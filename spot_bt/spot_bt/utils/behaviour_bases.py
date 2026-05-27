"""
Copyright (c) 2026 Chiara Ferraioli

This module contains foundational behavior classes used to build 
specific actions for the Spot robot. It includes a service timeout wrapper, 
a base class for navigation, and a base class for arm manipulation that 
handles coordinate frame trees and TF lookups.
"""
from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees_ros import action_clients

from rclpy.client import Client as ServiceClient
from rclpy.client import Future
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from synchros2.utilities import namespace_with
from spot_msgs.action import NavigateTo
from typing import Optional
from action_msgs.msg import GoalStatus
from py_trees.common import Access

from spot_msgs.action import Manipulation
from bosdyn.api import geometry_pb2
from bosdyn.api.image_pb2 import ImageSource
from geometry_msgs.msg import TransformStamped
from bosdyn.client.frame_helpers import add_edge_to_tree
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import time
import rclpy
from spot_bt.utils.constants import *


class ServiceClientTimeout(Behaviour):
    """
    A generic behavior that calls a ROS 2 service and waits for a response 
    until a specified timeout is reached.
    """
    def __init__(self, name: str, service_type, service_name: str, timeout_sec: float):
        """
        Initializes the service client timeout behavior.

        Args:
            name (str): The name of the behavior.
            service_type: The ROS 2 service type class.
            service_name (str): The name of the service to call.
            timeout_sec (float): Timeout in seconds before returning FAILURE.
        """
        super().__init__(name)
        self.service_type = service_type
        self.service_name = service_name
        self.timeout = Duration(seconds=timeout_sec)
        self.client: ServiceClient = None
        self.future: Future = None 
        self.node: Node = None
        self.start_time: Time = None
        self.latest_result = None
        
    def setup(self, **kwargs):
        """
        Sets up the ROS 2 service client by retrieving the node from kwargs.

        Args:
            **kwargs: Must contain the 'node' object.

        Raises:
            KeyError: If 'node' is not found in kwargs.
        """
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs[{self.qualified_name}]"
            raise KeyError(error_msg) from e

        # Check whether the node has a 'robot_name' attribute and, if so, get its value.
        robot_name = getattr(self.node, 'robot_name', None)

        self.client = self.node.create_client(self.service_type, namespace_with(robot_name, self.service_name))

        self.logger.info("Waiting for carry service...")
        self.client.wait_for_service()
    
    def initialise(self):
        """Initiates the async service call and records the starting time."""
        self.future = self.client.call_async(self.service_type.Request())
        self.start_time = self.node.get_clock().now()

    def update(self):
        """
        Evaluates the future's status. Returns SUCCESS if the call succeeds, 
        FAILURE on timeout or error, and RUNNING otherwise.

        Returns:
            Status: The current state of the behavior.
        """
        if self.future is not None:
            if self.future.done():
                result = self.future.result()
                self.latest_result = result
                return Status.SUCCESS if result.success else Status.FAILURE
            
            current_time = self.node.get_clock().now()

            if self.start_time and (current_time - self.start_time) >= self.timeout:
                self.logger.warning(f"{self.name}: service timeout")
                return Status.FAILURE
            
            return Status.RUNNING
        
        self.logger.warning(f"{self.name}: service not answering")
        return Status.FAILURE
    
    def terminate(self, new_status):
        """Clears the future and timer variables upon completion or cancellation."""
        self.future = None
        self.start_time = None

class NavigateBase(action_clients.FromBlackboard):
    """
    Base class for navigation behaviors. Inherits from py_trees_ros action_clients
    to interface seamlessly with the custom navigation action server.
    """
    def __init__(self, name: str, robot_name: Optional[str] = None):
        """
        Initializes the NavigateBase behavior.

        Args:
            name (str): The name of the behavior.
            robot_name (Optional[str]): The namespace of the robot.
        """
        self.goal_key = "navigation_goal"
        self.robot_name = robot_name
        
        super().__init__(
            name=name,
            action_type=NavigateTo,
            action_name=namespace_with(robot_name, "navigate_custom"),
            key=self.goal_key,
            generate_feedback_message=lambda msg: f"Navigation Feedback: {msg.feedback}"
        )
        
        self.blackboard.register_key(key="state", access=Access.READ)
        self.blackboard.register_key(key="state", access=Access.WRITE)
        self.blackboard.register_key(key=self.goal_key, access=Access.WRITE)
    
    def setup(self, **kwargs):
        """
        Validates parameters during the py_trees setup phase.

        Args:
            **kwargs: Must contain the 'node' object.

        Raises:
            KeyError: If 'node' is not found in kwargs.
        """
        super().setup(**kwargs)
        self.logger.debug(f"{self.name} [NavigateBase::setup()]")
        
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs[{self.qualified_name}]"
            raise KeyError(error_msg) from e
    
    def initialise(self):
        """Prepares the action client for execution."""
        return super().initialise()
    
    def _send_goal(self, waypoint_id: str, description: str = ""):
        """
        Helper method to construct and save the navigation goal to the blackboard.

        Args:
            waypoint_id (str): The target destination waypoint ID.
            description (str, optional): A brief log description for the action.
        """
        try:
            navigate_goal = NavigateTo.Goal()
            navigate_goal.waypoint_id = waypoint_id
            
            self.blackboard.set(name=self.goal_key, value=navigate_goal)
            
            log_msg = f"Sending navigation goal: {description}" if description else f"Sending goal to {waypoint_id}"
            self.logger.info(log_msg)
            
        except Exception as e:
            self.logger.error(f"Errore nella costruzione del goal: {e}")
            self.feedback_message = f"Goal construction error: {e}"
            raise
    
    def get_result_callback(self, future):
        """
        Common callback to handle the completion of the navigation action.
        Sets internal status based on the underlying GoalStatus.
        """
        result = future.result()
        self.result_message = result
        
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success:
            self.result_status = GoalStatus.STATUS_SUCCEEDED
            self.logger.info("Navigation successfully completed!")
            self.feedback_message = "Navigation succeeded"
            self._on_navigation_success()  
        else:
            self.result_status = GoalStatus.STATUS_ABORTED
            self.logger.warning("Navigation failed")
            self.feedback_message = "Navigation Failed"
            self._on_navigation_failure()  
        
        self.result_status_string = self.status_strings.get(
            self.result_status, 
            "UNKNOWN"
        )
    
    def _on_navigation_success(self):
        """Hook meant to be overridden by subclasses for post-success logic."""
        pass
    
    def _on_navigation_failure(self):
        """Hook meant to be overridden by subclasses for post-failure logic."""
        pass
    
    def cancel_response_callback(self, future):
        """Handles cancellation responses from the action server."""
        cancel_response = future.result()
        print(f"[CANCEL RESPONSE] goals_canceling={cancel_response.goals_canceling}", flush=True)
        super().cancel_response_callback(future)

GOAL_STATUS_STRINGS = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_ABORTED:   "ABORTED",
    GoalStatus.STATUS_CANCELED:  "CANCELED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_ACCEPTED:  "ACCEPTED",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    0: "UNKNOWN"
}


class ManipulationBase(action_clients.FromBlackboard):
    """
    Abstract base class for object manipulation behaviors in the Spot robot's behavior tree.

    This class extends the py_trees ROS 2 action client to interface with the custom 
    'manipulation' action server. It centralizes the heavy lifting of spatial coordinate 
    transformations required by the Boston Dynamics API. 
    
    Key responsibilities include:
    - Managing the TF2 buffer and listeners to track coordinate frames.
    - Caching the robot's static frame tree (e.g., from the body to individual fisheye cameras).
    - Dynamically looking up odometry and vision transforms at the exact timestamp of an object detection.
    - Constructing a complete `FrameTreeSnapshot` to ensure the robot has the full spatial 
      context needed to safely interact with objects.

    Note:
        This class is meant to be inherited by specific manipulation actions (e.g., PickObject, 
        WalkToObject). Subclasses MUST implement `_build_and_send_goal()` to define the exact 
        manipulation task, and `get_result_callback()` to handle the action server's response.
    """
    def __init__(
        self,
        name: str,
        goal_key: str,
        generate_feedback_message,
        robot_name: Optional[str] = None,
    ):
        """
        Initializes the manipulation base behavior.

        Args:
            name (str): The name of the behavior.
            goal_key (str): The blackboard key storing the target goal.
            generate_feedback_message (callable): Function to format feedback messages.
            robot_name (Optional[str]): The namespace of the robot.
        """
        self.goal_key     = goal_key
        self.robot_name   = robot_name
        self.body_frame   = namespace_with(self.robot_name, BODY_FRAME)
        self.odom_frame   = namespace_with(self.robot_name, ODOM_FRAME)
        self.vision_frame = namespace_with(self.robot_name, VISION_FRAME)

        super().__init__(
            name=name,
            action_type=Manipulation,
            action_name=namespace_with(self.robot_name, "manipulation"),
            key=self.goal_key,
            generate_feedback_message=generate_feedback_message,
        )

        self.blackboard.register_key(key="state",       access=Access.READ)
        self.blackboard.register_key(key="state",       access=Access.WRITE)
        self.blackboard.register_key(key=self.goal_key, access=Access.WRITE)

        self.static_transforms = {}
        self.pin_models = self._create_pin_models()
        self._init_failed = False

    def setup(self, **kwargs):
        """
        Initializes the TF buffer and builds the static frame tree cache.

        Args:
            **kwargs: Must contain the 'node' object.

        Raises:
            KeyError: If 'node' is not found in kwargs.
            RuntimeError: If the static frame tree fails to build.
        """
        super().setup(**kwargs)
        self.logger.debug(f"{self.name} [{self.__class__.__name__}::setup()]")

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            ) from e

        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self.node)

        if not self._build_static_frame_tree():
            raise RuntimeError("Failed to build static frame tree")

    def initialise(self):
        """
        Reads object detection info from the blackboard, retrieves the necessary 
        dynamic transforms (odom/vision to body) for the exact detection timestamp, 
        creates a frame tree snapshot, and delegates goal construction to the subclass.
        """
        self.logger.debug(f"{self.name} [{self.__class__.__name__}::initialise()]")
        self._init_failed = False

        try:
            spot_state = self.blackboard.state
            self.logger.warning(f"Object Detected: {spot_state.obj_info}")

            if spot_state.obj_info is None:
                self.logger.warning(f"{self.name}: obj_info is None, skipping goal construction")
                self.feedback_message = "No object detected"
                self._init_failed = True

            obj_info   = spot_state.obj_info
            frame_name = obj_info.frame_id
            center     = obj_info.center
            look_time  = obj_info.timestamp

            try:
                odom_to_body = self.tfBuffer.lookup_transform(
                    self.body_frame, self.odom_frame, look_time
                )
                vision_to_body = self.tfBuffer.lookup_transform(
                    self.body_frame, self.vision_frame, look_time
                )
            except TransformException as ex:
                self.logger.warning(f"Lookup failed: {ex}, retrying...")
                self.feedback_message = f"TF lookup failed: {ex}"
                return  

            snapshot = self._create_frame_tree_snapshot(
                self._tformstamp_to_tform(odom_to_body),
                self._tformstamp_to_tform(vision_to_body),
            )

            self._build_and_send_goal(center=center, snapshot=snapshot, frame_name=frame_name)
            self.logger.info("Frame Tree Snapshot Created")

        except KeyError as e:
            self.logger.error(f"Chiave mancante sulla blackboard: {e}")
            self.feedback_message = f"Missing blackboard key: {e}"
            return
        except Exception as e:
            self.logger.error(f"Errore nella costruzione del goal: {e}")
            self.feedback_message = f"Goal construction error: {e}"
            return

        super().initialise()

    def _build_and_send_goal(self, center, snapshot, frame_name) -> None:
        """
        Builds the manipulation specific goal and stores it on the blackboard.
        Must be implemented by subclasses.

        Raises:
            NotImplementedError: Because this is an abstract base method.
        """
        raise NotImplementedError(
            f"{self.__class__.__name__} must implement _build_and_send_goal()"
        )

    def get_result_callback(self, future) -> None:
        """
        Callback to process the outcome of the manipulation action.
        Must be implemented by subclasses.

        Raises:
            NotImplementedError: Because this is an abstract base method.
        """
        raise NotImplementedError(
            f"{self.__class__.__name__} must implement get_result_callback()"
        )

    def _create_pin_models(self) -> dict:
        """
        Generates PinholeCameraModel objects from the intrinsic parameters.

        Returns:
            dict: A dictionary mapping camera frame names to their respective models.
        """
        pin_models = {}

        for frame_name, intrinsics in CAMERA_INTRINSICS.items():
            focal_length = geometry_pb2.Vec2(x=intrinsics['fx'], y=intrinsics['fy'])
            center       = geometry_pb2.Vec2(x=intrinsics['cx'], y=intrinsics['cy'])

            camera_intrinsics = ImageSource.PinholeModel.CameraIntrinsics(
                focal_length=focal_length, principal_point=center
            )
            pin_models[frame_name] = ImageSource.PinholeModel(intrinsics=camera_intrinsics)

        return pin_models

    def _build_static_frame_tree(self) -> bool:
        """
        Constructs and caches ALL static transformations, including intermediate frames.
        Waits for each transform to become available in the TF tree before continuing.

        Returns:
            bool: True if all static transforms were successfully cached, False on timeout.
        """
        lookup_time = rclpy.time.Time()

        static_edges = [
            # Body → Head
            (BODY_FRAME, HEAD_FRAME),
            # Head → camera frames (intermediates)
            (HEAD_FRAME, FRONTRIGHT_FRAME),
            (HEAD_FRAME, FRONTLEFT_FRAME),
            (HEAD_FRAME, RIGHT_FRAME),
            (HEAD_FRAME, LEFT_FRAME),
            (HEAD_FRAME, BACK_FRAME),
            # Camera frames → fisheye (finals)
            (FRONTRIGHT_FRAME, FRONTRIGHT_CAMERA_FRAME),
            (FRONTLEFT_FRAME,  FRONTLEFT_CAMERA_FRAME),
            (RIGHT_FRAME,      RIGHT_CAMERA_FRAME),
            (LEFT_FRAME,       LEFT_CAMERA_FRAME),
            (BACK_FRAME,       BACK_CAMERA_FRAME),
        ]

        timeout = 10.0

        for parent, child in static_edges:
            parent = namespace_with(self.robot_name, parent)
            child  = namespace_with(self.robot_name, child)

            self.logger.info(f"Waiting for transform {parent} → {child}...")

            start = time.time()
            while (time.time() - start) < timeout:
                if self.tfBuffer.can_transform(
                    parent,
                    child,
                    lookup_time,
                    timeout=rclpy.duration.Duration(seconds=0.1),
                ):
                    try:
                        transform = self.tfBuffer.lookup_transform(parent, child, lookup_time)
                        key = f"{parent}_to_{child}"
                        self.static_transforms[key] = {
                            'parent':    parent,
                            'child':     child,
                            'transform': self._tformstamp_to_tform(transform, parent, child),
                        }
                        self.logger.info(f"✓ Cached: {parent} → {child}")
                        break

                    except TransformException as ex:
                        self.logger.warning(f"Lookup failed: {ex}, retrying...")

                rclpy.spin_once(self.node, timeout_sec=0.1)
                time.sleep(0.1)
            else:
                self.logger.error(f"✗ Timeout waiting for {parent} → {child}")
                return False

        self.logger.info(f"✓ Successfully cached {len(self.static_transforms)} static transforms")
        return True

    def _create_frame_tree_snapshot(self, odom_to_body, vision_to_body):
        """
        Compiles a comprehensive FrameTreeSnapshot including body, odom, vision, 
        and all cached static transformations for the specific timestamp.

        Args:
            odom_to_body: Dynamic transform from odom to body.
            vision_to_body: Dynamic transform from vision to body.

        Returns:
            FrameTreeSnapshot: The generated snapshot for the Boston Dynamics API.
        """
        frame_tree_edges = {}

        body_tf = geometry_pb2.SE3Pose(
            position=geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0),
            rotation=geometry_pb2.Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
        )
        frame_tree_edges = add_edge_to_tree(frame_tree_edges, body_tf, "", self.body_frame)

        frame_tree_edges = add_edge_to_tree(
            frame_tree_edges, odom_to_body, self.body_frame, self.odom_frame
        )
        frame_tree_edges = add_edge_to_tree(
            frame_tree_edges, vision_to_body, self.body_frame, self.vision_frame
        )

        for _, edge_data in self.static_transforms.items():
            frame_tree_edges = add_edge_to_tree(
                frame_tree_edges,
                edge_data['transform'],
                edge_data['parent'],
                edge_data['child'],
            )
            self.logger.debug(f"Added: {edge_data['parent']} → {edge_data['child']}")

        return geometry_pb2.FrameTreeSnapshot(child_to_parent_edge_map=frame_tree_edges)

    def _tformstamp_to_tform(self, tformstamp: TransformStamped, parent=None, child=None):
        """
        Converts a ROS 2 TransformStamped message into a Spot API SE3Pose object.
        Automatically handles orientation corrections for specific Spot cameras.

        Args:
            tformstamp (TransformStamped): The ROS transform to convert.
            parent (str, optional): The parent frame name.
            child (str, optional): The child frame name.

        Returns:
            SE3Pose: The equivalent Boston Dynamics transform.
        """
        traslation = tformstamp.transform.translation
        rotation1  = tformstamp.transform.rotation

        position  = geometry_pb2.Vec3(x=traslation.x, y=traslation.y, z=traslation.z)
        rotation2 = geometry_pb2.Quaternion(
            w=rotation1.w, x=rotation1.x, y=rotation1.y, z=rotation1.z
        )

        if parent is not None and child is not None:
            if parent == namespace_with(self.robot_name, HEAD_FRAME) and (
                child == namespace_with(self.robot_name, RIGHT_FRAME)
                or child == namespace_with(self.robot_name, BACK_FRAME)
            ):
                rotation2 = geometry_pb2.Quaternion(
                    w=-rotation1.w, x=-rotation1.x, y=-rotation1.y, z=-rotation1.z
                )

        transform = geometry_pb2.SE3Pose(position=position, rotation=rotation2)
        return transform
