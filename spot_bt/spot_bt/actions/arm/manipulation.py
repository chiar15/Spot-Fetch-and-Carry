"""
Copyright (c) 2026 Chiara Ferraioli

This module contains advanced manipulation behaviors for the Spot robot.
It implements picking logic with multi-stage success verification and 
image-based navigation to approach detected objects (WalkToObject).
"""

from __future__ import annotations

import time
from typing import Optional, Any

from action_msgs.msg import GoalStatus
from google.protobuf import wrappers_pb2
from py_trees.common import Status, Access

from bosdyn.api import manipulation_api_pb2
from bosdyn_msgs.conversions import convert
from spot_msgs.action import Manipulation
from synchros2.utilities import namespace_with

from spot_bt.utils.behaviour_bases import ManipulationBase, GOAL_STATUS_STRINGS
from spot_bt.utils.constants import HAND_FRAME


class PickObject(ManipulationBase):
    """
    Action behavior that commands Spot to pick up a detected object.

    This behavior extends ManipulationBase to perform an image-based grasp. 
    It implements a robust success verification logic that checks both 
    the API response state and the physical gripper aperture.

    Blackboard:
        manipulation_goal (Access.WRITE): Stores the generated goal for the action server.
        state (Access.READ/WRITE): Reads 'gripper_percentage' and 'obj_info'. 
            Updates 'holding_obj' and 'arm_stowed'.
    """

    def __init__(self, name: str, robot_name: Optional[str] = None):
        """
        Initializes the PickObject behavior and sets grasp parameters.

        Args:
            name (str): Name of the behavior.
            robot_name (Optional[str]): The namespace of the robot.
        """
        super().__init__(
            name=name,
            goal_key="manipulation_goal",
            generate_feedback_message=lambda msg: (
                f"Manipulation state: {msg.feedback.feedback.current_state}"
            ),
            robot_name=robot_name,
        )
        # Define the offset from palm to fingertip for precise grasping
        self.grasp_params = manipulation_api_pb2.GraspParams(
            grasp_palm_to_fingertip=0.5,
            grasp_params_frame_name=namespace_with(self.robot_name, HAND_FRAME),
        )

    def get_result_callback(self, future: Any) -> None:
        """
        Interprets the manipulation action result with deep inspection.

        The success is determined through three layers of verification:
        1. ROS Action status (must be STATUS_SUCCEEDED).
        2. Boston Dynamics API success flag.
        3. Internal state value (must be 6: MANIP_STATE_GRASP_SUCCEEDED).
        4. Sensor check: the gripper aperture must be > 5% to ensure an 
           object is actually held.

        Args:
            future: The ROS 2 action client future object.
        """
        result = future.result()
        self.result_message = result

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if result.result.success:
                final_state = result.result.result.current_state.value

                # State 6 corresponds to MANIP_STATE_GRASP_SUCCEEDED in BD API
                if final_state == 6:
                    # Physical verification via gripper sensors
                    if self.blackboard.state.gripper_percentage > 5.0:
                        self.result_status = GoalStatus.STATUS_SUCCEEDED
                        self.feedback_message = "Grasp succeeded"
                    else:
                        self.result_status = GoalStatus.STATUS_ABORTED
                        self.logger.info("✓ Grasp completed but gripper is empty!")
                        self.feedback_message = "Grasp failed (empty)"
                else:
                    self.result_status = GoalStatus.STATUS_ABORTED
                    self.logger.warning(
                        f"Goal completed but internal grasp failed. "
                        f"State: {final_state}, Message: {result.result.message}"
                    )
                    self.blackboard.set("state.holding_obj", False)
                    self.feedback_message = f"Grasp failed (state: {final_state})"
            else:
                self.result_status = GoalStatus.STATUS_ABORTED
                self.logger.error(f"✗ Manipulation logic failed: {result.result.message}")
                self.feedback_message = "Manipulation failed"
                self.blackboard.set("state.holding_obj", False)
        else:
            self.result_status = result.status
            self.logger.error(f"✗ ROS Goal failed with status: {result.status}")
            self.feedback_message = f"Goal failed (status: {result.status})"

        # Robot arm is no longer considered stowed after a pick attempt
        self.blackboard.set("state.arm_stowed", False)
        self.result_status_string = self.status_strings.get(self.result_status, "UNKNOWN")

    def update(self) -> Status:
        """
        Finalizes the behavior state based on sensory feedback.

        If the underlying action client reports SUCCESS, it performs a 
        final check on the 'gripper_percentage' (> 5.0) to set the 
        'holding_obj' flag on the blackboard.

        Returns:
            Status: SUCCESS if the object is held, FAILURE if initialization 
                failed or if the gripper is empty, otherwise returns the 
                action status (RUNNING/FAILURE).
        """
        if self._init_failed:
            return Status.FAILURE
        
        status = super().update()

        if status == Status.SUCCESS:
            gripper_pct = self.blackboard.state.gripper_percentage
            if gripper_pct > 5.0:
                self.blackboard.set("state.holding_obj", True)
                self.feedback_message = "Grasp succeeded"
                return Status.SUCCESS
            else:
                self.blackboard.set("state.holding_obj", False)
                self.logger.warning(f"✗ Physical grasp failed: gripper at {gripper_pct:.1f}%")
                self.feedback_message = "Grasp failed (sensor check)"
                return Status.FAILURE
            
        return status
    
    def _build_and_send_goal(self, center: Any, snapshot: Any, frame_name: str) -> None: 
        """
        Constructs and triggers the 'PickObjectInImage' command.

        Args:
            center: The pixel coordinates of the target.
            snapshot: The FrameTreeSnapshot for coordinate transformation.
            frame_name: The image sensor frame ID.
        """
        if self._init_failed:
            return

        grasp = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=center,
            transforms_snapshot_for_camera=snapshot,
            frame_name_image_sensor=frame_name,
            camera_model=self.pin_models[frame_name],
            grasp_params=self.grasp_params,
        )
        
        grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=grasp)
        grasp_goal = Manipulation.Goal()
        convert(grasp_request, grasp_goal.command)

        self.blackboard.set(name=self.goal_key, value=grasp_goal)
        self.logger.info("Sending grasp goal to action server...")


class WalkToObject(ManipulationBase):
    """
    Action behavior that commands Spot to approach a detected object.

    It calculates a navigation path to position the robot at a 1-meter 
    offset from the target, allowing for a better vantage point for grasping.

    Blackboard:
        walk_goal (Access.WRITE): Stores the approach goal for the action server.
        state (Access.WRITE): Resets 'obj_info' to None upon completion to 
            prevent re-triggering.
    """

    def __init__(self, name: str, robot_name: Optional[str] = None):
        """
        Initializes the WalkToObject behavior.

        Args:
            name (str): Name of the behavior.
            robot_name (Optional[str]): The namespace of the robot.
        """
        super().__init__(
            name=name,
            goal_key="walk_goal",
            generate_feedback_message=lambda msg: (
                f"Walk state: {msg.feedback.feedback.current_state}"
            ),
            robot_name=robot_name,
        )

    def get_result_callback(self, future: Any) -> None:
        """
        Handles the approach action result and resets detection state.

        If the approach succeeds, it resets 'obj_info' to None on the blackboard. 
        It includes a 3-second sleep to allow the robot's state and sensors 
        to stabilize before the next behavior (likely PickObject) starts.

        Args:
            future: The ROS 2 action client future object.
        """
        result = future.result()
        self.result_message = result

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if result.result.success:
                self.result_status = GoalStatus.STATUS_SUCCEEDED
                self.logger.info(f"✓ Walk completed: {result.result.message}")
                self.feedback_message = "Walk succeeded"
            else:
                self.result_status = GoalStatus.STATUS_ABORTED
                self.logger.error(f"✗ Walk failed (internal): {result.result.message}")
                self.feedback_message = "Manipulation failed"
        else:
            self.result_status = result.status
            self.logger.error(f"✗ ROS Goal failed with status: {result.status}")
            self.feedback_message = f"Goal failed (status: {result.status})"

        self.result_status_string = self.status_strings.get(self.result_status, "UNKNOWN")
        
        # Reset detection data to ensure a fresh scan for the next phase
        self.blackboard.set("state.obj_info", None)
        time.sleep(3.0)

    def _build_and_send_goal(self, center: Any, snapshot: Any, frame_name: str) -> None:
        """
        Constructs and triggers the 'WalkToObjectInImage' command.

        The command targets the object with a 1.0 meter offset distance 
        to ensure the robot stops at a safe and effective distance.

        Args:
            center: The pixel coordinates of the target.
            snapshot: The FrameTreeSnapshot for coordinate transformation.
            frame_name: The image sensor frame ID.
        """
        walk = manipulation_api_pb2.WalkToObjectInImage(
            pixel_xy=center,
            transforms_snapshot_for_camera=snapshot,
            frame_name_image_sensor=frame_name,
            camera_model=self.pin_models[frame_name],
            offset_distance=wrappers_pb2.FloatValue(value=1.0),
        )
        
        walk_request = manipulation_api_pb2.ManipulationApiRequest(walk_to_object_in_image=walk)
        walk_goal = Manipulation.Goal()
        convert(walk_request, walk_goal.command)

        self.blackboard.set(name=self.goal_key, value=walk_goal)
        self.logger.info("Sending walk goal (1m offset) to action server...")