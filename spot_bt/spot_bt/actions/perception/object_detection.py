"""
Copyright (c) 2026 Chiara Ferraioli

This module contains action behaviors for object perception and detection management.
It includes behaviors to trigger the object detection service and to manage 
cooldown periods after a failure to prevent immediate re-triggering.
"""

from __future__ import annotations

from py_trees.common import Status, Access
from py_trees.behaviour import Behaviour

from bosdyn.api.geometry_pb2 import Vec2

from spot_bt.utils.behaviour_bases import ServiceClientTimeout
from bt_interfaces.srv import Detection
from spot_bt.utils.data import DetectedObject
import rclpy


class ObjectDetection(ServiceClientTimeout):
    """
    Action behavior that triggers the object detection service.

    This node calls an external detection service and, upon success, packages 
    the result into a DetectedObject structure to be used by other nodes (e.g., picking).

    Blackboard:
        state (Access.WRITE): Updates 'obj_detected' flag and stores detection 
            details in 'obj_info'.
    """

    def __init__(self, name: str, timeout_sec: float = 30.0):
        """
        Initializes the ObjectDetection behavior.

        Args:
            name (str): Name of the behavior.
            timeout_sec (float): Service call timeout in seconds. Defaults to 30.0.
        """
        super().__init__(
            name=name,
            service_type=Detection,
            service_name="object_detection",
            timeout_sec=timeout_sec
        )

        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

    def initialise(self):
        """
        Initiates the object detection service call and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [ObjectDetection::initialise()]")
        return super().initialise()
    
    def update(self) -> Status:
        """
        Processes the result of the detection service.

        If the service returns SUCCESS, it sets 'obj_detected' to True and 
        constructs a DetectedObject containing the center coordinates (Vec2), 
        the ROS timestamp, and the frame ID, storing it in 'obj_info'.
        
        If the service returns FAILURE, it sets 'obj_detected' to False but 
        returns SUCCESS to allow the tree flow to continue (e.g., to exploration).

        Returns:
            Status: SUCCESS if the service completes (regardless of finding an 
                object), or RUNNING while waiting.
        """
        self.logger.debug(f"{self.name} [ObjectDetection::update()]")
        status = super().update()

        if status == Status.SUCCESS:
            # Object found and service succeeded
            self.blackboard.set("state.obj_detected", True)
            
            # Package detection data into a custom utility class
            obj_info = DetectedObject(
                center=Vec2(x=self.latest_result.x_center, y=self.latest_result.y_center),
                timestamp=rclpy.time.Time.from_msg(self.latest_result.timestamp),
                frame_id=self.latest_result.frame_id
            )
            self.blackboard.set("state.obj_info", obj_info)
            return Status.SUCCESS
        
        elif status == Status.FAILURE:
            # Service failed or timed out, reset detection flag
            self.blackboard.set("state.obj_detected", False)
            return Status.SUCCESS
        
        self.logger.debug(f"{self.name} [ObjectDetection::update()][RUNNING]")
        return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """
        Cleans up service variables and logs termination.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [ObjectDetection::terminate()]")
        return super().terminate(new_status)


class SetCooldown(Behaviour):
    """
    Action behavior that initializes a cooldown period on the blackboard.

    This node records the current time as the 'last_fail_time' to block 
    subsequent detection attempts for a specified duration.

    Blackboard:
        state (Access.WRITE): Updates 'last_fail_time' and 'cooldown_dr'.
    """

    def __init__(self, name: str, cooldown_sec: int = 30):
        """
        Initializes the SetCooldown behavior.

        Args:
            name (str): Name of the behavior.
            cooldown_sec (int): Duration of the cooldown in seconds. Defaults to 30.
        """
        super().__init__(name)
        self.cooldown_dr = rclpy.duration.Duration(seconds=cooldown_sec)
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

    def setup(self, **kwargs):
        """
        Retrieves the ROS clock from the node.

        Args:
            **kwargs: Arbitrary keyword arguments, including the ROS 2 node.
        """
        self.logger.debug(f"{self.qualified_name}::setup()")
        try:
            node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs[{self.qualified_name}]"
            raise KeyError(error_msg) from e
        
        self.clock = node.get_clock()

    def update(self) -> Status:
        """
        Updates the blackboard with the current time and cooldown duration.

        Returns:
            Status: Always returns SUCCESS.
        """
        self.logger.debug(f" {self.name} [SetCooldown::update()]")
        now = self.clock.now()

        self.blackboard.set("state.last_fail_time", now)
        self.blackboard.set("state.cooldown_dr", self.cooldown_dr)
        
        return Status.SUCCESS