"""
Copyright (c) 2026 Chiara Ferraioli

This module contains action behaviors to control Spot's body movement and posture.
It provides service-based actions for standing, sitting, and stopping, as well as 
complex movements like backward navigation and rotations using TF2-based goal calculation.
"""

from __future__ import annotations

import math
import time
from typing import Optional, Any

import rclpy
from py_trees.common import Status, Access
from py_trees_ros import action_clients
from tf2_ros import Buffer, TransformListener, TransformException

from std_srvs.srv import Trigger
from spot_msgs.action import RobotCommand
from synchros2.utilities import namespace_with
from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from spot_bt.utils.constants import BODY_FRAME, VISION_FRAME
from spot_bt.utils.behaviour_bases import ServiceClientTimeout


class Stand(ServiceClientTimeout):
    """
    Action behavior that commands Spot to transition to a standing posture.

    Blackboard:
        state (Access.WRITE): Updates 'standing' status flag.
    """

    def __init__(self, name: str, timeout_sec: float = 30.0):
        """
        Initializes the Stand behavior and registers blackboard keys.
        """
        super().__init__(
            name=name, 
            service_type=Trigger,
            service_name="stand",
            timeout_sec=timeout_sec
        )
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

    def initialise(self):
        """
        Initiates the stand service call and logs the start of the posture transition.
        """
        self.logger.debug(f"{self.name} [Stand::initialise()]")
        return super().initialise()
    
    def update(self) -> Status:
        """
        Processes the result of the stand service call.

        If the service returns SUCCESS, the blackboard flag 'state.standing' is 
        set to True. If it returns FAILURE, the flag is set to False to reflect 
        the inconsistent state.

        Returns:
            Status: SUCCESS if standing, FAILURE if the service failed, 
                or RUNNING while waiting.
        """
        self.logger.debug(f"{self.name} [Stand::update()]")
        status = super().update()

        if status == Status.SUCCESS:
            self.blackboard.set("state.standing", True)
            return Status.SUCCESS
        elif status == Status.FAILURE:
            self.blackboard.set("state.standing", False)
            return Status.FAILURE
        
        return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """
        Cleans up service variables and logs the behavior termination.
        """
        self.logger.debug(f"{self.name} [Stand::terminate()]")
        return super().terminate(new_status)


class Sit(ServiceClientTimeout):
    """
    Action behavior that commands Spot to transition to a sitting posture.

    Blackboard:
        state (Access.WRITE): Updates 'standing' status flag.
    """

    def __init__(self, name: str, timeout_sec: float = 30.0):
        """
        Initializes the Sit behavior and registers blackboard keys.
        """
        super().__init__(
            name=name, 
            service_type=Trigger,
            service_name="sit",
            timeout_sec=timeout_sec
        )
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

    def initialise(self):
        """
        Initiates the sit service call and logs the start of the posture transition.
        """
        self.logger.debug(f"{self.name} [Sit::initialise()]")
        return super().initialise()
    
    def update(self) -> Status:
        """
        Processes the result of the sit service call.

        If the service returns SUCCESS, 'state.standing' is set to False. 
        In case of FAILURE, it is set to True as the robot presumably 
        failed to sit.

        Returns:
            Status: SUCCESS, FAILURE, or RUNNING.
        """
        self.logger.debug(f"{self.name} [Sit::update()]")
        status = super().update()

        if status == Status.SUCCESS:
            self.blackboard.set("state.standing", False)
            return Status.SUCCESS
        elif status == Status.FAILURE:
            self.blackboard.set("state.standing", True)
            return Status.FAILURE
        
        return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """
        Cleans up service variables and logs the behavior termination.
        """
        self.logger.debug(f"{self.name} [Sit::terminate()]")
        return super().terminate(new_status)


class Stop(ServiceClientTimeout):
    """
    Action behavior that commands Spot to stop all body movement.

    Blackboard:
        state (Access.WRITE): Resets 'standing' flag upon success.
    """

    def __init__(self, name: str, timeout: float = 30.0):
        """
        Initializes the Stop behavior and registers blackboard keys.
        """
        super().__init__(
            name=name,
            service_name="stop",
            service_type=Trigger,
            timeout_sec=timeout
        )
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)
        
    def initialise(self):
        """
        Initiates the stop service call to halt robot movement.
        """
        self.logger.debug(f"{self.name} [Stop::initialise()]")
        super().initialise()

    def update(self) -> Status:
        """
        Processes the result of the stop service call.

        Upon SUCCESS, the 'state.standing' flag is reset to False. 
        Upon FAILURE, it remains or is set to True.

        Returns:
            Status: SUCCESS, FAILURE, or RUNNING.
        """
        self.logger.debug(f"{self.name} [Stop::update()]")
        status = super().update()

        if status == Status.SUCCESS:
            self.blackboard.set("state.standing", False)
            return Status.SUCCESS
        elif status == Status.FAILURE:
            self.blackboard.set("state.standing", True)
            return Status.FAILURE
        
        return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """
        Cleans up service variables and logs the behavior termination.
        """
        self.logger.debug(f"{self.name} [Stop::terminate()]")
        return super().terminate(new_status)
    

class MoveBackward(action_clients.FromBlackboard):
    """
    Action behavior that commands Spot to move backward relative to its current pose.

    It computes a target trajectory point in the Vision frame by applying 
    a negative X offset to the current robot localization.

    Blackboard:
        move_backward_goal (Access.WRITE): Stores the calculated trajectory goal.
    """

    def __init__(self, name: str, robot_name: Optional[str] = None, distance: float = -0.2):
        """
        Initializes the MoveBackward behavior and defines frames.
        """
        self.goal_key = 'move_backward_goal'
        self.distance = distance
        
        super().__init__(
            name=name,
            action_type=RobotCommand,
            action_name=namespace_with(robot_name, "robot_command"),
            key=self.goal_key,
            generate_feedback_message=lambda msg: f"Moving... ({msg.feedback})"
        )

        self.blackboard.register_key(key=self.goal_key, access=Access.WRITE)
        self.body_frame = namespace_with(robot_name, BODY_FRAME)
        self.vision_frame = namespace_with(robot_name, VISION_FRAME)

    def setup(self, **kwargs: Any):
        """
        Initializes TF2 components and waits for the Vision->Body transform.
        """
        super().setup(**kwargs)
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(f"Missing 'node' in setup kwargs [{self.qualified_name}]") from e
        
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self.node)
        self._wait_for_transform()

    def _wait_for_transform(self, timeout: float = 10.0):
        """
        Blocks until the necessary transform is available or timeout is reached.
        """
        start = time.time()
        while (time.time() - start) < timeout:
            try: 
                if self.tfBuffer.can_transform(self.vision_frame, self.body_frame, rclpy.time.Time()):
                    return
            except TransformException:
                pass
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)
        raise RuntimeError(f"TF Timeout: {self.vision_frame} -> {self.body_frame}")
    
    def initialise(self):
        """
        Initalizes the backward movement command.

        It performs a TF lookup of the current pose, calculates the target SE2 
        pose behind the robot, constructs the RobotCommand goal, and writes 
        it to the blackboard to trigger the base action client.
        """
        self.logger.debug(f"{self.name} [MoveBackward::initialise()]")

        try:
            world_t_robot = self.tfBuffer.lookup_transform(self.vision_frame, self.body_frame, rclpy.time.Time())
            
            world_t_robot_se2 = SE3Pose(
                world_t_robot.transform.translation.x,
                world_t_robot.transform.translation.y,
                world_t_robot.transform.translation.z,
                Quat(
                    world_t_robot.transform.rotation.w,
                    world_t_robot.transform.rotation.x,
                    world_t_robot.transform.rotation.y,
                    world_t_robot.transform.rotation.z,
                ),
            ).get_closest_se2_transform()
            
            robot_t_goal = SE2Pose(self.distance, 0.0, 0.0)
            world_t_goal = world_t_robot_se2 * robot_t_goal
            
            proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=world_t_goal.x, goal_y=world_t_goal.y, goal_heading=world_t_goal.angle,
                frame_name=self.vision_frame
            )
            
            action_goal = RobotCommand.Goal()
            convert(proto_goal, action_goal.command)
            
            self.blackboard.set(name=self.goal_key, value=action_goal)
            self.logger.info(f"Moving backward {abs(self.distance)}m")
            
        except Exception as e:
            self.logger.error(f"Goal construction failed: {e}")
            self.feedback_message = f"Goal error: {e}"
            return
        
        super().initialise()

    def terminate(self, new_status: Status):
        """
        Logs termination and pauses to ensure movement stabilization.
        """
        time.sleep(2.0)
        return super().terminate(new_status)
    

class RotateRight(action_clients.FromBlackboard):
    """
    Action behavior that commands Spot to rotate in place.

    It calculates a target orientation in the Vision frame based on a 
    relative angular offset from the current robot heading.

    Blackboard:
        rotate_right_goal (Access.WRITE): Stores the calculated rotation goal.
    """

    def __init__(self, name: str, robot_name: Optional[str] = None, angle: float = -math.pi / 2):
        """
        Initializes the RotateRight behavior and defines rotation parameters.
        """
        self.goal_key = 'rotate_right_goal'
        self.angle = angle

        super().__init__(
            name=name,
            action_type=RobotCommand,
            action_name=namespace_with(robot_name, "robot_command"),
            key=self.goal_key,
            generate_feedback_message=lambda msg: f"Rotating... ({msg.feedback})"
        )

        self.blackboard.register_key(key=self.goal_key, access=Access.WRITE)
        self.body_frame = namespace_with(robot_name, BODY_FRAME)
        self.vision_frame = namespace_with(robot_name, VISION_FRAME)

    def setup(self, **kwargs: Any):
        """
        Initializes TF2 components and waits for the Vision->Body transform.
        """
        super().setup(**kwargs)
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(f"Missing 'node' in setup kwargs [{self.qualified_name}]") from e

        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self.node)
        self._wait_for_transform()

    def _wait_for_transform(self, timeout: float = 10.0):
        """
        Blocks until the necessary transform is available.
        """
        start = time.time()
        while (time.time() - start) < timeout:
            try:
                if self.tfBuffer.can_transform(self.vision_frame, self.body_frame, rclpy.time.Time()):
                    return
            except TransformException:
                pass
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)
        raise RuntimeError(f"TF Timeout: {self.vision_frame} -> {self.body_frame}")

    def initialise(self):
        """
        Initializes the rotation command.

        It looks up the current localization, applies the angular offset (angle), 
        builds the RobotCommand trajectory goal, and stores it on the blackboard.
        """
        self.logger.debug(f"{self.name} [RotateRight::initialise()]")

        try:
            world_t_robot = self.tfBuffer.lookup_transform(self.vision_frame, self.body_frame, rclpy.time.Time())

            world_t_robot_se2 = SE3Pose(
                world_t_robot.transform.translation.x,
                world_t_robot.transform.translation.y,
                world_t_robot.transform.translation.z,
                Quat(
                    world_t_robot.transform.rotation.w, world_t_robot.transform.rotation.x,
                    world_t_robot.transform.rotation.y, world_t_robot.transform.rotation.z,
                ),
            ).get_closest_se2_transform()

            robot_t_goal = SE2Pose(0.0, 0.0, self.angle)
            world_t_goal = world_t_robot_se2 * robot_t_goal

            proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=world_t_goal.x, goal_y=world_t_goal.y, goal_heading=world_t_goal.angle,
                frame_name=self.vision_frame,
            )

            action_goal = RobotCommand.Goal()
            convert(proto_goal, action_goal.command)

            self.blackboard.set(name=self.goal_key, value=action_goal)
            self.logger.info(f"Rotating by {math.degrees(self.angle):.1f} degrees")

        except Exception as e:
            self.logger.error(f"Goal construction failed: {e}")
            self.feedback_message = f"Goal error: {e}"
            return

        super().initialise()

    def terminate(self, new_status: Status):
        """
        Logs termination and pauses for stabilization.
        """
        time.sleep(2.0)
        return super().terminate(new_status)