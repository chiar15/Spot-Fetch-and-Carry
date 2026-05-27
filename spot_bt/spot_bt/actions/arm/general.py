"""
Copyright (c) 2026 Chiara Ferraioli

This module contains general arm actions for the Spot robot.
It provides behaviors for basic manipulator states (stow, unstow, carry) 
using service calls, as well as more complex arm positioning and 
extension commands using action clients and TF2 transformations.
"""

from __future__ import annotations

import math
from typing import Optional, Any

import rclpy
from py_trees.common import Status, Access
from py_trees_ros import action_clients
from tf2_ros import Buffer, TransformListener, TransformException

from std_srvs.srv import Trigger
from spot_msgs.action import RobotCommand
from synchros2.utilities import namespace_with
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME

from spot_bt.utils.behaviour_bases import ServiceClientTimeout
from spot_bt.utils.constants import BODY_FRAME, HAND_FRAME


class ArmStow(ServiceClientTimeout):
    """
    Action behavior that commands Spot to stow its arm.

    Blackboard:
        state (Access.WRITE): Updates 'arm_stowed' to True and 'arm_carrying' 
            to False upon successful completion.
    """
    
    def __init__(self, name: str, timeout_sec: float = 30.0):
        """
        Initializes the ArmStow behavior.

        Args:
            name (str): Name of the behavior.
            timeout_sec (float): Service call timeout in seconds. Defaults to 30.0.
        """
        super().__init__(
            name=name,
            service_type=Trigger,
            service_name="arm_stow",
            timeout_sec=timeout_sec
        )
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

    def initialise(self):
        """
        Initiates the arm stow service call and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [ArmStow::initialise()]")
        return super().initialise()
    
    def update(self) -> Status:
        """
        Monitors the status of the stow service call.

        If the service returns SUCCESS, it updates the blackboard state 
        to reflect that the arm is securely stowed and ensures the carry 
        flag is disabled.

        Returns:
            Status: SUCCESS if the arm is stowed, FAILURE if the service fails, 
                or RUNNING while waiting.
        """
        self.logger.debug(f"{self.name} [ArmStow::update()]")
        status = super().update()

        if status == Status.SUCCESS:
            self.blackboard.set("state.arm_stowed", True)
            self.blackboard.set("state.arm_carrying", False)
            return Status.SUCCESS
        elif status == Status.FAILURE:
            self.blackboard.set("state.arm_stowed", False)
            return Status.FAILURE
        
        return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """
        Cleans up service-related variables and logs termination.
        """
        self.logger.debug(f"{self.name} [ArmStow::terminate()]")
        return super().terminate(new_status)
    

class ArmUnstow(ServiceClientTimeout):
    """
    Action behavior that commands Spot to unstow (deploy) its arm.

    Blackboard:
        state (Access.WRITE): Updates 'arm_stowed' to False and 'arm_carrying' 
            to False upon successful completion.
    """
    
    def __init__(self, name: str, timeout_sec: float = 30.0):
        """
        Initializes the ArmUnstow behavior.
        """
        super().__init__(
            name=name,
            service_type=Trigger,
            service_name="arm_unstow",
            timeout_sec=timeout_sec
        )
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

    def initialise(self):
        """
        Initiates the arm unstow service call and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [ArmUnstow::initialise()]")
        return super().initialise()
    
    def update(self) -> Status:
        """
        Monitors the status of the unstow service call.

        If the service returns SUCCESS, it updates the blackboard state 
        to reflect that the arm is deployed and no longer stowed.

        Returns:
            Status: SUCCESS if deployed, FAILURE if it fails, or RUNNING while waiting.
        """
        self.logger.debug(f"{self.name} [ArmUnstow::update()]")
        status = super().update()

        if status == Status.SUCCESS:
            self.blackboard.set("state.arm_stowed", False)
            self.blackboard.set("state.arm_carrying", False)
            return Status.SUCCESS
        elif status == Status.FAILURE:
            self.blackboard.set("state.arm_stowed", True)
            return Status.FAILURE
        
        return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """
        Cleans up service-related variables and logs termination.
        """
        self.logger.debug(f"{self.name} [ArmUnstow::terminate()]")
        return super().terminate(new_status)


class ArmCarry(ServiceClientTimeout):
    """
    Action behavior that commands Spot's arm to move into the carry position.

    Blackboard:
        state (Access.WRITE): Updates 'arm_carrying' to True upon success.
    """
    
    def __init__(self, name: str, timeout_sec: float = 30.0):
        """
        Initializes the ArmCarry behavior.
        """
        super().__init__(
            name=name,
            service_type=Trigger,
            service_name="arm_carry",
            timeout_sec=timeout_sec
        )
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

    def initialise(self):
        """
        Initiates the arm carry service call and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [ArmCarry::initialise()]")
        return super().initialise()
    
    def update(self) -> Status:
        """
        Monitors the status of the carry service call.

        If the service returns SUCCESS, it updates the blackboard state 
        to reflect that the arm is in the carry position.

        Returns:
            Status: SUCCESS if position reached, FAILURE otherwise, or RUNNING.
        """
        self.logger.debug(f"{self.name} [ArmCarry::update()]")
        status = super().update()

        if status == Status.SUCCESS:
            self.blackboard.set("state.arm_carrying", True)
            return Status.SUCCESS
        elif status == Status.FAILURE:
            self.blackboard.set("state.arm_carrying", False)
            return Status.FAILURE
        
        return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """
        Cleans up service-related variables and logs termination.
        """
        self.logger.debug(f"{self.name} [ArmCarry::terminate()]")
        return super().terminate(new_status)
    

class ArmSafe(action_clients.FromBlackboard):
    """
    Action behavior that moves Spot's arm to a 'ready' or 'safe' position.

    Blackboard:
        arm_safe_goal (Access.WRITE): Writes the generated RobotCommand goal 
            to the blackboard before triggering execution.
    """

    def __init__(self, name: str, robot_name: Optional[str] = None):
        """
        Initializes the ArmSafe behavior.
        """
        self.goal_key = "arm_safe_goal"

        super().__init__(
            name=name,
            action_type=RobotCommand,
            action_name=namespace_with(robot_name, 'robot_command'),
            key=self.goal_key,
            generate_feedback_message=lambda msg: f"Moving arm... ({msg.feedback})"
        )

        self.blackboard.register_key(key=self.goal_key, access=Access.WRITE)
        self.odom_frame = namespace_with(robot_name, ODOM_FRAME_NAME)
        self.grav_aligned_frame = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)

    def initialise(self):
        """
        Constructs the 'arm_ready' command, sets it on the blackboard, 
        and initiates the action client via the base class.
        """
        self.logger.debug(f"{self.name} [ArmSafe::initialise()]")

        try:
            command = RobotCommandBuilder.arm_ready_command()

            action_goal = RobotCommand.Goal()
            convert(command, action_goal.command)

            self.blackboard.set(name=self.goal_key, value=action_goal)
            self.logger.info("Moving arm in safe position")
        except Exception as e:
            self.logger.error(f"Error creating goal: {e}")
            self.feedback_message = f"Goal error: {e}"
            return
        
        super().initialise()


class ArmExtendForward(action_clients.FromBlackboard):
    """
    Action behavior that extends Spot's arm forward along the body X-axis.

    Blackboard:
        arm_extend_goal (Access.WRITE): Writes the calculated movement goal 
            to the blackboard before triggering execution.
    """

    def __init__(
        self, 
        name: str, 
        robot_name: Optional[str] = None, 
        extend_meters: float = 0.1
    ):
        """
        Initializes the ArmExtendForward behavior.
        """
        self.goal_key = "arm_extend_goal"
        self.robot_name = robot_name
        self.extend_meters = extend_meters
        self.body_frame = namespace_with(robot_name, BODY_FRAME)
        self.hand_frame = namespace_with(robot_name, HAND_FRAME)
        self.tfBuffer = None
        self.listener = None

        super().__init__(
            name=name,
            action_type=RobotCommand,
            action_name=namespace_with(robot_name, "robot_command"),
            key=self.goal_key,
            generate_feedback_message=lambda msg: f"Extending arm... ({msg.feedback})"
        )

        self.blackboard.register_key(key=self.goal_key, access=Access.WRITE)

    def setup(self, **kwargs: Any):
        """
        Initializes the TF2 buffer and listener.
        """
        super().setup(**kwargs)

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            ) from e

        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self.node)

    def initialise(self):
        """
        Calculates the target forward pose using a TF lookup of the hand relative 
        to the body, sets the goal on the blackboard, and initiates the action.
        """
        self.logger.debug(f"{self.name} [ArmExtendForward::initialise()]")

        try:
            transform = self.tfBuffer.lookup_transform(
                self.body_frame,
                self.hand_frame,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            self.logger.warning(f"{self.name}: TF lookup failed: {ex}")
            self.feedback_message = f"TF lookup failed: {ex}"
            return

        try:
            t = transform.transform.translation
            target_pose = SE3Pose(
                x=t.x + self.extend_meters,
                y=t.y,
                z=t.z,
                rot=Quat.from_pitch(math.radians(90)),
            ).to_proto()

            command = RobotCommandBuilder.arm_pose_command_from_pose(
                hand_pose=target_pose,
                frame_name=BODY_FRAME,
                seconds=3,
            )

            action_goal = RobotCommand.Goal()
            convert(command, action_goal.command)

            self.blackboard.set(name=self.goal_key, value=action_goal)
            self.logger.info(
                f"Extending arm forward by {self.extend_meters}m with gripper down"
            )
        except Exception as e:
            self.logger.error(f"Error creating goal: {e}")
            self.feedback_message = f"Goal error: {e}"
            return

        super().initialise()