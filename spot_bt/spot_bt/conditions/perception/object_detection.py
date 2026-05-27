"""
Copyright (c) 2026 Chiara Ferraioli

This module contains perception-related condition behaviors for the Spot robot.
It includes checks for object detection and cooldown states, allowing the 
behavior tree to react to the presence of target objects or to wait 
between detection attempts.
"""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from rclpy.clock import Clock


class IsObjectDetected(Behaviour):
    """
    Condition behavior that checks if a target object has been detected.

    Blackboard:
        state (Access.READ): Reads 'obj_detected' to determine if an object is present.
    """

    def __init__(self, name: str):
        """
        Initializes the behavior and attaches to the status blackboard.

        Args:
            name (str): Name of the behavior.
        """
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)
    
    def initialise(self):
        """
        Initiates the object detection status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsObjectDetected::initialise()]")

    def update(self) -> Status:
        """
        Evaluates the object detection status by checking the blackboard flag.

        Returns:
            Status: SUCCESS if 'obj_detected' is True, FAILURE otherwise.
        """
        self.logger.debug(f"{self.name} [IsObjectDetected::update()]")
        
        if self.blackboard.state.obj_detected:
            return Status.SUCCESS

        return Status.FAILURE
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsObjectDetected::terminate()][{self.status}->{new_status}]")


class IsInCooldown(Behaviour):
    """
    Condition behavior that checks if the robot is currently in a cooldown period.

    A cooldown is active if the time elapsed since the last grasp failure 
    or the last object dropping is less than the specified cooldown duration.

    Blackboard:
        state (Access.READ): Reads 'last_fail_time' and 'cooldown_dr' to calculate elapsed time.
    """

    def __init__(self, name: str):
        """
        Initializes the behavior and attaches to the status blackboard.

        Args:
            name (str): Name of the behavior.
        """
        super().__init__(name)
        self.clock: Clock = None
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def setup(self, **kwargs):
        """
        Initializes the ROS clock from the provided node.

        Args:
            **kwargs: Arbitrary keyword arguments, including the ROS 2 node.

        Raises:
            KeyError: If 'node' is not found in setup's kwargs.
        """
        self.logger.debug(f"{self.qualified_name}::setup()")
        try:
            node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs[{self.qualified_name}]"
            raise KeyError(error_msg) from e
        
        self.clock = node.get_clock()

    def initialise(self):
        """
        Initiates the cooldown status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsInCooldown::initialise()]")
    
    def update(self) -> Status:
        """
        Evaluates if the robot is currently in a cooldown state by calculating the elapsed time.

        It compares the current time with the 'last_fail_time' recorded on the blackboard 
        against the allowed 'cooldown_dr' duration.

        Returns:
            Status: SUCCESS if the robot is still within the cooldown period, 
                FAILURE if no previous failure is recorded or if the period has expired.
        """
        self.logger.debug(f"{self.name} [IsInCooldown::update()]")
        last = self.blackboard.state.last_fail_time

        if last is None:
            return Status.FAILURE
        
        # Check if the time difference is within the duration threshold
        elif self.clock.now() - last < self.blackboard.state.cooldown_dr:
            return Status.SUCCESS
        
        return Status.FAILURE
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsInCooldown::terminate()][{self.status}->{new_status}]")