"""
Copyright (c) 2026 Chiara Ferraioli

This module contains condition behaviors to monitor the state of Spot's gripper.
It includes checks for whether the gripper is open, closed, or currently 
grasping an object, using data from the robot's status blackboard.
"""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status


class IsGripperOpen(Behaviour):
    """
    Condition behavior that checks if Spot's gripper is currently open.

    Blackboard:
        state (Access.READ): Reads 'gripper_open' to evaluate the current gripper state.
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
        Initiates the gripper open status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsGripperOpen::initialise()]")
    
    def update(self) -> Status:
        """
        Evaluates the gripper's open status by checking the blackboard flag.

        Returns:
            Status: SUCCESS if 'gripper_open' is True, FAILURE otherwise.
        """
        self.logger.debug(f"{self.name} [IsGripperOpen::update()]")
        
        if self.blackboard.state.gripper_open:
            return Status.SUCCESS
        
        return Status.FAILURE
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsGripperOpen::terminate()][{self.status}->{new_status}]")


class IsGripperClosed(Behaviour):
    """
    Condition behavior that checks if Spot's gripper is currently closed.

    Blackboard:
        state (Access.READ): Reads 'gripper_open' to evaluate the current gripper state.
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
        Initiates the gripper closed status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsGripperClosed::initialise()]")
    
    def update(self) -> Status:
        """
        Evaluates the gripper's closed status by verifying the blackboard flag is False.

        Returns:
            Status: SUCCESS if 'gripper_open' is False, FAILURE otherwise.
        """
        self.logger.debug(f"{self.name} [IsGripperClosed::update()]")
        
        if self.blackboard.state.gripper_open:
            return Status.FAILURE
        
        return Status.SUCCESS
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsGripperClosed::terminate()][{self.status}->{new_status}]")


class IsGrasping(Behaviour):
    """
    Condition behavior that checks if Spot is currently grasping an object.

    Blackboard:
        state (Access.READ): Reads 'holding_obj' to determine if an object is held.
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
        Initiates the grasping status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsGrasping::initialise()]")

    def update(self) -> Status:
        """
        Evaluates the grasping status by checking the blackboard flag.

        Returns:
            Status: SUCCESS if 'holding_obj' is True, FAILURE otherwise.
        """
        self.logger.debug(f"{self.name} [IsGrasping::update()]")
        
        if self.blackboard.state.holding_obj:
            return Status.SUCCESS
        
        return Status.FAILURE
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsGrasping::terminate()][{self.status}->{new_status}]")