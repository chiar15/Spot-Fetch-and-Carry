"""
Copyright (c) 2026 Chiara Ferraioli

This module contains general condition behaviors for the Spot robot's arm.
These conditions query the robot's state from the blackboard to determine 
if the arm is currently stowed or in a carrying position.
"""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status


class IsArmStowed(Behaviour):
    """
    Condition behavior that checks if Spot's arm is currently stowed.

    Blackboard:
        state (Access.READ): Reads 'arm_stowed' to evaluate the current arm state.
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
        Initiates the arm stowed status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsArmStowed::initialise()]")
    
    def update(self) -> Status:
        """
        Evaluates the arm's stowed status by checking the blackboard flag.

        Returns:
            Status: SUCCESS if 'arm_stowed' is True, FAILURE otherwise.
        """
        self.logger.debug(f"{self.name} [IsArmStowed::update()]")
        
        if self.blackboard.state.arm_stowed:
            return Status.SUCCESS
        
        return Status.FAILURE
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsArmStowed::terminate()][{self.status}->{new_status}]")


class IsArmCarrying(Behaviour):
    """
    Condition behavior that checks if Spot's arm is in the carry position.

    Blackboard:
        state (Access.READ): Reads 'arm_carrying' to evaluate the current arm state.
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
        Initiates the arm carrying status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsArmCarrying::initialise()]")
    
    def update(self) -> Status:
        """
        Evaluates if the arm is currently in the carry position by checking the blackboard flag.

        Returns:
            Status: SUCCESS if 'arm_carrying' is True, FAILURE otherwise.
        """
        self.logger.debug(f"{self.name} [IsArmCarrying::update()]")
        
        if self.blackboard.state.arm_carrying:
            return Status.SUCCESS
        
        return Status.FAILURE
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsArmCarrying::terminate()][{self.status}->{new_status}]")