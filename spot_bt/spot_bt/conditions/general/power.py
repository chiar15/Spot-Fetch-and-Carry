"""
Copyright (c) 2026 Chiara Ferraioli

This module contains condition behaviors to monitor the power status of Spot's motors.
These behaviors check whether the robot's motors are powered on or off 
by querying the status blackboard.
"""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status


class IsPoweredOn(Behaviour):
    """
    Condition behavior that checks if Spot's motors are currently powered on.

    Blackboard:
        state (Access.READ): Reads 'powered_on' to evaluate the current motor power status.
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
        Initiates the motor power-on status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsPoweredOn::initialise()]")
    
    def update(self) -> Status:
        """
        Evaluates the power status of the motors by checking the blackboard flag.

        Returns:
            Status: SUCCESS if 'powered_on' is True, FAILURE otherwise.
        """
        self.logger.debug(f"{self.name} [IsPoweredOn::update()]")
        
        if self.blackboard.state.powered_on:
            return Status.SUCCESS
        
        return Status.FAILURE
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsPoweredOn::terminate()][{self.status}->{new_status}]")
        

class IsPoweredOff(Behaviour):
    """
    Condition behavior that checks if Spot's motors are currently powered off.

    Blackboard:
        state (Access.READ): Reads 'powered_on' to evaluate the current motor power status.
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
        Initiates the motor power-off status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsPoweredOff::initialise()]")

    def update(self) -> Status:
        """
        Evaluates the power status of the motors by verifying the 'powered_on' flag is False.

        Returns:
            Status: SUCCESS if 'powered_on' is False, FAILURE otherwise.
        """
        self.logger.debug(f"{self.name} [IsPoweredOff::update()]")
        
        if self.blackboard.state.powered_on:
            return Status.FAILURE
        
        return Status.SUCCESS
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsPoweredOff::terminate()][{self.status}->{new_status}]")