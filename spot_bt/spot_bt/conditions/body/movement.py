"""
Copyright (c) 2026 Chiara Ferraioli

This module contains condition behaviors for monitoring Spot's body movement 
and posture. It includes checks to determine if the robot is stationary, 
standing, or sitting, based on the state stored in the status blackboard.
"""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status


class IsStill(Behaviour):
    """
    Condition behavior that checks if Spot is currently stationary.

    Blackboard:
        state (Access.READ): Reads 'stopped' to evaluate the current motion state.
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
        Initiates the stillness status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsStill::initialise()]")

    def update(self) -> Status:
        """
        Evaluates if the robot is currently stationary by checking the blackboard flag.

        Returns:
            Status: SUCCESS if 'stopped' is True, FAILURE otherwise.
        """
        self.logger.debug(f"{self.name} [IsStill::update()]")
        
        if self.blackboard.state.stopped:
            self.logger.debug(f"{self.name} Spot is Still!")
            return Status.SUCCESS
        
        return Status.FAILURE
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsStill::terminate()][{self.status}->{new_status}]")
    

class IsStanding(Behaviour):
    """
    Condition behavior that checks if Spot is currently standing.

    Blackboard:
        state (Access.READ): Reads 'standing' to evaluate the current posture.
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
        Initiates the standing status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsStanding::initialise()]")

    def update(self) -> Status:
        """
        Evaluates if the robot is currently standing by checking the blackboard flag.

        Returns:
            Status: SUCCESS if 'standing' is True, FAILURE otherwise.
        """
        self.logger.debug(f"{self.name} [IsStanding::update()]")
        
        if self.blackboard.state.standing:
            return Status.SUCCESS
        
        return Status.FAILURE
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsStanding::terminate()][{self.status}->{new_status}]")


class IsSitting(Behaviour):
    """
    Condition behavior that checks if Spot is currently sitting.

    Blackboard:
        state (Access.READ): Reads 'standing' to evaluate the current posture.
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
        Initiates the sitting status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsSitting::initialise()]")

    def update(self) -> Status:
        """
        Evaluates if the robot is currently sitting by verifying the 'standing' flag is False.

        Returns:
            Status: SUCCESS if 'standing' is False, FAILURE otherwise.
        """
        self.logger.debug(f"{self.name} [IsSitting::update()]")
        
        if self.blackboard.state.standing:
            return Status.FAILURE
        
        return Status.SUCCESS
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsSitting::terminate()][{self.status}->{new_status}]")