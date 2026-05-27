"""
Copyright (c) 2026 Chiara Ferraioli

This module contains condition behaviors to monitor the robot's lease status.
These behaviors check whether the robot lease has been claimed or released 
by querying the status blackboard.
"""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status


class IsClaimed(Behaviour):
    """
    Condition behavior that checks if the robot lease is currently claimed.

    Blackboard:
        state (Access.READ): Reads 'lease_claimed' to evaluate the current control status.
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
        Initiates the lease claim status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsClaimed::initialise()]")

    def update(self) -> Status:
        """
        Evaluates the lease claim status by checking the blackboard flag.

        Returns:
            Status: SUCCESS if 'lease_claimed' is True, FAILURE otherwise.
        """
        self.logger.debug(f"{self.name} [IsClaimed::update()]")
        
        if self.blackboard.state.lease_claimed:
            return Status.SUCCESS
        
        return Status.FAILURE
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsClaimed::terminate()][{self.status}->{new_status}]")


class IsLeaseReleased(Behaviour):
    """
    Condition behavior that checks if the robot lease has been released.

    Blackboard:
        state (Access.READ): Reads 'lease_claimed' to evaluate the current control status.
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
        Initiates the lease release status check and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsLeaseReleased::initialise()]")

    def update(self) -> Status:
        """
        Evaluates the lease release status by verifying the 'lease_claimed' flag is False.

        Returns:
            Status: SUCCESS if 'lease_claimed' is False, FAILURE otherwise.
        """
        self.logger.debug(f"{self.name} [IsLeaseReleased::update()]")
        
        if self.blackboard.state.lease_claimed:
            return Status.FAILURE
        
        return Status.SUCCESS
    
    def terminate(self, new_status: Status):
        """
        Logs the termination and status transition of the behavior.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsLeaseReleased::terminate()][{self.status}->{new_status}]")