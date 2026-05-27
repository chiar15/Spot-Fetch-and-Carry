"""
Copyright (c) 2026 Chiara Ferraioli

This module contains action behaviors for emergency handling and error signaling
within the Spot robot's behavior tree.
"""

from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access


class SetCriticalFailure(Behaviour):
    """
    Behavior that signals a critical failure in the system.

    This node is used to trigger an immediate stop or a high-level emergency 
    branch by setting a global flag on the blackboard. It always returns 
    FAILURE to propagate the error status.

    Blackboard:
        state (Access.WRITE): Sets the 'critical_failure' flag to True.
    """
    
    def __init__(self, name: str, reason: str = "Unknown"):
        """
        Initializes the SetCriticalFailure behavior.

        Args:
            name (str): Name of the behavior.
            reason (str): A description of the failure cause. Defaults to "Unknown".
        """
        super().__init__(name)
        self.reason = reason
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)
    
    def update(self) -> Status:
        """
        Signals the critical failure and updates the blackboard.

        It logs an error message with the specified reason and sets the 
        'critical_failure' flag on the blackboard to True, allowing other 
        parts of the tree to react to the emergency.

        Returns:
            Status: Always returns FAILURE.
        """
        self.logger.error(f"CRITICAL FAILURE: {self.reason}")
        
        # Update the blackboard state to signal the emergency globally
        self.blackboard.set("state.critical_failure", True)
        
        return Status.FAILURE
