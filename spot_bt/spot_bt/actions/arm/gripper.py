"""
Copyright (c) 2026 Chiara Ferraioli

This module contains action behaviors to control Spot's gripper.
It provides simple service-based actions to fully open or close the 
manipulator's gripper and updates the corresponding status on the blackboard.
"""

from __future__ import annotations

from py_trees.common import Status, Access
from std_srvs.srv import Trigger

from spot_bt.utils.behaviour_bases import ServiceClientTimeout


class OpenGripper(ServiceClientTimeout):
    """
    Action behavior that commands Spot's gripper to open.

    Blackboard:
        state (Access.WRITE): Updates 'gripper_open' to True upon successful completion.
    """
    
    def __init__(self, name: str, timeout_sec: float = 30.0):
        """
        Initializes the OpenGripper behavior.

        Args:
            name (str): Name of the behavior.
            timeout_sec (float): Service call timeout in seconds. Defaults to 30.0.
        """
        super().__init__(
            name=name,
            service_type=Trigger,
            service_name="open_gripper",
            timeout_sec=timeout_sec
        )
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

    def initialise(self):
        """
        Initiates the open gripper service call and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [OpenGripper::initialise()]")
        return super().initialise()
    
    def update(self) -> Status:
        """
        Monitors the status of the open_gripper service call.

        If the service returns SUCCESS, it updates the blackboard state 
        to reflect that the gripper is open.

        Returns:
            Status: SUCCESS if the gripper is open, FAILURE if the service fails, 
                or RUNNING while waiting for the response.
        """
        self.logger.debug(f"{self.name} [OpenGripper::update()]")
        status = super().update()

        if status == Status.SUCCESS:
            self.blackboard.set("state.gripper_open", True)
            return Status.SUCCESS
        elif status == Status.FAILURE:
            return Status.FAILURE
        
        return Status.RUNNING
    

class CloseGripper(ServiceClientTimeout):
    """
    Action behavior that commands Spot's gripper to close.

    Blackboard:
        state (Access.WRITE): Updates 'gripper_open' to False upon successful completion.
    """
    
    def __init__(self, name: str, timeout_sec: float = 30.0):
        """
        Initializes the CloseGripper behavior.

        Args:
            name (str): Name of the behavior.
            timeout_sec (float): Service call timeout in seconds. Defaults to 30.0.
        """
        super().__init__(
            name=name,
            service_type=Trigger,
            service_name="close_gripper",
            timeout_sec=timeout_sec
        )
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

    def initialise(self):
        """
        Initiates the close gripper service call and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [CloseGripper::initialise()]")
        return super().initialise()
    
    def update(self) -> Status:
        """
        Monitors the status of the close_gripper service call.

        If the service returns SUCCESS, it updates the blackboard state 
        to reflect that the gripper is closed.

        Returns:
            Status: SUCCESS if the gripper is closed, FAILURE if the service fails, 
                or RUNNING while waiting for the response.
        """
        self.logger.debug(f"{self.name} [CloseGripper::update()]")
        status = super().update()

        if status == Status.SUCCESS:
            self.blackboard.set("state.gripper_open", False)
            return Status.SUCCESS
        elif status == Status.FAILURE:
            return Status.FAILURE
        
        return Status.RUNNING