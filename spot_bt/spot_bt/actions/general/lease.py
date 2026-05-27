"""
Copyright (c) 2026 Chiara Ferraioli

This module contains action behaviors to manage Spot's robot lease.
It provides behaviors to claim or release the lease via service calls, 
ensuring the robot's control state is correctly reflected on the blackboard.
"""

from __future__ import annotations

from spot_bt.utils.behaviour_bases import ServiceClientTimeout
from py_trees.common import Access
from py_trees.common import Status
from std_srvs.srv import Trigger

class ClaimLease(ServiceClientTimeout):
    """
    Action behavior that commands Spot to claim the robot lease.

    Blackboard:
        state (Access.WRITE): Updates 'lease_claimed' to True upon successful claim.
    """

    def __init__(self, name: str, timeout_sec: float = 30.0):
        """
        Initializes the ClaimLease behavior.

        Args:
            name (str): Name of the behavior.
            timeout_sec (float): Service call timeout in seconds. Defaults to 30.0.
        """
        super().__init__(
            name=name, 
            service_type=Trigger,
            service_name="claim",
            timeout_sec=timeout_sec
        )

        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

    def initialise(self):
        """
        Initiates the lease claim service call and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [Claim::initialise()]")
        return super().initialise()
    
    def update(self) -> Status:
        """
        Processes the result of the lease claim service call.

        If the service returns SUCCESS, it updates the blackboard to reflect 
        that the lease is now claimed. If it returns FAILURE, the flag is 
        explicitly set to False.

        Returns:
            Status: SUCCESS, FAILURE, or RUNNING while waiting for the service.
        """
        self.logger.debug(f"{self.name} [Claim::update()]")
        status = super().update()

        if status == Status.SUCCESS:
            self.blackboard.set("state.lease_claimed", True)
            return Status.SUCCESS
        elif status == Status.FAILURE:
            self.blackboard.set("state.lease_claimed", False)
            return Status.FAILURE
        
        self.logger.debug(f"{self.name} [Claim::update()][RUNNING]")
        return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """
        Cleans up service variables and logs the behavior termination.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [Claim::terminate()]")
        return super().terminate(new_status)
    
class ReleaseLease(ServiceClientTimeout):
    """
    Action behavior that commands Spot to release the robot lease.

    Blackboard:
        state (Access.WRITE): Updates 'lease_claimed' to False upon successful release.
    """

    def __init__(self, name: str, timeout_sec: float = 30.0):
        """
        Initializes the ReleaseLease behavior.

        Args:
            name (str): Name of the behavior.
            timeout_sec (float): Service call timeout in seconds. Defaults to 30.0.
        """
        super().__init__(
            name=name, 
            service_type=Trigger,
            service_name="release",
            timeout_sec=timeout_sec
        )

        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

    def initialise(self):
        """
        Initiates the lease release service call and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [ReleaseLease::initialise()]")
        return super().initialise()
    
    def update(self) -> Status:
        """
        Processes the result of the lease release service call.

        If the service returns SUCCESS, it updates the blackboard to reflect 
        that the lease has been released. If it returns FAILURE, the flag 
        is set to True as the robot presumably still holds the lease.

        Returns:
            Status: SUCCESS, FAILURE, or RUNNING while waiting for the service.
        """
        self.logger.debug(f"{self.name} [ReleaseLease::update()]")
        status = super().update()

        if status == Status.SUCCESS:
            self.blackboard.set("state.lease_claimed", False)
            return Status.SUCCESS
        elif status == Status.FAILURE:
            self.blackboard.set("state.lease_claimed", True)
            return Status.FAILURE
        
        self.logger.debug(f"{self.name} [ReleaseLease::update()][RUNNING]")
        return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """
        Cleans up service variables and logs the behavior termination.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [ReleaseLease::terminate()]")
        return super().terminate(new_status)