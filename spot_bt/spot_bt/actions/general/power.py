"""
Copyright (c) 2026 Chiara Ferraioli

This module contains action behaviors to manage the power state of Spot's motors.
It provides behaviors to power on or off the robot's motors via service calls,
reflecting the power status on the blackboard.
"""

from __future__ import annotations

from spot_bt.utils.behaviour_bases import ServiceClientTimeout
from py_trees.common import Access
from py_trees.common import Status
from std_srvs.srv import Trigger

class PowerOn(ServiceClientTimeout):
    """
    Action behavior that commands Spot to power on its motors.

    Blackboard:
        state (Access.WRITE): Updates 'powered_on' to True upon successful power on.
    """
    
    def __init__(self, name: str, timeout_sec: float = 30.0):
        """
        Initializes the PowerOn behavior.

        Args:
            name (str): Name of the behavior.
            timeout_sec (float): Service call timeout in seconds. Defaults to 30.0.
        """
        super().__init__(
            name=name,
            service_type=Trigger,
            service_name="power_on",
            timeout_sec=timeout_sec
        )

        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)
    
    def initialise(self):
        """
        Initiates the motor power-on service call and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [PowerOn::initialise()]")
        return super().initialise()
    
    def update(self) -> Status:
        """
        Processes the result of the power-on service call.

        If the service returns SUCCESS, it updates the blackboard state to indicate
        that the motors are powered on. If it returns FAILURE, the 'powered_on' flag
        is explicitly set to False.

        Returns:
            Status: SUCCESS if powered on, FAILURE if the service fails, 
                or RUNNING while waiting for the response.
        """
        self.logger.debug(f"{self.name} [PowerOn::update()]")
        status = super().update()

        if status == Status.SUCCESS:
            self.blackboard.set("state.powered_on", True)
            return Status.SUCCESS
        elif status == Status.FAILURE:
            self.blackboard.set("state.powered_on", False)
            return Status.FAILURE
        
        self.logger.debug(f"{self.name} [PowerOn::update()][RUNNING]")
        return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """
        Cleans up service variables and logs the behavior termination.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [PowerOn::terminate()]")
        return super().terminate(new_status)
    
class PowerOff(ServiceClientTimeout):
    """
    Action behavior that commands Spot to power off its motors.

    Blackboard:
        state (Access.WRITE): Updates 'powered_on' to False upon successful power off.
    """
    
    def __init__(self, name: str, timeout_sec: float = 30.0):
        """
        Initializes the PowerOff behavior.

        Args:
            name (str): Name of the behavior.
            timeout_sec (float): Service call timeout in seconds. Defaults to 30.0.
        """
        super().__init__(
            name=name,
            service_type=Trigger,
            service_name="power_off",
            timeout_sec=timeout_sec
        )

        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)
    
    def initialise(self):
        """
        Initiates the motor power-off service call and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [PowerOff::initialise()]")
        return super().initialise()
    
    def update(self) -> Status:
        """
        Processes the result of the power-off service call.

        If the service returns SUCCESS, it updates the blackboard state to indicate
        that the motors are no longer powered. If it returns FAILURE, the flag is
        set to True as the motors presumably failed to power off.

        Returns:
            Status: SUCCESS if powered off, FAILURE if the service fails, 
                or RUNNING while waiting for the response.
        """
        self.logger.debug(f"{self.name} [PowerOff::update()]")
        status = super().update()

        if status == Status.SUCCESS:
            self.blackboard.set("state.powered_on", False)
            return Status.SUCCESS
        elif status == Status.FAILURE:
            self.blackboard.set("state.powered_on", True)
            return Status.FAILURE
        
        self.logger.debug(f"{self.name} [PowerOff::update()][RUNNING]")
        return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """
        Cleans up service variables and logs the behavior termination.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [PowerOff::terminate()]")
        return super().terminate(new_status)