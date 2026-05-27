"""
Copyright (c) 2026 Chiara Ferraioli

This module contains dataclasses used to store the state of the Spot robot 
and information regarding detected objects. These classes are typically 
used in conjunction with py_trees blackboards.
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
from bosdyn.api.geometry_pb2 import Vec2
from rclpy.time import Time
from rclpy.duration import Duration
from typing import Optional

@dataclass
class DetectedObject:
    """
    Data structure representing a detected target object.
    Separated from the main Spot state for clarity.

    Attributes:
        center (Vec2): The (x, y) pixel coordinates of the object's center.
        timestamp (Time): The time at which the object was detected.
        frame_id (str): The coordinate frame ID of the camera that detected the object.
    """
    center: Vec2 = field(default_factory=lambda: Vec2(x=0.0, y=0.0)) # type: ignore[assignment]
    timestamp: Time = field(default_factory=Time)
    frame_id: str = field(default="")

@dataclass
class SpotState:
    """
    Main state container for the Spot robot.
    Holds the body status, arm status, and perception status.

    Attributes:
        lease_claimed (bool): True if the robot lease is acquired.
        powered_on (bool): True if motor power is engaged.
        standing (bool): True if the robot is standing.
        stopped (bool): True if the robot is completely still.
        critical_failure (bool): True if a critical failure has occurred.
        arm_stowed (bool): True if the manipulator arm is stowed.
        gripper_open (bool): True if the gripper is currently open.
        gripper_percentage (float): The openness of the gripper (0.0 to 1.0).
        arm_carrying (bool): True if the manipulator arm is in the carry position.
        holding_obj (bool): True if the gripper is currently holding an object.
        obj_detected (bool): True if the target object has been detected.
        obj_info (Optional[DetectedObject]): Details about the detected object.
        last_fail_time (Optional[Time]): Timestamp of the last grasping failure.
        cooldown_dr (Optional[Duration]): Cooldown duration for specific actions.
    """
    # Body Status
    lease_claimed: bool = field(default=False)
    powered_on: bool = field(default=False)
    standing: bool = field(default=False)
    stopped: bool = field(default=True)
    critical_failure: bool = field(default=False)
    
    # Arm Status
    arm_stowed: bool = field(default=True)
    gripper_open: bool = field(default=False)
    gripper_percentage: float = field(default=1.00)
    arm_carrying: bool = field(default=False)
    holding_obj: bool = field(default=False)

    # Perception Status
    obj_detected: bool = field(default=False)
    obj_info: Optional[DetectedObject] = field(default=None)
    last_fail_time: Optional[Time] = field(default=None)
    cooldown_dr: Optional[Duration] = field(default=None)

