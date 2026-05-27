"""
Copyright (c) 2026 Chiara Ferraioli

This module provides factory functions to create specific Selector subtrees 
for the Spot robot's behavior tree. These selectors implement fallback logic 
for various arm and body states, ensuring that actions are only executed 
if their corresponding target conditions are not already met.
"""
from py_trees.composites import Selector
from py_trees.composites import Sequence

from spot_bt.actions.arm.general import *
from spot_bt.conditions.arm.general import *
from spot_bt.actions.arm.manipulation import *
from spot_bt.actions.body.movement import *
from spot_bt.conditions.body.movement import *
from spot_bt.actions.body.navigation import *
from spot_bt.conditions.body.navigation import *
from spot_bt.actions.perception.object_detection import *
from spot_bt.composites import sequence
from py_trees.decorators import *

# Aggiungi all'inizio del file, dopo gli import

__all__ = [
    'create_carry_arm_selector',
    'create_stow_selector',
    'create_unstow_selector',
    'create_pick_with_recovery_selector',
    'create_home_selector'
]

def create_carry_arm_selector(
    name: str = "Check and Carry Arm", memory: bool = False
) -> Selector:
    """
    Creates a fallback selector to ensure the arm is in the carry position.

    It first checks if the arm is already in the carrying state. If not, 
    it falls back to executing the ArmCarry action.

    Args:
        name (str): The name of the selector node.
        memory (bool): If True, remembers the running child across ticks.

    Returns:
        Selector: The constructed subtree for carrying the arm.
    """
    selector = Selector(name, memory)
    selector.add_children(
        [
            IsArmCarrying("Is Arm Carrying?"),
            ArmCarry("Carry Arm"),
        ]
    )

    return selector

def create_stow_selector(
        name: str = "Check and Stow Arm", memory: bool = False
) -> Selector:
    """
    Creates a fallback selector to ensure the arm is securely stowed.

    It verifies if the arm is currently stowed. If the condition fails 
    (the arm is deployed), it triggers the ArmStow action.

    Args:
        name (str): The name of the selector node.
        memory (bool): If True, remembers the running child across ticks.

    Returns:
        Selector: The constructed subtree for stowing the arm.
    """
    selector = Selector(name, memory)
    selector.add_children(
        [
            IsArmStowed("Is Arm Stowed?"),
            ArmStow("Stow Arm"),
        ]
    )

    return selector

def create_unstow_selector(
        name: str = "Check and Unstow Arm", memory: bool = False
) -> Selector:
    """
    Creates a fallback selector to ensure the arm is deployed (unstowed).

    It uses an Inverter on the IsArmStowed condition. If the arm is already 
    unstowed, it returns SUCCESS. If it is stowed, it falls back to executing 
    the ArmUnstow action.

    Args:
        name (str): The name of the selector node.
        memory (bool): If True, remembers the running child across ticks.

    Returns:
        Selector: The constructed subtree for unstowing the arm.
    """
    selector = Selector(name, memory)
    selector.add_children(
        [
            Inverter(name = "Invertend Stow Check",child = IsArmStowed("Is Arm Stowed?")),
            ArmUnstow("Unstow Arm"),
        ]
    )

    return selector

def create_home_selector(
        name: str = "Check and Go Home", memory: bool = True, robot_name: Optional[str] = None
) -> Selector:
    """
    Creates a fallback selector to return the robot to its home waypoint.

    It checks if the robot is already at the home location. If not, it 
    initiates the GoHome navigation action.

    Args:
        name (str): The name of the selector node.
        memory (bool): If True, remembers the running child across ticks. Defaults to True.
        robot_name (Optional[str]): The namespace of the robot.

    Returns:
        Selector: The constructed subtree for returning home.
    """
    selector = Selector(name, memory)
    selector.add_children(
        [
            IsHome("Is Robot Home?"),
            GoHome("Go Home", robot_name=robot_name),
        ]
    )

    return selector

def create_pick_with_recovery_selector(
        name: str = "Pick Object and Recovery", memory: bool = True, robot_name: Optional[str] = None
) -> Selector:
    """
    Creates a complex selector that attempts to pick an object and provides 
    a recovery fallback if the operation fails or times out.

    The primary sequence attempts a detection followed by a picking action 
    (with a 30-second timeout). If this sequence fails, it falls back to a 
    recovery sequence. The recovery sequence is wrapped in a SuccessIsFailure 
    decorator so that, even if recovery succeeds, the overall selector still 
    reports FAILURE upstream (indicating the object wasn't actually picked).

    Args:
        name (str): The name of the selector node.
        memory (bool): If True, remembers the running child across ticks. Defaults to True.
        robot_name (Optional[str]): The namespace of the robot.

    Returns:
        Selector: The constructed subtree for picking and recovering.
    """
    selector = Selector(name, memory)
    selector.add_children(
        [
            Sequence(name="Detect and Pick", memory=True, children=[ObjectDetection(name="Detection Pre-Pick"), Timeout(name = "Pick Timeout", child = PickObject("Pick Object", robot_name=robot_name), duration=30.0)]),
            SuccessIsFailure(name = "Fail on Recovery", child = sequence.create_recovery_sequence(robot_name=robot_name))  
        ]
    )

    return selector

