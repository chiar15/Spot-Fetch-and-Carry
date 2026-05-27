"""
Copyright (c) 2026 Chiara Ferraioli

This module provides factory functions to create specific Sequence subtrees 
for the Spot robot's missions. These sequences define high-level workflows 
by chaining conditions, actions, and selectors into structured behaviors.
"""
from py_trees.composites import Sequence
from py_trees.decorators import *
from spot_bt.actions.arm.general import *
from spot_bt.actions.body.movement import *
from spot_bt.conditions.body.movement import *
from spot_bt.actions.perception.object_detection import *
from spot_bt.conditions.perception.object_detection import *
from spot_bt.actions.arm.manipulation import *
from spot_bt.composites import selector
from spot_bt.conditions.arm.gripper import *
from spot_bt.actions.arm.gripper import *
from spot_bt.actions.body.navigation import *

__all__ = [
    'create_recovery_sequence',
    'create_drop_sequence',
    'create_exploration_sequence',
    'create_stop_sequence',
    'create_pick_conditions_sequence',
    'create_pick_workflow_sequence'
]

def create_recovery_sequence(
        name: str = "Recovery from Failed Pick Object", memory: bool = True, robot_name: Optional[str] = None
) -> Sequence:
    """
    Creates a recovery sequence to return the robot to a safe state after a failed pick.

    This sequence ensures the arm is moved to a safe position, stowed, and the 
    gripper is closed. Finally, the robot moves backward to clear the area.

    Args:
        name (str): The name of the sequence node.
        memory (bool): If True, remembers the last running child.
        robot_name (Optional[str]): The namespace of the robot.

    Returns:
        Sequence: A subtree containing recovery actions (ArmSafe, stow, close gripper, move back).
    """
    sequence = Sequence(name, memory)
    sequence.add_children(
        [
            Timeout(name="Timeout Arm Safe", child=ArmSafe(name="Arm Safe", robot_name = robot_name), duration=20.0),
            ArmStow("Stow Arm"),
            CloseGripper("Close Gripper"),
            Timeout(name="Timeout Back",child=MoveBackward("Back", robot_name), duration=20),
        ]
    )

    return sequence

def create_drop_sequence(
        name: str = "Drop Object Sequence", memory: bool = True, robot_name: Optional[str] = None
) -> Sequence:
    """
    Creates a sequence to handle the controlled release of a carried object at home.

    The sequence verifies the robot is grasping an object, ensures the arm 
    posture and robot position (home) are correct, and then executes a 
    maneuver to extend the arm and drop the object before stowing.

    Args:
        name (str): The name of the sequence node.
        memory (bool): If True, remembers the last running child.
        robot_name (Optional[str]): The namespace of the robot.

    Returns:
        Sequence: A subtree managing the navigation to home and the object release logic.
    """
    sequence = Sequence(name, memory)
    sequence.add_children(
        [
            IsGrasping("Is Grasping Object?"),
            selector.create_carry_arm_selector(),
            selector.create_home_selector(robot_name=robot_name),
            Timeout(name="Turning Timeout", child=RotateRight(name="Rotate Right", robot_name=robot_name), duration=20.0),
            Timeout(name = "Timeout Extend Arm", child=ArmExtendForward("Extend Arm", robot_name=robot_name, extend_meters=0.1), duration=40.0),
            OpenGripper("Open Gripper"),
            CloseGripper("Close Gripper"),
            selector.create_stow_selector(),
            SetCooldown("After Drop Cooldown", cooldown_sec=4),
        ]
    )

    return sequence

def create_exploration_sequence(
        name: str = "Exploration Sequence", memory: bool = False, robot_name: Optional[str] = None
) -> Sequence:
    """
    Creates a sequence for exploring the environment when picking is not possible.

    If pick conditions are not met (e.g., no object detected or in cooldown), 
    the robot stows its arm and navigates to explore the area.

    Args:
        name (str): The name of the sequence node.
        memory (bool): If True, remembers the last running child.
        robot_name (Optional[str]): The namespace of the robot.

    Returns:
        Sequence: A subtree that triggers exploration navigation.
    """
    sequence = Sequence(name, memory)
    sequence.add_children(
        [
            Inverter(name="Inverter Pick Conditions", child=create_pick_conditions_sequence()),
            selector.create_stow_selector(),
            Navigate("Exploration", robot_name=robot_name),
        ]
    )

    return sequence

def create_stop_sequence(
        name: str = "Stop Sequence", memory: bool = False
) -> Sequence:
    """
    Creates a sequence to halt the robot and verify it is completely still.

    Args:
        name (str): The name of the sequence node.
        memory (bool): If True, remembers the last running child.

    Returns:
        Sequence: A subtree containing the stop action and stillness condition.
    """
    sequence = Sequence(name, memory)
    sequence.add_children(
        [
            Stop("Stop Robot"),
            IsStill("Is Robot Still?")
        ]
    )

    return sequence

def create_pick_conditions_sequence(
        name: str = "Pick Conditions Sequence", memory: bool = False
) -> Sequence:
    """
    Creates a sequence to verify if the conditions to start a pick are met.

    The robot must not be in a cooldown period and must have detected the target object.

    Args:
        name (str): The name of the sequence node.
        memory (bool): If True, remembers the last running child.

    Returns:
        Sequence: A subtree checking cooldown and object detection status.
    """
    sequence = Sequence(name, memory)
    sequence.add_children(
        [
            Inverter(name="Inverter Is in Cooldown?", child=IsInCooldown("Is Robot in Cooldown?")),
            IsObjectDetected("Is Object Detected?")
        ]
    )

    return sequence

def create_pick_workflow_sequence(
        name: str = "Pick Workflow Sequence", memory: bool = True, robot_name: Optional[str] = None
) -> Sequence:
    """
    Creates the main workflow sequence for approaching and picking an object.

    The workflow involves checking pick conditions, stopping the robot, 
    walking to the object (with retries), and finally executing the pick 
    with recovery logic. If the pick fails after retries, it sets a cooldown.

    Args:
        name (str): The name of the sequence node.
        memory (bool): If True, remembers the last running child.
        robot_name (Optional[str]): The namespace of the robot.

    Returns:
        Sequence: The full picking workflow subtree.
    """
    sequence = Sequence(name, memory)
    sequence.add_children(
        [
            create_pick_conditions_sequence(),
            create_stop_sequence(),
            Retry(name="Retry Walk To Object", child=Timeout(name="Timeout Walk To Object",child=WalkToObject(name="Walk To Object", robot_name=robot_name), duration=30.0), num_failures=5),
            create_stop_sequence(),
            selector.Selector(
                name="Pick and Set Cooldown Selector", 
                memory=True, 
                children=
                [
                    Retry(name="Retry Pick and Recovery", child=selector.create_pick_with_recovery_selector(robot_name=robot_name), num_failures=3),
                    SetCooldown("Set Cooldown After Pick Failure", cooldown_sec=20)
                ])
        ]
    )

    return sequence