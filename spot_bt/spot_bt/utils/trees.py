"""
Copyright (c) 2026 Chiara Ferraioli

This module provides factory functions to create specific subtrees 
for the robot's overall Behavior Tree mission.
"""
from spot_bt.composites import sequence
from spot_bt.composites import selector
from py_trees.decorators import *
from py_trees.composites import *
from spot_bt.conditions.perception.object_detection import *
from spot_bt.conditions.arm.gripper import *
from spot_bt.actions.perception.object_detection import *
from typing import Optional

def create_exploration_detection_subtree(
        name: str = "Exploration and Pick Subtree", robot_name: Optional[str] = None
) -> Sequence:
    """
    Creates a behavior tree sequence that attempts to pick an object, falling back to exploration.

    If the robot is not already holding an object, this subtree first tries 
    to execute the picking workflow. If the pick workflow fails (for example, 
    if no object is detected), it falls back to exploring the environment.

    Args:
        name (str): The name of the root sequence of this subtree.
        robot_name (Optional[str]): The robot's namespace.

    Returns:
        Sequence: The constructed sequence subtree for picking and exploring.
    """
    sel = Selector(name='Pick or Exploration', memory=False)
    sel.add_children(
        [
            sequence.create_pick_workflow_sequence(robot_name=robot_name),
            sequence.create_exploration_sequence(robot_name=robot_name)
        ]
    )

    subtree_sequence = Sequence(name, memory=True)
    subtree_sequence.add_children(
        [
            Inverter(name='Inverter Is Grasping Object', child=IsGrasping(name="Is Grasping Object?")),
            sel
        ]
    )
    
    return subtree_sequence

def create_mission_subtree(
        name: str = "Mission", robot_name: Optional[str] = None
) -> Selector:
    """
    Creates the main mission subtree, combining the drop sequence and the 
    exploration/picking sequence. The robot evaluates dropping an item first; 
    if not applicable, it falls back to exploring and picking.

    Args:
        name (str): The name of the root selector of this subtree.
        robot_name (Optional[str]): The robot's namespace.

    Returns:
        Selector: The constructed main mission subtree.
    """
    selector = Selector(name, memory=False)
    selector.add_children(
        [
            sequence.create_drop_sequence(robot_name=robot_name),
            create_exploration_detection_subtree(robot_name=robot_name),
        ]
    )

    return selector