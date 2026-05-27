"""
Copyright (c) 2026 Chiara Ferraioli

This module contains tick handler functions for the py_trees Behavior Tree.
These handlers are used for logging, displaying, and debugging the tree's 
execution at runtime.
"""
from __future__ import annotations

from py_trees.display import unicode_tree
from py_trees.trees import BehaviourTree
from py_trees.visitors import SnapshotVisitor


def generic_post_tick_handler(behavior_tree: BehaviourTree):
    """
    Prints a generic ASCII representation of the tree after a tick.

    Args:
        behavior_tree (BehaviourTree): The behavior tree instance being ticked.
    """
    print(unicode_tree(root=behavior_tree.root, show_status=True))


def generic_pre_tick_handler(behavior_tree: BehaviourTree):
    """
    Prints a banner showing the current tick count before execution.

    Args:
        behavior_tree (BehaviourTree): The behavior tree instance being ticked.
    """
    print(f"--------- Run {behavior_tree.count} ---------")


def snapshot_added_post_tick_handler(
    snapshot_visitor: SnapshotVisitor, behavior_tree: BehaviourTree
):
    """
    Prints an ASCII tree highlighting the current snapshot status.
    Useful to trace the exact execution path of the current tick.

    Args:
        snapshot_visitor (SnapshotVisitor): The visitor containing execution history.
        behavior_tree (BehaviourTree): The behavior tree instance being ticked.
    """
    print(
        unicode_tree(
            root=behavior_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited,
        )
    )
