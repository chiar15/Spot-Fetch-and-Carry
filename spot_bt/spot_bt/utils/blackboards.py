"""
Copyright (c) 2026 Chiara Ferraioli

This module provides utilities to create and connect to the py_trees
blackboards used by the behavior tree to share the robot's state.
"""
from py_trees.blackboard import Client
from py_trees.common import Access
from spot_bt.utils.data import SpotState

def create_status_blackboard(
) -> Client:
    """
    Creates and initializes the default Spot 'status' blackboard.
    
    This function sets up the blackboard with a new instance of SpotState
    and registers read and write access for the 'state' key.

    Returns:
        Client: The initialized py_trees blackboard client.
    """
    blackboard = Client(name="status")
    blackboard.register_key(key="state", access=Access.WRITE)
    blackboard.register_key(key="state", access=Access.READ)
    blackboard.state = SpotState()

    return blackboard

def connect_status_blackboard(
) -> Client:
    """
    Connects to an existing Spot 'status' blackboard.
    
    Unlike creation, this only registers read and write access for the 
    'state' key without overwriting the existing SpotState object.

    Returns:
        Client: The connected py_trees blackboard client.
    """
    blackboard = Client(name="status")
    blackboard.register_key(key="state", access=Access.WRITE)
    blackboard.register_key(key="state", access=Access.READ)

    return blackboard