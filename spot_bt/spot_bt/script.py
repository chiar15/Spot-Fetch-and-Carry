"""
Copyright (c) 2026 Chiara Ferraioli

Main entry point for the Spot robot's behavior tree mission.
This script initializes the ROS 2 environment, constructs the behavior tree 
root, registers pre-tick handlers for safety checks, and manages the main 
execution loop and graceful shutdown procedures.
"""
import argparse
from py_trees import logging
from py_trees.blackboard import Blackboard
from py_trees.common import Access
from py_trees.composites import Parallel, Sequence

from py_trees_ros.exceptions import TimedOutError
from py_trees_ros.trees import BehaviourTree

from spot_bt.ros_nodes.initialization_node import InitializationNode
from spot_bt.actions.perception.object_detection import *
from spot_bt.composites import sequence
from spot_bt.utils.subscribers import *
from spot_bt.utils import trees
from spot_bt.utils import blackboards
from spot_bt.utils.tick import generic_pre_tick_handler

from typing import Optional
import rclpy
import signal
import time

def emergency_pre_tick_handler(behaviour_tree):
    """
    Evaluates the robot's critical failure state before executing a behavior tree tick.

    If a critical failure is detected in the status blackboard, this handler forces 
    an immediate and safe shutdown of the behavior tree and the ROS 2 environment.

    Args:
        behaviour_tree (BehaviourTree): The behavior tree instance being executed.

    Raises:
        SystemExit: Exits the application with status code 1 if a critical failure is found.
    """
    blackboard = blackboards.connect_status_blackboard()
    
    is_critical = blackboard.get("state.critical_failure")
    
    if is_critical:
        behaviour_tree.node.get_logger().fatal("Shutting down behaviour tree!")
        
        # Safe Shutdown
        behaviour_tree.node.safe_shutdown()
        behaviour_tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

def create_root(robot_name: Optional[str] = None) -> Parallel:
    """
    Constructs the root node of the Spot mission behavior tree.

    The root is defined as a Parallel composite node (SuccessOnAll policy). 
    It runs the core robot monitoring behaviors (such as MovingScan and 
    ManipulatorScan) and the main perception loop (ObjectDetection) 
    in parallel with the primary mission logic defined by `create_mission_subtree`.

    Args:
        robot_name (Optional[str]): The namespace associated with the robot.

    Returns:
        Parallel: The constructed root node of the behavior tree.
    """
    root = Parallel("Demo", policy=ParallelPolicy.SuccessOnAll(synchronise=False))
    root.add_children(
        [
            MovingScan("Scan Moving", robot_name=robot_name),
            ManipulatorScan("Scan Manipulator", robot_name=robot_name),
            ObjectDetection(name="Object Detection"),
            trees.create_mission_subtree(robot_name=robot_name),
        ]
    )

    return root

def graceful_shutdown(node: InitializationNode, tree: BehaviourTree):
    """
    Executes a safe teardown of the robot's state and software nodes.

    This function triggers the robot's safe physical shutdown sequence, 
    terminates all running behaviors in the tree, destroys the ROS 2 node, 
    and attempts to shut down the rclpy context.

    Args:
        node (InitializationNode): The ROS 2 node managing the robot's initialization.
        tree (BehaviourTree): The running behavior tree to be terminated.
    """
    node.safe_shutdown()
    tree.shutdown()

    node.destroy_node()
    rclpy.try_shutdown()


shutdown_requested = False

def signal_handler(sig, frame):
    """
    Intercepts system signals (e.g., SIGINT) to trigger a graceful shutdown loop.

    Args:
        sig (int): The signal number received.
        frame: The current stack frame at the time of the signal.
    """
    global shutdown_requested
    shutdown_requested = True

def main(args=None):
    """
    Main execution loop for the Spot behavior tree application.

    This function handles command-line arguments, sets up the ROS 2 node, 
    initializes the robot's physical state, constructs the behavior tree, 
    and enters a ticking loop. It aggressively catches exceptions and keyboard 
    interrupts to ensure the robot always shuts down safely.

    Args:
        args (Optional[list]): Command-line arguments. Defaults to None.

    Raises:
        SystemExit: Exits the application with status code 1 on interruption, 
            initialization failure, or general execution exception.
    """
    parser = argparse.ArgumentParser(
        description="Spot manipulation mission with behaviour tree"
    )
    parser.add_argument(
        "--robot",
        type=str,
        default=None,
        help="Robot namespace (e.g., 'spot'). If not specified, no namespace is used."
    )
    args = parser.parse_args()
    
    rclpy.init()
    node = InitializationNode(robot_name=args.robot)

    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        node.initialize_robot()
    except KeyboardInterrupt:
        node.get_logger().error("Robot initialization interrupted!")
        Blackboard.clear()
        node.safe_shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(1)
    except RuntimeError:
        node.get_logger().error("Robot initialization failed!")
        Blackboard.clear()
        node.safe_shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(1)

    # Set BT debug level and activity stream
    logging.level = logging.Level.DEBUG
    Blackboard.enable_activity_stream(maximum_size=100)

    status_blackboard = blackboards.create_status_blackboard()

    root = create_root(robot_name=args.robot)
    tree = BehaviourTree(root)
    tree.add_pre_tick_handler(emergency_pre_tick_handler)  
    tree.add_pre_tick_handler(generic_pre_tick_handler)     


    try:
        tree.setup(node=node, timeout=30)
    except TimedOutError as e:
        node.get_logger().error(f"Failed to setup the tree, aborting [{e}]")
        Blackboard.clear()
        node.destroy_node()
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        node.get_logger().error("Tree setup interrupted!")
        Blackboard.clear()
        graceful_shutdown(node, tree)
        sys.exit(1)

    try:
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
            tree.tick()

            start_time = time.time()
            while (time.time() - start_time) < 0.5:
                if shutdown_requested or not rclpy.ok():
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as ex:
        print(f"Thrown Exception: {ex}")
        Blackboard.clear()
        graceful_shutdown(node, tree)
        sys.exit(1)
    
    Blackboard.clear()

    graceful_shutdown(node, tree)
    sys.exit(1)

