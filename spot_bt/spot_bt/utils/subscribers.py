"""
Copyright (c) 2026 Chiara Ferraioli

This module contains py_trees subscriber handlers that listen to specific 
ROS 2 topics (e.g., odometry, manipulation state) and update the 
behavior tree's blackboard accordingly.
"""
from bosdyn_api_msgs.msg import ManipulatorState
from py_trees_ros.subscribers import Handler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from typing import Optional
from synchros2.utilities import namespace_with
from py_trees.common import *
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import math


LINEAR_STILL_THRESHOLD  = 0.05   # m/s
ANGULAR_STILL_THRESHOLD = 0.05   # rad/s


class MovingScan(Handler):
    """
    Subscribes to the robot's odometry twist topic to determine if 
    the robot is currently moving or stopped. It updates the 'state.stopped' 
    flag on the status blackboard.
    """
    def __init__(self, name: str, robot_name: Optional[str] = None):
        """
        Initializes the MovingScan handler.

        Args:
            name (str): The name of the behavior.
            robot_name (Optional[str]): The robot namespace to prefix the topic.
        """
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        super().__init__(
            name=name,
            topic_name=namespace_with(robot_name, "odometry/twist"),
            topic_type=TwistWithCovarianceStamped,
            qos_profile=qos
        )

        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)
    
    def update(self):
        """
        Reads the latest twist message, calculates the 2D norm for linear 
        and angular speeds, and evaluates if the robot is stopped.

        Returns:
            Status: Always returns RUNNING as it continuously monitors the state.
        """
        self.logger.debug(f"{self.name} [MovingScan::update()]")

        with self.data_guard:
            if self.msg is None:
                self.feedback_message = "no message received yet"
                return Status.RUNNING
            
            linear  = self.msg.twist.twist.linear
            angular = self.msg.twist.twist.angular

            linear_speed  = math.sqrt(linear.x**2  + linear.y**2)
            angular_speed = math.sqrt(angular.x**2 + angular.y**2)

            stopped = (
                linear_speed  < LINEAR_STILL_THRESHOLD and
                angular_speed < ANGULAR_STILL_THRESHOLD
            )

            self.logger.warning(
                f"linear={linear_speed:.3f} m/s  "
                f"angular={angular_speed:.3f} rad/s  "
                f"→ stopped={stopped}"
            )

            self.blackboard.set("state.stopped", stopped)

        return Status.RUNNING

        
class ManipulatorScan(Handler):
    """
    Subscribes to the robot's manipulation state topic to monitor 
    the arm's status, specifically the gripper openness percentage.
    Updates the 'state.gripper_percentage' on the status blackboard.
    """
    def __init__(self, name: str, robot_name: Optional[str] = None):
        """
        Initializes the ManipulatorScan handler.

        Args:
            name (str): The name of the behavior.
            robot_name (Optional[str]): The robot namespace to prefix the topic.
        """
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        super().__init__(
            name=name,
            topic_name=namespace_with(robot_name, "/manipulation_state"),
            topic_type=ManipulatorState,
            qos_profile=qos
        )

        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)
    
    def update(self):
        """
        Processes the received ManipulatorState message to extract the gripper's 
        openness percentage and updates the corresponding blackboard state.
        If no message has been received yet, it sets a feedback message. 

        Returns:
            Status: Always returns RUNNING as it continuously monitors the state.
        """
        self.logger.debug(f"{self.name}  [ManipulatorScan::update()]")

        with self.data_guard:
            if self.msg is None:
                self.feedback_message = "no message received yet"
                return Status.RUNNING
            
            gripper_percentage = self.msg.gripper_open_percentage
            
            self.logger.warning(f"Gripper Percentage : {gripper_percentage}")
            self.blackboard.set("state.gripper_percentage", gripper_percentage)

            return Status.RUNNING