"""
Copyright (c) 2026 Chiara Ferraioli

This module contains navigation-related condition behaviors for the Spot robot.
It specifically implements the IsHome condition, which verifies the robot's 
current localization pose against a predefined home position.
"""

from __future__ import annotations

import json
from typing import Any, Optional

import numpy as np
from py_trees.common import Status

from spot_bt.utils.behaviour_bases import ServiceClientTimeout
from spot_msgs.srv import GraphNavGetLocalizationPose


class IsHome(ServiceClientTimeout):
    """
    Condition behavior that checks if the robot is at the designated home position.

    It utilizes the GraphNav localization service to retrieve the current pose 
    and compares it to a target position loaded from a configuration file.

    Blackboard:
        status (Client): Attached blackboard client to access global robot state.
    """

    def __init__(self, name: str, timeout_sec: float = 30.0):
        """
        Initializes the behavior and defines the proximity tolerance.

        Args:
            name (str): Name of the behavior.
            timeout_sec (float): Maximum time to wait for the localization service. 
                Defaults to 30.0.
        """
        super().__init__(
            name=name,
            service_type=GraphNavGetLocalizationPose,
            service_name="graph_nav_get_localization_pose",
            timeout_sec=timeout_sec
        )
        self.blackboard = self.attach_blackboard_client("status")
        self.tolerance = 0.2
        self.home_pose: Optional[dict[str, float]] = None

    def setup(self, **kwargs: Any):
        """
        Initializes the service client and loads the home position from assets.

        Args:
            **kwargs: Arbitrary keyword arguments, including the ROS 2 node.

        Raises:
            FileNotFoundError: If the home.json configuration file is missing.
            KeyError: If the expected 'pose' or 'position' keys are missing in the JSON.
        """
        super().setup(**kwargs)
        
        # Load the predefined home coordinates from the asset file
        with open("/home/chiara/Desktop/Tesi SPOT/src/spot_bt/spot_bt/assets/home.json", "r") as f:
            home_info = json.load(f)
        
        self.home_pose = home_info["pose"]["position"]

    def initialise(self):
        """
        Initiates the home status check via the localization service and logs the start of the operation.
        """
        self.logger.debug(f"{self.name} [IsHome::initialise()]")
        return super().initialise()

    def update(self) -> Status:
        """
        Processes the localization response and evaluates the distance to home.

        It calculates the 2D Euclidean distance between the current robot 
        coordinates (retrieved from GraphNav) and the home coordinates.

        Returns:
            Status: SUCCESS if the 2D distance is within the 0.2m tolerance, 
                FAILURE if the robot is outside the range or the service fails, 
                or RUNNING while waiting for the response.
        """
        self.logger.debug(f"{self.name} [IsHome::update()]")
        status = super().update()

        if status != Status.SUCCESS:
            return status
        
        if not self.latest_result.success:
            self.logger.warning("Failed to get localization pose from GraphNav")
            return Status.FAILURE
        
        curr_pose = self.latest_result.pose
        
        robot_x = curr_pose.pose.position.x
        robot_y = curr_pose.pose.position.y
        
        home_x = self.home_pose["x"]
        home_y = self.home_pose["y"]
        
        # Euclidean distance calculation
        distance_2d = np.sqrt(
            (robot_x - home_x)**2 + 
            (robot_y - home_y)**2
        )
        
        is_at_home = distance_2d <= self.tolerance
        
        self.logger.info(
            f"Robot position: ({robot_x:.2f}, {robot_y:.2f}), "
            f"Home position: ({home_x:.2f}, {home_y:.2f}), "
            f"Distance: {distance_2d:.2f}m, "
            f"Is home: {is_at_home}"
        )
        
        return Status.SUCCESS if is_at_home else Status.FAILURE
    
    def terminate(self, new_status: Status):
        """
        Cleans up service-related variables and logs the behavior termination.

        Args:
            new_status (Status): The status to which the behavior is transitioning.
        """
        self.logger.debug(f"{self.name} [IsHome::terminate()][{self.status}->{new_status}]")
        return super().terminate(new_status)