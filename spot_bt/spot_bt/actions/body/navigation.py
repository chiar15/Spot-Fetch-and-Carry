"""
Copyright (c) 2026 Chiara Ferraioli

This module contains navigation behaviors for the Spot robot using GraphNav.
It includes a greedy exploration strategy (Navigate) that visits clusters 
based on visit counts and a direct return-to-home behavior (GoHome).
"""

from __future__ import annotations

import json
import os
from typing import Optional, Any

import networkx as nx
import numpy as np
from py_trees.common import Status, Access

from bosdyn.api.graph_nav import map_pb2
from bosdyn.client.math_helpers import SE3Pose
from synchros2.utilities import namespace_with

from spot_bt.utils.behaviour_bases import NavigateBase


class Navigate(NavigateBase):
    """
    Action behavior that commands Spot to navigate to the nearest unvisited cluster.

    It implements a greedy strategy by maintaining a visit count for each cluster 
    and selecting the one with the minimum visits that is closest to the 
    robot's current known waypoint.

    Blackboard:
        navigation_goal (Access.WRITE): Inherited from NavigateBase. Stores the 
            calculated target waypoint goal for the action client.
    """
    
    def __init__(self, name: str, robot_name: Optional[str] = None):
        """
        Initializes the Navigate behavior and internal graph structures.

        Args:
            name (str): Name of the behavior.
            robot_name (Optional[str]): The namespace of the robot.
        """
        super().__init__(name=name, robot_name=robot_name)
        
        self.graph = nx.Graph()
        self.origin_waypoint: Optional[str] = None
        self.current_waypoint: Optional[str] = None
        self.centroids: Optional[dict[int, dict[str, str]]] = None
        self.cluster_ids: Optional[list[int]] = None
        self.clusters_visits: Optional[dict[int, int]] = None
        self.cluster: Optional[int] = None
        self.goal: Optional[dict[str, str]] = None

        self._active_cluster: Optional[int] = None
        self._active_goal_id: Optional[str] = None

    def setup(self, **kwargs: Any):
        """
        Loads the navigation graph and cluster assets.

        Initializes the NetworkX graph from the GraphNav map and loads 
        centroid information from the clusters.json asset.

        Args:
            **kwargs: Arbitrary keyword arguments, including the ROS 2 node.
        """
        super().setup(**kwargs)
        
        self.load_graph_nx()
        
        # Load cluster definitions
        with open("/home/chiara/Desktop/Tesi SPOT/src/spot_bt/spot_bt/assets/clusters.json", "r") as f:
            centroids_list = json.load(f)
        
        self.centroids = {
            int(item["cluster_id"]): {"id": item["id"], "name": item.get("name", item["id"])}
            for item in centroids_list
        }
        
        self.cluster_ids = sorted(self.centroids.keys())
        self.clusters_visits = {cid: 0 for cid in self.cluster_ids}
    
    def initialise(self):
        """
        Triggers the exploration logic.

        It calculates the target cluster using the greedy strategy, prepares 
         the navigation goal, and initiates the action via the base class.
        """
        self.logger.debug(f"{self.name} [Navigate::initialise()]")
        
        self.calculate_goal()
        
        description = f"cluster={self.cluster}, waypoint={self.goal.get('name', self.goal['id'])}"
        self._active_cluster = self.cluster
        self._active_goal_id = self.goal["id"]
        
        # Trigger navigation goal sending
        self._send_goal(self._active_goal_id, description)
        
        super().initialise()
    
    def _on_navigation_success(self):
        """
        Updates the internal state upon successful arrival at a waypoint.

        Increments the visit counter for the reached cluster and updates 
        the current robot position reference.
        """
        self.clusters_visits[self._active_cluster] += 1
        self.current_waypoint = self._active_goal_id
        self.logger.info(
            f"Cluster {self._active_cluster} visit count: "
            f"{self.clusters_visits[self._active_cluster]}"
        )

    def load_graph_nx(self):
        """
        Parses the GraphNav map to build a NetworkX representation.

        It processes waypoints and edges from the binary graph file, 
        calculating Euclidean distances between waypoints to use as edge weights.
        """
        positions = {}
        graph_path = os.path.join("/home/chiara/Desktop/Tesi SPOT/src/spot_bt/spot_bt/assets/map", "graph")
        
        with open(graph_path, "rb") as f:
            data = f.read()
            current_graph = map_pb2.Graph()
            current_graph.ParseFromString(data)

        # Identify waypoints and set origin
        for wp in current_graph.waypoints:
            self.graph.add_node(wp.id)
            if wp.annotations.name == "waypoint_56":
                self.origin_waypoint = wp.id

        if self.origin_waypoint is None:
            self.logger.warning("waypoint_56 not found, using the first available waypoint")
            self.origin_waypoint = current_graph.waypoints[0].id

        self.current_waypoint = self.origin_waypoint

        # BFS to calculate world coordinates from relative transforms
        queue = [(self.origin_waypoint, np.eye(4))]
        visited = set()

        while queue:
            curr_id, world_tform_wp = queue.pop(0)
            if curr_id in visited:
                continue
            visited.add(curr_id)

            positions[curr_id] = (world_tform_wp[0, 3], world_tform_wp[1, 3])

            for edge in current_graph.edges:
                if edge.id.from_waypoint == curr_id and edge.id.to_waypoint not in visited:
                    from_tform_to = SE3Pose.from_proto(edge.from_tform_to).to_matrix()
                    world_tform_to = world_tform_wp @ from_tform_to
                    queue.append((edge.id.to_waypoint, world_tform_to))
                elif edge.id.to_waypoint == curr_id and edge.id.from_waypoint not in visited:
                    to_tform_from = SE3Pose.from_proto(edge.from_tform_to).inverse().to_matrix()
                    world_tform_from = world_tform_wp @ to_tform_from
                    queue.append((edge.id.from_waypoint, world_tform_from))

        # Add edges with weights (Euclidean distance)
        for edge in current_graph.edges:
            u = edge.id.from_waypoint
            v = edge.id.to_waypoint
            if u in positions and v in positions:
                dist = float(np.linalg.norm(np.array(positions[v]) - np.array(positions[u])))
                self.graph.add_edge(u, v, weight=dist)
            else:
                self.graph.add_edge(u, v, weight=1.0)

    def _clusters_with_min_visits(self) -> list[int]:
        """
        Identifies clusters with the lowest number of successful visits.

        Returns:
            list[int]: A list of cluster IDs sharing the minimum visit count.
        """
        min_visits = min(self.clusters_visits.values()) if self.clusters_visits else 0
        return [cid for cid, v in self.clusters_visits.items() if v == min_visits]

    def _distance_wp_to_cluster_centroid(self, source_wp: str, cluster_id: int) -> float:
        """
        Calculates the shortest path distance between a waypoint and a cluster centroid.

        Args:
            source_wp (str): The starting waypoint ID.
            cluster_id (int): The target cluster ID.

        Returns:
            float: The path length based on graph weights.
        """
        target_wp = self.centroids[cluster_id]["id"]
        return float(nx.shortest_path_length(self.graph, source_wp, target_wp, weight="weight"))

    def calculate_goal(self):
        """
        Selects the target cluster based on the greedy exploration strategy.

        It chooses the cluster with the minimum visits that is closest to 
        the current waypoint.
        """
        candidate_clusters = self._clusters_with_min_visits()

        if len(candidate_clusters) == 0:
            raise RuntimeError("No clusters available for navigation.")

        # Find the closest cluster among those with minimum visits
        closest_cluster = min(
            candidate_clusters,
            key=lambda cid: self._distance_wp_to_cluster_centroid(self.current_waypoint, cid)
        )

        self.cluster = closest_cluster
        self.goal = self.centroids[closest_cluster]
        
        self.logger.info(
            f"Selected cluster {self.cluster} (visits: {self.clusters_visits[self.cluster]}) "
            f"with centroid {self.goal.get('name', self.goal['id'])}"
        )


class GoHome(NavigateBase):
    """
    Action behavior that commands Spot to return to the home waypoint.

    This behavior uses a fixed destination defined in the home.json asset.

    Blackboard:
        navigation_goal (Access.WRITE): Inherited from NavigateBase. Stores 
            the home waypoint ID for the action client.
    """
    
    def __init__(self, name: str, robot_name: Optional[str] = None):
        """
        Initializes the GoHome behavior and loads the home destination.

        Args:
            name (str): Name of the behavior.
            robot_name (Optional[str]): The namespace of the robot.
        """
        super().__init__(name=name, robot_name=robot_name)
        with open("/home/chiara/Desktop/Tesi SPOT/src/spot_bt/spot_bt/assets/home.json", "r") as f:
            home_information = json.load(f)

        self.home = home_information["waypoint_id"]
    
    def initialise(self):
        """
        Triggers the navigation to the home waypoint.

        It prepares the home destination and initiates the action via the base class.
        """
        self.logger.debug(f"{self.name} [GoHome::initialise()]")
        
        # Trigger navigation to the fixed home waypoint
        self._send_goal(self.home, "home waypoint")
        
        super().initialise()