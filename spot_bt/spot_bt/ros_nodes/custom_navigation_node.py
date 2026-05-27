"""
Copyright (c) 2026 Chiara Ferraioli

This module provides the CustomNavigationNode, a ROS 2 node that interfaces 
with the Boston Dynamics Spot GraphNav API. It exposes action servers for 
navigation and services to upload or clear the navigation graph.
"""
#!/usr/bin/env python3
import logging
import time
import traceback
import threading
import signal
import os
import json


import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


from std_srvs.srv import Trigger


from bosdyn.api import lease_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2
from bosdyn.client import create_standard_sdk
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.lease import Lease


from spot_driver.ros_helpers import get_from_env_and_fall_back_to_param
from spot_msgs.action import NavigateTo
from spot_msgs.srv import AcquireLease, ReturnLease, GraphNavUploadGraph

shutdown_requested = False

def signal_handler(sig, frame):
    """
    Handles system interrupt signals to initiate a graceful node shutdown.

    Args:
        sig (int): The signal number received (e.g., SIGINT).
        frame: The current stack frame.
    """
    global shutdown_requested
    shutdown_requested = True

class CustomNavigationNode(Node):
    """
    ROS 2 Node that wraps Boston Dynamics' GraphNav functionalities.

    This node provides an Action Server for navigating to specific waypoints 
    and Service Servers for uploading maps (graphs, waypoints, edge snapshots) 
    and clearing the current navigation graph from the robot's memory.
    """
    def __init__(self):
        """
        Initializes the node, sets up parameters, callback groups, SDK clients, 
        action servers, and service servers.
        """
        super().__init__("custom_navigation_node")

        self._action_group = MutuallyExclusiveCallbackGroup()
        self._clients_group = MutuallyExclusiveCallbackGroup()
        self._srv_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("spot_name", "")
        self.declare_parameter("poll_rate", 10.0)
        self._poll_rate: float = float(self.get_parameter("poll_rate").value)

        spot_name = self.get_parameter("spot_name").value
        namespace = f"/{spot_name}" if spot_name else ""

        self._acquire_lease_client = self.create_client(
            AcquireLease, f"{namespace}/acquire_lease", callback_group=self._clients_group
        )
        self._return_lease_client = self.create_client(
            ReturnLease, f"{namespace}/return_lease", callback_group=self._clients_group
        )
        self._stop_client = self.create_client(
            Trigger, f"{namespace}/stop", callback_group=self._clients_group
        )

        if not self._stop_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("[INIT] Stop service non disponibile")
            return

        if not self._acquire_lease_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("[INIT] AcquireLease service non disponibile")
            return

        if not self._return_lease_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("[INIT] ReturnLease service non disponibile")
            return

        username = get_from_env_and_fall_back_to_param(
            "BOSDYN_CLIENT_USERNAME", self, "username", "user"
        )
        password = get_from_env_and_fall_back_to_param(
            "BOSDYN_CLIENT_PASSWORD", self, "password", "password"
        )
        ip = get_from_env_and_fall_back_to_param("SPOT_IP", self, "hostname", "10.0.0.3")

        logging.basicConfig(format="[%(filename)s:%(lineno)d] %(message)s", level=logging.ERROR)

        try:
            sdk = create_standard_sdk("custom_navigation_node")
            self._robot = sdk.create_robot(ip)
            self._robot.authenticate(username, password)
            self._robot.time_sync.wait_for_sync()
            self._graph_nav_client = self._robot.ensure_client(GraphNavClient.default_service_name)
        except Exception:
            self.get_logger().error(f"[SDK] Errore connessione:\n{traceback.format_exc()}")
            return

        with open("/home/chiara/Desktop/Tesi SPOT/src/spot_bt/spot_bt/assets/home.json", "r") as f:
            home_information = json.load(f)
        self._home_waypoint_id = home_information["waypoint_id"]

        self._nav_lock = threading.Lock()
        self._reached_goal = False
        self._terminal_reason = None
        self._active_lease = None

        self.navigate_as = ActionServer(
            self,
            NavigateTo,
            "navigate_custom",
            execute_callback=self.handle_navigate_to,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_group,
        )

        self._graphnav_upload_srv = self.create_service(
            GraphNavUploadGraph,
            "graphnav_upload_graph_custom",
            self.handle_graphnav_upload_graph,
            callback_group=self._srv_group,
        )

        self._graphnav_clear_srv = self.create_service(
            Trigger,
            "graphnav_clear_graph_custom",
            self.handle_graphnav_clear_graph,
            callback_group=self._srv_group,
        )

        self.goal_handle: ServerGoalHandle | None = None
        self.run_navigate_to = False
        self.feedback_thread: threading.Thread | None = None

        self.get_logger().info("Node all ready!!")


    def call_service(self, client, request, timeout_sec=20.0, tag="SRV", spin_during_wait=False):
        """
        Synchronously calls a ROS 2 service while avoiding executor deadlocks.

        Args:
            client: The ROS 2 service client.
            request: The service request payload.
            timeout_sec (float, optional): Max seconds to wait for a response. Defaults to 20.0.
            tag (str, optional): A tag string used for error logging. Defaults to "SRV".
            spin_during_wait (bool, optional): Whether to call spin_once while waiting. Defaults to False.

        Returns:
            The service response object, or None if the call times out or throws an exception.
        """
        event = threading.Event()
        future = client.call_async(request)

        def done_cb(_fut):
            event.set()

        future.add_done_callback(done_cb)

        t_end = time.time() + timeout_sec
        while not event.is_set() and time.time() < t_end and rclpy.ok():
            if spin_during_wait:
                rclpy.spin_once(self, timeout_sec=0.1)
            else:
                time.sleep(0.01)

        if not event.is_set():
            self.get_logger().error(f"[{tag}] TIMEOUT <- {client.srv_name}")
            return None
        if future.exception() is not None:
            self.get_logger().error(f"[{tag}] EXC <- {client.srv_name}: {future.exception()}")
            return None
        return future.result()

    def _graphnav_download_healthcheck(self) -> tuple[bool, str]:
        """
        Tests the GraphNav connection by attempting to download the current graph.

        Returns:
            tuple[bool, str]: A boolean indicating success, and a status message.
        """
        try:
            g = self._graph_nav_client.download_graph()
            if g is None:
                return True, "download_graph OK (None)"
            return True, f"download_graph OK (wp={len(g.waypoints)} edges={len(g.edges)})"
        except Exception as e:
            return False, f"download_graph FAILED: {e}"
    
    def _upload_graph_and_snapshots_from_path(self, upload_path: str) -> tuple[bool, str]:
        """
        Uploads a saved graph, its waypoints, and edge snapshots from a directory to the robot.

        Includes fallback mechanisms to upload snapshots individually or in batches 
        if the payload exceeds the maximum byte limit.

        Args:
            upload_path (str): The file path containing the map data (graph, waypoint_snapshots, edge_snapshots).

        Returns:
            tuple[bool, str]: A boolean indicating success, and a detailed status message.
        """
        graph_path = os.path.join(upload_path, "graph")
        wp_dir = os.path.join(upload_path, "waypoint_snapshots")
        edge_dir = os.path.join(upload_path, "edge_snapshots")

        if not os.path.isfile(graph_path):
            return False, f"File non trovato: {graph_path}"
        if not os.path.isdir(wp_dir):
            return False, f"Directory non trovata: {wp_dir}"
        if not os.path.isdir(edge_dir):
            return False, f"Directory non trovata: {edge_dir}"

        with open(graph_path, "rb") as f:
            current_graph = map_pb2.Graph()
            current_graph.ParseFromString(f.read())

        waypoint_snaps: dict[str, map_pb2.WaypointSnapshot] = {}
        edge_snaps: dict[str, map_pb2.EdgeSnapshot] = {}

        for waypoint in current_graph.waypoints:
            if not waypoint.snapshot_id:
                continue
            p = os.path.join(wp_dir, waypoint.snapshot_id)
            if not os.path.exists(p):
                continue
            with open(p, "rb") as sf:
                s = map_pb2.WaypointSnapshot()
                s.ParseFromString(sf.read())
                waypoint_snaps[s.id] = s

        for edge in current_graph.edges:
            if not edge.snapshot_id:
                continue
            p = os.path.join(edge_dir, edge.snapshot_id)
            if not os.path.exists(p):
                continue
            with open(p, "rb") as sf:
                s = map_pb2.EdgeSnapshot()
                s.ParseFromString(sf.read())
                edge_snaps[s.id] = s

        resp = self._graph_nav_client.upload_graph(lease=None, graph=current_graph)

        if resp.status != graph_nav_pb2.UploadGraphResponse.STATUS_OK:
            try:
                status_name = graph_nav_pb2.UploadGraphResponse.Status.Name(resp.status)
            except Exception:
                status_name = str(resp.status)
            return False, f"upload_graph rifiutato: {status_name}"
        
        missing_wp = list(resp.unknown_waypoint_snapshot_ids)
        missing_edge = list(resp.unknown_edge_snapshot_ids)

        if len(missing_wp) == 0 and len(missing_edge) == 0:
            return True, (
                f"upload_graph OK (wp={len(current_graph.waypoints)}, edges={len(current_graph.edges)}), "
                "snapshots già presenti"
            )

        upload_individually = False
        try:
            self._graph_nav_client.upload_snapshots(
                graph_nav_pb2.UploadSnapshotsRequest.Snapshots(waypoint_snapshots=[], edge_snapshots=[]),
                lease=None,
            )
        except Exception:
            upload_individually = True

        if upload_individually:
            for sid in missing_wp:
                if sid not in waypoint_snaps:
                    return False, f"Manca waypoint snapshot su disco id={sid}"
                self._graph_nav_client.upload_waypoint_snapshot(waypoint_snaps[sid], lease=None)

            for sid in missing_edge:
                if sid not in edge_snaps:
                    return False, f"Manca edge snapshot su disco id={sid}"
                self._graph_nav_client.upload_edge_snapshot(edge_snaps[sid], lease=None)

            return True, f"upload snapshot OK (fallback) wp={len(missing_wp)} edge={len(missing_edge)}"

        kMaxBytes = 16 * 1024 * 1024

        batch = []
        nbytes = 0
        for sid in missing_wp:
            if sid not in waypoint_snaps:
                return False, f"Manca waypoint snapshot su disco id={sid}"
            s = waypoint_snaps[sid]
            b = s.ByteSize()
            if batch and (nbytes + b > kMaxBytes):
                self._graph_nav_client.upload_snapshots(
                    graph_nav_pb2.UploadSnapshotsRequest.Snapshots(waypoint_snapshots=batch, edge_snapshots=[]),
                    lease=None,
                )
                batch = []
                nbytes = 0
            batch.append(s)
            nbytes += b
        if batch:
            self._graph_nav_client.upload_snapshots(
                graph_nav_pb2.UploadSnapshotsRequest.Snapshots(waypoint_snapshots=batch, edge_snapshots=[]),
                lease=None,
            )

        batch = []
        nbytes = 0
        for sid in missing_edge:
            if sid not in edge_snaps:
                return False, f"Manca edge snapshot su disco id={sid}"
            s = edge_snaps[sid]
            b = s.ByteSize()
            if batch and (nbytes + b > kMaxBytes):
                self._graph_nav_client.upload_snapshots(
                    graph_nav_pb2.UploadSnapshotsRequest.Snapshots(waypoint_snapshots=[], edge_snapshots=batch),
                    lease=None,
                )
                batch = []
                nbytes = 0
            batch.append(s)
            nbytes += b
        if batch:
            self._graph_nav_client.upload_snapshots(
                graph_nav_pb2.UploadSnapshotsRequest.Snapshots(waypoint_snapshots=[], edge_snapshots=batch),
                lease=None,
            )

        return True, f"upload snapshot OK (batch) wp={len(missing_wp)} edge={len(missing_edge)}"

    def handle_graphnav_upload_graph(self, request, response):
        """
        Service callback to trigger a map upload to the robot.

        Args:
            request (GraphNavUploadGraph.Request): The service request containing the file path.
            response (GraphNavUploadGraph.Response): The service response to be populated.

        Returns:
            GraphNavUploadGraph.Response: The populated response indicating success or failure.
        """
        upload_path = request.upload_filepath
        if not upload_path:
            response.success = False
            response.message = "upload_filepath vuoto"
            self.get_logger().info("Upload graph failed: no file path")
            return response

        ok, msg = self._graphnav_download_healthcheck()
        if not ok:
            response.success = False
            response.message = msg
            self.get_logger().info("Upload graph pre-check failed")
            return response

        try:
            self.get_logger().info("Uploading graph...")
            t0 = time.time()
            ok2, msg2 = self._upload_graph_and_snapshots_from_path(upload_path)
            dt = time.time() - t0
            response.success = bool(ok2)
            response.message = f"{msg2} (t={dt:.2f}s)"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Eccezione upload: {e}"
            self.get_logger().info(f"Upload graph exception: {e}")
            return response

    def handle_graphnav_clear_graph(self, request, response):
        """
        Service callback to clear the currently loaded map from the robot's GraphNav memory.

        Args:
            request (Trigger.Request): The service request.
            response (Trigger.Response): The service response to be populated.

        Returns:
            Trigger.Response: The populated response indicating success or failure.
        """
        ok, msg = self._graphnav_download_healthcheck()
        if not ok:
            response.success = False
            response.message = msg
            self.get_logger().warning("Clear graph pre-check failed")
            return response

        try:
            self.get_logger().info("Clearing graph...")
            t0 = time.time()
            self._graph_nav_client.clear_graph(lease=None)
            dt = time.time() - t0

            ok2, msg2 = self._graphnav_download_healthcheck()
            response.success = True
            response.message = f"clear_graph OK (t={dt:.2f}s). Postcheck: {msg2}"
            return response
        except Exception as e:
            response.success = False
            response.message = f"clear_graph FAILED: {e}"
            self.get_logger().info(f"Clear graph exception: {e}")
            return response

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """
        Action server cancel callback. Rejects cancellation if the goal 
        has already been reached, otherwise accepts it.

        Args:
            goal_handle (ServerGoalHandle): The handle of the goal requesting cancellation.

        Returns:
            CancelResponse: The system's response (ACCEPT or REJECT) to the cancel request.
        """
        with self._nav_lock:
            already_reached = self._reached_goal

        if already_reached:
            self.get_logger().info("action cancel: REJECT (già raggiunto)")
            return CancelResponse.REJECT

        self.get_logger().info("action cancel: ACCEPT")
        return CancelResponse.ACCEPT

    def _nav_state(self, command_id: int | None) -> str:
        """
        Queries the GraphNav client for the current navigation status.

        Args:
            command_id (int | None): The active navigation command ID.

        Returns:
            str: The current state ("reached", "failed", or "running").
        """
        if command_id is None:
            return "running"
        try:
            status = self._graph_nav_client.navigation_feedback(command_id)
            if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
                return "reached"
            elif status.status in (
                graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST,
                graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK,
                graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED,
            ):
                return "failed"
            else:
                return "running"
        except Exception as e:
            return "running"

    def handle_navigate_to_feedback(self) -> None:
        """
        Runs on a separate thread to poll the robot's localization state 
        and continuously publish feedback to the action client.
        """
        rate_sleep = 1.0 / max(1e-6, float(self._poll_rate))
        while rclpy.ok() and self.run_navigate_to and self.goal_handle is not None:
            try:
                state = self._graph_nav_client.get_localization_state()
                if state.localization.waypoint_id and self.goal_handle is not None:
                    fb = NavigateTo.Feedback()
                    fb.waypoint_id = state.localization.waypoint_id
                    self.goal_handle.publish_feedback(fb)
            except Exception as e:
                self.get_logger().info(f"Feedback exception: {e}")
            time.sleep(rate_sleep)

    def handle_navigate_to(self, goal_handle: ServerGoalHandle) -> NavigateTo.Result:
        """
        Main execution callback for the NavigateTo action. 

        Acquires the robot lease, sends the GraphNav command to navigate to the 
        requested waypoint, and polls for success, failure, or cancellation. 
        Ensures the lease is properly returned upon completion.

        Args:
            goal_handle (ServerGoalHandle): The action server goal handle.

        Returns:
            NavigateTo.Result: The final result of the navigation action.
        """
        global shutdown_requested

        result = NavigateTo.Result()
        waypoint_id = goal_handle.request.waypoint_id
        is_home = (waypoint_id == self._home_waypoint_id)
        travel_params = None if is_home else graph_nav_pb2.TravelParams(ignore_final_yaw=True)

        with self._nav_lock:
            self._reached_goal = False
            self._terminal_reason = None

        acq_req = AcquireLease.Request()
        acq_req.client_name = "custom_navigation_node"
        acq_req.resource_name = "body"
        acq_req.force = True

        lease_resp = self.call_service(
            self._acquire_lease_client, acq_req, timeout_sec=20.0, tag="LEASE"
        )
        if lease_resp is None or not lease_resp.success:
            result.success = False
            result.message = "Lease non ottenuto"
            goal_handle.abort()
            return result

        lease_proto = lease_pb2.Lease(
            resource=lease_resp.lease.resource,
            epoch=lease_resp.lease.epoch,
            sequence=lease_resp.lease.sequence,
            client_names=lease_resp.lease.client_names,
        )
        lease = Lease(lease_proto)
        self._active_lease = lease_resp.lease

        self.goal_handle = goal_handle
        self.run_navigate_to = True
        self.feedback_thread = threading.Thread(target=self.handle_navigate_to_feedback, daemon=True)
        self.feedback_thread.start()

        nav_cmd_id = None

        try:
            rate_sleep = 1.0 / max(1e-6, float(self._poll_rate))

            while rclpy.ok() and goal_handle.is_active and not shutdown_requested:
                if goal_handle.is_cancel_requested:
                    state = self._nav_state(nav_cmd_id)
                    if state == "reached":
                        with self._nav_lock:
                            self._reached_goal = True
                            self._terminal_reason = "succeeded"
                        break
                    else:
                        _ = self.call_service(
                            self._stop_client, Trigger.Request(), timeout_sec=5.0, tag="STOP"
                        )
                        with self._nav_lock:
                            self._terminal_reason = "canceled"
                        break

                try:
                    nav_cmd_id = self._graph_nav_client.navigate_to(
                        waypoint_id,
                        1.0,
                        travel_params=travel_params,
                        leases=[lease.lease_proto],
                        command_id=nav_cmd_id,
                    )
                except Exception as e:
                    self.get_logger().info(f"Navigation exception: {e}")
                    with self._nav_lock:
                        self._terminal_reason = "aborted"
                    break

                state = self._nav_state(nav_cmd_id)
                if state == "reached":
                    with self._nav_lock:
                        self._reached_goal = True
                        self._terminal_reason = "succeeded"
                    break
                elif state == "failed":
                    with self._nav_lock:
                        self._terminal_reason = "aborted"
                    break

                lease = lease.create_newer()
                time.sleep(rate_sleep)

        finally:
            self.run_navigate_to = False
            if self.feedback_thread and self.feedback_thread.is_alive():
                self.feedback_thread.join(timeout=1.0)
            self.feedback_thread = None
            self.goal_handle = None

            try:
                ret_req = ReturnLease.Request()
                ret_req.lease = lease_resp.lease

                do_spin = shutdown_requested

                ret = self.call_service(
                    self._return_lease_client, ret_req, timeout_sec=20.0, tag="LEASE", spin_during_wait=do_spin
                )
                if ret and ret.success:
                    self._active_lease = None
            except Exception as e:
                self.get_logger().info(f"ReturnLease exception: {e}")

            with self._nav_lock:
                terminal = self._terminal_reason

            if terminal == "succeeded":
                result.success = True
                result.message = "Destinazione raggiunta!"
                self.get_logger().info("Destinazione Raggiunta!")
                goal_handle.succeed()
            elif terminal == "canceled":
                result.success = False
                result.message = "Navigazione cancellata"
                self.get_logger().info("Navigazione Cancellata")
                goal_handle.canceled()
            else:
                result.success = False
                result.message = "Navigazione fallita"
                self.get_logger().info("Navigazione Fallita")
                goal_handle.abort()
            return result

def main(args=None):
    """
    Main entry point for the CustomNavigationNode.

    Initializes the ROS 2 node and execution loop, and ensures 
    safe resource cleanup (like returning the lease) upon shutdown.

    Args:
        args (Optional[list]): Command-line arguments. Defaults to None.
    """
    rclpy.init()
    node = CustomNavigationNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while rclpy.ok() and not shutdown_requested:
            executor.spin_once(timeout_sec=0.1)
    finally:
        try:
            if node._active_lease is not None:
                ret_req = ReturnLease.Request()
                ret_req.lease = node._active_lease
                _ = node.call_service(
                    node._return_lease_client, ret_req, timeout_sec=10.0, tag="LEASE", spin_during_wait=True
                )
        except Exception as e:
            node.get_logger().info(f"[MAIN] ReturnLease shutdown EXC: {e}")
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
