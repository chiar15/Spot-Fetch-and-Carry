"""
Copyright (c) 2026 Chiara Ferraioli

This module contains the InitializationNode, a crucial ROS 2 node responsible 
for the startup and shutdown lifecycles of the Spot robot. It orchestrates 
motor power, posture, manipulator stowing, and the GraphNav map initialization.
"""
import argparse
import time
import rclpy
from rclpy.node import Node
from typing import Optional
import signal

from spot_msgs.srv import SetVelocity
from geometry_msgs.msg import Twist
from synchros2.utilities import namespace_with
from std_srvs.srv import Trigger
from spot_msgs.srv import GraphNavUploadGraph, GraphNavClearGraph, GraphNavSetLocalization, GraphNavGetLocalizationPose


class InitializationNode(Node):
    """
    ROS 2 Node that acts as the orchestrator for Spot's hardware and software initialization.

    It establishes connections to various Spot ROS 2 services and executes the 
    required sequences to safely bring the robot to a standing, localized state 
    ready for missions. It also provides a fault-tolerant shutdown sequence to 
    safely stow the arm, sit, and power off the motors.
    """
    def __init__(self, robot_name: Optional[str] = None) -> None:
        """
        Initializes the node, defines the static map path, and connects to services.

        Args:
            robot_name (Optional[str]): The namespace associated with the robot.
        """
        super().__init__("initialization_node")

        self.robot_name = robot_name
        self.file_path = "/home/chiara/Desktop/Tesi SPOT/src/spot_bt/spot_bt/assets/map"
        self.manage_services(robot_name)
    
    def manage_services(self, robot_name):
        """
        Creates and awaits the availability of all required Spot service clients.

        Args:
            robot_name (Optional[str]): The robot namespace.

        Returns:
            bool: True if all services are successfully found, False if any service times out.
        """
        self.claim_client = self.create_client(Trigger, namespace_with(robot_name, "claim"))
        self.release_client = self.create_client(Trigger, namespace_with(robot_name, "release_leases"))
        self.power_on_client = self.create_client(Trigger, namespace_with(robot_name, "power_on"))
        self.power_off_client = self.create_client(Trigger, namespace_with(robot_name, "power_off"))
        self.stand_client = self.create_client(Trigger, namespace_with(robot_name, "stand"))
        self.sit_client = self.create_client(Trigger, namespace_with(robot_name, "sit"))
        self.stop_client = self.create_client(Trigger, namespace_with(robot_name, "stop"))
        self.velocity_client = self.create_client(SetVelocity, namespace_with(robot_name, "max_velocity"))
        self.open_gripper_client = self.create_client(Trigger, namespace_with(robot_name, "open_gripper"))
        self.close_gripper_client = self.create_client(Trigger, namespace_with(robot_name, "close_gripper"))
        self.stow_client = self.create_client(Trigger, namespace_with(robot_name, "arm_stow"))

        self.clear_graph_client = self.create_client(Trigger, namespace_with(robot_name, "graphnav_clear_graph_custom"))
        self.upload_graph_client = self.create_client(GraphNavUploadGraph, namespace_with(robot_name, "graphnav_upload_graph_custom"))
        self.set_localization_client = self.create_client(GraphNavSetLocalization, namespace_with(robot_name, "graph_nav_set_localization"))
        self.get_localization_client = self.create_client(GraphNavGetLocalizationPose, namespace_with(robot_name, "graph_nav_get_localization_pose"))


        for name, client in [
            ("claim", self.claim_client),
            ("release_leases", self.release_client),
            ("power_on", self.power_on_client),
            ("power_off", self.power_off_client),
            ("stand", self.stand_client),
            ("sit", self.sit_client),
            ("stop", self.stop_client),
            ("max_velocity", self.velocity_client),
            ("open_gripper", self.open_gripper_client),
            ("close_gripper", self.close_gripper_client),
            ("arm_stow", self.stow_client),
            ("graphnav_clear_graph_custom", self.clear_graph_client),
            ("graphnav_upload_graph_custom", self.upload_graph_client),
            ("graph_nav_set_localization", self.set_localization_client),
            ("graph_nav_get_localization_pose", self.get_localization_client)
        ]:
            self.get_logger().info(f"Waiting for {name} service...")
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"{name} service not available!")
                return False
        
        return True
    
    def set_velocity(self):
        """
        Sets the maximum linear and angular velocity limits for the robot to ensure safe movement.

        Returns:
            bool: True if the velocity parameters were successfully set, False otherwise.
        """
        req = SetVelocity.Request()

        req.velocity_limit = Twist()
        req.velocity_limit.linear.x = 0.3
        req.velocity_limit.linear.y = 0.3
        req.velocity_limit.angular.z = 0.3

        future = self.velocity_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Service max_velocity was successful")
                return True
            else:
                self.get_logger().error(f"Service max_velocity failed with message: {response.message}")
                return False
        else:
            self.get_logger().error(f"Service max_velocity call timed out")
            return False

    def trigger_service(self, client, srv_name):
        """
        Helper method to synchronously call a standard ROS 2 Trigger service.

        Args:
            client: The ROS 2 service client instance.
            srv_name (str): The name of the service (used primarily for logging).

        Returns:
            bool: True if the service executed successfully, False on failure or timeout.
        """
        request = Trigger.Request()
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=50.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Service {srv_name} was successful")
                return True
            else:
                self.get_logger().error(f"Service {srv_name} failed with message: {response.message}")
                return False
        else:
            self.get_logger().error(f"Service {srv_name} call timed out")
            return False
        
    def claim_robot(self) -> bool:
        """
        Claims the robot lease.
        
        Returns:
            bool: True if successful, False otherwise.
        """
        self.get_logger().info("Claiming robot...")
        
        return self.trigger_service(self.claim_client, "claim_leases")

    def release_robot(self) -> bool:
        """
        Releases the robot lease.

        Returns:
            bool: True if successful, False otherwise.
        """
        self.get_logger().info("Releasing robot...")
        
        return self.trigger_service(self.release_client, "release_leases")

    def power_on_robot(self) -> bool:
        """
        Powers on the robot's motors.

        Returns:
            bool: True if successful, False otherwise.
        """
        self.get_logger().info("Powering on robot...")

        return self.trigger_service(self.power_on_client, "power_on")
    
    def power_off_robot(self) -> bool:
        """
        Powers off the robot's motors.

        Returns:
            bool: True if successful, False otherwise.
        """
        self.get_logger().info("Powering off robot...")

        return self.trigger_service(self.power_off_client, "power_off")
    
    def stand(self) -> bool:
        """
        Commands the robot to stand up.

        Returns:
            bool: True if successful, False otherwise.
        """
        self.get_logger().info("Standing...")
        
        return self.trigger_service(self.stand_client, "stand")

    def sit(self) -> bool:
        """
        Commands the robot to sit down.

        Returns:
            bool: True if successful, False otherwise.
        """
        self.get_logger().info("Sitting...")
        
        return self.trigger_service(self.sit_client, "sit")

    def stop(self) -> bool:
        """
        Stops the robot's current movement or action immediately.

        Returns:
            bool: True if successful, False otherwise.
        """
        self.get_logger().info("Stopping command...")

        return self.trigger_service(self.stop_client, "stop")
    
    def open_gripper(self) -> bool:
        """
        Commands the manipulator's gripper to open fully.

        Returns:
            bool: True if successful, False otherwise.
        """
        self.get_logger().info("Opening gripper...")

        return self.trigger_service(self.open_gripper_client, "open_gripper")
    
    def close_gripper(self) -> bool:
        """
        Commands the manipulator's gripper to close.

        Returns:
            bool: True if successful, False otherwise.
        """
        self.get_logger().info("Closing gripper...")

        return self.trigger_service(self.close_gripper_client, "close_gripper")
    
    def stow_arm(self) -> bool:
        """
        Commands the manipulator arm to return to its stowed position.

        Returns:
            bool: True if successful, False otherwise.
        """
        self.get_logger().info("Stowing arm...")
        
        return self.trigger_service(self.stow_client, "arm_stow")
    
    def clear_graph(self):
        """
        Clears the currently loaded GraphNav map from the robot's memory.

        Returns:
            bool: True if successful, False otherwise.
        """
        self.get_logger().info("Clearing graph...")
        
        return self.trigger_service(self.clear_graph_client, "graphnav_clear_graph_custom")
    
    def upload_graph(self):
        """
        Uploads the map graph and snapshots from the local file path to the robot.

        Returns:
            bool: True if the upload succeeds, False on failure or timeout.
        """
        req = GraphNavUploadGraph.Request()
        req.upload_filepath = self.file_path

        future = self.upload_graph_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec= 30.0)

        if future.result() is None:
            self.get_logger().error("Service graph_nav_upload_graph call timed out")
            return False

        resp = future.result()

        if resp.success:
            self.get_logger().info("Service graph_nav_upload_graph was successful")
            return True

        self.get_logger().error(f"Service graph_nav_upload_graph failed with message: {resp.message}")
        return False

    def set_localization(self):
        """
        Triggers the fiducial-based initial localization for the GraphNav system.

        Returns:
            bool: True if localization is successful, False otherwise.
        """
        req = GraphNavSetLocalization.Request()
        req.method = "fiducial"

        future = self.set_localization_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=40.0)

        if future.result() is None:
            self.get_logger().error("Service graph_nav_set_localization call timed out")
            return False

        resp = future.result()
        if resp.success:
            self.get_logger().info("Service graph_nav_set_localization was successful")
            return True

        self.get_logger().error(f"Service graph_nav_set_localization failed with message: {resp.message}")
        return False

    def get_localization_pose(self):
        """
        Retrieves the robot's current localization pose from the GraphNav system.

        Returns:
            bool: True if the pose is successfully retrieved, False otherwise.
        """
        req = GraphNavGetLocalizationPose.Request()

        future = self.get_localization_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("Service graph_nav_get_localization_pose call timed out")
            return False

        resp = future.result()
        if resp.success:
            self.get_logger().info("Service graph_nav_get_localization_pose was successful")
            return True

        self.get_logger().error(f"Service graph_nav_get_localization_pose failed with message: {resp.message}")
        return False
    
    def initialize_robot(self):
        """
        Executes the full sequential boot-up procedure for the robot.
        
        This sequence powers on the motors, clears and uploads the map graph, 
        stands the robot up, sets localization, and configures max velocity.

        Raises:
            RuntimeError: If any critical step in the initialization sequence fails.
        """
        if not self.power_on_robot():
            raise RuntimeError("Failed to power on the motors")
        
        if not self.clear_graph():
            raise RuntimeError("Failed to clear graph map")
        
        if not self.upload_graph():
            raise RuntimeError("Failed to upload graph map")

        if not self.stand():
            raise RuntimeError("Failed to make robot stand")
        
        time.sleep(3.0)
        if not self.set_localization():
            raise RuntimeError("Failed to set localization")
        
        if not self.set_velocity():
            raise RuntimeError("Failed to set max velocity")
    
    def safe_shutdown(self):
        """
        Executes a fault-tolerant shutdown sequence to ensure the robot is in a 
        safe physical state before terminating the software.
        
        It attempts to stop current movements, manage the gripper, stow the arm, 
        sit the robot down, and power off the motors. Failures during this process 
        are logged rather than raised to ensure the sequence attempts to complete.
        """
        if not self.stop():
            self.get_logger().error("Failed to stop command")

        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper")
        else:
            time.sleep(0.5)
            if not self.close_gripper():
                self.get_logger().error("Failed to close gripper")
        
        if not self.stow_arm():
            self.get_logger().error("Failed to stow arm")

        if not self.sit():
            self.get_logger().error("Failed to sit")
        
        if not self.power_off_robot():
            self.get_logger().error("Failed to power off the motors")

shutdown_requested = False

def signal_handler(sig, frame):
    """
    Handles system interrupt signals to trigger a graceful exit.

    Args:
        sig (int): The signal number.
        frame: The current stack frame.
    """
    global shutdown_requested
    shutdown_requested = True

def main(args=None):
    """
    Main entry point for the InitializationNode. Parses arguments, initializes 
    the robot physically, and maintains the node loop until interrupted.

    Args:
        args (Optional[list]): Command-line arguments. Defaults to None.
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
        
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as ex:
        print(f"Thrown Exception: {ex}")
        node.safe_shutdown()
        node.destroy_node()
        rclpy.shutdown()
    
    node.safe_shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()