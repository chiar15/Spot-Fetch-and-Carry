"""
Copyright (c) 2026 Chiara Ferraioli

This module contains the DetectionNode, which runs an object detection model 
(YOLO) on the Spot robot's synchronized camera and depth feeds. It processes 
the images to find specific targets, filters them based on physical heuristics 
(size, distance), and serves the coordinates of the closest object via a ROS 2 service.
"""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from typing import Optional
import numpy as np


from bt_interfaces.srv import Detection
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from spot_msgs.msg import Feedback
from synchros2.utilities import namespace_with
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ultralytics import YOLO
from ultralytics.utils import ThreadingLocked
import threading



class DetectionNode(Node):
    """
    ROS 2 Node that synchronizes camera and depth feeds, executes YOLO object 
    detection, and provides a service to retrieve the closest detected target.

    It optionally manages a separate thread to display a real-time OpenCV 
    grid visualization of all five robot cameras with drawn bounding boxes.
    """
    def __init__(self, robot_name: Optional[str] = None) -> None:
        """
        Initializes the DetectionNode, setting up the YOLO model, the ROS 2 
        service, subscriptions, and the optional visualization thread.

        Args:
            robot_name (Optional[str]): The namespace of the robot.
        """
        super().__init__("detection_node")

        self.roi_size = 5
        
        self.br = CvBridge()
        
        self.manage_subscriptions(robot_name)

        self.model = YOLO("/home/chiara/Desktop/Tesi SPOT/src/spot_bt/spot_bt/assets/tennis_ball.pt")
        self.detections_lock = threading.Lock()
        
        service_name = namespace_with(robot_name, 'object_detection')
        self.detection_srv = self.create_service(
            Detection, 
            service_name,
            self.detection_callback, 
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.standing = False
        self.standing_lock = threading.Lock()

        self.detections = {}

        self.declare_parameter('show_grid', True)
        self.show_grid = self.get_parameter('show_grid').value
        
        if self.show_grid:
            self.setup_grid_view()
            self.viz_thread = threading.Thread(target=self.visualization_loop, daemon=True)
            self.viz_thread.start()
    
    def setup_grid_view(self):
        """
        Configures the memory buffer and layout mapping for the 2x3 camera grid 
        used in the OpenCV visualization.
        """
        self.cell_size = (640, 480)  
        grid_h = 2 * self.cell_size[1]  
        grid_w = 3 * self.cell_size[0]  
        
        self.grid_image = np.zeros((grid_h, grid_w, 3), dtype=np.uint8)
        self.grid_lock = threading.Lock() 
        
        self.grid_layout = {
            'frontright_fisheye': (0, 0),
            'frontleft_fisheye':  (0, 1), 
            'right_fisheye':      (0, 2),
            'left_fisheye':       (1, 0),
            'back_fisheye':       (1, 1)
        }
        
        self.get_logger().info("Grid visualization enabled")
    
    def visualization_loop(self):
        """
        Executes a continuous OpenCV loop in a separate daemon thread to safely 
        render the shared grid image without blocking the ROS 2 executor.
        """
        cv2.namedWindow('Detection Grid', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Detection Grid', 1920, 960)
        
        while rclpy.ok():
            with self.grid_lock:
                display_img = self.grid_image.copy()
            
            cv2.imshow('Detection Grid', display_img)
            if cv2.waitKey(30) & 0xFF == ord('q'):  
                break
        
        cv2.destroyAllWindows()

    def manage_subscriptions(self, robot_name: Optional[str] = None):
        """
        Creates and registers synchronized image and depth subscribers for all 
        five Spot cameras, alongside a feedback subscriber for the robot's state.

        Args:
            robot_name (Optional[str]): The namespace of the robot.
        """
        cb_groups = {
            'frontright': MutuallyExclusiveCallbackGroup(),
            'frontleft':  MutuallyExclusiveCallbackGroup(),
            'right':      MutuallyExclusiveCallbackGroup(),
            'left':       MutuallyExclusiveCallbackGroup(),
            'back':       MutuallyExclusiveCallbackGroup(),
        }

        for cam in cb_groups.keys():

            img_topic = namespace_with(robot_name, f"camera/{cam}/image")
            depth_topic = namespace_with(robot_name, f"depth_registered/{cam}/image")

            img_sub = Subscriber(
                self, Image, img_topic,
                callback_group=cb_groups[cam]
            )

            depth_sub = Subscriber(
                self, Image, depth_topic,
                callback_group=cb_groups[cam]
            )

            ts = ApproximateTimeSynchronizer(
                [img_sub, depth_sub],
                queue_size=10,
                slop=0.5
            )
            ts.registerCallback(self.process_camera)

            setattr(self, f"{cam}_image_sub", img_sub)
            setattr(self, f"{cam}_depth_sub", depth_sub)
            setattr(self, f"{cam}_ts", ts)
        
        self.feedback_sub = self.create_subscription(Feedback, namespace_with(robot_name, "status/feedback"), callback=self.feedback_callback, callback_group = MutuallyExclusiveCallbackGroup(), qos_profile=10)
        self.get_logger().info("Subscriptions and synchronizers set up.")
    
    def feedback_callback(self, msg: Feedback):
        """
        Updates the internal standing state based on the robot's feedback message.

        Args:
            msg (Feedback): The feedback message received from the robot.
        """
        with self.standing_lock:
            self.standing = msg.standing

    @ThreadingLocked()
    def thread_safe_predict(self, image):
        """
        Executes a thread-safe YOLO model prediction on the provided image.

        Args:
            image (np.ndarray): The OpenCV image matrix to run inference on.

        Returns:
            list: The inference results returned by the YOLO model.
        """
        results = self.model(image, verbose=False, conf=0.6)
        return results
    
    def process_camera(self, image: Image, depth_map):
        """
        Callback triggered by the time synchronizer. Converts the image and depth 
        data, runs YOLO detection, filters results by physical heuristics (distance, 
        area, aspect ratio), and updates the global dictionary of detections.

        Args:
            image (Image): The synchronized RGB image message.
            depth_map (Image): The corresponding registered depth map message.
        """
        with self.standing_lock:
            if not self.standing:
                return

        try:
            frame = self.br.imgmsg_to_cv2(image, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        if depth_map.encoding not in ['16UC1', '32FC1']:
            self.get_logger().error(f"Formato depth non supportato: {depth_map.encoding}")
            return

        dtype = np.uint16 if depth_map.encoding == '16UC1' else np.float32
        depth_data = np.frombuffer(depth_map.data, dtype=dtype).reshape(depth_map.height, depth_map.width)

        if depth_data is None:
            return
        
        img_width = image.width
        img_height = image.height
        frame_id = image.header.frame_id
        timestamp = image.header.stamp
        
        result = self.thread_safe_predict(frame)
        boxes = result[0].boxes  
        filtered_boxes = []      

        closest_obj = None

        if len(boxes) > 0:
            for box in boxes:
                x_center, y_center, width, height = box.xywh[0].cpu().numpy().astype(int)

                aspect_ratio = width / height if height > 0 else 0
                if aspect_ratio > 1.5 or aspect_ratio < 0.75:
                    continue

                area = width * height

                if area > 20000:
                    continue

                y_min = max(0, y_center - self.roi_size)
                y_max = min(y_center + self.roi_size, img_height)
                x_min = max(0, x_center - self.roi_size)
                x_max = min(x_center + self.roi_size, img_width)

                roi = depth_data[y_min:y_max, x_min:x_max]
                valid_depths = roi[roi > 0]

                if valid_depths.size == 0:
                    continue

                depth = np.median(valid_depths) / 1000.0
                if depth > 2.00:
                    continue

                filtered_boxes.append(box)

                self.get_logger().info(f"Object found with distance {depth}")
                obj = {
                    'center': (x_center, y_center),
                    'distance': depth,
                    'timestamp': timestamp
                }

                if closest_obj is None or closest_obj['distance'] > depth:
                    closest_obj = obj
        
        if self.show_grid and frame_id in self.grid_layout:
            self.update_grid_cell(frame_id, frame, filtered_boxes)

        with self.detections_lock:
            if closest_obj is None and frame_id in self.detections:
                self.detections.pop(frame_id)
            elif closest_obj is not None:
                self.detections.update({frame_id: closest_obj})
                self.get_logger().info(f"Object found with camera {frame_id}")

    def update_grid_cell(self, frame_id, frame, boxes):
        """
        Draws bounding boxes onto the target camera frame and safely copies 
        it into the designated cell of the global grid visualization.

        Args:
            frame_id (str): The coordinate frame ID identifying the camera.
            frame (np.ndarray): The OpenCV image matrix for this camera.
            boxes (list): The list of filtered YOLO bounding boxes to draw.
        """
        if frame_id not in self.grid_layout:
            return
        
        row, col = self.grid_layout[frame_id]
        
        cell_img = frame.copy()  
        
        if len(boxes) > 0:
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cv2.rectangle(cell_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                conf = box.conf[0].cpu().numpy()
                cv2.putText(cell_img, f'{conf:.1f}', (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        cv2.putText(cell_img, frame_id, (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        with self.grid_lock:
            y_start = row * self.cell_size[1]
            x_start = col * self.cell_size[0]
            self.grid_image[y_start:y_start+self.cell_size[1], 
                        x_start:x_start+self.cell_size[0]] = cell_img
    
    def detection_callback(self, request, response):
        """
        Handles requests to the object detection service. Iterates over current 
        detections to identify and return the closest valid target.

        Args:
            request (Detection.Request): The incoming service request.
            response (Detection.Response): The pre-allocated service response object.

        Returns:
            Detection.Response: The populated response containing the success status, 
                frame ID, coordinates, and timestamp of the closest target.
        """
        frame_id = None
        
        with self.detections_lock:

            detections = self.detections.copy()

        if len(detections) == 0:

            response.success = False
            return response
        
        else:
            
            for cam in detections:
                if frame_id is None:
                    frame_id = cam
                else:
                    if detections[cam]['distance'] < detections[frame_id]['distance']:
                        frame_id = cam
        
        if frame_id is None:
            response.success = False
        else:
            det = detections[frame_id]
            response.success = True
            response.frame_id = frame_id
            response.x_center = float(det['center'][0])
            response.y_center = float(det['center'][1])
            response.timestamp = det['timestamp']
        
        return response
        
def main(args=None):
    """
    Main entry point for the DetectionNode. Parses arguments, initializes ROS 2, 
    starts the multi-threaded executor, and ensures safe shutdown on interruption.

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
    node = DetectionNode(robot_name=args.robot)
    executor = MultiThreadedExecutor()
    executor.add_node(node)


    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
