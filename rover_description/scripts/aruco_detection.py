#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs

class ArUcoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # 1. Configuration Parameters
        self.detection_window = [] # Store last few detections
        self.required_consecutive_frames = 3
        self.global_goal_sent = False
        self.dist_threshold = 6.5
        self.mission_started = False
        self.start_sub = self.create_subscription(Bool, '/start_search', self.start_cb, 10)
        f = 554.26
        self.marker_size = 0.2  # 20cm as defined in your Xacro
        # self.matrix_coefficients = np.array([[1662.76, 0, 960.5],
        #                                     [0, 1662.76, 540.5],
        #                                     [0, 0, 1]], dtype=np.float32)
        self.matrix_coefficients = np.array([[f, 0, 320],
                                            [0, f, 240],
                                            [0, 0, 1]], dtype=np.float32)
        self.distortion_coefficients = np.array([1e-08, 1e-08, 1e-08, 1e-08, 1e-08], dtype=np.float32)

        # 2. Publishers and Subscribers
        # Updated to camera_2 as per your rover's latest configuration
        self.image_sub = self.create_subscription(Image, '/rgbd_camera/image', self.image_callback, 10)
        self.ar_signal_pub = self.create_publisher(Bool, '/AR', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.debug_img_pub = self.create_publisher(Image, '/aruco_debug_image', 10)
        self.sync_sub = self.create_subscription(Bool, '/marker_goal_reached', self.sync_callback, 10)
        self.sync_pub = self.create_publisher(Bool, '/marker_goal_reached', 10)
    
        # 3. TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.bridge = CvBridge()
        # self.ar_active = True
        self.marker_found = False  # Defaults to TRUE so it WAITS for a signal to start publishing

        # 4. ArUco Detector Initialization (Modern API)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.minMarkerPerimeterRate = 0.12
        self.aruco_params.polygonalApproxAccuracyRate = 0.015
        self.aruco_params.errorCorrectionRate = 0.0
        self.aruco_params.adaptiveThreshConstant = 15
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Object points for solvePnP: corners of a square marker in its own frame
        self.obj_points = np.array([
            [-self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2, -self.marker_size/2, 0],
            [-self.marker_size/2, -self.marker_size/2, 0]
        ], dtype=np.float32)

        self.get_logger().info("ArUco Detection Node (Camera 2) Started")

    def start_cb(self, msg):
        if msg.data:
            self.mission_started = True
            self.get_logger().info("Search signal received. Starting operation!")
    
    def sync_callback(self, msg):
        """Updates the local lock based on what other cameras have found."""
        self.global_goal_sent = msg.data
        if self.global_goal_sent:
            self.get_logger().info("Global Goal Lock Received. Silencing this node.")

    def image_callback(self, msg):

        if not self.mission_started:
            return

        if self.global_goal_sent:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            self.detection_window.append(True)
            if len(self.detection_window) > self.required_consecutive_frames:
                self.detection_window.pop(0)
        else:
            self.detection_window = []

        if len(self.detection_window) >= self.required_consecutive_frames:
            for i in range(len(ids)):
                # solvePnP replaces the deprecated estimatePoseSingleMarkers
                _, rvec, tvec = cv2.solvePnP(
                    self.obj_points, corners[i], 
                    self.matrix_coefficients, self.distortion_coefficients, 
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                dist = np.linalg.norm(tvec)

                if dist < self.dist_threshold and dist>0.5:
                    self.get_logger().info(f"Marker {ids[i]} found within range ({dist:.2f}m). Locking goal!")
                    
                    # Only stop spiral and lock once we are close enough to be accurate
                    self.global_goal_sent = True 
                    self.sync_pub.publish(Bool(data=True)) 
                    self.ar_signal_pub.publish(Bool(data=True))

                # Visualization for Rviz/Debug
                    cv2.aruco.drawDetectedMarkers(cv_image, corners)
                    cv2.drawFrameAxes(cv_image, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.1)

                    # Mode P Logic: Publish goal if AR is NOT active (search complete)
                    self.process_and_publish_goal(tvec.flatten(), rvec.flatten())
                else:
                    self.get_logger().warn(f"Marker detected but too far ({dist:.2f}m). Continuing search...")

        self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    def process_and_publish_goal(self, tvec, rvec):
        camera_pose = PoseStamped()
        camera_pose.header.stamp = self.get_clock().now().to_msg()
        # Ensure frame_id matches your URDF camera frame
        camera_pose.header.frame_id = "camera_link"
        
        # OpenCV Z (forward) -> ROS X (forward)
        # OpenCV X (right)   -> ROS -Y (right)
        camera_pose.pose.position.x = float(tvec[2]) 
        camera_pose.pose.position.y = float(-tvec[0])
        camera_pose.pose.position.z = 0.0
        camera_pose.pose.orientation.w = 1.0 

        try:
            # Look up transform from odom to the specific camera frame
            transform = self.tf_buffer.lookup_transform(
                "odom", 
                camera_pose.header.frame_id, 
                rclpy.time.Time(),
            )
            
            # Use do_transform_pose for accurate world coordinates
            world_pose = tf2_geometry_msgs.do_transform_pose(camera_pose.pose, transform)
            
            goal_msg = PoseStamped()
            goal_msg.header.stamp = rclpy.time.Time().to_msg()
            goal_msg.header.frame_id = "odom"
            goal_msg.pose = world_pose

            self.goal_pub.publish(goal_msg)
            self.goal_sent = True
            self.get_logger().info(f"Published Goal at odom: x={goal_msg.pose.position.x:.2f}, y={goal_msg.pose.position.y:.2f}")
            
        except Exception as e:
            self.get_logger().warn(f"TF Lookup Failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()