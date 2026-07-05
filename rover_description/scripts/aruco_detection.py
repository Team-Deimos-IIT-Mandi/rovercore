#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs

class ArUcoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # 1. Configuration Parameters
        self.detection_window = [] # Store last few detections
        self.global_goal_sent = False
        self.goal_sent = False
        
        qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST
        )
        self.state_sub = self.create_subscription(String, 'rover/mission_state', self.global_state_callback, qos_profile)
        self.fsm_client = self.create_client(Trigger, 'mission/complete_aruco_search')
        self.global_mission_state = 'BOOTING'

        # Camera topic and frame parameters — override at launch to support multiple cameras
        self.declare_parameter('camera_topic', '/cam/front/image_raw')
        self.declare_parameter('camera_frame', 'camera_link')

        # Focal length parameters.
        # Formula: f = (image_width / 2) / tan(camera_hfov / 2)
        # Default: 640x480 resolution with 60-degree (1.0472 rad) horizontal FOV
        #   f = (640 / 2) / tan(1.0472 / 2) = 320 / tan(30°) = 320 / 0.5774 ≈ 554.26
        self.declare_parameter('image_width', 640)
        self.declare_parameter('camera_hfov', 1.0472)  # 60 degrees in radians
        self.declare_parameter('required_consecutive_frames', 3)
        self.declare_parameter('dist_threshold', 6.5)

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        camera_hfov = self.get_parameter('camera_hfov').get_parameter_value().double_value
        if camera_hfov <= 0.0:
            self.get_logger().error('camera_hfov must be positive')
            camera_hfov = 1.0472  # fallback to 60 degrees
        self.required_consecutive_frames = self.get_parameter('required_consecutive_frames').get_parameter_value().integer_value
        self.dist_threshold = self.get_parameter('dist_threshold').get_parameter_value().double_value

        # Compute focal length from image dimensions and field of view
        f = (image_width / 2.0) / math.tan(camera_hfov / 2.0)
        cx = image_width / 2.0
        cy = cx * (3.0 / 4.0)  # assume 4:3 aspect ratio for default 640x480

        self.marker_size = 0.2  # 20cm as defined in your Xacro
        # self.matrix_coefficients = np.array([[1662.76, 0, 960.5],
        #                                     [0, 1662.76, 540.5],
        #                                     [0, 0, 1]], dtype=np.float32)
        self.matrix_coefficients = np.array([[f, 0, cx],
                                            [0, f, cy],
                                            [0, 0, 1]], dtype=np.float32)
        self.distortion_coefficients = np.array([1e-08, 1e-08, 1e-08, 1e-08, 1e-08], dtype=np.float32)

        # 2. Publishers and Subscribers
        self.image_sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)
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
        self.is_finished_reported = False

        # 4. ArUco Detector Initialization (Modern API)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
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

        self.get_logger().info(
            f"ArUco Detection Node Started | topic={camera_topic} | "
            f"frame={self.camera_frame} | f={f:.4f} (width={image_width}, hfov={camera_hfov:.4f} rad)"
        )

    def global_state_callback(self, msg):
        self.global_mission_state = msg.data

    def sync_callback(self, msg):
        """Updates the local lock based on what other cameras have found."""
        self.global_goal_sent = msg.data
        if self.global_goal_sent:
            self.get_logger().info("Global Goal Lock Received. Silencing this node.")

    def image_callback(self, msg):
        if self.global_mission_state != 'ARUCO_SEARCH':
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
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

                # Visualization for Rviz/Debug
                cv2.drawFrameAxes(cv_image, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.1)

                if self.global_mission_state == 'ARUCO_SEARCH' and not self.global_goal_sent:
                    if dist < self.dist_threshold and dist>0.5:
                        self.get_logger().info(f"Marker {ids[i]} found within range ({dist:.2f}m). Locking goal!")

                        # Only stop spiral and lock once we are close enough to be accurate
                        self.global_goal_sent = True
                        self.sync_pub.publish(Bool(data=True))
                        self.ar_signal_pub.publish(Bool(data=True))
                        
                        if not self.is_finished_reported:
                            self.notify_mission_control_complete()

                        # Mode P Logic: Publish goal if AR is NOT active (search complete)
                        self.process_and_publish_goal(tvec.flatten(), rvec.flatten())
                    else:
                        self.get_logger().warn(f"Marker detected but too far ({dist:.2f}m). Continuing search...")

        self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        cv2.imshow("Aruco Detection Debug", cv_image)
        cv2.waitKey(1)

    def process_and_publish_goal(self, tvec, rvec):
        camera_pose = PoseStamped()
        camera_pose.header.stamp = self.get_clock().now().to_msg()
        # Ensure frame_id matches your URDF camera frame
        camera_pose.header.frame_id = self.camera_frame

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
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = "odom"
            goal_msg.pose = world_pose

            self.goal_pub.publish(goal_msg)
            self.goal_sent = True
            self.get_logger().info(f"Published Goal at odom: x={goal_msg.pose.position.x:.2f}, y={goal_msg.pose.position.y:.2f}")

        except Exception as e:
            self.get_logger().warn(f"TF Lookup Failed: {e}")

    def notify_mission_control_complete(self):
        self.is_finished_reported = True

        if not self.fsm_client.service_is_ready():
            self.get_logger().info("Waiting for Mission Manager aruco-search service...")
            self.is_finished_reported = False
            return

        req = Trigger.Request()
        future = self.fsm_client.call_async(req)
        future.add_done_callback(self.fsm_response_callback)

    def fsm_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info("SUCCESS: Central FSM acknowledged ArUco search complete.")
            else:
                self.get_logger().error(f"FSM rejected ArUco search completion: {res.message}")
                self.is_finished_reported = False
        except Exception as e:
            self.get_logger().error(f"Service communication failed: {e}")
            self.is_finished_reported = False

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
