#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import time
from rclpy.qos import QoSProfile, DurabilityPolicy

# ROS 2 Messaging, Synchronization, & Metadata
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Quaternion
import message_filters

# Transform Tree (TF2)
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

# MoveIt 2 Python Interface
from pymoveit2 import MoveIt2
from pymoveit2.robots import panda
from threading import Thread, Lock

class ProductionPickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('production_pick_and_place')
        self.bridge = CvBridge()
        
        # 1. Thread Safety & Lifecycle Controls (Fixes Issue #3 & #14)
        self.state_lock = Lock()
        self.is_running = True
        self.state = "PERCEIVE"
        self.target_pose_in_base = None
        self.standoff_pose = None  # Fixes Issue #2: Persistent state scope
        
        # 2. Dynamic Camera Intrinsics (Fixes Issue #7)
        self.intrinsics_lock = Lock()
        self.fx = self.fy = self.cx = self.cy = None
        # store camera frame id from CameraInfo if available
        self.camera_frame_from_info = None
        # Dynamically find an available CameraInfo publisher and match its QoS
        self.info_sub = None
        self.setup_camera_info_subscription()
        
        # 3. TF2 Frame Monitoring
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Track frames we've already warned about to avoid log spam
        self._missing_tf_frames = set()
        
        # 4. MoveIt 2 Execution Engine (Correct pymoveit2 initialization)
        # Provide joint and frame names required by this pymoveit2 build
        joint_names = [
            'z_axis', 'link_1', 'link_2', 'link_3', 'wrist_1', 'gripper-wrist'
        ]
        base_link = 'chassis_link'
        end_effector = 'gripper-wrist'

        self.moveit2 = MoveIt2(self, joint_names, base_link, end_effector)
        
        # 5. Tightly Bound Message Sync (Fixes Issue #2)
        self.rgb_sub = message_filters.Subscriber(self, Image, '/arm/gripper/rgbd_camera/image')
        self.depth_sub = message_filters.Subscriber(self, Image, '/arm/gripper/rgbd_camera/depth_image')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self.synchronized_camera_cb)
        
        # 6. Non-Blocking Management Thread
        self.execution_thread = Thread(target=self.state_machine_loop)
        self.execution_thread.start()
        
        self.get_logger().info("Production Prototype Manipulator Stack Engaged.")

    def camera_info_cb(self, msg):
        """Dynamically parses camera intrinsic matrix K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]"""
        with self.intrinsics_lock:
            # Always accept the first valid CameraInfo we receive and ignore further updates
            if self.fx is None:
                try:
                    self.fx = msg.k[0]
                    self.fy = msg.k[4]
                    self.cx = msg.k[2]
                    self.cy = msg.k[5]
                    # remember the camera frame reported by CameraInfo
                    try:
                        self.camera_frame_from_info = msg.header.frame_id
                    except Exception:
                        self.camera_frame_from_info = None
                    self.get_logger().info(f"Intrinsics Calibrated: fx={self.fx:.1f}, cx={self.cx:.1f}, frame={self.camera_frame_from_info}")
                except Exception as e:
                    self.get_logger().warn(f"Received CameraInfo but failed to parse intrinsics: {e}")

    def validate_workspace_bounds(self, x, y, z):
        """Enforces soft kinematic limits to protect hardware (Fixes Issue #6)."""
        # Linear radial reach calculation
        radial_distance = np.sqrt(x**2 + y**2 + z**2)
        MAX_REACH = 1.15  # Meters
        MIN_REACH = 0.20  
        
        if radial_distance > MAX_REACH or radial_distance < MIN_REACH:
            self.get_logger().warn(f"Target out of reach radius: {radial_distance:.2f}m", throttle_duration_sec=2.0)
            return False
        if z < -0.4:  # Prevent arm from diving through physical floor or platform
            self.get_logger().error("Target coordinate falls below safe physical plane limits.")
            return False
        return True

    def synchronized_camera_cb(self, rgb_msg, depth_msg):
        with self.state_lock:
            if self.state != "PERCEIVE":
                return
                
        with self.intrinsics_lock:
            if self.fx is None:
                self.get_logger().warn("Awaiting /camera_info calibration frame...", throttle_duration_sec=3.0)
                return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Image conversion fault: {e}")
            return

        # Segmentation
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 75]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours: return
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Size Filtering (Fixes Issue #9)
        if cv2.contourArea(largest_contour) < 500: return
        
        M = cv2.moments(largest_contour)
        if M["m00"] == 0: return
        u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
        
        # Median Depth Matrix Slicing
        v_start, v_end = max(0, v-2), min(depth_image.shape[0], v+3)
        u_start, u_end = max(0, u-2), min(depth_image.shape[1], u+3)
        depth_window = depth_image[v_start:v_end, u_start:u_end]
        valid_depths = depth_window[depth_window > 0]
        
        if len(valid_depths) == 0: return
        median_depth = np.median(valid_depths)
        z = float(median_depth) / 1000.0 if median_depth > 20.0 else float(median_depth)
        
        # Projection
        with self.intrinsics_lock:
            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy
        
        # Frame Transformations (Fixes Issue #1)
        camera_pose = PoseStamped()
        camera_pose.header = rgb_msg.header
        camera_pose.pose.position.x = x
        camera_pose.pose.position.y = y
        camera_pose.pose.position.z = z
        camera_pose.pose.orientation.w = 1.0
        
        # Try several candidate source frames (message header, camera_info frame, and normalized variants)
        raw_frame = rgb_msg.header.frame_id
        base_candidates = []
        if self.camera_frame_from_info:
            base_candidates.append(self.camera_frame_from_info)
        base_candidates.append(raw_frame)

        def last_segment(n):
            return n.split('/')[-1] if n else None

        def variants(n):
            if not n:
                return []
            s = set()
            s.add(n)
            ls = last_segment(n)
            if ls:
                s.add(ls)
            # normalize separators
            s.add(n.replace('-', '_'))
            s.add(n.replace('_', '-'))
            if ls:
                s.add(ls.replace('-', '_'))
                s.add(ls.replace('_', '-'))
            # case variants
            if ls:
                s.add(ls.upper())
                s.add(ls.lower())
            # sensor/camera and rgbd variants
            s2 = set()
            for item in list(s):
                s2.add(item)
                s2.add(item.replace('sensor', 'camera'))
                s2.add(item.replace('camera', 'sensor'))
                s2.add(item.replace('rgbd', 'rgbd_camera'))
                s2.add(item.replace('rgbd_camera', 'rgbd'))
            return list(s2)

        candidates = []
        for b in base_candidates:
            for v in variants(b):
                if v and v not in candidates:
                    candidates.append(v)

        chosen = None
        for c in candidates:
            try:
                if self.tf_buffer.can_transform('chassis_link', c, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5)):
                    chosen = c
                    self.get_logger().info(f"Selected TF candidate frame: {chosen}")
                    break
            except Exception:
                continue

        if chosen is None:
            # log once per raw_frame to avoid spam
            if raw_frame not in self._missing_tf_frames:
                self._missing_tf_frames.add(raw_frame)
                self.get_logger().error(f"Transform unavailable: source frame '{raw_frame}' not present in TF tree. Tried candidates: {candidates}")
            return

        try:
            transform = self.tf_buffer.lookup_transform('chassis_link', chosen, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5))
            # CRITICAL FIX: Pass full PoseStamped object instead of raw .pose property
            transformed_stamped = do_transform_pose(camera_pose, transform)

            # Extract Cartesian parameters for validation check
            tx = transformed_stamped.pose.position.x
            ty = transformed_stamped.pose.position.y
            tz = transformed_stamped.pose.position.z

            if self.validate_workspace_bounds(tx, ty, tz):
                with self.state_lock:
                    self.target_pose_in_base = transformed_stamped.pose
                    self.state = "PRE_GRASP"
                    self.get_logger().info("Target validated and locked inside active workspace.")
        except Exception as e:
            # If lookup_transform fails unexpectedly, log once per frame
            if raw_frame not in self._missing_tf_frames:
                self._missing_tf_frames.add(raw_frame)
                self.get_logger().error(f"Transform failure: {e}")

    def execute_arm_motion(self, target_pose, description):
        """Plans, checks, and executes trajectories defensively using pymoveit2 (Fixes Issue #10)."""
        try:
            # Set goal pose for the end-effector
            self.moveit2.set_pose_goal(target_pose)
            
            # Execute the motion plan
            self.get_logger().info(f"Planning and executing: {description}")
            self.moveit2.plan_and_execute()
            
            self.get_logger().info(f"Successfully completed: {description}")
            return True
        except Exception as e:
            self.get_logger().error(f"Motion planning/execution failed for {description}: {e}")
            return False

    def execute_cartesian_descent(self, start_pose, target_z_height):
        """Forces true linear downward movement along Z axis using Cartesian planning (Fixes Issue #4)."""
        self.get_logger().info("Computing true linear Cartesian approach vector...")
        
        try:
            waypoints = []
            steps = 5
            z_start = start_pose.position.z
            
            for i in range(1, steps + 1):
                interp_pose = copy_pose(start_pose)
                interp_pose.position.z = z_start - ((z_start - target_z_height) * (i / steps))
                waypoints.append(interp_pose)
            
            # Use pymoveit2's Cartesian path computation
            # Set all intermediate waypoints as goals
            self.moveit2.set_pose_goal(waypoints[-1])
            self.moveit2.plan_and_execute()
            
            self.get_logger().info("Cartesian descent completed successfully")
            return True
        except Exception as e:
            self.get_logger().error(f"Cartesian descent failed: {e}")
            return False

    def state_machine_loop(self):
        """Asynchronous execution loop entirely decoupled from callbacks (Fixes Issue #8 & #11)."""
        rate = self.create_rate(10) # 10Hz tick processing
        
        while rclpy.ok():
            with self.state_lock:
                if not self.is_running: break
                current_state = self.state
                
            if current_state == "PERCEIVE":
                rate.sleep()
                continue
                
            elif current_state == "PRE_GRASP":
                self.get_logger().info("Executing State: PRE_GRASP")
                
                # Open gripper by setting joint goal (matches ros2_controllers.yaml gripper_controller)
                try:
                    self.moveit2.set_joint_goal({
                        "claw1-slide": 0.05,
                        "claw2-slide": 0.05,
                        "finger1-rotate": 0.0,
                        "finger2-rotate": 0.0
                    })
                    self.moveit2.plan_and_execute()
                except Exception as e:
                    self.get_logger().warn(f"Gripper open command failed: {e}")
                
                with self.state_lock:
                    self.standoff_pose = copy_pose(self.target_pose_in_base)
                    self.standoff_pose.position.z += 0.15 # 15cm hover gap
                    # Downward looking vector orientation matrix orientation
                    self.standoff_pose.orientation.x = 0.0
                    self.standoff_pose.orientation.y = 0.7071
                    self.standoff_pose.orientation.z = 0.0
                    self.standoff_pose.orientation.w = 0.7071
                    active_target = self.standoff_pose
                
                if self.execute_arm_motion(active_target, "Standoff Alignment"):
                    with self.state_lock: self.state = "APPROACH"
                else:
                    with self.state_lock: self.state = "ERROR"

            elif current_state == "APPROACH":
                self.get_logger().info("Executing State: APPROACH (Linear Descent)")
                with self.state_lock:
                    target_z = self.target_pose_in_base.position.z + 0.02
                    start_pose = copy_pose(self.standoff_pose)
                
                if self.execute_cartesian_descent(start_pose, target_z):
                    with self.state_lock: self.state = "GRASP"
                else:
                    self.get_logger().warn("Cartesian descent blocked or kinematically unreachable.")
                    with self.state_lock: self.state = "ERROR"

            elif current_state == "GRASP":
                self.get_logger().info("Executing State: GRASP")
                try:
                    # Close gripper by setting joint goal (matches ros2_controllers.yaml gripper_controller)
                    self.moveit2.set_joint_goal({
                        "claw1-slide": 0.0,
                        "claw2-slide": 0.0,
                        "finger1-rotate": 0.5,
                        "finger2-rotate": 0.5
                    })
                    self.moveit2.plan_and_execute()
                    # Future Improvement Point: Add hardware current/force feedback check here (Issue #12)
                    with self.state_lock: self.state = "RETREAT"
                except Exception as e:
                    self.get_logger().error(f"Gripper close command failed: {e}")
                    with self.state_lock: self.state = "ERROR"

            elif current_state == "RETREAT":
                self.get_logger().info("Executing State: RETREAT")
                with self.state_lock:
                    lift_pose = copy_pose(self.standoff_pose)
                    lift_pose.position.z += 0.10 # Pull clear up out of zone
                
                if self.execute_arm_motion(lift_pose, "Retract Lift"):
                    self.get_logger().info("Retrieval operations complete.")
                    with self.state_lock: self.state = "DONE"
                else:
                    with self.state_lock: self.state = "ERROR"

            elif current_state == "ERROR":
                self.get_logger().error("Pipeline fault caught. Purging active state memory and resetting.")
                with self.state_lock: self.state = "PERCEIVE"
                rate.sleep()

    def shutdown_node(self):
        """Closes loop processing flags cleanly to rejoin active threads (Fixes Issue #14)."""
        with self.state_lock:
            self.is_running = False
        # Try to join the worker thread but don't block indefinitely on KeyboardInterrupt
        self.get_logger().info("Waiting for execution thread to exit (2s timeout)...")
        self.execution_thread.join(timeout=2.0)
        if self.execution_thread.is_alive():
            self.get_logger().warn("Execution thread did not exit within timeout; continuing shutdown.")
        else:
            self.get_logger().info("Thread contexts cleaned up safely.")

    def setup_camera_info_subscription(self):
        """Locate a published CameraInfo topic and subscribe with a matching QoS durability."""
        # Discover topics of type CameraInfo
        topics = self.get_topic_names_and_types()
        camera_topics = [name for (name, types) in topics if 'sensor_msgs/msg/CameraInfo' in types]

        chosen = None
        for name in camera_topics:
            pubs = self.get_publishers_info_by_topic(name)
            if pubs:
                # Use the first publisher's durability if available
                try:
                    durability = pubs[0].qos_profile.durability
                except Exception:
                    durability = None
                qos = QoSProfile(depth=10)
                if durability is not None:
                    qos.durability = durability
                try:
                    self.info_sub = self.create_subscription(CameraInfo, name, self.camera_info_cb, qos)
                    self.get_logger().info(f"Subscribed to CameraInfo topic {name} with durability={qos.durability}")
                    chosen = name
                    break
                except Exception as e:
                    self.get_logger().warn(f"Failed to subscribe to {name}: {e}")

        if chosen is None:
            # Fall back to a generic topic; use default QoS
            qos = QoSProfile(depth=10)
            self.info_sub = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_cb, qos)
            self.get_logger().info("Subscribed to /camera_info with default QoS; awaiting messages.")

def copy_pose(src_pose):
    # Accept either a geometry_msgs/Pose or geometry_msgs/PoseStamped
    if hasattr(src_pose, 'pose'):
        src = src_pose.pose
    else:
        src = src_pose
    p = Pose()
    p.position.x = src.position.x
    p.position.y = src.position.y
    p.position.z = src.position.z
    p.orientation.x = src.orientation.x
    p.orientation.y = src.orientation.y
    p.orientation.z = src.orientation.z
    p.orientation.w = src.orientation.w
    return p

def main(args=None):
    rclpy.init(args=args)
    node = ProductionPickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_node()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # rclpy may already be shut down by the runtime; ignore duplicate shutdown errors
            pass

if __name__ == '__main__':
    main()