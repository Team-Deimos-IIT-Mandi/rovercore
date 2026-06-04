#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import threading
import sys, select, termios, tty

# ── YOUR SYSTEM CONSTANTS ──
MAX_LINEAR_SPEED = 0.5      
MAX_ANGULAR_SPEED = 0.8     
ARM_GROUP_NAME = "arm"      
END_EFFECTOR_LINK = "gripper-base"  
CHASSIS_HEIGHT = 0.43 

ARM_JOINT_NAMES = ['z_axis', 'link_1', 'link_2', 'link_3', 'wrist_1','gripper-wrist']
GRIPPER_TOPIC = '/gripper_controller/joint_trajectory'
GRIPPER_JOINT_NAMES = ['claw1-slide', 'claw2-slide']

# Teleop settings
TELEOP_STEP = 0.05       # 5cm per keystroke
ROLL_STEP = 0.2          # Radians per keystroke

class WholeBodyController(Node):
    def __init__(self):
        super().__init__('whole_body_controller')
        self.ik_busy = False

        # Create callback group to prevent deadlocks
        self.cb_group = ReentrantCallbackGroup()
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        
        # Listeners
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.latest_joint_state = JointState()
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        
        # MoveIt Client
        # MoveIt Client (Updated)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.cb_group)
        
        # ... (other code) ...

        # Control Loops (Updated)
        self.timer = self.create_timer(0.05, self.control_loop, callback_group=self.cb_group)
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt IK service...')
            
        # State Variables
        self.state = "IDLE"
        self.target_global_x = 0.0
        self.target_global_y = 0.0
        self.target_global_z = 0.0
        
        # Nav targets
        self.park_target_x = 0.0
        self.park_target_y = 0.0
        self.park_target_yaw = 0.0
        
        # Predictive Geometry state (For drift detection)
        self.pred_local_x = 0.0
        self.pred_local_y = 0.0
        
        self.last_valid_ik_solution = None  
        self.target_ee_quat = None 
        
        self.gripper_open = True
        self.teleop_active = False
        
        # Control Loops
        self.timer = self.create_timer(0.05, self.control_loop)
        self.input_thread = threading.Thread(target=self.terminal_listener)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.get_logger().info("✅ Dual-Mode Controller Active!")
        self.get_logger().info("Type 'cm dx dy dz' for auto-navigation, or 'teleop' for WASD mode.")

    def joint_state_cb(self, msg):
        self.latest_joint_state = msg

    # ==========================================
    # 🎮 TELEOP JOGGING (NO ROVER MOVEMENT)
    # ==========================================

    def toggle_gripper(self):
        """Snaps the prismatic claws open or closed."""
        self.gripper_open = not self.gripper_open
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = GRIPPER_JOINT_NAMES
        point = JointTrajectoryPoint()
        
        # Your URDF prismatic limits are -0.053 to 0.053.
        target_pos = 0.05 if self.gripper_open else 0.0
        point.positions = [target_pos, target_pos]
        point.time_from_start.sec = 1
        traj_msg.points.append(point)
        
        self.gripper_pub.publish(traj_msg)
        state_str = "OPENED" if self.gripper_open else "CLOSED"
        print(f"\r\n[Gripper] -> {state_str}")

    def process_teleop_command(self, dx, dy, dz):
        """Processes a single keystroke. Strict rule: Wheels stay locked."""
        try:
            t = self.tf_buffer.lookup_transform('base_link', END_EFFECTOR_LINK, rclpy.time.Time())
            current_ee_x = t.transform.translation.x
            current_ee_y = t.transform.translation.y
            current_ee_z = t.transform.translation.z
            self.target_ee_quat = t.transform.rotation  
        except Exception:
            return
            
        local_x = current_ee_x + dx
        local_y = current_ee_y + dy
        local_z = current_ee_z + dz
        
        if self.check_ik_feasibility(local_x, local_y, local_z, suppress_warnings=True):
            print(f"\r\n[Teleop] Jogging to X:{local_x:.2f} Y:{local_y:.2f} Z:{local_z:.2f}")
            self.execute_moveit_reach()
        else:
            print("\r\n[Teleop] ⚠️ Boundary reached or collision detected. Arm locked.")

    def rotate_wrist(self, direction):
        """Finds the wrist_1 joint and nudges its angle directly."""
        if 'wrist_1' not in self.latest_joint_state.name:
            return
            
        idx = self.latest_joint_state.name.index('wrist_1')
        current_angle = self.latest_joint_state.position[idx]
        new_angle = current_angle + (ROLL_STEP * direction)
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['wrist_1']
        point = JointTrajectoryPoint()
        point.positions = [new_angle]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000 
        traj_msg.points.append(point)
        
        self.arm_traj_pub.publish(traj_msg)
        print(f"\r\n[Teleop] Wrist rotated to {new_angle:.2f} rad")

    def run_teleop_loop(self):
        """Takes over the terminal for raw WASD input."""
        self.teleop_active = True
        print("\n" + "="*40)
        print("🎮 TELEOP MODE ENGAGED")
        print("W/S : Move Forward/Back (X)")
        print("A/D : Move Left/Right (Y)")
        print("R/F : Move Up/Down (Z)")
        print("Q/E : Rotate Wrist Roll")
        print("Z   : Open/Close Gripper")
        print("ESC : Exit Teleop Mode")
        print("="*40 + "\n")
        
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while self.teleop_active and rclpy.ok():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1).lower()
                    
                    if key == '\x1b': 
                        self.teleop_active = False
                        break
                    
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    
                    if key == 'w': self.process_teleop_command(TELEOP_STEP, 0, 0)
                    elif key == 's': self.process_teleop_command(-TELEOP_STEP, 0, 0)
                    elif key == 'a': self.process_teleop_command(0, TELEOP_STEP, 0)
                    elif key == 'd': self.process_teleop_command(0, -TELEOP_STEP, 0)
                    elif key == 'r': self.process_teleop_command(0, 0, TELEOP_STEP)
                    elif key == 'f': self.process_teleop_command(0, 0, -TELEOP_STEP)
                    elif key == 'q': self.rotate_wrist(1)
                    elif key == 'e': self.rotate_wrist(-1)
                    elif key == 'z': self.toggle_gripper()
                    
                    tty.setraw(sys.stdin.fileno())
                    
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\n[System] Exited Teleop Mode. Awaiting 'cm' commands.\n")

    # ==========================================
    # 🧠 COMMAND MODE (FSM)
    # ==========================================
    
    def process_relative_command(self, dx, dy, dz):
        try:
            t = self.tf_buffer.lookup_transform('base_link', END_EFFECTOR_LINK, rclpy.time.Time())
            current_ee_x = t.transform.translation.x
            current_ee_y = t.transform.translation.y
            current_ee_z = t.transform.translation.z
            self.target_ee_quat = t.transform.rotation  
        except Exception:
            self.get_logger().error(f"TF Error: Could not locate the gripper.")
            return
            
        local_x = current_ee_x + dx
        local_y = current_ee_y + dy
        local_z = current_ee_z + dz
        
        self.get_logger().info(f"Final Destination Math -> X:{local_x:.2f}, Y:{local_y:.2f}, Z:{local_z:.2f}")
        self.process_local_target(local_x, local_y, local_z)

    def process_local_target(self, local_x, local_y, local_z):
        current_x, current_y, current_yaw = self.get_current_pose()
        if current_x is None: return

        if self.check_ik_feasibility(local_x, local_y, local_z):
            self.get_logger().info("✅ Target reachable from here! Deploying arm.")
            self.state = "REACHING"
            self.execute_moveit_reach()
            return

        target_global_x = current_x + (local_x * math.cos(current_yaw)) - (local_y * math.sin(current_yaw))
        target_global_y = current_y + (local_x * math.sin(current_yaw)) + (local_y * math.cos(current_yaw))
        
        self.get_logger().info("Target unreachable from current pose. Initiating candidate search for park pose...")
        
        search_radii = [0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80]
        search_angles = [math.radians(d) for d in range(0, 360, 15)]
        
        candidates = []
        for r in search_radii:
            for theta in search_angles:
                cand_x = target_global_x - r * math.cos(theta)
                cand_y = target_global_y - r * math.sin(theta)
                
                drive_dx = cand_x - current_x
                drive_dy = cand_y - current_y
                dist_to_cand = math.sqrt(drive_dx**2 + drive_dy**2)
                
                if dist_to_cand < 0.05:
                    continue
                    
                expected_yaw = math.atan2(drive_dy, drive_dx)
                
                target_dx = target_global_x - cand_x
                target_dy = target_global_y - cand_y
                
                pred_local_x = target_dx * math.cos(-expected_yaw) - target_dy * math.sin(-expected_yaw)
                pred_local_y = target_dx * math.sin(-expected_yaw) + target_dy * math.cos(-expected_yaw)
                
                candidates.append({
                    'dist': dist_to_cand,
                    'cand_x': cand_x,
                    'cand_y': cand_y,
                    'cand_yaw': expected_yaw,
                    'pred_local_x': pred_local_x,
                    'pred_local_y': pred_local_y
                })
                
        candidates.sort(key=lambda c: c['dist'])
        
        best_park_x = None
        best_park_y = None
        found_pose = False
        
        for cand in candidates:
            if self.check_ik_feasibility(cand['pred_local_x'], cand['pred_local_y'], local_z, suppress_warnings=True):
                best_park_x = cand['cand_x']
                best_park_y = cand['cand_y']
                self.park_target_yaw = cand['cand_yaw']
                self.pred_local_x = cand['pred_local_x']
                self.pred_local_y = cand['pred_local_y']
                found_pose = True
                break

        if found_pose:
            self.get_logger().info(f"✅ Found valid park pose at X:{best_park_x:.2f}, Y:{best_park_y:.2f}. Driving to park...")
            self.park_target_x = best_park_x
            self.park_target_y = best_park_y
            self.target_global_x = target_global_x
            self.target_global_y = target_global_y
            self.target_global_z = local_z
            self.state = "NAVIGATING"
            return
            
        self.get_logger().error("❌ ABORT: Exhaustive candidate search failed.")
        self.state = "IDLE"

    # [FIX 6] TODO: FUTURE UPGRADE
    # Replace GetPositionIK with MoveGroup.plan() action call.
    # Currently checks "IK exists" but does not guarantee the trajectory is collision-free. 
    # Transitioning to MoveGroup actions will solve "reachable but blocked by collision" scenarios.
    def check_ik_feasibility(self, target_x, target_y, target_z, suppress_warnings=False):

        if self.ik_busy:
            if not suppress_warnings:
                self.get_logger().warn(
                    f"IK request skipped - busy "
                    f"(x={target_x:.2f}, y={target_y:.2f}, z={target_z:.2f})"
                )
            return False

        self.ik_busy = True

        try:
            if not suppress_warnings:
                self.get_logger().info(
                    f"[IK DEBUG] START "
                    f"x={target_x:.2f} "
                    f"y={target_y:.2f} "
                    f"z={target_z:.2f}"
                )

            req = GetPositionIK.Request()
            req.ik_request.group_name = ARM_GROUP_NAME
            req.ik_request.pose_stamped.header.frame_id = "base_link"
            req.ik_request.robot_state.joint_state = self.latest_joint_state
            req.ik_request.pose_stamped.pose.position.x = float(target_x)
            req.ik_request.pose_stamped.pose.position.y = float(target_y)
            req.ik_request.pose_stamped.pose.position.z = float(target_z)

            if self.target_ee_quat is not None:
                req.ik_request.pose_stamped.pose.orientation = self.target_ee_quat
            else:
                req.ik_request.pose_stamped.pose.orientation.w = 1.0

            future = self.ik_client.call_async(req)

            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=2.0
            )

            if not future.done():
                if not suppress_warnings:
                    self.get_logger().error("[IK DEBUG] IK SERVICE TIMEOUT")
                return False

            response = future.result()
            error_code = response.error_code.val

            if error_code == 1:
                self.last_valid_ik_solution = response.solution.joint_state
                if not suppress_warnings:
                    self.get_logger().info("[IK DEBUG] VALID IK SOLUTION FOUND")
                return True

            if not suppress_warnings:
                self.get_logger().warn(f"MoveIt Error Code: {error_code}")
                if float(target_z) < CHASSIS_HEIGHT:
                    self.get_logger().error(f"FATAL COLLISION TRAP: Z={target_z}m is below chassis!")

            return False

        except Exception as e:
            if not suppress_warnings:
                self.get_logger().error(f"[IK DEBUG] Exception: {str(e)}")
            return False

        finally:
            self.ik_busy = False

    # ==========================================
    # 🛞 THE WHEELS & HARDWARE EXECUTION
    # ==========================================

    def get_current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return x, y, yaw
        except Exception:
            return None, None, None

    def control_loop(self):
        if self.state != "NAVIGATING": return
            
        current_x, current_y, current_yaw = self.get_current_pose()
        if current_x is None: return

        dx = self.park_target_x - current_x
        dy = self.park_target_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        yaw_error = self.park_target_yaw - current_yaw
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi
        
        # Enforcing distance AND target orientation
        if distance <= 0.08 and abs(yaw_error) < 0.10:
            
            self.cmd_vel_pub.publish(Twist())
            self.state = "REACHING"

            self.get_logger().info(
                f"Arrived at Park Position. Distance Error: {distance:.3f}m, Yaw Error: {abs(yaw_error):.3f}rad"
            )

            final_dx = self.target_global_x - current_x
            final_dy = self.target_global_y - current_y

            actual_local_x = (
                final_dx * math.cos(-current_yaw)
                - final_dy * math.sin(-current_yaw)
            )

            actual_local_y = (
                final_dx * math.sin(-current_yaw)
                + final_dy * math.cos(-current_yaw)
            )
            
            error_x = actual_local_x - self.pred_local_x
            error_y = actual_local_y - self.pred_local_y
            
            self.get_logger().info(
                f"[DRIFT DETECT] Drift Error X: {error_x:.4f}m, Drift Error Y: {error_y:.4f}m"
            )

            reachable = self.check_ik_feasibility(
                actual_local_x,
                actual_local_y,
                self.target_global_z
            )

            if reachable:
                self.get_logger().info("[DEBUG] Executing arm trajectory")
                self.execute_moveit_reach()
            else:
                self.get_logger().error("Terrain shift caused IK to fail. Aborting.")
                self.state = "IDLE"

            return

        target_yaw = math.atan2(dy, dx)
        heading_error = target_yaw - current_yaw
        
        while heading_error > math.pi: heading_error -= 2 * math.pi
        while heading_error < -math.pi: heading_error += 2 * math.pi

        twist = Twist()
        
        # Drive logic: Align heading first, then drive while correcting, finally turn to park yaw
        if distance > 0.08:
            if abs(heading_error) > 0.2:
                twist.angular.z = max(min(heading_error * 1.5, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)
            else:
                twist.linear.x = min(distance * 0.8, MAX_LINEAR_SPEED)
                twist.angular.z = heading_error * 1.0
        else:
            # We are in position but need to fix our final yaw orientation
            twist.angular.z = max(min(yaw_error * 1.5, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)

        self.cmd_vel_pub.publish(twist)

    def execute_moveit_reach(self):
        if self.last_valid_ik_solution is None: return

        traj_msg = JointTrajectory()
        traj_msg.joint_names = ARM_JOINT_NAMES
        
        point = JointTrajectoryPoint()
        target_positions = []
        
        for joint in ARM_JOINT_NAMES:
            if joint in self.last_valid_ik_solution.name:
                idx = self.last_valid_ik_solution.name.index(joint)
                target_positions.append(self.last_valid_ik_solution.position[idx])
            else:
                return

        point.positions = target_positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000  
        
        traj_msg.points.append(point)
        self.arm_traj_pub.publish(traj_msg)
        self.state = "IDLE"

    # ==========================================
    # ⌨️ TERMINAL LISTENER
    # ==========================================
    
    def terminal_listener(self):
        while rclpy.ok():
            try:
                if self.teleop_active: 
                    continue
                    
                user_input = input().strip().split()
                if not user_input: continue
                
                if user_input[0] == "teleop":
                    self.run_teleop_loop()
                
                elif user_input[0] == "cm" and len(user_input) == 4:
                    dx = float(user_input[1])
                    dy = float(user_input[2])
                    dz = float(user_input[3])
                    self.process_relative_command(dx, dy, dz)
                else:
                    print("Invalid. Use 'cm dx dy dz' or type 'teleop'")
            except Exception as e:
                    print(f"[INPUT THREAD ERROR] {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WholeBodyController()
    
    # Use MultiThreadedExecutor so callbacks don't block each other
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist()) 
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()