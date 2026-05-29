#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (MotionPlanRequest, Constraints, PositionConstraint,
                              OrientationConstraint, BoundingVolume)
from moveit_msgs.srv import GetPositionFK, GetCartesianPath
from geometry_msgs.msg import PoseStamped, Vector3
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
import math
import threading
import time

class CartesianController(Node):
    def __init__(self):
        # 1. Force the script to sync with Gazebo's simulated clock
        super().__init__('cartesian_controller',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self._cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')

        self._joint_state = None
        self._js_lock = threading.Lock()
        
        # 2. Use the Sensor Data QoS profile to guarantee connection to Gazebo
        self.create_subscription(JointState, '/joint_states', self._js_callback, qos_profile_sensor_data)

    def _js_callback(self, msg):
        with self._js_lock:
            self._joint_state = msg

    def wait_for_servers(self):
        self.get_logger().info("Waiting for /move_action action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Waiting for /compute_fk service...")
        self._fk_client.wait_for_service()
        self.get_logger().info("Waiting for /compute_cartesian_path service...")
        self._cartesian_client.wait_for_service()
        self.get_logger().info("Ready.")

    def get_current_pose(self):
        with self._js_lock:
            js = self._joint_state

        if js is None:
            self.get_logger().error("No joint state received yet")
            return None

        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"          
        req.fk_link_names = ["gripper-base"]       
        req.robot_state.joint_state = js

        future = self._fk_client.call_async(req)
        start = time.time()
        while not future.done():
            if time.time() - start > 5.0:
                self.get_logger().error("FK timed out")
                return None
            time.sleep(0.05)

        result = future.result()
        if result and result.error_code.val == 1:
            return result.pose_stamped[0]
        self.get_logger().error("FK failed. Is the arm connected to the tf tree?")
        return None

    def go_to(self, x, y, z, qx=None, qy=None, qz=None, qw=None):
        """Move to Absolute XYZ (Relative to base_link)."""
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z

        if qx is None:
            target.pose.orientation.x = 0.0
            target.pose.orientation.y = 0.0
            target.pose.orientation.z = 0.0
            target.pose.orientation.w = 1.0
            use_ori_constraint = False
        else:
            target.pose.orientation.x = qx
            target.pose.orientation.y = qy
            target.pose.orientation.z = qz
            target.pose.orientation.w = qw
            use_ori_constraint = True

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "gripper-base"
        pos_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        
        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [target.pose]
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pos_constraint]

        if use_ori_constraint:
            ori_constraint = OrientationConstraint()
            ori_constraint.header.frame_id = "base_link"
            ori_constraint.link_name = "gripper-base"
            ori_constraint.orientation = target.pose.orientation
            ori_constraint.absolute_x_axis_tolerance = 0.1  # Tightened for accuracy
            ori_constraint.absolute_y_axis_tolerance = 0.1
            ori_constraint.absolute_z_axis_tolerance = 0.1
            ori_constraint.weight = 1.0
            goal_constraints.orientation_constraints = [ori_constraint]

        req = MotionPlanRequest()
        req.group_name = "arm"
        req.num_planning_attempts = 50
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5

        self.get_logger().info(f"Planning Absolute Move to X:{x:.3f} Y:{y:.3f} Z:{z:.3f} ...")

        future = self._action_client.send_goal_async(goal)
        start = time.time()
        while not future.done():
            if time.time() - start > 10.0:
                self.get_logger().error("Goal send timed out")
                return False
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by controller")
            return False

        result_future = goal_handle.get_result_async()
        start = time.time()
        while not result_future.done():
            if time.time() - start > 30.0:
                self.get_logger().error("Execution timed out")
                return False
            time.sleep(0.05)

        code = result_future.result().result.error_code.val
        if code == 1:
            self.get_logger().info("Execution Success.")
            return True
        else:
            self.get_logger().error(f"Execution Failed with Error Code: {code}")
            return False

    def move_by_cartesian(self, dx=0.0, dy=0.0, dz=0.0):
        """Relative Straight-Line Cartesian Move."""
        current = self.get_current_pose()
        if current is None:
            return

        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x = current.pose.position.x + dx
        target.pose.position.y = current.pose.position.y + dy
        target.pose.position.z = current.pose.position.z + dz
        target.pose.orientation = current.pose.orientation

        self.get_logger().info(f"Planning Cartesian Step DX:{dx:.3f} DY:{dy:.3f} DZ:{dz:.3f} ...")

        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.group_name = "arm"
        req.link_name = "gripper-base"
        req.waypoints = [current.pose, target.pose]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        future = self._cartesian_client.call_async(req)
        start = time.time()
        while not future.done():
            if time.time() - start > 10.0:
                self.get_logger().error("Cartesian path service timed out")
                return
            time.sleep(0.05)

        result = future.result()
        if result.fraction < 0.9:
            self.get_logger().error(f"Move blocked! Only {result.fraction*100:.1f}% of path achievable.")
            return

        exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        exec_client.wait_for_server()

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = result.solution

        future = exec_client.send_goal_async(exec_goal)
        start = time.time()
        while not future.done():
            if time.time() - start > 10.0:
                return
            time.sleep(0.05)

        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        start = time.time()
        while not result_future.done():
            if time.time() - start > 30.0:
                return
            time.sleep(0.05)

        self.get_logger().info("Cartesian move complete.")

    def rotate_by(self, d_roll=0.0, d_pitch=0.0, d_yaw=0.0):
        """Rotate end effector in place (Inputs in Radians)"""
        current = self.get_current_pose()
        if current is None:
            return
            
        o = current.pose.orientation
        p = current.pose.position

        # Convert current quaternion to Euler angles (Roll, Pitch, Yaw)
        sinr_cosp = 2.0 * (o.w * o.x + o.y * o.z)
        cosr_cosp = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (o.w * o.y - o.z * o.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Add the requested deltas
        roll += d_roll
        pitch += d_pitch
        yaw += d_yaw

        # Convert new Euler angles back to Quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        self.get_logger().info(f"Rotating in place to Roll:{roll:.2f} Pitch:{pitch:.2f} Yaw:{yaw:.2f} ...")
        self.go_to(p.x, p.y, p.z, qx, qy, qz, qw)

def print_help():
    print("""
------------------------------------------------------
ROBOTIC ARM COMMAND-LINE CONTROL
Reference Frame: base_link
End-Effector:    gripper-base
------------------------------------------------------
Commands:
  pos                 → Show current X Y Z & Orientation
  go <X> <Y> <Z>      → Move absolutely to coordinates
  cm <dX> <dY> <dZ>   → Move relatively in a straight line
  rot <dR> <dP> <dY>  → Rotate in place (Roll, Pitch, Yaw in Radians)
  q                   → Quit
------------------------------------------------------""")

def main():
    rclpy.init()
    node = CartesianController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.wait_for_servers()
    print_help()

    try:
        while rclpy.ok():
            try:
                cmd = input("Command: ").strip().split()
            except EOFError:
                break
            if not cmd:
                continue

            try:
                if cmd[0] == 'pos':
                    pose = node.get_current_pose()
                    if pose:
                        p = pose.pose.position
                        o = pose.pose.orientation
                        print(f"  Current XYZ : {p.x:.4f}  {p.y:.4f}  {p.z:.4f}")
                        print(f"  Current QUAT: {o.x:.4f}  {o.y:.4f}  {o.z:.4f}  {o.w:.4f}")

                elif cmd[0] == 'go' and len(cmd) == 4:
                    node.go_to(float(cmd[1]), float(cmd[2]), float(cmd[3]))

                elif cmd[0] == 'cm' and len(cmd) == 4:
                    node.move_by_cartesian(float(cmd[1]), float(cmd[2]), float(cmd[3]))

                elif cmd[0] == 'rot' and len(cmd) == 4:
                    node.rotate_by(float(cmd[1]), float(cmd[2]), float(cmd[3]))

                elif cmd[0] == 'q':
                    break
                else:
                    print("Invalid command. Check format.")

            except ValueError:
                print("Invalid number formatting. Example: rot 1.57 0 0")
            except Exception as e:
                node.get_logger().error(f"Error executing command: {e}")

    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()