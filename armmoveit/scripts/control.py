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


from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from moveit_msgs.srv import GetPositionFK, GetCartesianPath
from geometry_msgs.msg import PoseStamped, Vector3, Point # Added Point here
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PlanningOptions  # Ensure this import is at the top of your file
from armmoveit.srv import GetIKSeed
from moveit_msgs.msg import RobotState
from rclpy.callback_groups import ReentrantCallbackGroup
from scipy.spatial.transform import Rotation as R # Ensure this is at the top of your file
import math
import threading
import numpy as np
import time

class CartesianController(Node):
    def __init__(self):
        super().__init__('cartesian_controller',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
       # self._seed_client = self.create_client(GetIKSeed, 'get_ik_seed')

        self.group = ReentrantCallbackGroup() # Allows concurrent callbacks
        
        # Use the group for your clients
        self._seed_client = self.create_client(GetIKSeed, 'get_ik_seed', callback_group=self.group)
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self._cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')

        self._joint_state = None
        self._marker_pub = self.create_publisher(MarkerArray, '/target_visualization', 10)
        self._js_lock = threading.Lock()
        self.create_subscription(JointState, '/joint_states', self._js_callback, qos_profile_sensor_data)

    
    def print_mission_report(self, target_x, target_y, target_z, init, s1, final, s2_success):
        """Calculates and prints the unified mission metrics."""
        import numpy as np
        import math

        self.get_logger().info("\n--- MISSION REPORT ---")
        self.get_logger().info(f"Target Pos:  ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
        self.get_logger().info(f"Initial Pos: ({init['pos'].x:.3f}, {init['pos'].y:.3f}, {init['pos'].z:.3f})")
        
        # Determine Final Pos (Fallback to Stage 1 if Stage 2 fails)
        final_pos = final['pos'] if s2_success else s1['pos']
        self.get_logger().info(f"Final Pos:   ({final_pos.x:.3f}, {final_pos.y:.3f}, {final_pos.z:.3f})")
        
        # 1. Distance Moved (Start to Finish)
        dist_moved = math.sqrt((final_pos.x - init['pos'].x)**2 + (final_pos.y - init['pos'].y)**2 + (final_pos.z - init['pos'].z)**2)
        self.get_logger().info(f"Distance Moved: {dist_moved:.4f}m")

        # 2. Target Error (Difference between requested location and actual final location)
        target_err = math.sqrt((final_pos.x - target_x)**2 + (final_pos.y - target_y)**2 + (final_pos.z - target_z)**2)
        self.get_logger().info(f"Target Positional Error: {target_err:.4f}m")
        
        # Direction Report
        self.get_logger().info(f"Initial Direction Vector: {init['dir']}")
        if s2_success:
            dot = np.clip(np.dot(init['dir'], final['dir']), -1.0, 1.0)
            angle_err = math.degrees(math.acos(dot))
            self.get_logger().info(f"Final Direction Vector: {final['dir']}")
            self.get_logger().info(f"Orientation Error: {angle_err:.2f} degrees")
        else:
            self.get_logger().warn("Stage 2 Status: FAILED - Orientation unchanged.")
        self.get_logger().info("----------------------\n")

    # CHANGE THIS DEFINITION IN YOUR FILE:
    def publish_state_markers(self, init, s1, final):
        ma = MarkerArray()
        # Now this matches the 3 arguments you pass in go_to()
        states = [ (init, 'init', (0,0,1)), (s1, 's1', (1,1,0)), (final, 'final', (0,1,0)) ]
    # ... rest of your code ...
        
        for i, (state, ns, color) in enumerate(states):
            # Sphere
            m = Marker(); m.header.frame_id = 'base_link'; m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns; m.id = i; m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position = state['pos']; m.scale.x = 0.04; m.scale.y = 0.04; m.scale.z = 0.04
            m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], 0.8
            ma.markers.append(m)
            # Arrow
            arr = Marker(); arr.header.frame_id = 'base_link'; arr.header.stamp = m.header.stamp
            arr.ns = ns + '_dir'; arr.id = i + 10; arr.type = Marker.ARROW; arr.action = Marker.ADD
            arr.points = [state['pos'], Point(x=state['pos'].x + state['dir'][0]*0.2, y=state['pos'].y + state['dir'][1]*0.2, z=state['pos'].z + state['dir'][2]*0.2)]
            arr.scale.x = 0.02; arr.scale.y = 0.04; arr.scale.z = 0.04
            arr.color.r, arr.color.g, arr.color.b, arr.color.a = color[0], color[1], color[2], 1.0
            ma.markers.append(arr)
        self._marker_pub.publish(ma)

    def publish_target(self, x, y, z):
        """Publishes visualization markers for the target."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = now
        m.ns = 'target_goal'; m.id = 0
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position = Point(x=float(x), y=float(y), z=float(z))
        m.scale.x = 0.05; m.scale.y = 0.05; m.scale.z = 0.05
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.8
        ma.markers.append(m)
        self._marker_pub.publish(ma)

    def get_current_joint_state(self):
        """Helper to safely get current joint state."""
        with self._js_lock:
            return self._joint_state

    def get_gripper_direction(self):
        pose = self.get_current_pose()
        if not pose: 
            self.get_logger().error("Could not retrieve pose for direction vector.")
            return None
        
        # Get orientation quaternion
        q = pose.pose.orientation
        rot = R.from_quat([q.x, q.y, q.z, q.w])
        
        # Rotate the unit vector [0, 0, 1] by the orientation
        # This gives you the vector the gripper is pointing along
        direction = rot.apply([0,-1,0])
        return direction
    
    def _js_callback(self, msg):
        with self._js_lock:
            self._joint_state = msg

    

    def wait_for_servers(self):
        self.get_logger().info("Waiting for servers...")
        self._action_client.wait_for_server()
        self._fk_client.wait_for_service()
        self._cartesian_client.wait_for_service()
        self.get_logger().info("Ready.")

    def get_ik_seed(self, x, y, z):
        if not self._seed_client.service_is_ready():
            self.get_logger().warn("Seeder service not ready.")
            return None
        req = GetIKSeed.Request()
        req.x, req.y, req.z = float(x), float(y), float(z)
        future = self._seed_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.joints if (response and response.joints) else None

    def get_fk_for_joints(self, joints):
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = ["gripper-base"]
        req.robot_state.joint_state.name = ["z_axis", "link_1", "link_2", "link_3", "wrist_1", "gripper-wrist"]
        req.robot_state.joint_state.position = joints
        future = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        return res.pose_stamped[0] if (res and res.error_code.val == 1) else None

    def get_current_pose(self):
        with self._js_lock: js = self._joint_state
        if js is None: return None
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = ["gripper-base"]
        req.robot_state.joint_state = js
        future = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        return res.pose_stamped[0] if (res and res.error_code.val == 1) else None
    
    def get_current_wrist_pose(self):
        """FK to wrist_1 link — used as the Stage 1 position target."""
        with self._js_lock: js = self._joint_state
        if js is None: return None
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = ["wrist1"]
        req.robot_state.joint_state = js
        future = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        return res.pose_stamped[0] if (res and res.error_code.val == 1) else None

    def execute_motion(self, req):
        """Helper to send a MotionPlanRequest and wait for the result."""
        # 6. Construct PlanningOptions
        plan_opts = PlanningOptions()
        plan_opts.plan_only = False
        plan_opts.replan = True
        plan_opts.replan_attempts = 5

        # 7. Construct Goal
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = plan_opts
        
        # 8. Execution
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        code = result_future.result().result.error_code.val
        
        if code == 1:
            self.get_logger().info("Execution Success.")
            return True
        else:
            self.get_logger().error(f"Execution Failed with Error Code: {code}")
            return False
        
    def report_accuracy(self, x, y, z):
        final_pose = self.get_current_pose()
        if final_pose:
            fx, fy, fz = final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z
            error = math.sqrt((x - fx)**2 + (y - fy)**2 + (z - fz)**2)
            self.get_logger().info("--- Final Accuracy Report ---")
            self.get_logger().info(f"Target: X:{x:.4f} Y:{y:.4f} Z:{z:.4f}")
            self.get_logger().info(f"Actual: X:{fx:.4f} Y:{fy:.4f} Z:{fz:.4f}")
            self.get_logger().info(f"Final Error (Euclidean Distance): {error:.4f} meters")
            self.get_logger().info("-----------------------------")

    def solve_ik_for_orientation(self, x, y, z, orientation, seed_joint_state=None,
                                   position_tolerance=None):
        """
        Call MoveIt IK for a full 6-DOF pose.

        seed_joint_state  : JointState seed — solver starts here, keeping the
                            solution near the current arm configuration.
        position_tolerance: if set (metres), adds a position constraint with a
                            sphere of this radius around (x,y,z) so the solver
                            can accept solutions with small positional drift
                            (e.g. after wrist reorientation moves gripper-base
                            slightly). If None, exact position is required.
        """
        from moveit_msgs.srv import GetPositionIK
        if not hasattr(self, '_ik_client'):
            self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
        if not self._ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("IK service not available.")
            return None

        if seed_joint_state is None:
            with self._js_lock:
                seed_joint_state = self._joint_state
        if seed_joint_state is None:
            self.get_logger().error("No joint state available for IK seed.")
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position = Point(x=float(x), y=float(y), z=float(z))
        req.ik_request.pose_stamped.pose.orientation = orientation
        req.ik_request.robot_state.joint_state = seed_joint_state

        if position_tolerance is not None:
            # A loose position constraint lets the solver return solutions
            # where gripper-base drifts up to `position_tolerance` metres from
            # (x,y,z) — needed when only wrist joints are being adjusted and
            # full 6-DOF satisfaction at the exact point isn't possible.
            from geometry_msgs.msg import Pose as GmPose
            tol_pose = GmPose()
            tol_pose.position.x = float(x)
            tol_pose.position.y = float(y)
            tol_pose.position.z = float(z)
            tol_pose.orientation.w = 1.0
            tol_sphere = SolidPrimitive(
                type=SolidPrimitive.SPHERE,
                dimensions=[float(position_tolerance)])
            pc = PositionConstraint()
            pc.header.frame_id = "base_link"
            pc.link_name = "gripper-base"
            pc.constraint_region = BoundingVolume(
                primitives=[tol_sphere], primitive_poses=[tol_pose])
            pc.weight = 1.0
            req.ik_request.constraints.position_constraints.append(pc)

        future = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.error_code.val == 1:
            return res.solution.joint_state
        self.get_logger().error(f"IK failed, error code: {res.error_code.val if res else 'no response'}")
        return None

    def go_to(self, x, y, z):
        import numpy as np
        
        # 0. INITIAL CAPTURE (With Safety Checks)
        start_pose = self.get_current_pose()
        start_wrist = self.get_current_wrist_pose()
        start_dir = self.get_gripper_direction()

        if start_pose is None or start_wrist is None or start_dir is None:
            self.get_logger().error("Cannot get initial pose. Aborting.")
            return False
            
        # Initialize our flight data recorder
        init_state = {'pos': start_wrist.pose.position, 'dir': start_dir}
        s1_state = init_state  # Fallback if S1 fails
        final_state = init_state # Fallback if S2 fails

        q0 = start_pose.pose.orientation
        rot0 = R.from_quat([q0.x, q0.y, q0.z, q0.w])
        original_approach = rot0.apply([0, -1, 0])

        # ── STAGE 1: Move wrist_1 to (x, y, z) ───────────────────────────────
        offset = np.array([
            start_pose.pose.position.x - start_wrist.pose.position.x,
            start_pose.pose.position.y - start_wrist.pose.position.y,
            start_pose.pose.position.z - start_wrist.pose.position.z,
        ])
        
        gx_t = float(x) + offset[0]
        gy_t = float(y) + offset[1]
        gz_t = float(z) + offset[2]
        self.get_logger().info(f"Stage 1: Moving wrist_1 to ({x:.3f}, {y:.3f}, {z:.3f}) via gripper-base target ({gx_t:.3f}, {gy_t:.3f}, {gz_t:.3f}) ...")

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "gripper-base"
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position = Point(x=gx_t, y=gy_t, z=gz_t)
        target_pose.pose.orientation = start_pose.pose.orientation
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.05])
        pos_constraint.constraint_region = BoundingVolume(primitives=[sphere], primitive_poses=[target_pose.pose])
        pos_constraint.weight = 1.0

        req1 = MotionPlanRequest()
        req1.group_name = "arm"
        req1.goal_constraints = [Constraints(position_constraints=[pos_constraint])]
        req1.start_state.is_diff = True

        if not self.execute_motion(req1):
            self.get_logger().error("Stage 1 failed.")
            return False

        self.get_logger().info("Stage 1 done. Waiting for joint states to settle...")
        time.sleep(0.5)
        
        # --- STAGE 1 SNAPSHOT ---
        s1_wrist = self.get_current_wrist_pose()
        s1_dir = self.get_gripper_direction()
        if s1_wrist and s1_dir is not None:
            s1_state = {'pos': s1_wrist.pose.position, 'dir': s1_dir}

        # ── STAGE 2: Reorient gripper-base ────────────────────────────────────
        self.get_logger().info("Stage 2: Reorienting gripper ...")
        with self._js_lock:
            seed_js = self._joint_state

        post_s1_gripper = self.get_current_pose()
        if post_s1_gripper is None:
            self.get_logger().error("Cannot get gripper pose after Stage 1.")
            return False
            
        gx = post_s1_gripper.pose.position.x
        gy = post_s1_gripper.pose.position.y
        gz = post_s1_gripper.pose.position.z

        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "gripper-base"
        oc.orientation = q0
        oc.absolute_x_axis_tolerance = 0.15
        oc.absolute_y_axis_tolerance = 0.10
        oc.absolute_z_axis_tolerance = 0.10
        oc.weight = 1.0

        pc2 = PositionConstraint()
        pc2.header.frame_id = "base_link"
        pc2.link_name = "gripper-base"
        from geometry_msgs.msg import Pose as GmPose
        tol_pose = GmPose()
        tol_pose.position.x = gx
        tol_pose.position.y = gy
        tol_pose.position.z = gz
        tol_pose.orientation.w = 1.0
        pc2.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.15])],
            primitive_poses=[tol_pose])
        pc2.weight = 1.0

        req2 = MotionPlanRequest()
        req2.group_name = "arm"
        req2.goal_constraints = [Constraints(orientation_constraints=[oc], position_constraints=[pc2])]
        req2.start_state = RobotState(joint_state=seed_js) # Fixed RobotState wrap
        req2.num_planning_attempts = 5
        req2.allowed_planning_time = 5.0

        # Capture success/failure without returning early
        s2_success = self.execute_motion(req2)
        
        if s2_success:
            self.get_logger().info("Stage 2 Success. Orientation restored.")
        else:
            self.get_logger().error("Stage 2 failed — wrist position preserved, orientation unchanged.")

        # --- FINAL SNAPSHOT ---
        final_wrist = self.get_current_wrist_pose()
        final_dir = self.get_gripper_direction()
        if final_wrist and final_dir is not None:
            final_state = {'pos': final_wrist.pose.position, 'dir': final_dir}

        # --- REPORTING & VISUALIZATION ---
        self.print_mission_report(float(x), float(y), float(z), init_state, s1_state, final_state, s2_success)
        self.publish_state_markers(init_state, s1_state, final_state)
        
        # You can now delete or comment out the old report_accuracy call!
        # self.report_accuracy(x, y, z) 
        
        return s2_success

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
        # Stage 2 of go_to will restore orientation — but rotate_by needs
        # to temporarily override the "original" orientation with the desired one.
        # Simplest correct fix: directly call solve_ik_for_orientation + execute.
        from geometry_msgs.msg import Quaternion
        target_orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        with self._js_lock:
            seed_js = self._joint_state
        ik_result_js = self.solve_ik_for_orientation(p.x, p.y, p.z, target_orientation, seed_js)
        if ik_result_js is None:
            self.get_logger().error("rotate_by: IK failed.")
            return
        arm_joints = ["z_axis", "link_1", "link_2", "link_3", "wrist_1", "gripper-wrist"]
        name_to_pos = dict(zip(ik_result_js.name, ik_result_js.position))
        ordered_positions = [name_to_pos[j] for j in arm_joints if j in name_to_pos]
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        traj = JointTrajectory()
        traj.joint_names = arm_joints
        pt = JointTrajectoryPoint()
        pt.positions = ordered_positions
        pt.time_from_start.sec = 2
        traj.points = [pt]
        exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        exec_client.wait_for_server()
        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = traj
        future = exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

def print_help():
    print("""
------------------------------------------------------
ROBOTIC ARM COMMAND-LINE CONTROL
Reference Frame: base_link
End-Effector:    gripper-base
------------------------------------------------------
Commands:
  pos                 → Show current X Y Z & Orientation
  go <X> <Y> <Z>      → Move wrist_1 to coordinates, then reorient gripper
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
                        
                        # Calculate Roll, Pitch, Yaw from Quaternion
                        sinr_cosp = 2.0 * (o.w * o.x + o.y * o.z)
                        cosr_cosp = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)
                        roll = math.atan2(sinr_cosp, cosr_cosp)

                        sinp = 2.0 * (o.w * o.y - o.z * o.x)
                        pitch = math.asin(sinp) if abs(sinp) <= 1.0 else math.copysign(math.pi / 2.0, sinp)

                        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
                        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
                        yaw = math.atan2(siny_cosp, cosy_cosp)

                        print(f"  Current XYZ : {p.x:.4f}  {p.y:.4f}  {p.z:.4f}")
                        print(f"  Current RPY : {roll:.4f}  {pitch:.4f}  {yaw:.4f} (radians)")
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