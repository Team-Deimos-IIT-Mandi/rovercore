#!/usr/bin/env python3
"""
valve_approach_controller.py
─────────────────────────────
State machine driven valve approach.

States
──────
SEARCHING        : No bbox seen. Logs error every watchdog tick.
TARGET_LOCKED    : Bbox visible. Accumulating stable detections before committing.
MOVING_TO_PREGRASP : Arm in motion toward standoff pose. Bbox IGNORED.
FINAL_ALIGNMENT  : Arm arrived. Re-checks bbox for fine correction.
GRASP            : Within grasp distance. Holds position. (extend with grasp action)

Key design decisions
─────────────────────
* Once a move goal is sent, the state is MOVING_TO_PREGRASP and ALL bbox
  callbacks are ignored until the action completes (success or failure).
* Detection stability filter: requires N consecutive detections within a
  spatial tolerance before committing to a move — kills jitter-triggered loops.
* Snapshot: the target pose is FROZEN when the move is sent. Mid-motion
  detections never update it.
* On motion failure, state falls back to SEARCHING with a cooldown so the
  node doesn't immediately re-trigger.
* REQUIRE_USER_CONFIRMATION gate sits between TARGET_LOCKED → MOVING_TO_PREGRASP.
"""

import math
import threading
import enum

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints,
                              OrientationConstraint, PositionConstraint)
from shape_msgs.msg import SolidPrimitive
import tf2_ros

try:
    from tf2_geometry_msgs import do_transform_pose_stamped
except ImportError:
    from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped


# ══════════════════════════════════════════════════════════════════════════════
class State(enum.Enum):
    SEARCHING         = "SEARCHING"
    TARGET_LOCKED     = "TARGET_LOCKED"
    MOVING_TO_PREGRASP = "MOVING_TO_PREGRASP"
    FINAL_ALIGNMENT   = "FINAL_ALIGNMENT"
    GRASP             = "GRASP"


# ══════════════════════════════════════════════════════════════════════════════
class ValveApproachController(Node):

    # ── tunable parameters ────────────────────────────────────────────────
    REQUIRE_USER_CONFIRMATION = True    # set False for autonomous

    DETECTION_TIMEOUT_SEC  = 2.0        # SEARCHING error threshold
    STABILITY_COUNT        = 5          # consecutive detections needed to commit
    STABILITY_TOL_M        = 0.05       # max centroid drift between stable frames (m)
    STANDOFF_DISTANCE      = 0.20       # metres short of valve for pregrasp
    FINAL_ALIGN_DISTANCE   = 0.08       # inside this → FINAL_ALIGNMENT
    GRASP_DISTANCE         = 0.05       # inside this → GRASP
    MAX_REACH_RADIUS       = 1.0        # reject detections beyond this (m)
    FAILURE_COOLDOWN_SEC   = 3.0        # wait after motion failure before retrying
    # ─────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('valve_approach_controller')

        # ── MoveIt ───────────────────────────────────────────────────────
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.planning_frame    = 'base_link'
        self.planning_group    = 'arm'
        self.end_effector_link = 'gripper-base'
        self.camera_frame      = 'gripper_rgbd_camera'

        # ── TF ───────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── camera intrinsics ─────────────────────────────────────────────
        self.cx = 320.0;  self.cy = 240.0
        self.fx = 640.0 / (2.0 * np.tan(1.5 / 2.0))
        self.fy = self.fx

        # ── state machine ─────────────────────────────────────────────────
        self._state               = State.SEARCHING
        self._state_lock          = threading.Lock()

        # stability buffer
        self._stable_detections   = []   # list of (x,y,z) in base_link
        self._frozen_target       = None  # PoseStamped frozen when move sent

        # timing
        self._last_detection_time = None
        self._failure_cooldown_end = None

        # prompt guard
        self._confirm_thread_alive = False

        # ── locked orientation (captured once at startup) ─────────────────
        self.locked_orientation          = None
        self._orientation_capture_attempts = 0
        self._capture_timer = self.create_timer(0.5, self._try_capture_orientation)

        # ── timers ────────────────────────────────────────────────────────
        self.create_timer(1.0, self._watchdog)

        # ── subscription ──────────────────────────────────────────────────
        self.create_subscription(
            Point, '/arm/gripper/valve_center', self._bbox_callback, 10)

        self.get_logger().info(
            f"Valve controller ready | "
            f"REQUIRE_USER_CONFIRMATION={self.REQUIRE_USER_CONFIRMATION}")

    # ══════════════════════════════════════════════════════════════════════
    # PROPERTIES
    # ══════════════════════════════════════════════════════════════════════
    @property
    def state(self):
        return self._state

    def _transition(self, new_state: State):
        old = self._state
        self._state = new_state
        self.get_logger().info(f"[STATE] {old.value} → {new_state.value}")

    # ══════════════════════════════════════════════════════════════════════
    # ORIENTATION CAPTURE
    # ══════════════════════════════════════════════════════════════════════
    def _try_capture_orientation(self):
        if self.locked_orientation is not None:
            self._capture_timer.cancel()
            return

        self._orientation_capture_attempts += 1
        if self._orientation_capture_attempts > 10:
            self.get_logger().warn(
                "Could not capture EEF orientation — defaulting to identity.")
            self.locked_orientation = (0.0, 0.0, 0.0, 1.0)
            self._capture_timer.cancel()
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.planning_frame, self.end_effector_link,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.3))
            r = tf.transform.rotation
            self.locked_orientation = (r.x, r.y, r.z, r.w)
            self.get_logger().info(
                f"EEF orientation locked: "
                f"({r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f})")
            self._capture_timer.cancel()
        except Exception as e:
            self.get_logger().debug(
                f"Orientation capture attempt "
                f"{self._orientation_capture_attempts}: {e}")

    # ══════════════════════════════════════════════════════════════════════
    # WATCHDOG TIMER
    # ══════════════════════════════════════════════════════════════════════
    def _watchdog(self):
        with self._state_lock:
            state = self._state

        # only warn about missing bbox when we actually need to see it
        if state in (State.SEARCHING, State.TARGET_LOCKED):
            if self._last_detection_time is None:
                self.get_logger().error(
                    "No bbox received yet. Is the perception node running? "
                    f"(topic: /arm/gripper/valve_center)")
                return

            elapsed = (self.get_clock().now()
                       - self._last_detection_time).nanoseconds / 1e9
            if elapsed > self.DETECTION_TIMEOUT_SEC:
                self.get_logger().error(
                    f"Bbox lost for {elapsed:.1f} s.")
                with self._state_lock:
                    if self._state == State.TARGET_LOCKED:
                        self._stable_detections.clear()
                        self._transition(State.SEARCHING)
  
    def _is_pose_reachable(self, pose: PoseStamped) -> bool:
        """
        Fast IK feasibility check using MoveIt's /compute_ik service.
        Returns True if a valid joint solution exists.
        """

        if not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/compute_ik service unavailable.")
            return False

        req = GetPositionIK.Request()

        ik_req = PositionIKRequest()
        ik_req.group_name = self.planning_group
        ik_req.pose_stamped = pose
        ik_req.ik_link_name = self.end_effector_link
        ik_req.timeout.sec = 1

        # apply locked orientation
        qx, qy, qz, qw = self.locked_orientation
        ik_req.pose_stamped.pose.orientation.x = qx
        ik_req.pose_stamped.pose.orientation.y = qy
        ik_req.pose_stamped.pose.orientation.z = qz
        ik_req.pose_stamped.pose.orientation.w = qw

        req.ik_request = ik_req

        future = self.ik_client.call_async(req)

        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done():
            self.get_logger().error("IK request timed out.")
            return False

        result = future.result()

        if result.error_code.val == 1:
            self.get_logger().info("IK solution found.")
            return True

        self.get_logger().warn(
            f"No IK solution. MoveIt error code: "
            f"{result.error_code.val}"
        )

        return False

    # ══════════════════════════════════════════════════════════════════════
    # BBOX CALLBACK  –  the only entry point for sensor data
    # ══════════════════════════════════════════════════════════════════════
    def _bbox_callback(self, msg: Point):
        self._last_detection_time = self.get_clock().now()

        with self._state_lock:
            state = self._state

        # ── HARD GATE: ignore bbox entirely while arm is moving ───────────
        if state in (State.MOVING_TO_PREGRASP,):
            return

        # ── basic sanity checks ───────────────────────────────────────────
        if self.locked_orientation is None:
            return
        if msg.z <= 0.05:
            self.get_logger().warn(f"Depth {msg.z:.3f} m implausible — skip.")
            return

        # ── failure cooldown guard ────────────────────────────────────────
        if self._failure_cooldown_end is not None:
            remaining = (self._failure_cooldown_end
                         - self.get_clock().now()).nanoseconds / 1e9
            if remaining > 0:
                self.get_logger().debug(
                    f"In failure cooldown ({remaining:.1f} s remaining).")
                return
            self._failure_cooldown_end = None

        # ── 1. project pixel → base_link valve position ───────────────────
        valve_base = self._project_to_base(msg)
        if valve_base is None:
            return

        vx = valve_base.pose.position.x
        vy = valve_base.pose.position.y
        vz = valve_base.pose.position.z

        # ── 2. check distance from EEF ────────────────────────────────────
        try:
            eef_tf = self.tf_buffer.lookup_transform(
                self.planning_frame, self.end_effector_link,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.3))
        except Exception as e:
            self.get_logger().error(f"EEF TF lookup failed: {e}")
            return

        ex = eef_tf.transform.translation.x
        ey = eef_tf.transform.translation.y
        ez = eef_tf.transform.translation.z
        dist = math.sqrt((vx-ex)**2 + (vy-ey)**2 + (vz-ez)**2)

        if dist > self.MAX_REACH_RADIUS:
            self.get_logger().error(
                f"Valve {dist:.2f} m away — beyond {self.MAX_REACH_RADIUS} m limit.")
            return

        # ── 3. state-specific logic ───────────────────────────────────────
        with self._state_lock:
            state = self._state   # re-read inside lock

            if state == State.GRASP:
                return   # done, nothing to do

            if state == State.FINAL_ALIGNMENT:
                if dist <= self.GRASP_DISTANCE:
                    self._transition(State.GRASP)
                    self.get_logger().info(
                        "Within grasp distance. GRASP state reached. "
                        "Trigger your grasp action here.")
                else:
                    # send a fine correction move (no user confirmation needed)
                    goal_pose = self._build_standoff_pose(
                        vx, vy, vz, ex, ey, ez,
                        standoff=self.GRASP_DISTANCE)
                    if goal_pose:
                        self._transition(State.MOVING_TO_PREGRASP)


                        if not self._is_pose_reachable(goal_pose): 
                            self.get_logger().warn( "Target pose is not reachable by this arm." )
                            return
                        
                        self._send_move_goal(goal_pose, next_state_on_success=State.FINAL_ALIGNMENT)
                return

            # SEARCHING or TARGET_LOCKED
            self._update_stability_buffer(vx, vy, vz)

            if len(self._stable_detections) >= self.STABILITY_COUNT:
                # ── commit: average the stable detections ─────────────────
                avg = np.mean(self._stable_detections, axis=0)
                self._stable_detections.clear()

                goal_pose = self._build_standoff_pose(
                    avg[0], avg[1], avg[2], ex, ey, ez,
                    standoff=self.STANDOFF_DISTANCE)

                if goal_pose is None:
                    return

                self._transition(State.TARGET_LOCKED)

                if self.REQUIRE_USER_CONFIRMATION:
                    # release lock before blocking thread
                    pass  # transition already done above
                    self._request_confirmation(goal_pose, avg[0], avg[1], avg[2])
                else:
                    self._transition(State.MOVING_TO_PREGRASP)
                    self._frozen_target = goal_pose
                    self._send_move_goal(goal_pose,
                                         next_state_on_success=State.FINAL_ALIGNMENT)

    # ══════════════════════════════════════════════════════════════════════
    # STABILITY BUFFER
    # ══════════════════════════════════════════════════════════════════════
    def _update_stability_buffer(self, x, y, z):
        """
        Only accept a detection if it's within STABILITY_TOL_M of the
        previous one — resets the buffer on jitter.
        """
        if self._stable_detections:
            last = self._stable_detections[-1]
            d = math.sqrt((x-last[0])**2 + (y-last[1])**2 + (z-last[2])**2)
            if d > self.STABILITY_TOL_M:
                self.get_logger().debug(
                    f"Centroid jumped {d:.3f} m — resetting stability buffer.")
                self._stable_detections.clear()

        self._stable_detections.append((x, y, z))
        self.get_logger().debug(
            f"Stability buffer: {len(self._stable_detections)}/{self.STABILITY_COUNT}")

    # ══════════════════════════════════════════════════════════════════════
    # PROJECTION HELPER
    # ══════════════════════════════════════════════════════════════════════
    def _project_to_base(self, msg: Point):
        """Unproject pixel+depth → base_link PoseStamped (raw, no standoff)."""
        x_cam = (msg.x - self.cx) * msg.z / self.fx
        y_cam = (msg.y - self.cy) * msg.z / self.fy
        z_cam = msg.z

        cam_pose = PoseStamped()
        cam_pose.header.frame_id    = self.camera_frame
        cam_pose.header.stamp       = self.get_clock().now().to_msg()
        cam_pose.pose.position.x    = float(x_cam)
        cam_pose.pose.position.y    = float(y_cam)
        cam_pose.pose.position.z    = float(z_cam)
        cam_pose.pose.orientation.w = 1.0

        try:
            tf = self.tf_buffer.lookup_transform(
                self.planning_frame, self.camera_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5))
            return do_transform_pose_stamped(cam_pose, tf)
        except Exception as e:
            self.get_logger().error(f"Camera→base_link TF failed: {e}")
            return None

    # ══════════════════════════════════════════════════════════════════════
    # STANDOFF POSE BUILDER
    # ══════════════════════════════════════════════════════════════════════
    def _build_standoff_pose(self, vx, vy, vz,
                              ex, ey, ez, standoff):
        """
        Return a PoseStamped that is `standoff` metres back from the valve
        along the straight-line EEF → valve vector.
        """
        dx, dy, dz = vx - ex, vy - ey, vz - ez
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist < 1e-3:
            self.get_logger().warn("EEF already at valve position.")
            return None

        nx, ny, nz = dx/dist, dy/dist, dz/dist
        goal = PoseStamped()
        goal.header.frame_id    = self.planning_frame
        goal.pose.position.x    = vx - nx * standoff
        goal.pose.position.y    = vy - ny * standoff
        goal.pose.position.z    = vz - nz * standoff
        goal.pose.orientation.w = 1.0
        return goal

    # ══════════════════════════════════════════════════════════════════════
    # USER CONFIRMATION
    # ══════════════════════════════════════════════════════════════════════
    def _request_confirmation(self, goal_pose, vx, vy, vz):
        if self._confirm_thread_alive:
            return

        def _prompt():
            self._confirm_thread_alive = True
            print(
                f"\n[ValveApproach] Stable valve centroid in {self.planning_frame}:\n"
                f"  valve  → x={vx:.3f}  y={vy:.3f}  z={vz:.3f}\n"
                f"  goal   → x={goal_pose.pose.position.x:.3f}"
                f"  y={goal_pose.pose.position.y:.3f}"
                f"  z={goal_pose.pose.position.z:.3f}\n"
                f"Move end-effector? [y/N] ", end='', flush=True)
            try:
                answer = input().strip().lower()
            except EOFError:
                answer = 'n'

            if answer == 'y':
                with self._state_lock:
                    self._frozen_target = goal_pose
                    self._transition(State.MOVING_TO_PREGRASP)
                # re-enter executor via one-shot timer
                self.create_timer(
                    0.0,
                    lambda: self._send_move_goal(
                        goal_pose,
                        next_state_on_success=State.FINAL_ALIGNMENT))
            else:
                self.get_logger().info("User declined — returning to SEARCHING.")
                with self._state_lock:
                    self._transition(State.SEARCHING)

            self._confirm_thread_alive = False

        threading.Thread(target=_prompt, daemon=True).start()

    # ══════════════════════════════════════════════════════════════════════
    # MOVEGROUP GOAL
    # ══════════════════════════════════════════════════════════════════════
    def _send_move_goal(self, goal_pose: PoseStamped,
                        next_state_on_success: State):
        if not self.move_group_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("MoveGroup action server unavailable.")
            with self._state_lock:
                self._transition(State.SEARCHING)
            return

        qx, qy, qz, qw = self.locked_orientation

        x = goal_pose.pose.position.x
        y = goal_pose.pose.position.y
        z = goal_pose.pose.position.z

        # position constraint
        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.planning_frame
        pos_c.header.stamp    = self.get_clock().now().to_msg()
        pos_c.link_name       = self.end_effector_link
        box = SolidPrimitive(type=SolidPrimitive.BOX,
                             dimensions=[0.04, 0.04, 0.04])
        tgt = PoseStamped()
        tgt.pose.position.x  = float(x)
        tgt.pose.position.y  = float(y)
        tgt.pose.position.z  = float(z)
        tgt.pose.orientation.w = 1.0
        vol = BoundingVolume()
        vol.primitives.append(box)
        vol.primitive_poses.append(tgt.pose)
        pos_c.constraint_region = vol
        pos_c.weight = 1.0

        # orientation constraint (locked from startup)
        ori_c = OrientationConstraint()
        ori_c.header.frame_id = self.planning_frame
        ori_c.header.stamp    = self.get_clock().now().to_msg()
        ori_c.link_name       = self.end_effector_link
        ori_c.orientation.x   = qx
        ori_c.orientation.y   = qy
        ori_c.orientation.z   = qz
        ori_c.orientation.w   = qw
        ori_c.absolute_x_axis_tolerance = 1.57
        ori_c.absolute_y_axis_tolerance = 0.3
        ori_c.absolute_z_axis_tolerance = 0.3
        ori_c.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_c)
        constraints.orientation_constraints.append(ori_c)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name            = self.planning_group
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info(
            f"Sending goal → x={x:.3f}  y={y:.3f}  z={z:.3f}")

        future = self.move_group_client.send_goal_async(goal_msg)
        # carry next_state through closure
        future.add_done_callback(
            lambda f, ns=next_state_on_success: self._goal_response_cb(f, ns))

    # ══════════════════════════════════════════════════════════════════════
    # ACTION CALLBACKS
    # ══════════════════════════════════════════════════════════════════════
    def _goal_response_cb(self, future, next_state_on_success: State):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error(
                "MoveGroup goal REJECTED — check IK / collision scene.")
            self._enter_failure_cooldown()
            return
        self.get_logger().info("Goal accepted — arm moving.")
        gh.get_result_async().add_done_callback(
            lambda f, ns=next_state_on_success: self._result_cb(f, ns))

    def _result_cb(self, future, next_state_on_success: State):
        result = future.result().result
        if result.error_code.val == 1:   # MoveItErrorCodes.SUCCESS
            self.get_logger().info("Motion completed successfully.")
            with self._state_lock:
                self._transition(next_state_on_success)
        else:
            self.get_logger().error(
                f"Motion FAILED (MoveIt error {result.error_code.val}).")
            self._enter_failure_cooldown()

    def _enter_failure_cooldown(self):
        self._failure_cooldown_end = (
            self.get_clock().now()
            + rclpy.duration.Duration(seconds=self.FAILURE_COOLDOWN_SEC))
        self._stable_detections.clear()
        with self._state_lock:
            self._transition(State.SEARCHING)
        self.get_logger().warn(
            f"Entering {self.FAILURE_COOLDOWN_SEC} s failure cooldown.")


# ══════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = ValveApproachController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()