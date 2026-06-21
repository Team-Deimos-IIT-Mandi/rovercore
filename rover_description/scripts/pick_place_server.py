#!/usr/bin/env python3
"""
Pick-Place Server — Nav2 + arm controller pick-and-place pipeline.

State machine: IDLE → NAV_TO_PREGRASP → PICK → STOW → NAV_TO_DROP → PLACE → DONE

When arm_enabled=False: navigation still runs; arm steps are skipped gracefully.
Publishes /pick_place/status (std_msgs/String) for operator dashboard.

Subscribed topics:
  /aruco/pose  (geometry_msgs/PoseStamped)  — target object world pose

Action clients:
  /navigate_to_pose     (nav2_msgs/action/NavigateToPose)
  /arm_controller/follow_joint_trajectory   (control_msgs/action/FollowJointTrajectory)
  /gripper_controller/follow_joint_trajectory

Action server:
  /pick_place  (std_srvs/srv/Trigger — simplified; full action TBD)
"""

import math

# ROS imports are optional so the pure-Python state machine is importable in unit tests
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from std_msgs.msg import String, Bool
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False


# ---------------------------------------------------------------------------
# Named arm poses (joint values in radians; must match arm_urdf.srdf)
# ---------------------------------------------------------------------------
ARM_JOINTS = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6']
GRIPPER_JOINTS = ['Finger_1', 'Finger_2']

ARM_POSES = {
    'stow':          [0.0, -1.57,  1.57,  0.0,  0.0,  0.0],
    'deploy_ready':  [0.0, -0.5,   0.8,   0.0, -0.3,  0.0],
    'science_probe': [0.0,  0.3,   1.2,   0.0, -1.57, 0.0],
}
GRIPPER_OPEN   = [0.0,      0.0]       # Finger_1, Finger_2
GRIPPER_CLOSED = [0.0355, -0.0355]


# ---------------------------------------------------------------------------
# State machine (pure Python, no ROS — enables unit testing)
# ---------------------------------------------------------------------------
class PickPlaceStateMachine:
    STATES = ('IDLE', 'NAV_TO_PREGRASP', 'PICK', 'STOW',
              'NAV_TO_DROP', 'PLACE', 'DONE', 'FAILED')

    def __init__(self, arm_enabled: bool = True):
        self.arm_enabled = arm_enabled
        self.state = 'IDLE'
        self.target_pose = None
        self.drop_pose = None
        self._transitions: list[str] = []

    def start(self, target_pose, drop_pose):
        assert self.state == 'IDLE', f'Cannot start from state {self.state}'
        self.target_pose = target_pose
        self.drop_pose = drop_pose
        self._transition('NAV_TO_PREGRASP')

    def nav_success(self):
        if self.state == 'NAV_TO_PREGRASP':
            self._transition('PICK' if self.arm_enabled else 'STOW')
        elif self.state == 'NAV_TO_DROP':
            self._transition('PLACE' if self.arm_enabled else 'DONE')

    def nav_fail(self):
        self._transition('FAILED')

    def arm_success(self):
        if self.state == 'PICK':
            self._transition('STOW')
        elif self.state == 'STOW':
            self._transition('NAV_TO_DROP')
        elif self.state == 'PLACE':
            self._transition('DONE')

    def arm_fail(self):
        self._transition('FAILED')

    def reset(self):
        self.state = 'IDLE'
        self.target_pose = None
        self.drop_pose = None
        self._transitions.clear()

    def _transition(self, next_state: str):
        self._transitions.append(f'{self.state}→{next_state}')
        self.state = next_state

    @property
    def is_terminal(self) -> bool:
        return self.state in ('DONE', 'FAILED')


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------
_NodeBase = Node if _ROS_AVAILABLE else object


class PickPlaceServer(_NodeBase):
    def __init__(self):
        super().__init__('pick_place_server')

        self.declare_parameter('arm_enabled', False)
        self.declare_parameter('nav_goal_timeout', 120.0)
        self.declare_parameter('pregrasp_offset_m', 0.5)

        arm_enabled = self.get_parameter('arm_enabled').value
        self._nav_timeout = self.get_parameter('nav_goal_timeout').value
        self._pregrasp_offset = self.get_parameter('pregrasp_offset_m').value

        self._sm = PickPlaceStateMachine(arm_enabled=arm_enabled)

        # Publishers
        self._status_pub = self.create_publisher(String, '/pick_place/status', 10)

        # Subscribers
        self._aruco_sub = self.create_subscription(
            PoseStamped, '/aruco/pose', self._aruco_cb, 10)
        self._trigger_sub = self.create_subscription(
            Bool, '/pick_place/trigger', self._trigger_cb, 10)

        # Action clients
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        # Service server — operator can trigger manually
        self._trigger_srv = self.create_service(
            Trigger, '/pick_place/start', self._trigger_srv_cb)

        self._target_pose: PoseStamped | None = None
        self._active = False

        self.get_logger().info(
            f'PickPlaceServer ready — arm_enabled={arm_enabled}')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _aruco_cb(self, msg: PoseStamped):
        self._target_pose = msg

    def _trigger_cb(self, msg: Bool):
        if msg.data and not self._active:
            self._start_sequence()

    def _trigger_srv_cb(self, _req, response):
        if self._active:
            response.success = False
            response.message = f'Already running: {self._sm.state}'
        elif self._target_pose is None:
            response.success = False
            response.message = 'No ArUco target received yet'
        else:
            self._start_sequence()
            response.success = True
            response.message = 'Pick-place sequence started'
        return response

    # ------------------------------------------------------------------
    # Sequence execution
    # ------------------------------------------------------------------
    def _start_sequence(self):
        if self._target_pose is None:
            self.get_logger().error('No target pose — cannot start pick-place')
            return

        drop_pose = PoseStamped()
        drop_pose.header.frame_id = 'map'
        drop_pose.pose.position.x = 0.0
        drop_pose.pose.position.y = 0.0
        drop_pose.pose.orientation.w = 1.0

        self._sm.reset()
        self._sm.start(self._target_pose, drop_pose)
        self._active = True
        self._publish_status()
        self._run_nav_to_pregrasp()

    def _run_nav_to_pregrasp(self):
        goal = NavigateToPose.Goal()
        goal.pose = self._sm.target_pose
        # Offset the goal so rover stops pregrasp_offset metres short of marker
        goal.pose.pose.position.x -= self._pregrasp_offset
        self._send_nav_goal(goal, self._on_pregrasp_nav_done)

    def _on_pregrasp_nav_done(self, future):
        result = future.result()
        if result and result.status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self._sm.nav_success()
            self._publish_status()
            if self._sm.state == 'PICK':
                self._run_arm_pick()
            else:
                self._run_stow()
        else:
            self._sm.nav_fail()
            self._publish_status()
            self._active = False

    def _run_arm_pick(self):
        self._send_arm_goal('deploy_ready', self._on_pick_arm_ready)

    def _on_pick_arm_ready(self, future):
        if self._arm_future_ok(future):
            self._send_gripper_goal(GRIPPER_CLOSED, self._on_gripper_closed)
        else:
            self._sm.arm_fail()
            self._publish_status()
            self._active = False

    def _on_gripper_closed(self, future):
        if self._arm_future_ok(future):
            self._sm.arm_success()  # PICK → STOW
            self._publish_status()
            self._run_stow()
        else:
            self._sm.arm_fail()
            self._publish_status()
            self._active = False

    def _run_stow(self):
        if not self._sm.arm_enabled:
            self._sm.arm_success()  # skip stow, go straight to NAV_TO_DROP
            self._publish_status()
            self._run_nav_to_drop()
            return
        self._send_arm_goal('stow', self._on_stow_done)

    def _on_stow_done(self, future):
        if self._arm_future_ok(future):
            self._sm.arm_success()  # STOW → NAV_TO_DROP
            self._publish_status()
            self._run_nav_to_drop()
        else:
            self._sm.arm_fail()
            self._publish_status()
            self._active = False

    def _run_nav_to_drop(self):
        goal = NavigateToPose.Goal()
        goal.pose = self._sm.drop_pose
        self._send_nav_goal(goal, self._on_drop_nav_done)

    def _on_drop_nav_done(self, future):
        result = future.result()
        if result and result.status == 4:
            self._sm.nav_success()
            self._publish_status()
            if self._sm.state == 'PLACE':
                self._run_arm_place()
            else:
                self._active = False  # DONE
        else:
            self._sm.nav_fail()
            self._publish_status()
            self._active = False

    def _run_arm_place(self):
        self._send_arm_goal('deploy_ready', self._on_place_arm_ready)

    def _on_place_arm_ready(self, future):
        if self._arm_future_ok(future):
            self._send_gripper_goal(GRIPPER_OPEN, self._on_gripper_opened)
        else:
            self._sm.arm_fail()
            self._publish_status()
            self._active = False

    def _on_gripper_opened(self, future):
        if self._arm_future_ok(future):
            self._sm.arm_success()  # PLACE → DONE
        else:
            self._sm.arm_fail()
        self._publish_status()
        # Stow arm after place regardless of success
        if self._sm.arm_enabled:
            self._send_arm_goal('stow', lambda _: None)
        self._active = False

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _send_nav_goal(self, goal, callback):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            self._sm.nav_fail()
            self._publish_status()
            self._active = False
            return
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_nav_goal_accepted(f, callback))

    def _on_nav_goal_accepted(self, future, callback):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav goal rejected')
            self._sm.nav_fail()
            self._publish_status()
            self._active = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(callback)

    def _send_arm_goal(self, pose_name: str, callback):
        if not self._arm_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Arm controller not available')
            callback(None)
            return
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = ARM_POSES[pose_name]
        pt.time_from_start = Duration(sec=3, nanosec=0)
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = self._arm_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_arm_goal_accepted(f, callback))

    def _send_gripper_goal(self, positions: list, callback):
        if not self._gripper_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Gripper controller not available')
            callback(None)
            return
        traj = JointTrajectory()
        traj.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=2, nanosec=0)
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_arm_goal_accepted(f, callback))

    def _on_arm_goal_accepted(self, future, callback):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm/gripper goal rejected')
            callback(None)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(callback)

    def _arm_future_ok(self, future) -> bool:
        if future is None:
            return False
        result = future.result()
        return result is not None and result.status == 4

    def _publish_status(self):
        msg = String()
        msg.data = self._sm.state
        self._status_pub.publish(msg)
        self.get_logger().info(
            f'PickPlace: {" → ".join(self._sm._transitions[-3:])} [{self._sm.state}]'
            if self._sm._transitions else f'PickPlace: {self._sm.state}')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
