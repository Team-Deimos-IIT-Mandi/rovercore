#!/usr/bin/env python3
"""
Science Mission Node — full autonomous science pipeline.

State machine:
  IDLE → NAV_TO_SITE → PROBE → DRILL → SENSE_SOIL →
  SPECTROMETER → MICROSCOPE → LOG → RETRACT → DONE

Fault tolerance (per sensor):
  drill fail   — non-fatal, flag drill_ok=False, continue
  soil fail    — non-fatal, flag soil_ok=False
  spec fail    — non-fatal, flag spec_ok=False
  micro fail   — non-fatal, flag micro_ok=False
  ALL sensors dead (soil+spec+micro all False) — FAILED (no data to log)

Published topics:
  /science/status       (std_msgs/String)  — current state
  /science/site_complete (std_msgs/Bool)   — True when site fully processed

Subscribed topics:
  /science/soil_readings (std_msgs/Float32MultiArray) — pH, moisture, temp from CM4
  /pick_place/trigger    (std_msgs/Bool)              — optional external start

Services called:
  /science/sample      (std_srvs/Trigger) — logs current soil data row
  /science/drill_start (std_srvs/Trigger) — enables drill on CM4
  /science/drill_stop  (std_srvs/Trigger) — disables drill on CM4
  /science/spectrometer (std_srvs/Trigger) — triggers 3s spectrometer scan
  /science/microscope   (std_srvs/Trigger) — captures microscope frame
"""

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from std_msgs.msg import String, Bool, Float32MultiArray
    from std_srvs.srv import Trigger
    from nav2_msgs.action import NavigateToPose
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False


# ---------------------------------------------------------------------------
# Named arm poses — must match arm_urdf.srdf
# ---------------------------------------------------------------------------
ARM_JOINTS = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6']
ARM_POSES = {
    'stow':          [0.0, -1.57,  1.57,  0.0,  0.0,  0.0],
    'science_probe': [0.0,  0.3,   1.2,   0.0, -1.57, 0.0],
}


# ---------------------------------------------------------------------------
# Science mission state machine (pure Python)
# ---------------------------------------------------------------------------
class ScienceMissionStateMachine:
    STATES = ('IDLE', 'NAV_TO_SITE', 'PROBE', 'DRILL', 'SENSE_SOIL',
              'SPECTROMETER', 'MICROSCOPE', 'LOG', 'RETRACT', 'DONE', 'FAILED')

    def __init__(self, arm_enabled: bool = True):
        self.arm_enabled = arm_enabled
        self.state = 'IDLE'
        self.site_pose = None

        # Sensor health flags (set False on failure)
        self.drill_ok = True
        self.soil_ok = True
        self.spec_ok = True
        self.micro_ok = True

        self._transitions: list[str] = []

    # ------------------------------------------------------------------
    # Public events
    # ------------------------------------------------------------------
    def start(self, site_pose):
        assert self.state == 'IDLE'
        self.site_pose = site_pose
        self._go('NAV_TO_SITE')

    def nav_success(self):
        if self.state == 'NAV_TO_SITE':
            self._go('PROBE' if self.arm_enabled else 'DRILL')

    def nav_fail(self):
        self._go('FAILED')

    def probe_success(self):
        if self.state == 'PROBE':
            self._go('DRILL')

    def probe_fail(self):
        self._go('FAILED')

    def drill_success(self):
        if self.state == 'DRILL':
            self._go('SENSE_SOIL')

    def drill_fail(self):
        """Non-fatal — record flag and continue."""
        if self.state == 'DRILL':
            self.drill_ok = False
            self._go('SENSE_SOIL')

    def soil_success(self):
        if self.state == 'SENSE_SOIL':
            self.soil_ok = True
            self._go('SPECTROMETER')

    def soil_fail(self):
        if self.state == 'SENSE_SOIL':
            self.soil_ok = False
            self._go('SPECTROMETER')

    def spec_success(self):
        if self.state == 'SPECTROMETER':
            self.spec_ok = True
            self._go('MICROSCOPE')

    def spec_fail(self):
        if self.state == 'SPECTROMETER':
            self.spec_ok = False
            self._go('MICROSCOPE')

    def micro_success(self):
        if self.state == 'MICROSCOPE':
            self.micro_ok = True
            self._check_sensor_abort_or_log()

    def micro_fail(self):
        if self.state == 'MICROSCOPE':
            self.micro_ok = False
            self._check_sensor_abort_or_log()

    def log_success(self):
        if self.state == 'LOG':
            self._go('RETRACT' if self.arm_enabled else 'DONE')

    def log_fail(self):
        if self.state == 'LOG':
            self._go('FAILED')

    def retract_success(self):
        if self.state == 'RETRACT':
            self._go('DONE')

    def retract_fail(self):
        if self.state == 'RETRACT':
            self._go('FAILED')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _check_sensor_abort_or_log(self):
        """All sensors failed → FAILED. At least one succeeded → LOG."""
        if not self.soil_ok and not self.spec_ok and not self.micro_ok:
            self._go('FAILED')
        else:
            self._go('LOG')

    def _go(self, next_state: str):
        self._transitions.append(f'{self.state}→{next_state}')
        self.state = next_state

    @property
    def is_terminal(self) -> bool:
        return self.state in ('DONE', 'FAILED')

    @property
    def any_sensor_data(self) -> bool:
        return self.soil_ok or self.spec_ok or self.micro_ok

    def reset(self):
        self.state = 'IDLE'
        self.site_pose = None
        self.drill_ok = True
        self.soil_ok = True
        self.spec_ok = True
        self.micro_ok = True
        self._transitions.clear()


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------
_NodeBase = Node if _ROS_AVAILABLE else object


class ScienceMissionNode(_NodeBase):
    def __init__(self):
        super().__init__('science_mission')

        self.declare_parameter('arm_enabled', False)
        self.declare_parameter('drill_duration', 5.0)
        self.declare_parameter('sensor_read_timeout', 3.0)

        arm_enabled = self.get_parameter('arm_enabled').value
        self._drill_duration = self.get_parameter('drill_duration').value
        self._sensor_timeout = self.get_parameter('sensor_read_timeout').value

        self._sm = ScienceMissionStateMachine(arm_enabled=arm_enabled)
        self._soil_data: list[float] | None = None
        self._active = False

        # Publishers
        self._status_pub = self.create_publisher(String, '/science/status', 10)
        self._site_done_pub = self.create_publisher(Bool, '/science/site_complete', 10)

        # Subscribers
        self.create_subscription(
            Float32MultiArray, '/science/soil_readings', self._soil_cb, 10)
        self.create_subscription(
            Bool, '/science/trigger', self._trigger_cb, 10)

        # Service clients
        self._sample_client = self.create_client(Trigger, '/science/sample')
        self._drill_start_client = self.create_client(Trigger, '/science/drill_start')
        self._drill_stop_client = self.create_client(Trigger, '/science/drill_stop')
        self._spec_client = self.create_client(Trigger, '/science/spectrometer')
        self._micro_client = self.create_client(Trigger, '/science/microscope')

        # Action clients
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        # Service server
        self.create_service(Trigger, '/science/start_mission', self._start_srv_cb)

        self.get_logger().info(f'ScienceMission ready — arm_enabled={arm_enabled}')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _soil_cb(self, msg: Float32MultiArray):
        self._soil_data = list(msg.data)

    def _trigger_cb(self, msg: Bool):
        if msg.data and not self._active:
            self._start_sequence(site_pose=None)

    def _start_srv_cb(self, _req, response):
        if self._active:
            response.success = False
            response.message = f'Already running: {self._sm.state}'
        else:
            self._start_sequence(site_pose=None)
            response.success = True
            response.message = 'Science mission started'
        return response

    # ------------------------------------------------------------------
    # Sequence
    # ------------------------------------------------------------------
    def _start_sequence(self, site_pose):
        self._sm.reset()
        self._sm.start(site_pose)
        self._active = True
        self._publish_status()
        self._run_nav()

    def _run_nav(self):
        if self._sm.site_pose is None:
            # No explicit nav target — skip nav, start at probe
            self._sm.nav_success()
            self._publish_status()
            self._run_probe()
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._sm.site_pose
        self._send_nav_goal(goal, self._on_nav_done)

    def _on_nav_done(self, future):
        result = future.result()
        if result and result.status == 4:
            self._sm.nav_success()
            self._publish_status()
            self._run_probe()
        else:
            self._sm.nav_fail()
            self._publish_status()
            self._finish()

    def _run_probe(self):
        if not self._sm.arm_enabled:
            self._sm.probe_success()
            self._publish_status()
            self._run_drill()
            return
        self._send_arm_goal('science_probe', self._on_probe_done)

    def _on_probe_done(self, future):
        if self._arm_future_ok(future):
            self._sm.probe_success()
            self._publish_status()
            self._run_drill()
        else:
            self._sm.probe_fail()
            self._publish_status()
            self._finish()

    def _run_drill(self):
        future = self._call_service(self._drill_start_client, '/science/drill_start')
        if future is None:
            self._sm.drill_fail()
            self._publish_status()
            self._run_sense_soil()
            return
        future.add_done_callback(self._on_drill_start)

    def _on_drill_start(self, future):
        try:
            result = future.result()
            if result.success:
                self.create_timer(self._drill_duration, self._stop_drill_once)
                return
        except Exception:
            pass
        self._sm.drill_fail()
        self._publish_status()
        self._run_sense_soil()

    def _stop_drill_once(self):
        # Timer fires once; stop drill then continue pipeline
        stop_future = self._call_service(self._drill_stop_client, '/science/drill_stop')
        self._sm.drill_success()
        self._publish_status()
        self._run_sense_soil()

    def _run_sense_soil(self):
        future = self._call_service(self._sample_client, '/science/sample')
        if future is None:
            self._sm.soil_fail()
        else:
            future.add_done_callback(self._on_soil_done)
            return
        self._publish_status()
        self._run_spectrometer()

    def _on_soil_done(self, future):
        try:
            result = future.result()
            if result.success:
                self._sm.soil_success()
            else:
                self._sm.soil_fail()
        except Exception:
            self._sm.soil_fail()
        self._publish_status()
        self._run_spectrometer()

    def _run_spectrometer(self):
        future = self._call_service(self._spec_client, '/science/spectrometer')
        if future is None:
            self._sm.spec_fail()
            self._publish_status()
            self._run_microscope()
        else:
            future.add_done_callback(self._on_spec_done)

    def _on_spec_done(self, future):
        try:
            result = future.result()
            if result.success:
                self._sm.spec_success()
            else:
                self._sm.spec_fail()
        except Exception:
            self._sm.spec_fail()
        self._publish_status()
        self._run_microscope()

    def _run_microscope(self):
        future = self._call_service(self._micro_client, '/science/microscope')
        if future is None:
            self._sm.micro_fail()
            self._publish_status()
            self._after_sensors()
        else:
            future.add_done_callback(self._on_micro_done)

    def _on_micro_done(self, future):
        try:
            result = future.result()
            if result.success:
                self._sm.micro_success()
            else:
                self._sm.micro_fail()
        except Exception:
            self._sm.micro_fail()
        self._publish_status()
        self._after_sensors()

    def _after_sensors(self):
        if self._sm.is_terminal:
            self._finish()
            return
        # LOG state
        future = self._call_service(self._sample_client, '/science/sample')
        if future is None:
            self._sm.log_fail()
            self._publish_status()
            self._finish()
        else:
            future.add_done_callback(self._on_log_done)

    def _on_log_done(self, future):
        try:
            result = future.result()
            if result.success:
                self._sm.log_success()
            else:
                self._sm.log_fail()
        except Exception:
            self._sm.log_fail()
        self._publish_status()
        if self._sm.state == 'RETRACT':
            self._run_retract()
        else:
            self._finish()

    def _run_retract(self):
        if not self._sm.arm_enabled:
            self._sm.retract_success()
            self._publish_status()
            self._finish()
            return
        self._send_arm_goal('stow', self._on_retract_done)

    def _on_retract_done(self, future):
        if self._arm_future_ok(future):
            self._sm.retract_success()
        else:
            self._sm.retract_fail()
        self._publish_status()
        self._finish()

    def _finish(self):
        self._active = False
        done_msg = Bool()
        done_msg.data = (self._sm.state == 'DONE')
        self._site_done_pub.publish(done_msg)
        self.get_logger().info(
            f'Science mission {self._sm.state}. '
            f'drill={self._sm.drill_ok} soil={self._sm.soil_ok} '
            f'spec={self._sm.spec_ok} micro={self._sm.micro_ok}')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _call_service(self, client, name: str):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'Service {name} not available — skipping')
            return None
        return client.call_async(Trigger.Request())

    def _send_nav_goal(self, goal, callback):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available')
            self._sm.nav_fail()
            self._publish_status()
            self._finish()
            return
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_nav_goal_accepted(f, callback))

    def _on_nav_goal_accepted(self, future, callback):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._sm.nav_fail()
            self._publish_status()
            self._finish()
            return
        goal_handle.get_result_async().add_done_callback(callback)

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
            lambda f: self._on_arm_accepted(f, callback))

    def _on_arm_accepted(self, future, callback):
        goal_handle = future.result()
        if not goal_handle.accepted:
            callback(None)
            return
        goal_handle.get_result_async().add_done_callback(callback)

    def _arm_future_ok(self, future) -> bool:
        if future is None:
            return False
        result = future.result()
        return result is not None and result.status == 4

    def _publish_status(self):
        msg = String()
        msg.data = self._sm.state
        self._status_pub.publish(msg)


def main(args=None):
    if not _ROS_AVAILABLE:
        raise RuntimeError('ROS 2 not available')
    rclpy.init(args=args)
    node = ScienceMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
