import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2
import numpy as np


MISSION_STATES = [
    'BOOTING', 'DOME_EXIT', 'ASTRONAUT_SEARCH',
    'FUEL_TRAIL_FOLLOW', 'OXYGEN_TANK_NAV', 'DOME_RETURN',
    'SEQUENCE_COMPLETE', 'EMERGENCY_STOP'
]

STATE_TRANSITIONS = {
    'DOME_EXIT': ('mission/complete_dome_exit', 'ASTRONAUT_SEARCH'),
    'ASTRONAUT_SEARCH': ('mission/complete_astronaut_search', 'FUEL_TRAIL_FOLLOW'),
    'FUEL_TRAIL_FOLLOW': ('mission/complete_fuel_trail', 'OXYGEN_TANK_NAV'),
    'OXYGEN_TANK_NAV': ('mission/complete_oxygen_tank_nav', 'DOME_RETURN'),
    'DOME_RETURN': ('mission/complete_dome_return', 'SEQUENCE_COMPLETE'),
}

DEFAULT_CAMERA_TOPICS = {
    'fuel_trail_debug': '/fuel_trail/debug_frame',
    'fuel_trail_mask': '/fuel_trail/debug_mask',
    'airlock_debug': '/airlock_debug',
    'front_camera': '/cam/front/image_raw',
}


class ROSBridgeNode(Node):
    def __init__(self, shared):
        super().__init__('rover_dashboard_bridge')
        self.shared = shared
        self.bridge = CvBridge()
        self.camera_subs = {}

        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.state_sub = self.create_subscription(
            String, 'rover/mission_state', self.state_callback, qos_latched
        )

        self.service_clients = {}
        for state_name, (srv_name, _) in STATE_TRANSITIONS.items():
            client = self.create_client(Trigger, srv_name)
            self.service_clients[state_name] = client

        self.estop_client = self.create_client(Trigger, 'mission/emergency_stop')

        for label, topic in self.shared['camera_topics'].items():
            self._create_camera_sub(label, topic)

        self.get_logger().info('ROS Bridge node initialized')

    def _create_camera_sub(self, label, topic):
        sub = self.create_subscription(
            ROSImage, topic,
            lambda msg, lbl=label: self.camera_callback(msg, lbl),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.camera_subs[label] = sub

    def subscribe_camera(self, label, topic):
        if label in self.camera_subs:
            self.destroy_subscription(self.camera_subs.pop(label))
        self._create_camera_sub(label, topic)
        self.shared['camera_topics'][label] = topic

    def unsubscribe_camera(self, label):
        if label in self.camera_subs:
            self.destroy_subscription(self.camera_subs.pop(label))
            self.shared['cameras'].pop(label, None)
            self.shared['camera_topics'].pop(label, None)

    def update_camera_topic(self, label, new_topic):
        if label not in self.camera_subs:
            return
        if self.shared['camera_topics'].get(label) == new_topic:
            return
        self.destroy_subscription(self.camera_subs.pop(label))
        self._create_camera_sub(label, new_topic)
        self.shared['camera_topics'][label] = new_topic

    def state_callback(self, msg):
        self.shared['state'] = msg.data

    def camera_callback(self, msg, label):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            self.shared['cameras'][label] = jpeg.tobytes()
        except Exception:
            pass

    def call_service_async(self, service_name, done_callback=None):
        client_map = {
            'mission/emergency_stop': self.estop_client,
        }
        for state_name, (srv_name, _) in STATE_TRANSITIONS.items():
            client_map[srv_name] = self.service_clients[state_name]

        client = client_map.get(service_name)
        if not client:
            return

        if not client.wait_for_service(timeout_sec=0.5):
            return

        req = Trigger.Request()
        future = client.call_async(req)
        if done_callback:
            future.add_done_callback(done_callback)
        return future


class ROSBridge:
    def __init__(self):
        self.shared = {
            'state': 'UNKNOWN',
            'logs': deque(maxlen=500),
            'cameras': {},
            'camera_topics': DEFAULT_CAMERA_TOPICS.copy(),
        }
        self._node = None
        self._executor = None
        self._spin_thread = None

    def start(self):
        rclpy.init(args=[])
        self._node = ROSBridgeNode(self.shared)
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        while rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    def stop(self):
        if self._node:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if self._spin_thread:
            self._spin_thread.join(timeout=2.0)

    def get_state(self):
        return self.shared['state']

    def get_logs(self, n=100):
        logs = list(self.shared['logs'])
        return logs[-n:]

    def add_log(self, level, name, msg):
        import time as ttime
        entry = {
            'time': ttime.time(),
            'level_str': level,
            'name': name,
            'msg': msg,
        }
        self.shared['logs'].append(entry)

    def get_camera_frame(self, label):
        return self.shared['cameras'].get(label)

    def get_camera_topics(self):
        return dict(self.shared['camera_topics'])

    def add_camera_topic(self, label, topic):
        if label in self.shared['camera_topics']:
            return False
        self._node.subscribe_camera(label, topic)
        return True

    def remove_camera_topic(self, label):
        self._node.unsubscribe_camera(label)

    def update_camera_topic(self, label, topic):
        self._node.update_camera_topic(label, topic)

    def advance_state(self):
        current = self.shared['state']
        if current in STATE_TRANSITIONS:
            srv_name, _ = STATE_TRANSITIONS[current]
            self._node.call_service_async(srv_name)
            return True
        return False

    def emergency_stop(self):
        self._node.call_service_async('mission/emergency_stop')
        return True
