# Arc Night Mission

This sub-package is the **main mission pipeline** for the Arc Night competition track. It orchestrates a sequence of autonomous behaviors — from exiting a dome structure to returning home — and provides a **web-based dashboard** for real-time monitoring and control.

The mission is designed to run without Nav2 autonomy — all navigation is odometry-based with simple proportional controllers, and all perception is pure OpenCV vision processing.

---

## Mission Flow

The central finite state machine in `mission_manager.py` drives the mission through the following states:

```
BOOTING → DOME_EXIT → ASTRONAUT_SEARCH → FUEL_TRAIL_FOLLOW → OXYGEN_TANK_NAV → DOME_RETURN → SEQUENCE_COMPLETE
```

An `EMERGENCY_STOP` state is available at any point. `SEQUENCE_COMPLETE` and `EMERGENCY_STOP` are terminal states with no outgoing transitions.

### State Details

| State | Node Responsible | Description |
|---|---|---|
| `BOOTING` | `mission_manager` | Initial state on startup. Immediately self-transitions to `DOME_EXIT` in `main()`. |
| `DOME_EXIT` | `dome_exit.py` (`airlock_exit_node`) | Vision-guided exit from the starting dome using front/left/right cameras. Blind push forward by 0.9 m, then approach an equipment kit. |
| `ASTRONAUT_SEARCH` | `astronaut_searcher.py` | Proportional heading controller drives the rover to odometry target `(15.5, -11.4)`. Uses `/gps/odom` and `/imu/filtered`. |
| `FUEL_TRAIL_FOLLOW` | `fuel_trail_follower.py` | HSV-based line tracking to follow a dark fuel trail. Searches for an orange rocket, then images the damage hole. |
| `OXYGEN_TANK_NAV` | `oxygen_tank_navigator.py` | Proportional heading controller drives the rover to odometry target `(25.8, -6.0)`. Uses `/gps/odom` and `/imu/filtered`. |
| `DOME_RETURN` | `dome_return_navigator.py` | Proportional heading controller drives the rover to odometry target `(-5.0, 3.5)`. Uses `/gps/odom` and `/imu/filtered`. |
| `SEQUENCE_COMPLETE` | — | Terminal state — mission finished successfully. |
| `EMERGENCY_STOP` | — | Terminal state — zero velocity published, all motion halted. |

### State Transitions

All transitions are triggered by `Trigger` service calls. The `mission_manager` provides a service server per transition. Worker nodes call the appropriate service when their task is complete.

| Current State | Service to Advance | Next State |
|---|---|---|
| `BOOTING` | *(auto-transition in `main()`)* | `DOME_EXIT` |
| `DOME_EXIT` | `mission/complete_dome_exit` | `ASTRONAUT_SEARCH` |
| `ASTRONAUT_SEARCH` | `mission/complete_astronaut_search` | `FUEL_TRAIL_FOLLOW` |
| `FUEL_TRAIL_FOLLOW` | `mission/complete_fuel_trail` | `OXYGEN_TANK_NAV` |
| `OXYGEN_TANK_NAV` | `mission/complete_oxygen_tank_nav` | `DOME_RETURN` |
| `DOME_RETURN` | `mission/complete_dome_return` | `SEQUENCE_COMPLETE` |
| *(any)* | `mission/emergency_stop` | `EMERGENCY_STOP` |

---

## Nodes

### `mission_manager.py` — Central FSM

The orchestrator for the entire mission. Maintains the current state, publishes it on `/rover/mission_state` (with `TRANSIENT_LOCAL` durability), and provides a `Trigger` service for each allowed transition.

#### ROS Interface

| Direction | Topic / Service | Type | QoS |
|---|---|---|---|
| Published | `rover/mission_state` | `std_msgs/String` | `TRANSIENT_LOCAL` |
| Service server | `mission/complete_dome_exit` | `std_srvs/srv/Trigger` | — |
| Service server | `mission/complete_astronaut_search` | `std_srvs/srv/Trigger` | — |
| Service server | `mission/complete_fuel_trail` | `std_srvs/srv/Trigger` | — |
| Service server | `mission/complete_oxygen_tank_nav` | `std_srvs/srv/Trigger` | — |
| Service server | `mission/complete_dome_return` | `std_srvs/srv/Trigger` | — |
| Service server | `mission/emergency_stop` | `std_srvs/srv/Trigger` | — |

#### Valid States

```python
VALID_STATES = {
    'BOOTING', 'DOME_EXIT', 'ASTRONAUT_SEARCH',
    'OXYGEN_TANK_NAV', 'DOME_RETURN',
    'FUEL_TRAIL_FOLLOW', 'SEQUENCE_COMPLETE', 'EMERGENCY_STOP'
}
```

#### Auto-Transition

In `main()`, the manager immediately advances from `BOOTING` to `DOME_EXIT`:

```python
if __name__ == '__main__':
    node = MissionManager()
    node.change_state('DOME_EXIT')  # auto-advance from BOOTING
```

On startup, the initial state is set to `'BOOTING'`, then immediately changed to `'DOME_EXIT'`. Worker nodes subscribe to `rover/mission_state` and gate on their assigned state.

---

### `dome_exit.py` — Airlock Exit (Node name: `airlock_exit_node`)

Autonomous exit from the starting dome structure using three camera feeds (front, left, right) and odometry.

#### ROS Interface

| Direction | Topic / Service | Type |
|---|---|---|
| Subscribed | `rover/mission_state` | `std_msgs/String` |
| Subscribed | `/cam/front/image_raw` | `sensor_msgs/Image` |
| Subscribed | `/cam/left/image_raw` | `sensor_msgs/Image` |
| Subscribed | `/cam/right/image_raw` | `sensor_msgs/Image` |
| Subscribed | `/odom` | `nav_msgs/Odometry` |
| Published | `/cmd_vel` | `geometry_msgs/Twist` |
| Published | `/airlock_debug` | `sensor_msgs/Image` |
| Client | `mission/complete_dome_exit` | `std_srvs/srv/Trigger` |

#### Internal State Machine (4 phases)

**State 0 — Track Exit Door:**
- Applies a binary brightness threshold (value 50) to the front camera grayscale feed.
- Finds the largest contour (the dome exit opening).
- Computes the centroid `cx` of the contour.
- Proportional steering: `angular.z = -(cx - frame_center_x) * angular_kp` (note negative sign).
- Drives forward at `target_forward_speed = 1.0 m/s`.
- Progresses to State 1 when `min_front_contour >= 20000` (the exit fills the frame).

**State 1 — Blind Push:**
- Triggered by the tripwire system: left and right cameras look for a bright area (threshold 50) with contour area `>= 15000` pixels.
- Once activated, drives forward blindly for `0.9 m` (monitored via `/odom` displacement).
- This ensures the rover's tail clears the dome exit before turning toward the kit.

**State 2 — Kit Approach:**
- Uses a binary threshold (`kit_threshold_value = 180`) on the front camera.
- Approaches the kit until the contour width reaches a target pixel width or the bottom edge of the contour reaches the image bottom, then stops.

**State 3 — Finished:**
- Publishes zero velocity.
- Calls `mission/complete_dome_exit` to advance the FSM to `ASTRONAUT_SEARCH`.

#### Control Parameters (hardcoded)

| Parameter | Value | Description |
|---|---|---|
| `target_forward_speed` | 1.0 | Forward speed during exit tracking (m/s) |
| `angular_kp` | 0.003 | Proportional gain for centering on door |
| `min_front_contour` | 20000 | Min contour area to trigger push phase (px²) |
| `tripwire_threshold` | 15000 | Min contour area on side cameras to trigger tripwire (px²) |
| `blind_push_distance` | 0.9 | Distance to push forward blindly after tripwire (m) |
| `kit_threshold_value` | 180 | Binary threshold for detecting the equipment kit |

---

### `astronaut_searcher.py` — Astronaut Waypoint Navigation

Drives the rover to a fixed odometry target using a proportional heading controller. **Uses `/gps/odom` for position and `/imu/filtered` for yaw.**

#### ROS Interface

| Direction | Topic / Service | Type |
|---|---|---|
| Subscribed | `rover/mission_state` | `std_msgs/String` |
| Subscribed | `/gps/odom` | `nav_msgs/Odometry` |
| Subscribed | `/imu/filtered` | `sensor_msgs/Imu` |
| Published | `/cmd_vel` | `geometry_msgs/Twist` |
| Client | `mission/complete_astronaut_search` | `std_srvs/srv/Trigger` |

#### Control Algorithm

```python
# Extract yaw from IMU quaternion
yaw = quaternion_to_yaw(msg.orientation) - math.pi / 2

# Compute heading error
dx = TARGET_X - curr_x
dy = TARGET_Y - curr_y
target_yaw = math.atan2(dy, dx)
yaw_error = normalize_angle(target_yaw - yaw)

# Distance to target
distance = math.sqrt(dx**2 + dy**2)

# Angular control (proportional)
angular.z = clamp(yaw_error * angular_kp, -max_angular_speed, max_angular_speed)

# Linear control (speed ramp near target)
if abs(yaw_error) < heading_drive_threshold:
    speed_scale = min(distance / slowdown_distance, 1.0)
    linear.x = max(min_linear_speed, max_linear_speed * speed_scale)
else:
    linear.x = 0.0  # Turn in place until heading is close
```

#### Constants & Parameters (hardcoded)

| Parameter | Value | Description |
|---|---|---|
| `TARGET_X` | 15.5 | Target x position in odom frame |
| `TARGET_Y` | -11.40 | Target y position in odom frame |
| `ARRIVAL_TOLERANCE` | 1.0 | Distance threshold to consider arrived (m) |
| `max_linear_speed` | 1.5 | Maximum forward speed (m/s) |
| `min_linear_speed` | 0.2 | Minimum forward speed (m/s) |
| `max_angular_speed` | 0.8 | Maximum rotation speed (rad/s) |
| `angular_kp` | 1.5 | Proportional gain for yaw correction |
| `slowdown_distance` | 3.0 | Distance at which deceleration begins (m) |
| `heading_drive_threshold` | 0.35 | Max yaw error before driving forward (rad) |

#### Helper Methods (static, shared across all navigator nodes)

```python
@staticmethod
def quaternion_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

@staticmethod
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

@staticmethod
def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))
```

---

### `fuel_trail_follower.py` — Line Follower & Rocket Scanner

Vision-based node that follows a dark fuel trail by HSV color masking, then scans for an orange rocket and images its damage hole. This is the most perception-heavy node.

#### ROS Interface

| Direction | Topic / Service | Type |
|---|---|---|
| Subscribed | `rover/mission_state` | `std_msgs/String` |
| Subscribed | `/cam/front/image_raw` | `sensor_msgs/Image` |
| Published | `/cmd_vel` | `geometry_msgs/Twist` |
| Published | `/vision_task_complete` | `std_msgs/Bool` |
| Published | `/fuel_trail/debug_frame` | `sensor_msgs/Image` |
| Published | `/fuel_trail/debug_mask` | `sensor_msgs/Image` |
| Client | `mission/complete_fuel_trail` | `std_srvs/srv/Trigger` |

#### ROS Parameters (declared)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `linear_speed` | double | 1.0 | Forward speed while following trail (m/s) |
| `kp_line` | double | 0.005 | Proportional gain for trail centering |
| `horizon_ratio` | double | 0.45 | Fraction of image top to ignore (sky/horizon) |

#### HSV Color Profiles

Three hardcoded color profiles for different detection targets. Each can be tuned at runtime via OpenCV trackbars when the debug window is active.

| Profile | Label | Lower HSV | Upper HSV | Purpose |
|---|---|---|---|---|
| `trail` | Dark trail | `[0, 19, 14]` | `[168, 255, 48]` | Fuel trail line following |
| `orange` | Orange rocket | `[10, 97, 20]` | `[28, 128, 255]` | Rocket body detection |
| `hole` | Rocket hole | `[10, 97, 20]` | `[28, 128, 255]` | White damage hole alignment |

#### Internal State Machine (6 states)

**WAITING:**
- Initial state before FSM activation.
- Transitions to `FOLLOW_TRAIL` when global state is `FUEL_TRAIL_FOLLOW`.

**FOLLOW_TRAIL / SEARCH:**
- Converts the front camera frame to HSV.
- Applies the `trail` HSV mask (dark line), cropped below `horizon_ratio`.
- Finds the largest contour and applies proportional steering.
- If no trail contour is detected for 100+ consecutive frames, transitions to `SEARCHING_HOLE`.
- In every frame, also checks for orange rocket detection using the `orange` mask. If orange is found, transitions to `SEARCHING_HOLE`.

**SEARCHING_HOLE:**
- Rotates in place at `0.2 rad/s`.
- Applies the `orange` HSV mask and looks for contours above 2000 px².
- When found, transitions to `ALIGNING_HOLE`.

**ALIGNING_HOLE:**
- Applies the `hole` HSV mask (same values as `orange`).
- Centers the largest white contour using proportional control (gain `0.003`).
- Stops the rover when the hole is centered (error < 20 px).
- Saves the current frame as `rocket_damage_report.jpg`.
- Publishes `True` on `/vision_task_complete`.
- Calls `mission/complete_fuel_trail` service to advance the FSM.
- Transitions to `DONE`.

**DONE:**
- Idle state — zero velocity, no further action.

#### Tuning Interface

When graphical display is available (OpenCV window), trackbars appear for each color profile, allowing real-time tuning of HSV lower/upper bounds. Tuned values are saved per profile via `save_trackbar_values()`.

---

### `oxygen_tank_navigator.py` — Oxygen Tank Waypoint

Structurally similar to `astronaut_searcher.py` — same control law, same helper methods, different target coordinates. **Uses `/gps/odom` and `/imu/filtered`.**

#### ROS Interface

| Direction | Topic / Service | Type |
|---|---|---|
| Subscribed | `rover/mission_state` | `std_msgs/String` |
| Subscribed | `/gps/odom` | `nav_msgs/Odometry` |
| Subscribed | `/imu/filtered` | `sensor_msgs/Imu` |
| Published | `/cmd_vel` | `geometry_msgs/Twist` |
| Client | `mission/complete_oxygen_tank_nav` | `std_srvs/srv/Trigger` |

#### Constants (hardcoded)

| Parameter | Value |
|---|---|
| `TARGET_X` | 25.8 |
| `TARGET_Y` | -6.0 |
| `ARRIVAL_TOLERANCE` | 4.0 |
| `max_linear_speed` | 1.5 |
| `max_angular_speed` | 0.8 |
| `angular_kp` | 1.5 |
| `slowdown_distance` | 3.0 |
| `heading_drive_threshold` | 0.35 |

---

### `dome_return_navigator.py` — Dome Return Waypoint

Structurally similar to the other odom-based navigators. Drives the rover back to the dome. **Uses `/gps/odom` and `/imu/filtered`.**

#### ROS Interface

| Direction | Topic / Service | Type |
|---|---|---|
| Subscribed | `rover/mission_state` | `std_msgs/String` |
| Subscribed | `/gps/odom` | `nav_msgs/Odometry` |
| Subscribed | `/imu/filtered` | `sensor_msgs/Imu` |
| Published | `/cmd_vel` | `geometry_msgs/Twist` |
| Client | `mission/complete_dome_return` | `std_srvs/srv/Trigger` |

#### Constants (hardcoded)

| Parameter | Value |
|---|---|
| `TARGET_X` | -5.0 |
| `TARGET_Y` | 3.5 |
| `ARRIVAL_TOLERANCE` | 0.8 |
| `max_linear_speed` | 2.0 |
| `max_angular_speed` | 0.8 |
| `angular_kp` | 1.5 |
| `slowdown_distance` | 3.0 |
| `heading_drive_threshold` | 0.35 |

---

## Complete ROS Interface Reference

### All Topics

| Direction | Topic | Type | Publisher | Subscribers |
|---|---|---|---|---|
| Published | `rover/mission_state` | `std_msgs/String` | `mission_manager` | All nodes, `ros_bridge` |
| Published | `/cmd_vel` | `geometry_msgs/Twist` | All worker nodes | Diff-drive controller |
| Published | `/airlock_debug` | `sensor_msgs/Image` | `dome_exit` | Debug viewer |
| Published | `/fuel_trail/debug_frame` | `sensor_msgs/Image` | `fuel_trail_follower` | Debug viewer |
| Published | `/fuel_trail/debug_mask` | `sensor_msgs/Image` | `fuel_trail_follower` | Debug viewer |
| Published | `/vision_task_complete` | `std_msgs/Bool` | `fuel_trail_follower` | Downstream consumers |
| Subscribed | `/cam/front/image_raw` | `sensor_msgs/Image` | Camera driver | `dome_exit`, `fuel_trail_follower` |
| Subscribed | `/cam/left/image_raw` | `sensor_msgs/Image` | Camera driver | `dome_exit` |
| Subscribed | `/cam/right/image_raw` | `sensor_msgs/Image` | Camera driver | `dome_exit` |
| Subscribed | `/odom` | `nav_msgs/Odometry` | Wheel odometry | `dome_exit` |
| Subscribed | `/gps/odom` | `nav_msgs/Odometry` | GPS odometry | `astronaut_searcher`, `oxygen_tank_navigator`, `dome_return_navigator` |
| Subscribed | `/imu/filtered` | `sensor_msgs/Imu` | IMU filter | `astronaut_searcher`, `oxygen_tank_navigator`, `dome_return_navigator` |

### All Services

| Service Name | Type | Server | Client(s) |
|---|---|---|---|
| `mission/complete_dome_exit` | `Trigger` | `mission_manager` | `dome_exit`, `ros_bridge` |
| `mission/complete_astronaut_search` | `Trigger` | `mission_manager` | `astronaut_searcher`, `ros_bridge` |
| `mission/complete_fuel_trail` | `Trigger` | `mission_manager` | `fuel_trail_follower`, `ros_bridge` |
| `mission/complete_oxygen_tank_nav` | `Trigger` | `mission_manager` | `oxygen_tank_navigator`, `ros_bridge` |
| `mission/complete_dome_return` | `Trigger` | `mission_manager` | `dome_return_navigator`, `ros_bridge` |
| `mission/emergency_stop` | `Trigger` | `mission_manager` | `ros_bridge` |

---

## Web Dashboard (`webapp/`)

The Flask-based web dashboard provides real-time mission monitoring and control from any browser without requiring a ROS terminal.

### Directory Structure

```
webapp/
├── __init__.py                  # Package marker (empty)
├── app.py                       # Flask application — routes, REST API, Jinja2 templates
├── ros_bridge.py                # ROS 2 background node + thread-safe wrapper
├── node_manager.py              # Subprocess lifecycle manager (start/stop nodes)
├── static/css/dashboard.css     # Bootstrap 5 dark-theme stylesheet (509 lines)
└── templates/
    ├── base.html                # Jinja2 layout — nav bar, sidebar, footer
    ├── index.html               # Main dashboard — state, cameras, logs, node controls
    └── about.html               # Project info page
```

### Running the Web App

```bash
cd webapp
pip install -r ../requirements.txt   # flask>=2.3, opencv-python-headless>=4.8, numpy>=1.24
python app.py
```

Then open **http://127.0.0.1:5000** in a browser.

![Arc Night Dashboard](dashboard.png)

The dashboard polls every **1.5 seconds** for state/node/log updates and every **0.5 seconds** for camera frames.

### REST API Reference

#### State & Mission

| Method | Endpoint | Description |
|---|---|---|
| GET | `/api/state` | Returns `{"state": "DOME_EXIT", "can_advance": true, "next_state": "ASTRONAUT_SEARCH", "valid_states": [...]}` |
| POST | `/api/mission/advance` | Calls the appropriate transition service based on current state |
| POST | `/api/mission/estop` | Calls `mission/emergency_stop` |

#### Node Management

| Method | Endpoint | Description |
|---|---|---|
| GET | `/api/nodes` | Returns status of all managed nodes (running/stopped, PID, uptime) |
| POST | `/api/node/<name>/start` | Spawn a node as a subprocess |
| POST | `/api/node/<name>/stop` | SIGINT → 5s timeout → SIGKILL |
| POST | `/api/node/stop_all` | Stop all managed nodes |

#### Logs

| Method | Endpoint | Description |
|---|---|---|
| GET | `/api/logs` | Returns up to 200 recent log entries with level, name, message, and timestamp |

#### Cameras

| Method | Endpoint | Description |
|---|---|---|
| GET | `/api/camera/<label>` | Returns the latest JPEG bytes for the named camera feed |
| GET | `/api/cameras` | Returns the list of configured `{label, topic}` pairs |
| POST | `/api/cameras/add` | Add a new camera subscription (`{label, topic}`) |
| POST | `/api/cameras/remove` | Remove a camera subscription (`{label}`) |
| POST | `/api/cameras/update` | Change the topic for an existing label |

### Default Camera Topics

| Label | Topic |
|---|---|
| `fuel_trail_debug` | `/fuel_trail/debug_frame` |
| `fuel_trail_mask` | `/fuel_trail/debug_mask` |
| `airlock_debug` | `/airlock_debug` |
| `front_camera` | `/cam/front/image_raw` |

### Node Manager (`node_manager.py`)

Manages ROS nodes as subprocesses. Defines which nodes it can control:

```python
NODES = [
    'mission_manager', 'dome_exit', 'astronaut_searcher',
    'fuel_trail_follower', 'oxygen_tank_navigator', 'dome_return_navigator'
]
```

Key methods:

- **`start(node_name)`** — Sources the workspace (`source install/setup.bash`), then spawns `python3 <node_name>.py` with `preexec_fn=os.setsid` (process group isolation). Reads stdout/stderr line-by-line and feeds logs to the callback.
- **`stop(node_name)`** — Sends SIGINT to the process group. Waits 5 seconds for graceful shutdown, then sends SIGKILL.
- **`get_status(node_name)`** — Returns `{running: bool, pid: int, uptime: str, display_name: str}`.
- **`stop_all()`** — Iterates and stops every managed node.

### ROS Bridge (`ros_bridge.py`)

Runs a ROS 2 node in a background daemon thread using `SingleThreadedExecutor`. The `ROSBridge` wrapper class provides a thread-safe API for Flask.

#### ROSBridgeNode (inner ROS node)

```python
class ROSBridgeNode(Node):
    def __init__(self, shared):
        self.sub_state = self.create_subscription(
            String, 'rover/mission_state', self.state_callback, ...)
```

- **State tracking:** Subscribes to `rover/mission_state` with `TRANSIENT_LOCAL` QoS. On each update, stores the state string in a shared dict.
- **Camera subscriptions:** Dynamically subscribes to any camera topic with `BEST_EFFORT` reliability. Converts `sensor_msgs/Image` → JPEG bytes via `cv_bridge` and `cv2.imencode('.jpg', ...)`. Stores the latest frame per label in the shared dict.
- **Service clients:** Creates clients for all defined transitions at startup.

#### State Transition Mapping

```python
STATE_TRANSITIONS = {
    'DOME_EXIT':            ('mission/complete_dome_exit', 'ASTRONAUT_SEARCH'),
    'ASTRONAUT_SEARCH':     ('mission/complete_astronaut_search', 'FUEL_TRAIL_FOLLOW'),
    'FUEL_TRAIL_FOLLOW':    ('mission/complete_fuel_trail', 'OXYGEN_TANK_NAV'),
    'OXYGEN_TANK_NAV':      ('mission/complete_oxygen_tank_nav', 'DOME_RETURN'),
    'DOME_RETURN':          ('mission/complete_dome_return', 'SEQUENCE_COMPLETE'),
}
```

Note: `BOOTING` is not in the bridge transitions — the auto-transition from `BOOTING` to `DOME_EXIT` happens inside `mission_manager.py`'s `main()`, so the bridge only handles transitions from `DOME_EXIT` onward.

---

## Important Notes

- **Startup order:** `mission_manager` must be started first — it publishes the initial `BOOTING`/`DOME_EXIT` state. Worker nodes subscribe to `rover/mission_state` and will not activate until their assigned state appears.
- **Odometry sources:** `dome_exit.py` uses `/odom` (wheel odometry) for the blind push distance. The three navigator nodes (`astronaut_searcher`, `oxygen_tank_navigator`, `dome_return_navigator`) use `/gps/odom` for position and `/imu/filtered` for yaw (with a `-π/2` offset applied to the IMU heading).
- **Fuel trail advancement:** The `fuel_trail_follower` calls the `mission/complete_fuel_trail` service itself upon completing the rocket imaging, so no manual advancement is required.
- **Dead code / orphaned files:** `align_and_move_node.py` and `final_turn_node.py` do not exist on disk (only `__pycache__` bytecode remains). They are vestiges of an earlier mission phase and cannot be reached by the current FSM.
- **No `__init__.py`** exists in `arc_night_mission/` itself — only `webapp/__init__.py` exists (marking the webapp as a Python package). The mission nodes are designed to be run as standalone scripts, not imported as a package.

---

## Dependencies

### ROS 2 Packages
- `rclpy` — ROS 2 Python client library
- `std_msgs` — `String`, `Bool`
- `std_srvs` — `Trigger`
- `geometry_msgs` — `Twist`
- `nav_msgs` — `Odometry`
- `sensor_msgs` — `Image`, `Imu`
- `cv_bridge` — ROS ↔ OpenCV image conversion

### Python Packages (from `requirements.txt`)
- `flask>=2.3` — Web framework
- `opencv-python-headless>=4.8` — Computer vision
- `numpy>=1.24` — Numerical operations

---

## Running the Full Mission

### Step 1 — Launch the Rover Simulation

```bash
ros2 launch rover_description full_autonomy.launch.py world:=mars
```

Verify that `/odom`, `/gps/odom`, `/imu/filtered`, camera topics, and `/cmd_vel` are active:

```bash
ros2 topic list
```

### Step 2 — Source the Workspace

```bash
source install/setup.bash
```

### Step 3 — Launch the Arc Night Mission

Open a **second terminal**, source the workspace, and launch the Arc Night Mission:

```bash
source install/setup.bash
ros2 launch rover_description arc_night_mission.launch.py
```

#### Option B: Web Dashboard

```bash
cd arc_night_mission/webapp
python app.py
```

Then go to **http://localhost:5000** or the printed IP address. From the dashboard, start each node via the UI, or start them all and monitor progress.
