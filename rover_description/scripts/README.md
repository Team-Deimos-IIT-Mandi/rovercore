# Rover Description — ROS 2 Nodes

This directory contains **30 ROS 2 Python nodes** and a sub-package (`arc_night_mission/`) that implement the autonomy, state estimation, hardware drivers, computer vision, and science pipeline for the **Assem10 six-wheeled Mars rover** (ROS 2 Humble).

---

## Navigation & Teleoperation

| Node | Description |
|---|---|
| `cmd_vel_relay.py` | Relays `/cmd_vel` to `/diff_drive_controller/cmd_vel_unstamped` for the diff-drive controller. |
| `teleop.py` | Keyboard teleop (WASD) publishing `/cmd_vel`. |
| `joy_teleop.py` | Joystick teleoperation node. |
| `mission_initializer.py` | Converts GPS lat/lon parameters into a map-frame `/goal_pose` for Nav2; signals `/start_search`. |
| `spiral_search.py` | Generates spiral waypoints from the rover's start position, sending sequential goals until an ArUco marker is found. |
| `gate_detection_node.py` | Detects gates/poles for navigation through narrow passages. |
| `gps_nav_node.py` | GPS-aided waypoint navigation helper. |

## State Estimation & EKF Constraints

| Node | Description |
|---|---|
| `nonholonomic_node.py` | Publishes non-holonomic constraint (`Vy=0`, `Vz=0` at 50 Hz) for `robot_localization` EKF. |
| `zupt_node.py` | Zero-Velocity Update — detects stationary rover via IMU gyro + wheel speed, publishes zero-velocity constraint. |
| `slip_detector_node.py` | Compares wheel odometry vs. optical flow; inflates covariance on slip. |
| `imu_filter_node.py` | Notch filter for IMU data; publishes filtered `/imu/filtered`. |
| `flow_derotation_node.py` | Removes gyroscopic rotation bias from optical flow velocity using IMU angular velocity. |
| `joint_state_restamper.py` | Re-stamps `/joint_states_raw` timestamps with the current ROS clock to fix Gazebo bridge jitter. |
| `robot_description_publisher.py` | Publishes URDF string as a latched `/robot_description` topic (bridges `ros_gz_sim` `create`). |

## GPS Drivers & Processing

| Node | Description |
|---|---|
| `gps_driver_node.py` | NMEA serial GPS driver — parses `$GNGGA`/`$GNRMC`, publishes `/gps/fix` and `/gps/vel`. |
| `gps_fix_covariance_node.py` | Injects sensible default covariance into GPS NavSatFix (Gazebo bridge outputs zero covariance). |
| `gps_gating_node.py` | Mahalanobis 3-sigma gate on GPS; compares `/gps/odom` vs. `/odometry/local`, publishes `/gps/odom_gated`. |

## IMU & Optical Flow Hardware Drivers

| Node | Description |
|---|---|
| `icm20948_driver_node.py` | Hardware driver for ICM-20948 IMU over I2C; publishes `/imu/data`. |
| `mtf01_driver_node.py` | Driver for MTF-01 optical flow + rangefinder over serial (MSP v2 protocol). |
| `mtf01_inspector.py` | Debug/inspection tool for MTF-01 sensor data. |
| `optical_flow_node.py` | Simulates PMW3901 optical flow using Farneback dense optical flow on a downward camera + rangefinder height. |

## Computer Vision

| Node | Description |
|---|---|
| `aruco_detection.py` | Detects ArUco markers (DICT_4X4_50) via OpenCV `solvePnP`; publishes `/AR` signal and `/goal_pose`. |
| `fuel_trail_follower.py` | Vision-based fuel trail follower using HSV color masking and OpenCV contour tracking. |
| `task_fuel_trail.py` | Higher-level state machine for the fuel trail task (WAITING → FOLLOWING → COMPLETE). |

## Drone Collaboration

| Node | Description |
|---|---|
| `drone_aruco_relay.py` | Receives ArUco detections from a companion drone, transforms to the rover map frame, and publishes as Nav2 goals. |
| `drone_map_ingestor.py` | Receives aerial occupancy grid tiles from a drone and republishes as a Nav2 costmap layer. |

## Science Mission

| Node | Description |
|---|---|
| `science_mission.py` | Autonomous science pipeline state machine: IDLE → NAV → PROBE → DRILL → SENSE_SOIL → SPECTROMETER → MICROSCOPE → LOG → RETRACT → DONE. |
| `science_logger.py` | Logs science data (GPS, odometry, soil readings) to CSV on a `/science/sample` service call. |
| `pick_place_server.py` | Pick-and-place state machine: NAV_TO_PREGRASP → PICK → STOW → NAV_TO_DROP → PLACE → DONE. |

## Safety & Terrain

| Node | Description |
|---|---|
| `slope_detector.py` | Terrain safety monitor — reads IMU orientation, computes roll/pitch, publishes tilt warnings and E-STOP signals. |

## Arc Night Mission (Sub-package)

`arc_night_mission/` is the **main mission package** for the Arc Night competition track. It contains a self-contained mission pipeline (dome exit → astronaut search → fuel trail → oxygen tank → dome return) and a **Flask web dashboard** for real-time rover control:

| Node | Description |
|---|---|
| `mission_manager.py` | Central mission FSM coordinating the full sequence. |
| `dome_exit.py` | Vision + blind push handling for exiting the starting airlock/dome. |
| `astronaut_searcher.py` | Drives to an odometry target to locate the astronaut. |
| `fuel_trail_follower.py` | HSV-based line tracker for the fuel trail. |
| `oxygen_tank_navigator.py` | Odometry-target drive to reach the oxygen tank. |
| `dome_return_navigator.py` | Odometry-target drive to return to the dome. |
| `align_and_move_node.py` | Precision alignment and movement node. |
| `final_turn_node.py` | Final turn maneuver to conclude the mission. |

### Running the Arc Night Mission

**Step 1 — Terminal 1: Launch the full autonomy stack:**
```bash
ros2 launch rover_description full_autonomy.launch.py world:=mars
```

This starts the rover in a Mars environment with `/odom`, camera feeds (`/cam/front/image_raw`, `/cam/left/image_raw`, `/cam/right/image_raw`), and `/cmd_vel` active.

**Step 2 — Terminal 2: Launch the Arc Night Mission (Option A: Terminal Launch):**
```bash
source install/setup.bash
ros2 launch rover_description arc_night_mission.launch.py
```

### Web Dashboard

The `webapp/` directory inside `arc_night_mission/` provides a **Flask web dashboard** that lets you monitor and control the rover from a browser.

**To start the web app:**

```bash
cd arc_night_mission/webapp
python app.py
```

Then open the displayed URL (typically `http://127.0.0.1:5000`) in a browser to view telemetry, send commands, and monitor the Arc Night mission state.

---

## Dependencies

- **ROS 2 Humble** — `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2_ros`, etc.
- **Navigation** — `nav2_bringup`, `robot_localization`, `cartographer_ros`
- **Simulation** — `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `xacro`, `rviz2`
- **Control** — `controller_manager`, `diff_drive_controller`, `joint_state_broadcaster`
- **Python** — `opencv-python`, `cv_bridge`, `numpy`, `scipy`, `utm`, `pyserial`, `smbus2`, `flask`

See `package.xml` in the parent directory for the full dependency list.

---

## Naming Conventions

- **Files & nodes:** `snake_case` (e.g., `cmd_vel_relay.py`, `class CmdVelRelay`)
- **Classes:** `PascalCase`
- **Methods & functions:** `snake_case`; private methods prefixed with `_`
- **Topics:** `/lowercase/with_slashes`
- **Constants:** `UPPER_CASE`
- **Parameters:** Declared via `self.declare_parameter('param_name', default)` in `__init__`

---

## Common Patterns

All nodes follow the same boilerplate:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class SomeNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # declare parameters, publishers, subscribers, timers, service clients

def main(args=None):
    rclpy.init(args=args)
    node = SomeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

State machines (`science_mission.py`, `pick_place_server.py`) extract core logic into pure-Python classes independent of ROS for testability.
