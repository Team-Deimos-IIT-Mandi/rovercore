# RoverCore World File — Mars Rover Autonomy Stack

A ROS 2 (Humble) workspace for **Assem10**, a 4-wheel differential-drive Mars rover with simulation, hardware drivers, multi-sensor autonomy, a 6-DOF robotic arm.

## Robot Platform

| Attribute | Specification |
|---|---|
| Drive | 4-wheel differential (continuous joints), skid-steer |
| Chassis mass | 9.9 kg |
| Wheel radius | 0.14 m |
| Wheel separation | 0.62 m |
| Total mass (est.) | ~20.6 kg |
| Compute | Jetson Orin NX (rover) + Raspberry Pi 5 (arm) + CM4 (science sensors) |
| Real HW bus | CAN-FD 5 Mbit/s (AK80-9 motors), UART sensors, GPIO safety |

### Sensor Suite

| Sensor | Qty | Specs |
|---|---|---|
| RGBD camera (depth) | 1 | 640×480, 1.047 rad HFOV, 0.1–100 m |
| RGB camera (front) | 1 | 1280×720, 80° HFOV, spot headlight |
| RGB camera (FL, FR, left, right, rear) | 5 | 640×480, 90° HFOV |
| Science camera | 1 | 1280×720, 70° HFOV, 0.05–10 m close-range |
| Downward optical flow | 1 | 120×120 px |
| Rangefinder (lidar) | 1 | 1 beam, 0.01–8 m |
| IMU (ICM-20948) | 1 | 200 Hz, I²C addr 0x69 |

### State Estimation

**Dual extended Kalman filter** (`robot_localization`) fusing 5 sensor modalities:

- **EKF Local** (`odom` frame, 30 Hz) — IMU (notch-filtered, gravity-removed) + wheel odometry (vx) + optical flow (vx, vy) + nonholonomic constraint (vy=0, vz=0) + ZUPT
- **EKF Global** (`map` frame, 30 Hz) — IMU (relative) + local odometry (vx, vy, yaw_vel) + gated GPS (x/y, 3σ Mahalanobis outlier rejection)
- **NavSat transform** — GPS lat/lon → map-frame XY via UTM, with datum [39.8942, 32.7845, 900.0]

## Workspace Layout

```
.
├── rover_description/       # Main ROS 2 package
│   ├── scripts/             # 41 Python nodes (see below)
│   ├── launch/              # 7 launch files
│   ├── config/              # 14 configuration files
│   ├── urdf/                # Robot model (URDF/XACRO)
│   ├── meshes/              # STL mesh files
│   ├── worlds/              # Simulation world files
│   ├── models/              # Gazebo model assets
│   ├── maps/                # Cartographer SLAM maps
│   ├── rviz/                # RViz2 config
│   └── tests/               # Unit tests
├── rover_hardware/          # C++ ros2_control serial interface plugin
├── models/                  # Top-level model assets
├── worlds/                  # Top-level world files
├── ws_moveit_backup/        # MoveIt 2 source dependencies
├── build/                   # Colcon build artifacts
├── install/                 # Colcon install space
└── log/                     # Build logs
```

## Launch Files

### `sim.launch.py`
Start the rover in a Gazebo simulation world.

| Argument | Default | Options |
|---|---|---|
| `world` | `depot` | `depot`, `mars`, `moon`, `mars_array`, `moon_array` |

Launches: `robot_state_publisher`, `ros_gz_sim` Gazebo, ROS-Gazebo bridge (18 topics), sensor TF publishers, joint state restamper, GPS covariance injector, RViz2.

### `full_autonomy.launch.py`
Full autonomous mission stack.

| Argument | Default | Description |
|---|---|---|
| `world` | `depot` | Simulation world |
| `arm_enabled` | `false` | Enable arm pick-and-place |
| `target_lat` | `39.8940850` | Mission target latitude |
| `target_lon` | `32.7846688` | Mission target longitude |

Launches: `sim.launch.py` + `state_estimation.launch.py` + `nav2.launch.py` + mission initializer, spiral search, ArUco detection, cmd_vel relay.

### `state_estimation.launch.py`
Dual EKF + sensor fusion. Launches EKF nodes, NavSat transform, sensor conditioning (nonholonomic constraint, slip detector, ZUPT, GPS gating). When `use_sim_time:=false`, also launches real hardware sensor drivers.

### `sensors_hw.launch.py`
Real hardware sensor drivers.

| Argument | Default | Description |
|---|---|---|
| `gps_port` | `/dev/rover_gps` | NMEA GPS serial port |
| `flow_port` | `/dev/rover_flow` | MTF-01 optical flow serial port |
| `imu_bus` | `1` | I²C bus |
| `imu_addr` | `105` (0x69) | IMU I²C address |
| `imu_rate_hz` | `100.0` | IMU publish rate |

Launches: GPS NMEA driver, ICM-20948 I²C driver, IMU notch filter, MTF-01 MSP serial driver, flow derotation.

### `nav2.launch.py`
Nav2 navigation stack with custom behavior tree. Launches Nav2 bringup, depth-to-pointcloud conversion.

### `slam.launch.py`
Cartographer SLAM.

| Argument | Description |
|---|---|
| `mode` | `mapping` or `localization` |
| `map_file_path` | Path to pre-built map for localization mode |

### `drone_bringup.launch.py`
MAVROS drone companion node. Launches MAVROS, ArUco relay, and aerial map ingestor for drone-rover collaboration.

## Python Nodes (41 scripts)

### Simulation & Hardware Support
- `optical_flow_node.py` — PMW3901 simulator (Farneback flow + SQUAL covariance)
- `imu_filter_node.py` — Notch filter (45 Hz) for motor vibration rejection
- `flow_derotation_node.py` — De-rotate optical flow by IMU gyro (pitch/roll)
- `nonholonomic_node.py` — Enforce Vy=0, Vz=0 constraint for car-like motion
- `slip_detector_node.py` — Detect wheel slip via flow vs odometry comparison
- `zupt_node.py` — Zero-velocity update when stationary
- `gps_gating_node.py` — 3σ Mahalanobis outlier rejection on GPS
- `gps_fix_covariance_node.py` — Inject default GPS covariance
- `joint_state_restamper.py` — Re-stamp joint states with sim clock
- `robot_description_publisher.py` — Publish robot_description as latched topic
- `cmd_vel_relay.py` — Relay Nav2 `/cmd_vel` to diff_drive_controller

### Hardware Drivers
- `icm20948_driver_node.py` — I²C ICM-20948 IMU driver
- `mtf01_driver_node.py` — MSP v2 serial driver for MTF-01 optical flow/rangefinder
- `mtf01_inspector.py` — Standalone MSP packet inspector (non-ROS)
- `gps_driver_node.py` — NEMA serial GPS driver ($GNGGA/$GNRMC)

### Navigation & Autonomy
- `gps_nav_node.py` — GPS waypoint sequencer (lat/lon → UTM → Nav2 action)
- `spiral_search.py` — Expanding spiral waypoint generator
- `mission_initializer.py` — Send initial Nav2 goal from GPS datum
- `gate_detection_node.py` — LiDAR-based gate detection (Euclidean clustering of 2 posts)
- `slope_detector.py` — Terrain safety: roll/pitch monitoring, dynamic speed limiting

### Vision & Perception
- `aruco_detection.py` — ArUco marker detection with PnP pose estimation
- `fuel_trail_follower.py` — HSV-based fuel trail line following
- `task_fuel_trail.py` — Vision worker for fuel trail (ROS action)

### Mission State Machines
- `science_mission.py` — Autonomous science pipeline: IDLE → NAV → PROBE → DRILL → SENSE → SPECTROMETER → MICROSCOPE → LOG → RETRACT → DONE
- `science_logger.py` — Science data CSV logging service
- `pick_place_server.py` — Pick-and-place: IDLE → NAV_TO_PREGRASP → PICK → STOW → NAV_TO_DROP → PLACE → DONE

### Teleoperation
- `joy_teleop.py` — PS4/Xbox joystick → Twist (deadman button)
- `teleop.py` — Keyboard WASD → Twist

### Arc Night Mission (`arc_night_mission/`)
Competition mission state machine with Flask web dashboard.

**Mission FSM:** BOOTING → DOME_EXIT → ASTRONAUT_SEARCH → FUEL_TRAIL_FOLLOW → OXYGEN_TANK_NAV → DOME_RETURN → SEQUENCE_COMPLETE

| Node | Purpose |
|---|---|
| `mission_manager.py` | Central mission FSM, state transitions |
| `dome_exit.py` | Vision-guided airlock door exit |
| `astronaut_searcher.py` | Navigate to fixed astronaut coordinate (x=-14, y=16) |
| `fuel_trail_follower.py` | HSV fuel trail line following (FSM-integrated) |
| `oxygen_tank_navigator.py` | Navigate to oxygen tank (x=25.8, y=-6.0) |
| `dome_return_navigator.py` | Navigate to dome (x=-5.0, y=3.5) |

**Web Dashboard** (`arc_night_mission/webapp/`):
- `app.py` — Flask server: dashboard UI, REST API, camera streams, node management
- `ros_bridge.py` — ROS 2 subscriber/bridge thread
- `node_manager.py` — Subprocess launcher for mission nodes

## Robot Model (URDF)

| File | Description |
|---|---|
| `Assem10.urdf` | Primary simulation model (773 lines): chassis, 4 wheels, all sensors, Gazebo plugins |
| `rover.urdf` | Earlier model variant (819 lines) with rocker-bogie hints |
| `rover_with_arm.urdf.xacro` | Combined rover + 6-DOF arm + gripper (finger 1/2) |
| `Assem10_real.urdf.xacro` | Real hardware variant with `RoverSerialInterface` plugin on `/dev/ttyTHS0` |

**Frame tree:** `map` → `odom` → `base_link` → `chassis_link` → sensor frames

**Gazebo plugins:** DiffDrive (`/model/Rover/cmd_vel`, 50 Hz odom), JointStatePublisher, ros2_control controller manager.

## Configuration Files

| File | Purpose |
|---|---|
| `nav2_params.yaml` | Full Nav2 stack: AMCL, BT navigator, controller server (FollowPath, 20 Hz, 0.3m tolerance), planners (GridBased, Smoother), global/local costmaps |
| `rover_controllers.yaml` | `diff_drive_controller` (L/R wheel pairs, 0.5874m separation, 0.11m radius) |
| `combined_controllers.yaml` | Merged diff_drive + arm_controller 6-DOF + gripper controller |
| `hardware_params.yaml` | CAN-FD bus (can0, 5 Mbit/s, motor IDs 0x01–0x14), GPIO pins, UART ports, I²C addresses |
| `ekf_local.yaml` | EKF Local: 30 Hz, 2D mode, IMU + wheel odom + flow + nonholonomic + ZUPT |
| `ekf_global.yaml` | EKF Global: 30 Hz, 2D mode, IMU + local odom + gated GPS, 0.5s lag smoothing |
| `navsat.yaml` | NavSat transform: yaw offset correction, datum config, odometry yaw enabled |
| `cartographer.lua` | Cartographer 2D SLAM: 0.05m grid, CSM + Ceres scan matching, loop closure every 90 nodes |
| `mission_config.yaml` | Tunable parameters: acceptance radii, speeds, ArUco/gate/slope/thresholds, watchdog timeouts, arm/ science durations |
| `mission_waypoints.yaml` | ARC 2026 GPS waypoints (approach, gate, search, science) |
| `imu_calibration.yaml` | Notch filter: 45 Hz center, 5 Hz bandwidth |
| `mavros_params.yaml` | MAVROS UDP config, plugin whitelist, TF frames |
| `joint_names_Assem10.yaml` | Arm joint names: `LF, RF, LB, RB` |
| `navigate_and_search_bt.xml` | Custom Nav2 BT: 6 retries, replan+follow, recovery (clear costmaps, spin, wait, backup) |

## Simulation Worlds

| World | File |
|---|---|
| Mars terrain | `mars.world.sdf` |
| Empty / depot | `empty.world` |
| Agriculture (barn + solar farm) | `agriculture.world` |
| Agriculture model | `models/cpr_agriculture/` (Collada mesh, wood/metal/solar textures) |
| Gale Crater patch | `models/gale_crater_patch2/` |

Additional models: `fuel_trail/`, `ramp.stl`, `rocks/`.

## Hardware Interface

`rover_hardware/RoverSerialInterface` — C++ `ros2_control` `hardware_interface::SystemInterface` plugin.

- Serial UART on `/dev/ttyTHS0`
- Real-time CAN-FD communication (5 Mbit/s) with motor controllers
- Replaces Gazebo IgnitionSystem plugin for physical rover deployment

## MoveIt 2 Backup

`ws_moveit_backup/` — Separate colcon workspace (COLCON_IGNORE'd from root) containing MoveIt 2 source for arm motion planning:

- `moveit2/` — Main MoveIt 2 library
- `moveit2_tutorials/`, `moveit_resources/`, `moveit_visual_tools/`
- `moveit_task_constructor/`, `rosparam_shortcuts/`, `srdfdom/`, `launch_param_builder/`

## Quick Start

```bash
# Source the workspace
source install/setup.bash

# Launch in Mars simulation world
ros2 launch rover_description sim.launch.py world:=mars

# Full autonomy with dual EKF, Nav2, and mission nodes
ros2 launch rover_description full_autonomy.launch.py world:=mars

# SLAM mapping mode
ros2 launch rover_description slam.launch.py mode:=mapping

# Real hardware sensor drivers
ros2 launch rover_description sensors_hw.launch.py

# Arc Night mission dashboard
ros2 run rover_description arc_night_mission.webapp.app
```

## Running Tests

```bash
colcon test --packages-select rover_description
```

## Dependencies

| Category | Packages |
|---|---|
| **ROS 2 Core** | `robot_state_publisher`, `joint_state_publisher_gui`, `xacro`, `rviz2`, `tf2_ros`, `std_msgs`, `geometry_msgs` |
| **Simulation** | `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces` |
| **Control** | `controller_manager`, `diff_drive_controller`, `joint_state_broadcaster`, `ros2_control` |
| **Localization** | `robot_localization`, `cartographer_ros` |
| **Navigation** | `nav2_bringup`, `nav2_common`, `nav2_msgs` |
| **Perception** | `cv_bridge`, `python3-opencv`, `depth_image_proc` |
| **Python** | `numpy`, `scipy`, `utm`, `flask`, `rclpy` |
| **Arm (MoveIt)** | `moveit2`, `moveit_task_constructor`, `srdfdom` |
| **Drone** | `mavros` |

## Package Metadata

- **Package name:** `rover_description`
- **Version:** `0.0.0`
- **License:** Apache-2.0
- **Maintainer:** Mars Rover Team
- **Build type:** `ament_cmake` (with `ament_cmake_python`)
