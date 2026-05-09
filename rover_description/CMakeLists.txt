cmake_minimum_required(VERSION 3.8)
project(rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  PROGRAMS
    scripts/teleop.py
    scripts/cmd_vel_relay.py
    scripts/aruco_detection.py
    scripts/spiral_search.py
    scripts/mission_initializer.py
    scripts/robot_description_publisher.py
    scripts/optical_flow_node.py
    scripts/imu_filter_node.py
    scripts/flow_derotation_node.py
    scripts/nonholonomic_node.py
    scripts/slip_detector_node.py
    scripts/gps_gating_node.py
    scripts/zupt_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf meshes launch rviz config worlds maps models
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
