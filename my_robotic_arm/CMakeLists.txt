cmake_minimum_required(VERSION 3.8)
project(my_robotic_arm)  

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies needed for this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

# Install all simulation assets in one clean block
install(DIRECTORY
  config
  launch
  meshes
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()