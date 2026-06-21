# Install script for directory: /home/remandey/my-programs/rovercore/rover_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/remandey/my-programs/rovercore/install/rover_description")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rover_description" TYPE PROGRAM FILES
    "/home/remandey/my-programs/rovercore/rover_description/scripts/teleop.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/cmd_vel_relay.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/aruco_detection.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/spiral_search.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/mission_initializer.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/arc_night_mission/mission_manager.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/arc_night_mission/dome_exit.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/arc_night_mission/astronaut_searcher.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/arc_night_mission/oxygen_tank_navigator.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/arc_night_mission/dome_return_navigator.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/arc_night_mission/fuel_trail_follower.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/robot_description_publisher.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/optical_flow_node.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/imu_filter_node.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/flow_derotation_node.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/nonholonomic_node.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/slip_detector_node.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/gps_gating_node.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/gps_fix_covariance_node.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/zupt_node.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/gps_driver_node.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/icm20948_driver_node.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/mtf01_driver_node.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/joint_state_restamper.py"
    "/home/remandey/my-programs/rovercore/rover_description/scripts/joy_teleop.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rover_description" TYPE DIRECTORY FILES
    "/home/remandey/my-programs/rovercore/rover_description/urdf"
    "/home/remandey/my-programs/rovercore/rover_description/meshes"
    "/home/remandey/my-programs/rovercore/rover_description/launch"
    "/home/remandey/my-programs/rovercore/rover_description/rviz"
    "/home/remandey/my-programs/rovercore/rover_description/config"
    "/home/remandey/my-programs/rovercore/rover_description/worlds"
    "/home/remandey/my-programs/rovercore/rover_description/maps"
    "/home/remandey/my-programs/rovercore/rover_description/models"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/remandey/my-programs/rovercore/build/rover_description/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/rover_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/remandey/my-programs/rovercore/build/rover_description/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/rover_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rover_description/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rover_description/environment" TYPE FILE FILES "/home/remandey/my-programs/rovercore/build/rover_description/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rover_description/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rover_description/environment" TYPE FILE FILES "/home/remandey/my-programs/rovercore/build/rover_description/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rover_description" TYPE FILE FILES "/home/remandey/my-programs/rovercore/build/rover_description/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rover_description" TYPE FILE FILES "/home/remandey/my-programs/rovercore/build/rover_description/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rover_description" TYPE FILE FILES "/home/remandey/my-programs/rovercore/build/rover_description/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rover_description" TYPE FILE FILES "/home/remandey/my-programs/rovercore/build/rover_description/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rover_description" TYPE FILE FILES "/home/remandey/my-programs/rovercore/build/rover_description/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/remandey/my-programs/rovercore/build/rover_description/ament_cmake_index/share/ament_index/resource_index/packages/rover_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rover_description/cmake" TYPE FILE FILES
    "/home/remandey/my-programs/rovercore/build/rover_description/ament_cmake_core/rover_descriptionConfig.cmake"
    "/home/remandey/my-programs/rovercore/build/rover_description/ament_cmake_core/rover_descriptionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rover_description" TYPE FILE FILES "/home/remandey/my-programs/rovercore/rover_description/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/remandey/my-programs/rovercore/build/rover_description/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
