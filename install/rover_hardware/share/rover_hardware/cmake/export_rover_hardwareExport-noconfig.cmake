#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rover_hardware::rover_hardware" for configuration ""
set_property(TARGET rover_hardware::rover_hardware APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rover_hardware::rover_hardware PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "hardware_interface::hardware_interface;rclcpp::rclcpp;rclcpp_lifecycle::rclcpp_lifecycle"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librover_hardware.so"
  IMPORTED_SONAME_NOCONFIG "librover_hardware.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rover_hardware::rover_hardware )
list(APPEND _IMPORT_CHECK_FILES_FOR_rover_hardware::rover_hardware "${_IMPORT_PREFIX}/lib/librover_hardware.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
