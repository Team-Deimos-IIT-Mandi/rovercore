cmake_minimum_required(VERSION 3.22)
project(armmoveit)

find_package(ament_cmake REQUIRED)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch scripts models
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# ── YOUR NEW EXECUTABLE SCRIPT REGISTRATION ──
install(PROGRAMS
  scripts/control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fullcontrol.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/manip.py
  DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

# ── THIS MUST ALWAYS BE THE ABSOLUTE LAST LINE ──
ament_package()