cmake_minimum_required(VERSION 3.8)
project(quadruped_gait_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# FIND DEPENDENCIES
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ============================================================================
# INCLUDE DIRECTORIES
# ============================================================================
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ============================================================================
# GAIT CONTROLLER NODE
# ============================================================================
add_executable(gait_controller 
  src/gait_controller.cpp 
  src/trajecgen.cpp 
  src/Kinematics.cpp
)

ament_target_dependencies(gait_controller
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# ============================================================================
# JOINT COMMAND RELAY NODE (Convertit /joint_commands → /joint_states)
# ============================================================================
add_executable(joint_command_relay 
  src/joint_command_relay.cpp
)

ament_target_dependencies(joint_command_relay
  rclcpp
  std_msgs
  sensor_msgs
)

# ============================================================================
# INSTALL EXECUTABLES
# ============================================================================
install(TARGETS
  gait_controller
  joint_command_relay
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# INSTALL DIRECTORIES
# ============================================================================

# Headers
install(DIRECTORY include/
  DESTINATION include/
)

# Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# RViz configs
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Meshes (si vous en avez)
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Config files (si vous en avez)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# ============================================================================
# TESTING
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
