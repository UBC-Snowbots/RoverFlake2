cmake_minimum_required(VERSION 3.8)
project(drive_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(rover_arm_common REQUIRED) # needed by controller_config.h (motor_addressing.h)

# mjbots moteus C++ client library (header-only, exposes moteus::cpp target)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external_pkgs/moteus moteus_build)

# Include directories (header files)
include_directories(include)

# Define executables
add_executable(drive_control_node src/drive_control.cpp)
add_executable(motor_control_node src/motor_control.cpp)
add_executable(wheel_speed_node src/wheel_speed.cpp)

# Link dependencies for drive_control_node
ament_target_dependencies(drive_control_node
  rclcpp
  geometry_msgs
  sensor_msgs
  rover_arm_common
)

# Gamepad mappings + process_joy_input live in arm_control's controller_config.h.
# Header-only use, so include it straight from the source tree (same pattern as
# arm_hardware_interface's COMMS_DIR) instead of find_package(arm_control),
# which would drag in the whole MoveIt dependency stack.
target_include_directories(drive_control_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../arm_control/include
)

# Link dependencies for motor_control_node
ament_target_dependencies(motor_control_node
  rclcpp
  geometry_msgs
  sensor_msgs
  rover_msgs
)

# Link dependencies for wheel_speed_node
ament_target_dependencies(wheel_speed_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs  # Added std_msgs
  rover_msgs
)

# Link the moteus client library
target_link_libraries(motor_control_node moteus::cpp)

# Set include directories
target_include_directories(drive_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(motor_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(wheel_speed_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the launch directory
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install both executables
install(TARGETS
  drive_control_node
  motor_control_node  
  wheel_speed_node
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Required for ROS 2 packages
ament_package()
