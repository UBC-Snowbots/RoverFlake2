cmake_minimum_required(VERSION 3.5)
project(control_base_hardware_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(rover_msgs REQUIRED)

include_directories(
  include
)

# Add executable
add_executable(arduino_serial_publisher src/arduino_serial_publisher.cpp)
ament_target_dependencies(arduino_serial_publisher rclcpp std_msgs serial rover_msgs)

# Install
install(TARGETS
  arduino_serial_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()