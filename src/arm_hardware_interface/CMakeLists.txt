cmake_minimum_required(VERSION 3.8)
project(arm_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rover_arm_common REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(include)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external_pkgs/moteus moteus_build)

# CAN-FD driver executable
add_executable(moteus_driver src/moteus_driver_node.cpp)
target_link_libraries(moteus_driver moteus::cpp)
ament_target_dependencies(moteus_driver rclcpp rover_msgs std_msgs sensor_msgs rover_arm_common)

# MoveIt Servo output -> /arm/command (shadow mode by default)
add_executable(servo_to_arm_bridge src/servo_to_arm_bridge.cpp)
ament_target_dependencies(servo_to_arm_bridge rclcpp rover_msgs trajectory_msgs rover_arm_common)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY include/ DESTINATION include/)

install(TARGETS moteus_driver servo_to_arm_bridge
    DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
