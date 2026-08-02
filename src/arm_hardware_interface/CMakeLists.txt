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

# Where comms.h / device_ids.h live. Only the header is needed (we use the
# MSG_TYPE__* ids and the id masks), nothing from comms.c gets linked.
# Override with: colcon build --cmake-args -DCOMMS_INCLUDE_DIR=/path/to/comms
set(COMMS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../FirmFlake/common_libraries/comms"
CACHE PATH "Directory containing comms.h and device_ids.h")

if(NOT EXISTS "${COMMS_DIR}/comms.h")
message(FATAL_ERROR
"comms.h not found in ${COMMS_DIR}. "
"Set -DCOMMS_INCLUDE_DIR=<dir containing comms.h>.")
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external_pkgs/moteus moteus_build)

include_directories(include ${COMMS_DIR})

# CAN-FD driver for arm
add_executable(moteus_driver src/moteus_driver_node.cpp)
target_link_libraries(moteus_driver moteus::cpp)
ament_target_dependencies(moteus_driver rclcpp rover_msgs std_msgs sensor_msgs rover_arm_common)


# Socet CAN driver for other loops
add_executable(generic_can_hardware_interface src/generic_can_hardware_interface.cpp src/socket_can.cpp ${COMMS_DIR}/comms.c)
# target_link_libraries(generic_can_hardware_interface )?
ament_target_dependencies(generic_can_hardware_interface rclcpp rover_msgs)

add_executable(lighting_translator
  src/lighting_translator.cpp
  ${COMMS_DIR}/comms.c
)
ament_target_dependencies(lighting_translator rclcpp rover_msgs)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY include/ DESTINATION include/)

#? Not too sure about this...
install(PROGRAMS launch/can_bringup.sh
  DESTINATION share/${PROJECT_NAME}/launch)


install(TARGETS 
    moteus_driver
    generic_can_hardware_interface
    lighting_translator
    DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
