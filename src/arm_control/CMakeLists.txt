cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  rover_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_servo
  moveit_ros_planning
  pluginlib
  rclcpp
  rclcpp_components
  realtime_tools
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs
  rover_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# This library provides a way of loading parameters for servo
# generate_parameter_library(
#   moveit_servo_lib_parameters
#   config/servo_parameters.yaml
# )

# # This library provides a C++ interface for sending realtime twist or joint commands to a robot
# add_library(moveit_servo_lib_cpp SHARED
#   src/collision_monitor.cpp
#   src/servo.cpp
#   src/utils/common.cpp
#   src/utils/command.cpp

# )



# rclcpp_components_register_node(
#   moveit_servo_lib_ros
#   PLUGIN "moveit_servo::ServoNode"
#   EXECUTABLE servo_node
# )
include_directories(include)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED
add_executable(old_arm_servo src/old_arm_servo.cpp)
ament_target_dependencies(old_arm_servo rclcpp std_msgs moveit_common moveit_servo ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_executable(joy_arm_control src/joy_arm_control.cpp include/joy_arm_control.h)
ament_target_dependencies(joy_arm_control rclcpp rover_msgs sensor_msgs moveit_servo  ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_executable(moveit_control src/moveit_control.cpp include/moveit_control.h)
ament_target_dependencies(moveit_control rclcpp rover_msgs sensor_msgs control_msgs moveit_servo ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
 #   old_arm_servo
    joy_arm_control
    moveit_control
    # servo_cpp_interface_demo
  DESTINATION
    lib/${PROJECT_NAME}
)


ament_package()
