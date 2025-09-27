cmake_minimum_required(VERSION 3.10)
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
  rover_utils
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
  moteus_msgs
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
#add_executable(old_arm_servo src/old_arm_servo.cpp)
#ament_target_dependencies(old_arm_servo rclcpp std_msgs moveit_common moveit_servo ${THIS_PACKAGE_INCLUDE_DEPENDS})

# For armControlParams, we add it as a regular .h file here because its in the same package. For any other package, we would just put arm_control in ament_target_dependencies
add_executable(joy_arm_control src/joy_arm_control.cpp include/joy_arm_control.h)
ament_target_dependencies(joy_arm_control rclcpp rover_msgs sensor_msgs moveit_servo  ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_executable(moveit_control src/moveit_control.cpp include/moveit_control.h include/armControlParams.h)
ament_target_dependencies(moveit_control rclcpp rover_msgs sensor_msgs control_msgs moveit_servo trajectory_msgs ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_executable(sim_helper src/sim_helper_node.cpp include/armControlParams.h)
ament_target_dependencies(sim_helper rclcpp rover_msgs sensor_msgs std_msgs rover_utils ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_executable(cbs_arm_interface src/cbs_interface.cpp include/armControlParams.h include/cbs_interface.h)
ament_target_dependencies(cbs_arm_interface rclcpp rover_msgs sensor_msgs std_msgs rover_utils ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(moteus_control_adapter src/moteus_control_adapter_node.cpp include/moteus_control_adapter_node.h)
ament_target_dependencies(moteus_control_adapter ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(
  TARGETS
 #   old_arm_servo
    cbs_arm_interface
    joy_arm_control
    moveit_control
    sim_helper
    moteus_control_adapter
    # servo_cpp_interface_demo
  DESTINATION
    lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}/include
)


ament_export_include_directories(include)
ament_package()
