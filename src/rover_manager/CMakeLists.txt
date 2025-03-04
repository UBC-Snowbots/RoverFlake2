cmake_minimum_required(VERSION 3.10)
project(rover_manager)

# This package 


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_utils REQUIRED)
find_package(serial REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

add_executable(watchdog_node src/watchdogNode.cpp include/watchdogNode.h)
ament_target_dependencies(watchdog_node rclcpp rover_utils rover_msgs)

add_executable(cbs_hardware_manager_node src/cbsHardwareManagerNode.cpp include/cbsHardwareManagerNode.h src/cbsDevice.cpp include/cbsDevice.h include/cbsDefinitions.h)
ament_target_dependencies(cbs_hardware_manager_node rclcpp rover_utils serial rover_msgs)

add_executable(cbs_display_manager_node src/cbsDisplayManagerNode.cpp include/cbsDisplayManagerNode.h)
ament_target_dependencies(cbs_display_manager_node rclcpp rover_utils rover_msgs)

install(TARGETS
  watchdog_node
  cbs_hardware_manager_node
  cbs_display_manager_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
launch
DESTINATION
share/${PROJECT_NAME}/launch/
)

ament_package()
