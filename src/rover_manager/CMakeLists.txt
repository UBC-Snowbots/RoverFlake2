cmake_minimum_required(VERSION 3.8)
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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

add_executable(watchdog_node src/watchdogNode.cpp include/watchdogNode.h)
ament_target_dependencies(watchdog_node rclcpp rover_utils)

install(TARGETS
  watchdog_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
launch
DESTINATION
share/${PROJECT_NAME}/launch/
)

ament_package()
