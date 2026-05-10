cmake_minimum_required(VERSION 3.10)
project(rover_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(rover_utils REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(alert_example_node src/alert_example/alert_example.cpp src/alert_example/alert_example.h)
ament_target_dependencies(alert_example_node rclcpp rover_msgs rover_utils)

include_directories(include)
include_directories(config)

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}/include/)

install(DIRECTORY
  config/
  DESTINATION include/${PROJECT_NAME}/config/)

install(TARGETS 
  alert_example_node 
  DESTINATION lib/${PROJECT_NAME})

  ament_export_include_directories(include) #needed to include .h files in other packages

ament_package()
