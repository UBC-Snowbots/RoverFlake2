cmake_minimum_required(VERSION 3.10)
project(integration_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../external_pkgs/Catch2 ${CMAKE_CURRENT_BINARY_DIR}/external_pkgs/Catch2) # Include Catch2 for unit tests

include_directories(include/)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)


add_executable(integration_tester src/assert_node.cpp)
ament_target_dependencies(integration_tester rclcpp sensor_msgs rover_utils example_interfaces)

target_link_libraries(integration_tester 
  Catch2::Catch2)

install(
  TARGETS 
  integration_tester
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
