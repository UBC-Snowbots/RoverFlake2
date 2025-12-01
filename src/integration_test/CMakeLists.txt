cmake_minimum_required(VERSION 3.10)
project(integration_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Catch2 did not work very well. We tried, its just difficult to integrate in a scalable way.
# leaving this stale code just commented to keep record that we tried it. 
# add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../external_pkgs/Catch2 ${CMAKE_CURRENT_BINARY_DIR}/external_pkgs/Catch2) # Include Catch2 for unit tests

include_directories(include/)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(integration_tester src/assert_node.cpp)
add_executable(example_multiply_by_two_node src/example_multiply_by_two_node.cpp)
ament_target_dependencies(integration_tester rclcpp sensor_msgs rover_utils example_interfaces)
ament_target_dependencies(example_multiply_by_two_node rclcpp sensor_msgs rover_utils example_interfaces)

if(BUILD_TESTING)
  # Integration tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_launch_test.py)
  add_ros_isolated_launch_test(test/test_arm_pipeline.py)
  # Add more tests here
endif()

# target_link_libraries(integration_tester 
#   Catch2::Catch2)

install(
  TARGETS 
  integration_tester
  example_multiply_by_two_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
