cmake_minimum_required(VERSION 3.8)
project(sweepcpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Sweep REQUIRED)

# Create the executable target
add_executable(sweep_scan_node src/sweep_scan_node.cpp)

# Link ROS2 dependencies
ament_target_dependencies(sweep_scan_node
  rclcpp
  sensor_msgs
)

# Link the Sweep library. If your Sweep package exports a target, you can instead do:
# target_link_libraries(sweep_scan_node PUBLIC sweep::sweep)
target_link_libraries(sweep_scan_node
  ${LIBSWEEP_LIBRARY}
)

# Include the Sweep include directories
target_include_directories(sweep_scan_node PUBLIC
  ${LIBSWEEP_INCLUDE_DIR}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following lines skip certain linter checks. Remove these if you add proper copyright/license.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  sweep_scan_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
