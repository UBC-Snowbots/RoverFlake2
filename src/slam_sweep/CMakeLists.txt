cmake_minimum_required(VERSION 3.8)
project(slam_sweep)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Specify the path to the bundled Sweep SDK, gagwegtew
set(SWEEP_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/sweep-sdk)
include_directories(${SWEEP_SDK_DIR}/include)
link_directories(${SWEEP_SDK_DIR}/lib)

# Execute sweep node
add_executable(sweep_node src/sweep_node.cpp)
ament_target_dependencies(sweep_node rclcpp sensor_msgs pcl_conversions PCL)

# Link against ROS, PCL, and the Sweep SDK libraries
target_link_libraries(sweep_node
  ${PCL_LIBRARIES}
  sweep
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(TARGETS sweep_node
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
