cmake_minimum_required(VERSION 3.10)
project(mock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rover_utils REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(rover_arm_common REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(
  camera/include
  ${OpenCV_INCLUDE_DIRS}
)



add_executable(mock_camera_node camera/src/mock_camera.cpp camera/include/mock_camera.h)
ament_target_dependencies(mock_camera_node rclcpp sensor_msgs cv_bridge image_transport rover_utils)
target_link_libraries(mock_camera_node ${OpenCV_LIBRARIES})

# Simulated arm driver: /arm/command -> /joint_states (see arm/src/mock_arm.cpp)
add_executable(mock_arm_node arm/src/mock_arm.cpp)
ament_target_dependencies(mock_arm_node rclcpp sensor_msgs rover_msgs rover_arm_common)

# Digital twin of the real arm: /arm/feedback (telemetry) -> /joint_states,
# plus optional twin-vs-sim error (see arm/src/arm_twin.cpp)
add_executable(arm_twin_node arm/src/arm_twin.cpp)
ament_target_dependencies(arm_twin_node rclcpp sensor_msgs std_msgs rover_msgs rover_arm_common)


install(
  DIRECTORY
    camera/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install (
  DIRECTORY
    camera/config/
  DESTINATION share/${PROJECT_NAME}/camera/config
)

install(
  TARGETS
  mock_camera_node
  mock_arm_node
  arm_twin_node
  DESTINATION lib/${PROJECT_NAME}
)


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
