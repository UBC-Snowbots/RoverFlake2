cmake_minimum_required(VERSION 3.8)
project(drive_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#! not sure how to find dependencies for phid
include_directories(/usr/include/)
include_directories(include)
add_executable(drive_node src/drive_control.cpp include/drive_control.h)
ament_target_dependencies(drive_node rclcpp geometry_msgs)
target_link_libraries(drive_node phidget22)

add_executable(chassis_joy_ctrl src/chassis_joy_ctrl.cpp include/chassis_joy_ctrl.h)
ament_target_dependencies(chassis_joy_ctrl rclcpp geometry_msgs sensor_msgs)


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

install(TARGETS
  drive_node
  chassis_joy_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
