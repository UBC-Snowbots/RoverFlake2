cmake_minimum_required(VERSION 3.8)
project(comms_base_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(death_ray_motor_control_node src/death_ray_motor_control.cpp)
add_executable(death_ray_tracking_node src/death_ray_tracking.cpp)

ament_target_dependencies(death_ray_motor_control_node
  rclcpp
  std_msgs
)

ament_target_dependencies(death_ray_tracking_node
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(death_ray_motor_control_node gpiod)

target_include_directories(death_ray_motor_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/comms_base_control>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(death_ray_tracking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/comms_base_control>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  death_ray_motor_control_node
  death_ray_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()