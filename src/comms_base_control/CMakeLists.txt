cmake_minimum_required(VERSION 3.8)
project(comms_base_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include/comms_base_control)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rover_msgs REQUIRED)

# comms_relay talks to pigpiod over a socket, so it only needs the client
# library (pigpiod_if2), not the root-only in-process pigpio library.
find_path(PIGPIOD_IF2_INCLUDE_DIR pigpiod_if2.h)
find_library(PIGPIOD_IF2_LIBRARY pigpiod_if2)


add_executable(death_ray_motor_control_node src/death_ray_motor_control.cpp)
add_executable(death_ray_cbs_input_node src/death_ray_cbs_input_node.cpp)


ament_target_dependencies(death_ray_motor_control_node
  rclcpp
  std_msgs
)
ament_target_dependencies(death_ray_cbs_input_node
  rclcpp
  std_msgs
  rover_msgs
)


target_link_libraries(death_ray_motor_control_node gpiod)
# target_link_libraries(death_ray_cbs_input_node rclcpp)

target_include_directories(death_ray_motor_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/comms_base_control>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  death_ray_motor_control_node
  death_ray_cbs_input_node
  DESTINATION lib/${PROJECT_NAME}
)

if(PIGPIOD_IF2_INCLUDE_DIR AND PIGPIOD_IF2_LIBRARY)
  find_package(Threads REQUIRED)
  add_executable(comms_relay src/comms_relay.cpp)
  target_include_directories(comms_relay PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/comms_base_control>
    $<INSTALL_INTERFACE:include>
  )
  target_include_directories(comms_relay PRIVATE ${PIGPIOD_IF2_INCLUDE_DIR})
  target_link_libraries(comms_relay ${PIGPIOD_IF2_LIBRARY} Threads::Threads rt)
  ament_target_dependencies(comms_relay rclcpp std_msgs rover_msgs)
  install(TARGETS comms_relay DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "pigpiod_if2 not found; skipping comms_relay target")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()