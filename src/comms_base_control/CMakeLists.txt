cmake_minimum_required(VERSION 3.8)
project(comms_base_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_path(PIGPIO_INCLUDE_DIR pigpio.h)
find_library(PIGPIO_LIBRARY pigpio)

add_executable(death_ray_motor_control_node src/death_ray_motor_control.cpp)

ament_target_dependencies(death_ray_motor_control_node
  rclcpp
  std_msgs
)


target_link_libraries(death_ray_motor_control_node gpiod)

target_include_directories(death_ray_motor_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/comms_base_control>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  death_ray_motor_control_node
  DESTINATION lib/${PROJECT_NAME}
)

if(PIGPIO_INCLUDE_DIR AND PIGPIO_LIBRARY)
  add_executable(comms_relay src/comms_relay.cpp)
  target_include_directories(comms_relay PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/comms_base_control>
    $<INSTALL_INTERFACE:include>
  )
  target_include_directories(comms_relay PRIVATE ${PIGPIO_INCLUDE_DIR})
  target_link_libraries(comms_relay ${PIGPIO_LIBRARY})
  ament_target_dependencies(comms_relay rclcpp std_msgs)
  install(TARGETS comms_relay DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "pigpio not found; skipping comms_relay target")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()