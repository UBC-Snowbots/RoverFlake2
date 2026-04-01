cmake_minimum_required(VERSION 3.8)
project(arm_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rover_hmi_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(include)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external_pkgs/moteus moteus_build)

# CAN-FD driver executable
add_executable(moteus_driver src/moteus_driver_node.cpp)
target_link_libraries(moteus_driver moteus::cpp)
ament_target_dependencies(moteus_driver rclcpp rover_msgs std_msgs sensor_msgs)

# HMI plugins — separate shared library, does not link the CAN library
add_library(arm_hardware_interface_hmi SHARED
    src/motor_status_module.cpp
    src/plotting_module.cpp
    src/motor_config_module.cpp
    src/send_command_module.cpp
    src/command_log_module.cpp
    include/motor_status_module.h
    include/plotting_module.h
    include/motor_config_module.h
    include/send_command_module.h
    include/command_log_module.h
)
target_link_libraries(arm_hardware_interface_hmi
    Qt5::Widgets
    rover_hmi_core::rover_hmi_core
)
ament_target_dependencies(arm_hardware_interface_hmi rclcpp rover_msgs std_msgs pluginlib)
pluginlib_export_plugin_description_file(rover_hmi_core plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY include/ DESTINATION include/)

install(TARGETS moteus_driver
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS arm_hardware_interface_hmi
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

ament_export_include_directories(include)

ament_package()
