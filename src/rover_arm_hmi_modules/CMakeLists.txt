cmake_minimum_required(VERSION 3.8)
project(rover_arm_hmi_modules)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rover_hmi_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(include)

add_library(rover_arm_hmi_modules SHARED
    src/motor_config_module.cpp
    src/plotting_module.cpp
    src/send_command_module.cpp
    src/command_log_module.cpp
    include/send_command_module.h
)

target_link_libraries(rover_arm_hmi_modules
    Qt5::Widgets
    rover_hmi_core::rover_hmi_core
)

ament_target_dependencies(rover_arm_hmi_modules
    rclcpp rover_msgs std_msgs ament_index_cpp pluginlib)

pluginlib_export_plugin_description_file(rover_hmi_core plugins.xml)

install(TARGETS rover_arm_hmi_modules
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
