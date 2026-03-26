cmake_minimum_required(VERSION 3.8)
project(rover_hmi_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(ament_index_cpp REQUIRED)

set(CMAKE_AUTOMOC ON)

include_directories(include)

add_executable(moteus_gui
    src/moteus_gui.cpp
    src/moteus_data_bus.cpp
    src/plot_widget.cpp
    src/motor_status_module.cpp
    src/plotting_module.cpp
    src/send_command_module.cpp
    src/command_log_module.cpp
    src/motor_config_module.cpp
    src/tiling_container.cpp
    src/rviz_module.cpp
    include/moteus_data_bus.h
    include/plot_widget.h
    include/tiling_container.h
    include/rviz_module.h
)
target_link_libraries(moteus_gui Qt5::Widgets)
ament_target_dependencies(moteus_gui rclcpp rover_msgs std_msgs ament_index_cpp)

install(TARGETS moteus_gui DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
