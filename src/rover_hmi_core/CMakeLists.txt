cmake_minimum_required(VERSION 3.8)
project(rover_hmi_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(include)

add_library(rover_hmi_core SHARED
    src/plot_widget.cpp
    include/rover_hmi_core/plot_widget.h
)
target_link_libraries(rover_hmi_core Qt5::Widgets)
ament_target_dependencies(rover_hmi_core rclcpp)

target_include_directories(rover_hmi_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS rover_hmi_core
    EXPORT rover_hmi_coreTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(rover_hmi_coreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Qt5Widgets)

ament_package()
