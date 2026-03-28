cmake_minimum_required(VERSION 3.8)
project(rover_hmi_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(include)

# Shared library: GuiModule interface, TilingContainer layout engine, PlotWidget
add_library(rover_hmi_core SHARED
    src/plot_widget.cpp
    src/tiling_container.cpp
    include/rover_hmi_core/plot_widget.h
    include/rover_hmi_core/tiling_container.h
)
target_link_libraries(rover_hmi_core Qt5::Widgets)
ament_target_dependencies(rover_hmi_core rclcpp pluginlib)

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
ament_export_dependencies(rclcpp pluginlib Qt5Widgets)

# Generic HMI host executable — discovers and tiles all registered GuiModules
add_executable(rover_hmi src/hmi_host.cpp)
target_link_libraries(rover_hmi rover_hmi_core Qt5::Widgets)
ament_target_dependencies(rover_hmi rclcpp pluginlib)
install(TARGETS rover_hmi DESTINATION lib/${PROJECT_NAME})

ament_package()
