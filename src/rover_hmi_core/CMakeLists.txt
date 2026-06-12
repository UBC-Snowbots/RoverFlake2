cmake_minimum_required(VERSION 3.8)
project(rover_hmi_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(include)

# ---------------------------------------------------------------------------
# Core library: GuiModule interface, TilingContainer layout engine, PlotWidget
# ---------------------------------------------------------------------------
add_library(rover_hmi_core SHARED
    src/plot_widget.cpp
    src/tiling_container.cpp
    include/rover_hmi_core/plot_widget.h
    include/rover_hmi_core/tiling_container.h
)
target_link_libraries(rover_hmi_core Qt5::Core Qt5::Widgets)
ament_target_dependencies(rover_hmi_core rclcpp pluginlib)

target_include_directories(rover_hmi_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# HMI plugin modules: arm / science / electrical
# ---------------------------------------------------------------------------
add_library(rover_hmi_modules SHARED
    # Electrical
    src/electrical/power_summary_module.cpp
    src/electrical/drivetrain_stop_module.cpp
    src/electrical/wheel_telemetry_module.cpp
    src/electrical/lighting_module.cpp
    src/electrical/vaw_tree_module.cpp
    src/electrical/power_timeline_module.cpp
    # Arm
    src/arm/motor_status_module.cpp
    src/arm/motor_config_module.cpp
    src/arm/plotting_module.cpp
    src/arm/send_command_module.cpp
    src/arm/command_log_module.cpp
    # MoveIt — header listed explicitly so AUTOMOC scans it (RvizModule
    # is the only module that places Q_OBJECT in its header; the rest use
    # the inline-Q_OBJECT-in-.cpp pattern and don't need this).
    src/moveit/rviz_module.cpp
    include/rover_hmi_core/moveit/rviz_module.h
    # Science
    src/science/science_module.cpp
    src/science/science_pipeline_module.cpp
    src/science/science_analysis_module.cpp
    # Drive
    src/drive/drive_module.cpp
)

# Each subdirectory's headers are findable with bare names inside their own CPP files
target_include_directories(rover_hmi_modules PRIVATE
    include/rover_hmi_core/electrical
    include/rover_hmi_core/arm
    include/rover_hmi_core/moveit
    include/rover_hmi_core/science
    include/rover_hmi_core/drive
)

target_link_libraries(rover_hmi_modules
    rover_hmi_core
    Qt5::Widgets
)
ament_target_dependencies(rover_hmi_modules rclcpp rover_msgs std_msgs sensor_msgs geometry_msgs pluginlib ament_index_cpp)
pluginlib_export_plugin_description_file(rover_hmi_core plugins.xml)

# ---------------------------------------------------------------------------
# Generic HMI host executable
# ---------------------------------------------------------------------------
add_executable(rover_hmi src/hmi_host.cpp)
target_link_libraries(rover_hmi rover_hmi_core Qt5::Widgets)
ament_target_dependencies(rover_hmi rclcpp pluginlib)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS rover_hmi_core
    EXPORT rover_hmi_coreTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
install(TARGETS rover_hmi_modules
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS rover_hmi DESTINATION lib/${PROJECT_NAME})

ament_export_targets(rover_hmi_coreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp pluginlib Qt5Widgets)

ament_package()
