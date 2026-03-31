cmake_minimum_required(VERSION 3.8)
project(rover_electrical_hmi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_hmi_core REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(src)

add_library(rover_electrical_hmi SHARED
  src/power_summary_module.cpp
  src/drivetrain_stop_module.cpp
  src/wheel_telemetry_module.cpp
  src/lighting_module.cpp
  src/vaw_tree_module.cpp
  src/power_timeline_module.cpp
)

target_include_directories(rover_electrical_hmi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

ament_target_dependencies(rover_electrical_hmi
  rclcpp rover_hmi_core rover_msgs std_msgs pluginlib
)

target_link_libraries(rover_electrical_hmi
  Qt5::Widgets
  rover_hmi_core::rover_hmi_core
)

pluginlib_export_plugin_description_file(rover_hmi_core plugins.xml)

install(TARGETS rover_electrical_hmi
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

ament_package()
