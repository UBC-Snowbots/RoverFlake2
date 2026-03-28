cmake_minimum_required(VERSION 3.22)
project(dev_arm_moveit_config_v3)

find_package(ament_cmake REQUIRED)
find_package(rover_hmi_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(include)

# HMI plugin — embedded RViz panel
add_library(dev_arm_moveit_config_v3_hmi SHARED
    src/rviz_module.cpp
    include/rviz_module.h
)
target_link_libraries(dev_arm_moveit_config_v3_hmi
    Qt5::Widgets
    rover_hmi_core::rover_hmi_core
)
ament_target_dependencies(dev_arm_moveit_config_v3_hmi pluginlib ament_index_cpp)
pluginlib_export_plugin_description_file(rover_hmi_core plugins.xml)

install(TARGETS dev_arm_moveit_config_v3_hmi
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
