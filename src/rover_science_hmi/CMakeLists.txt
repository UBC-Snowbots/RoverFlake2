cmake_minimum_required(VERSION 3.8)
project(rover_science_hmi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(rover_hmi_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(include)

add_library(rover_science_hmi SHARED
    src/science_module.cpp
    include/science_module.h
    src/science_pipeline_module.cpp
    include/science_pipeline_module.h
    src/science_analysis_module.cpp
    include/science_analysis_module.h
)
target_link_libraries(rover_science_hmi
    Qt5::Widgets
    rover_hmi_core::rover_hmi_core
)
ament_target_dependencies(rover_science_hmi rclcpp rover_msgs pluginlib)
pluginlib_export_plugin_description_file(rover_hmi_core plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS rover_science_hmi
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/)

ament_export_include_directories(include)
ament_package()
