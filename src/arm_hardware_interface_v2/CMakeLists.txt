cmake_minimum_required(VERSION 3.8)
project(arm_hardware_interface_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../arm_hardware_interface/moteus moteus_build)

add_executable(moteus_driver src/moteus_driver_node.cpp)
target_link_libraries(moteus_driver moteus::cpp)
ament_target_dependencies(moteus_driver rclcpp rover_msgs std_msgs)

install(TARGETS moteus_driver DESTINATION lib/${PROJECT_NAME})

ament_package()
