cmake_minimum_required(VERSION 3.8)
project(arm_hardware_interface_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../arm_hardware_interface/moteus moteus_build)

add_executable(moteus_poll src/poll.cpp)
target_link_libraries(moteus_poll moteus::cpp)
ament_target_dependencies(moteus_poll rclcpp)

install(TARGETS moteus_poll DESTINATION lib/${PROJECT_NAME})

ament_package()
