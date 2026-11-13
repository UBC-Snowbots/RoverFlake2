cmake_minimum_required(VERSION 3.8)
project(rover_moteus_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# moteus C++ client is header-only; we only need its include path
set(MOTEUS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../external_pkgs/moteus/lib/cpp/mjbots/moteus)

add_library(${PROJECT_NAME} SHARED src/moteus_bus.cpp src/moteus_node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${MOTEUS_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/moteus>)
ament_target_dependencies(${PROJECT_NAME} rclcpp rover_msgs std_msgs)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${MOTEUS_INCLUDE_DIR}/ DESTINATION include/moteus FILES_MATCHING PATTERN "*.h")
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rover_msgs std_msgs)
ament_package()