cmake_minimum_required(VERSION 3.8)
project(rover_arm_common)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Contract headers stay header-only; this small library is just the
# motor_config.yaml loader/saver. yaml-cpp is PRIVATE — consumers never see it.
add_library(rover_arm_common SHARED src/motor_config_io.cpp)
target_include_directories(rover_arm_common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(rover_arm_common PRIVATE yaml-cpp)
ament_target_dependencies(rover_arm_common PUBLIC ament_index_cpp)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS rover_arm_common
    EXPORT rover_arm_commonTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

ament_export_targets(rover_arm_commonTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(ament_index_cpp)

ament_package()
