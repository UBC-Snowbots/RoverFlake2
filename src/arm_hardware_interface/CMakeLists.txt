cmake_minimum_required(VERSION 3.8)
project(arm_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rover_arm_common REQUIRED)

include_directories(include)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external_pkgs/moteus moteus_build)

# CAN-FD driver executable
add_executable(moteus_driver src/moteus_driver_node.cpp)
target_link_libraries(moteus_driver moteus::cpp)
ament_target_dependencies(moteus_driver rclcpp rover_msgs std_msgs sensor_msgs rover_arm_common)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Hardware-free conf-get regression test: drives the vendored moteus client
  # against a scripted fdcanusb emulator on a pty (test/fdcanusb_emulator.py).
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(conf_get_path_test test/conf_get_path_test.cpp)
  target_link_libraries(conf_get_path_test moteus::cpp)
  ament_target_dependencies(conf_get_path_test rover_arm_common)
  target_compile_definitions(conf_get_path_test PRIVATE
    EMULATOR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fdcanusb_emulator.py")

  # Pure-math test for the axis 5/6 differential wrist mapping.
  ament_add_gtest(wrist_math_test test/wrist_math_test.cpp)
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY include/ DESTINATION include/)

install(TARGETS moteus_driver
    DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
