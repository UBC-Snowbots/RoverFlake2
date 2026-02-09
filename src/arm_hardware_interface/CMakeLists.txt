cmake_minimum_required(VERSION 3.8)
project(arm_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(arm_control REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)
include_directories(include ${arm_control_INCLUDE_DIRS})

add_subdirectory(moteus)
add_library(${PROJECT_NAME} src/armprotocol.cpp
                            include/arm_hardware_interface/ArmSerialProtocol.h)

# For old and new arm
add_executable(
  serial_arm_driver src/ArmSerialInterface.cpp include/ArmSerialInterface.h
                    include/arm_hardware_interface/ArmSerialProtocol.h)
add_executable(
  moteus_arm_driver src/MoteusInterface.cpp include/ArmMoteusInterface.h
                    include/arm_hardware_interface/ArmSerialProtocol.h)

target_compile_definitions(moteus_arm_driver PRIVATE BUILDING_NEW_DRIVER=1)

target_link_libraries(moteus_arm_driver moteus::cpp)
ament_target_dependencies(serial_arm_driver rclcpp sensor_msgs serial
                          rover_msgs arm_control)
ament_target_dependencies(moteus_arm_driver rclcpp sensor_msgs serial
                          rover_msgs arm_control)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/launch)

# Install headers for other packages (for arm serial protocol)
install(DIRECTORY include/ DESTINATION include/)

install(
  TARGETS serial_arm_driver moteus_arm_driver ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
