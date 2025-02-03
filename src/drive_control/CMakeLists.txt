cmake_minimum_required(VERSION 3.8)
project(drive_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) # Ensure sensor_msgs is found

include_directories(include)

# Define the executable
add_executable(drive_node src/drive_control.cpp include/drive_control.h)

# Link dependencies
ament_target_dependencies(drive_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

target_link_libraries(drive_node phidget22)

# Set include directories
target_include_directories(drive_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the executable
install(TARGETS
  drive_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
