cmake_minimum_required(VERSION 3.8)
project(rv_sample_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(joy REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include/)
#add_executable adds a 'target' for CMake to go in and do it's magic
#The target's name here is 'sample_node', and while the naming doesn't need to match the source files, its good practice to keep them the same as this is what you use with ros2 run
add_executable(sample_node src/sample_node.cpp include/sample_node.h)
ament_target_dependencies(sample_node rclcpp joy)




install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  TARGETS 
  sample_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
