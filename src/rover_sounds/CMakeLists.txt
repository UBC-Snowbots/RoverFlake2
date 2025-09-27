cmake_minimum_required(VERSION 3.10)
project(rover_sounds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# Find SFML shared libraries
find_package(SFML 2.5 COMPONENTS system audio REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include sounds)  # Include the OpenCV header files


add_executable(speaker_node src/speaker_node.cpp include/speaker_node.h)
ament_target_dependencies(speaker_node rclcpp ament_index_cpp std_msgs)
target_link_libraries(speaker_node sfml-system sfml-audio)  # Link against the OpenCV libraries
install(TARGETS
speaker_node
DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY 
    sounds/
  DESTINATION share/${PROJECT_NAME}/sounds
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
