cmake_minimum_required(VERSION 3.8)
project(rover_hmi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find the gtkmm package
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTKMM gtkmm-3.0)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# find_package(rover_utils REQUIRED)
include_directories(include)
# Add the include directories
include_directories(${GTKMM_INCLUDE_DIRS})

# Add the linker flags
link_directories(${GTKMM_LIBRARY_DIRS})


add_executable(main_hmi_node src/RoverHMI.cpp include/RoverHMI.h) # 1080p
# add_executable(right_screen src/right_screen.cpp)  # 1080p
# add_executable(middle_screen src/middle_screen.cpp) # 1440p
ament_target_dependencies(main_hmi_node rclcpp ament_index_cpp std_msgs rover_msgs)
target_link_libraries(main_hmi_node ${GTKMM_LIBRARIES})


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

install(TARGETS 
  main_hmi_node
  DESTINATION lib/${PROJECT_NAME}/
  )

install(DIRECTORY 
  glade_files
  DESTINATION share/${PROJECT_NAME}/)


ament_package()
