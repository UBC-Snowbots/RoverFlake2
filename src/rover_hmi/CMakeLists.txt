cmake_minimum_required(VERSION 3.8)
project(rover_hmi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find the gtkmm package
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTKMM gtkmm-3.0)
# pkg_check_modules(OPENCV2 open-3.0)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(arm_hardware_interface REQUIRED)
find_package(arm_control REQUIRED)
# find_package(rover_utils REQUIRED)
include_directories(include)
# Add the include directories
include_directories(${GTKMM_INCLUDE_DIRS})
# include_directories(${arm_hardware_interface_INCLUDE_DIRS})
# Add the linker flags
link_directories(${GTKMM_LIBRARY_DIRS})
# link_directories(${arm_hardware_interface_LIBRARY_DIRS})


add_executable(main_hmi_node src/RoverHMI.cpp include/RoverHMI.h src/helper_functions.cpp src/system_overview_functions.cpp)
# add_executable(middle_rover_viz_node src/RoverViz.cpp include/RoverViz.h)
add_executable(arm_hmi_node src/ArmTestingHMI.cpp include/RoverHMI.h include/ArmTestingHMI.h src/helper_functions.cpp src/system_overview_functions.cpp)
# add_executable(right_screen src/right_screen.cpp)  # 1080p
# add_executable(middle_screen src/middle_screen.cpp) # 1440p
ament_target_dependencies(main_hmi_node rclcpp ament_index_cpp std_msgs rover_msgs OpenCV sensor_msgs cv_bridge geometry_msgs arm_hardware_interface arm_control)
ament_target_dependencies(arm_hmi_node rclcpp ament_index_cpp std_msgs rover_msgs OpenCV sensor_msgs cv_bridge geometry_msgs arm_hardware_interface arm_control)
# ament_target_dependencies(middle_rover_viz_node rclcpp ament_index_cpp std_msgs rover_msgs rviz_visual_tools arm_hardware_interface)
target_link_libraries(main_hmi_node ${GTKMM_LIBRARIES})
target_link_libraries(arm_hmi_node ${GTKMM_LIBRARIES})


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
  #middle_rover_viz_node
  arm_hmi_node
  DESTINATION lib/${PROJECT_NAME}/
  )

install(DIRECTORY 
  glade_files
  config
  DESTINATION share/${PROJECT_NAME}/)
  install(DIRECTORY 
  css_files
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
