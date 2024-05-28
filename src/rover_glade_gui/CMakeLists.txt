cmake_minimum_required(VERSION 3.8)
project(rover_glade_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(ament_index_cpp REQUIRED)
pkg_check_modules(GTK REQUIRED gtkmm-3.0) #gtkmm for c++, gtk+ for c. not confusing at all
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  ${GTK_INCLUDE_DIRS})
include_directories(include)
add_executable(arm_panel src/arm_panel_node.cpp include/arm_panel_node.h)
add_executable(main_display_left src/main_display_left.cpp include/main_display_left.h)
ament_target_dependencies(arm_panel rclcpp rover_msgs ament_index_cpp)
ament_target_dependencies(main_display_left rclcpp rover_msgs ament_index_cpp)
target_link_libraries(arm_panel ${GTK_LIBRARIES})
target_link_libraries(main_display_left ${GTK_LIBRARIES})
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

install(DIRECTORY 
  glade_files/
  DESTINATION share/${PROJECT_NAME}/glade_files)
install(DIRECTORY
  include/
  DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
  arm_panel
  main_display_left
  #main_display_right 
  #main_display_center
  DESTINATION lib/${PROJECT_NAME}/
  )

ament_package()
