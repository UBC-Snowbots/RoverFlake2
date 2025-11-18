cmake_minimum_required(VERSION 3.10)
project(rv_sample_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(joy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rover_utils REQUIRED)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include/)
#add_executable adds a 'target' for CMake to go in and do it's magic
#The target's name here is 'sample_node', and while the naming doesn't need to match the source files, its good practice to keep them the same as this is what you use with ros2 run
add_executable(sample_node src/sample_node.cpp include/sample_node.h include/sample_node_defines.h)
ament_target_dependencies(sample_node rclcpp joy rover_utils)

# add_executable(test_example test/unit_test_example.cpp) # add a test, its just another executable
# target_link_libraries(test_example PRIVATE Catch2::Catch2WithMain)
# ament_add_catch2(test_example test/unit_test_example.cpp)

# if(BUILD_TESTING)
#   include(CTest)

#   ###############
#   ## UNIT TEST ##
#   ###############
#   add_executable(example_unit_test
#     test/unit_test_example.cpp
#   )
#   # Linking a unit test executable with catch_ros2::catch_ros2_with_main
#   # provides the default Catch2 main function to the executable.
#   target_link_libraries(example_unit_test
#     catch_ros2::catch_ros2_with_main
#   )
#   add_test(NAME ExampleUnitTest COMMAND example_unit_test)
# endif()


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

# find_package(ament_lint_auto REQUIRED)
# ament_lint_auto_find_test_dependencies()
ament_package()
