cmake_minimum_required(VERSION 3.8)
project(rover_arm_common)

find_package(ament_cmake REQUIRED)

# Header-only package: nothing to compile, just export the include directory.
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)

ament_package()
