cmake_minimum_required(VERSION 3.5)
project(ipcamerafeed)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFloat64.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
