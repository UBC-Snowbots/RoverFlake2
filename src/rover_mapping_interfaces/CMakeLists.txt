cmake_minimum_required(VERSION 3.8)
project(rover_mapping_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TagPoint.srv"
  "srv/Segment.srv"
  "srv/StartMission.srv"
  "srv/ExportMap.srv"
)

ament_package()
