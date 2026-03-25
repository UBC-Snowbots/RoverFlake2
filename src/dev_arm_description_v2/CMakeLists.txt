cmake_minimum_required(VERSION 3.8)

project(dev_arm_description_v2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf textures
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
