cmake_minimum_required(VERSION 3.5)

project(dev_arm_description)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
# find_package(joint_state_publisher_gui REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install directories for configuration, launch files, meshes, and URDFs
install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_package()

