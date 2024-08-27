cmake_minimum_required(VERSION 3.8)
project(cameras_system)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(compressed_image_transport REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable for publisher node
add_executable(camera_sys_publisher_node src/camera_sys_publisher_node.cpp)
ament_target_dependencies(camera_sys_publisher_node rclcpp sensor_msgs std_msgs OpenCV cv_bridge image_transport)

# Add executable for sample message node
add_executable(sample_msg_node src/sample_msg_node.cpp)
ament_target_dependencies(sample_msg_node rclcpp std_msgs)

# Add executable for subscriber node
add_executable(camera_sys_subscriber_node src/camera_sys_subscriber_node.cpp)
ament_target_dependencies(camera_sys_subscriber_node rclcpp sensor_msgs OpenCV cv_bridge image_transport)

install(TARGETS
  camera_sys_publisher_node
  camera_sys_subscriber_node
  sample_msg_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
