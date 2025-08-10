cmake_minimum_required(VERSION 3.5)
project(aruco_detector)

# Add include directory for header files
include_directories(
  include  
)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # Add this for std_msgs
find_package(OpenCV REQUIRED)

# Add executable
add_executable(aruco_detector_node src/DetectMarker.cpp)
# add_executable(color_encoder src/Encoder.cpp)
# add_executable(color_encoder src/EncodeFeed.cpp)
# Link dependencies to your executable
ament_target_dependencies(aruco_detector_node
  rclcpp
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs  
  OpenCV
)
# ament_target_dependencies(color_encoder
#   rclcpp
#   image_transport
#   cv_bridge
#   sensor_msgs
#   std_msgs  
#   OpenCV
# )

# ament_target_dependencies(color_encoder
#   rclcpp
#   image_transport
#   cv_bridge
#   sensor_msgs
#   std_msgs  
#   OpenCV
# )

# Install the executable
install(TARGETS
  aruco_detector_node
  # color_encoder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
