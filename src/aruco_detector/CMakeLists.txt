cmake_minimum_required(VERSION 3.5)
project(aruco_detector)

# Add include directory for header files
include_directories(
  include  # Ensure the 'include' folder is added
)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # Add this for std_msgs
find_package(OpenCV REQUIRED)
# find_package(cv2_aruco REQUIRED)  

# Add executable
add_executable(aruco_detector_node src/DetectMarker.cpp)

# Link dependencies to your executable
ament_target_dependencies(aruco_detector_node
  rclcpp
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs  
  OpenCV
  # cv2_aruco  
)

# Install the executable
install(TARGETS
  aruco_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
