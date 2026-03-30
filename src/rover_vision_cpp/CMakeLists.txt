cmake_minimum_required(VERSION 3.8)
project(rover_vision_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# Aruco node
add_executable(aruco_node src/aruco.cpp)
target_include_directories(aruco_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(aruco_node 
rclcpp 
sensor_msgs 
std_msgs 
cv_bridge 
image_transport 
OpenCV
)
install(TARGETS 
aruco_node 
  DESTINATION lib/${PROJECT_NAME}
)

# Cameras node
add_executable(cameras_node src/cameras.cpp)
target_include_directories(cameras_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cameras_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)
target_link_libraries(cameras_node
  realsense2::realsense2
)
install(TARGETS cameras_node
  DESTINATION lib/${PROJECT_NAME}
)

# YOLO detector node
add_executable(object_detection_node src/object.cpp)
target_include_directories(object_detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(object_detection_node
  rclcpp
  sensor_msgs
  OpenCV
)
install(TARGETS object_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
