cmake_minimum_required(VERSION 3.5)
project(usb_parser)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install Python scripts 
ament_python_install_package(usb_parser)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Mark other files for installation (if needed)
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/
# )

# Add any additional setup you may need, such as finding and linking other dependencies
# find_package(some_dependency REQUIRED)
# target_link_libraries(your_node ${some_dependency_LIBRARIES})

ament_package()
