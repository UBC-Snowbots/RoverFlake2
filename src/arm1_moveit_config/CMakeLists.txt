cmake_minimum_required(VERSION 3.5)

project(arm1_moveit_config)

# Find the ament_cmake package
find_package(ament_cmake REQUIRED)


# find_package(launch_ros REQUIRED)

# Install directories
foreach(dir config launch meshes urdf)
    install(DIRECTORY ${dir}/
        DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

# If you're creating a library or executable, you'd list them here, but since you didn't in the original CMakeLists.txt, I'm omitting that.

# This is the ament_cmake equivalent to catkin_package(). 
# It's required at the end of your CMakeLists.txt.
ament_package()

