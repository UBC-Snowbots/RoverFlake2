This package is for nodes and files related to controlling the arm.

This includes:

1. moveit_control.cpp - MoveIt! control (inverse kinematics)

2. joy_arm_control.cpp - joystick control for arm. Meant to be used with PS4 controller by default, with joy_linux

3. sim_helper_node.cpp - Makes a bridge so we can communicate with the simulated arm over the same topics as the physical arm. 

<!-- Aaron's Notes -->
ros2 topic pub --once /arm/command rover_msgs/msg/ArmCommand   "{cmd_type: 86, velocities: [5.0, 0.0, 0.0, 0.0, 0.0, 0.0], end_effector: 0.0}"