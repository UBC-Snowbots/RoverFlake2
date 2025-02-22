This package is for nodes and files related to controlling the arm.

This includes:

1. moveit_control.cpp - MoveIt! control (inverse kinematics)

2. joy_arm_control.cpp - joystick control for arm. Meant to be used with PS4 controller by default, with joy_linux

3. sim_helper_node.cpp - Makes a bridge so we can communicate with the simulated arm over the same topics as the physical arm. 