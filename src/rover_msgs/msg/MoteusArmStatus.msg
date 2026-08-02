# One for each motor. 
rover_msgs/BldcServoConfig[] config
rover_msgs/BldcServoStatus[] status
bool[] limit_switches

# CAN link health (drives the HMI connection banner)
string can_device        # fdcanusb node the driver has open; empty = none found
uint8 motors_replying    # motors that answered the last CAN cycle
