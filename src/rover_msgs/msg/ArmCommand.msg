char cmd_type
float64[] positions
float64[] velocities
float64 end_effector
int16 cmd_value 
# If its a home command, cmd_value will be the index (0 = axis 1) of the axis to home. 0xAA = home all axes except EE.
# 0xAE = Home all axes including EE