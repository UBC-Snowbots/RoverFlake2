/*
    Converts differential drive requests to motor outputs.
    Directions given from bird eye view and top down view of each motor.

    ex. A5 = -5, A6 = -3 means spin A5 -5 R to L and A6 down by 3.
    This requires M5 to move 3 from L to R (positive direction) and M6 to move 3 from R to L (negative direction)
    from a top down view of each motor. The A5 spin also requires M5 to move 5 more in the positive direction. 
    (We choose to move the motor to execute the spin as the one that can continue in the current direction of motion)

    takes:
        axis5_input: Unitless input for axis 5 
        axis6_input: Unitless input for axis 6
    returns: 
        (motor5_output, motor6_output) Unitless values for motors 5 and 6
*/
inline void differential_drive(float axis5_input, float axis6_input, float &motor5_output, float &motor6_output) {

    // If the inputs have the same sign, to continue in the same direction of motion, use axis 5 for rotation
    if((axis5_input > 0 && axis6_input > 0) || (axis5_input < 0 && axis6_input < 0)) {
        motor5_output = - axis5_input - axis6_input;
        motor6_output = motor5_output;
    }

    // Else, use axis 6 for rotation
    else {
        motor5_output = - axis6_input;
        motor6_output = axis6_input;
    }  

    return;
}