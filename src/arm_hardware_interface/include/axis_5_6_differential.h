#pragma once

// =============================================================================
// Differential Wrist Drive  (axis_5_6_differential.h)
// =============================================================================
//
// Axes 5 (Wrist Roll) and 6 (End Effector) share a differential drive
// mechanism.  The user commands joint-space values for each axis, but the
// actual motors are coupled: both must move together to achieve a single DOF,
// or in opposite directions for the other DOF.
//
// This function converts joint-space inputs into motor-space commands that
// are then sent over CAN.  It is applied to velocity commands (and position
// commands when axes 5 & 6 are commanded together).
//
// Call site: moteus_driver_node.cpp poll() step 2b for motors at indices 4 & 5.
// =============================================================================

inline void differential_drive(float axis5_input, float axis6_input,
                                float& motor5_output, float& motor6_output)
{
    if ((axis5_input > 0 && axis6_input > 0) ||
        (axis5_input < 0 && axis6_input < 0))
    {
        motor5_output = -axis5_input - axis6_input;
        motor6_output = motor5_output;
    }
    else
    {
        motor5_output = -axis6_input;
        motor6_output =  axis6_input;
    }
}
