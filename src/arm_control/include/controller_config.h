/**
 * controller_config.h
 * 
 * This is for gamepads / game controllers, not for the control base joysticks
 * 
 * Modular controller configuration for arm teleoperation.
 * To switch controllers, change the ACTIVE_CONTROLLER define below.
 * Button/axis indices can be verified by running: ros2 topic echo /joy
 * 
 * Joy -> Ros2 package that works with event based (/dev/input/eventx)
 *          - This has bugs? Cannot seem to set parameters
 * Joy_linux -> Ros2 package that does the same thing as joy, but uses /dev/input/by-id
 *          - This was the ros1 joy package and used to be the standard. We are able to set parameters and stuff 
 * 
 */
#pragma once

#include <sensor_msgs/msg/joy.hpp>
#include <rover_arm_common/ArmSerialProtocol.h>
inline static constexpr int MAX_BUTTONS = 20; // can be decreased
inline static constexpr int MAX_AXES    = 10;


// ============================================================
//  PS4 Controller (Using Joy Linux)
// ============================================================
namespace ps4_index {
    namespace axes {
        static inline constexpr int LEFT_JOYSTICK_X     = 0;
        static inline constexpr int LEFT_JOYSTICK_Y     = 1;
        static inline constexpr int RIGHT_JOYSTICK_X    = 3;
        static inline constexpr int RIGHT_JOYSTICK_Y    = 4;

        static inline constexpr int L2                  = 2; // Left Trigger
        static inline constexpr int R2                  = 5; // Right trigger

        static inline constexpr int DPAD_X              = 6;
        static inline constexpr int DPAD_Y              = 7;

    }

    namespace buttons { // Can also be switches
        static inline constexpr int X                   = 0;
        static inline constexpr int CIRCLE              = 1;
        static inline constexpr int TRIANGLE            = 2;
        static inline constexpr int SQUARE              = 3;
        static inline constexpr int L1                  = 4; // Left bumper
        static inline constexpr int R1                  = 5; // Right bumber
        static inline constexpr int SHARE               = 8; // Share button (to the left of the middle trackpad)
        static inline constexpr int OPTIONS             = 9; // Options button (to the right of the middle trackpad)
        static inline constexpr int PS_BUTTON           = 10; // Middle playstation button in between joysticks
        static inline constexpr int L3                  = 11; // Left joystick button
        static inline constexpr int R3                  = 12; // Right joystic button

        
    }

    //Note for PS4 Controller: Currently there is no 

    // If we need to support both joy and joy_linux, maybe something like:
    /*
    namespace your_controller
    {
        namespace joy_event {
            axes...
            buttons...
        }
            
        namespace joy_linux {
            axes...
            buttons...
        }
    }
    
    */

}


// ============================================================
//  Nintendo Switch Pro Controller (joy, event based - NOT JOY LINUX WE CANT GET SWITCH WORKING WITH JOY LINUX) 
// ============================================================

namespace switch_index {
    namespace axes {
        static inline constexpr int LEFT_JOYSTICK_X     = 0; // Positive is Left
        static inline constexpr int LEFT_JOYSTICK_Y     = 1; // Positive is up
        static inline constexpr int RIGHT_JOYSTICK_X    = 2; // Positive is left 
        static inline constexpr int RIGHT_JOYSTICK_Y    = 3; // Positive is up
        static inline constexpr int LEFT_TRIGGER_ZL     = 4; // Positive is unclicked. Negative is clicked. Only -1 and 1
        static inline constexpr int RIGHT_TRIGGER_ZR    = 5; // Positive is unclicked. Negative is clicked. Only -1 and 1

    }

    namespace buttons {
        static inline constexpr int A                   = 0;
        static inline constexpr int B                   = 1;
        static inline constexpr int X                   = 2;
        static inline constexpr int Y                   = 3;
        static inline constexpr int MINUS               = 4; // the (-) button to the left of the switch logo
        static inline constexpr int HOME                = 5; // the (home) button to the bottom right of the switch logo
        static inline constexpr int PLUS                = 6; // the (+) button to the right of the switch logo
        static inline constexpr int L3                  = 7; // Left joystick button
        static inline constexpr int R3                  = 8; // Right joystic button
        static inline constexpr int L1                  = 9; // Left bumper
        static inline constexpr int R1                  = 10; // Right bumper

        static inline constexpr int DPAD_UP             = 11; 
        static inline constexpr int DPAD_DOWN           = 12; 
        static inline constexpr int DPAD_LEFT           = 13; 
        static inline constexpr int DPAD_RIGHT          = 14; 
        static inline constexpr int SQURE_CICLE         = 15; // Weird button to the bottom left of the switch logo

    }
}


// ============================================================
//  Cyborg Joystick (joy linux)
// ============================================================

namespace cyborg_index {
    namespace axes {
        static inline constexpr int JOYSTICK_X          = 0; // Left is positive
        static inline constexpr int JOYSTICK_Y          = 1; // up is positive
        static inline constexpr int THROTTLE            = 2; // up is 1, all the way down is -1
        static inline constexpr int JOYSTICK_Z          = 3; // CCW is positive

        // Thumb 4way is the dongle thing between buttons 5 and 6
        static inline constexpr int THUMB_4WAY_X        = 4; // left is positive
        static inline constexpr int THUMB_4WAY_Y        = 5; // left is positive
    }

    namespace buttons {
        static inline constexpr int TRIGGER             = 0;
        static inline constexpr int BUTTON_2            = 1;
        static inline constexpr int BUTTON_3            = 2;
        static inline constexpr int BUTTON_4            = 3;
        static inline constexpr int BUTTON_5            = 4; // kinda sticky
        static inline constexpr int BUTTON_6            = 5; // kinda sticky
        static inline constexpr int F1                  = 6;
        static inline constexpr int F2                  = 7;
        static inline constexpr int F3                  = 8;
        static inline constexpr int F4                  = 9;
        static inline constexpr int LEFT_UPARROW        = 10;
        static inline constexpr int RIGHT_UPARROW       = 11;
    }
}


namespace ArmControllerConfig { // Can make into a class later?

    enum class GameController {
        PS4_JOY_LINUX, // Dualshock 4 (PS4) controller, ran from joy_linux (not joy!)
        SWITCH_PRO_CONTROLLER,
        CYBORG_JOYSTICK

        //Also possible here:
        // AARON_STYLE_SWITCH_PRO
        // ROWAN_PRECISE_PS4
        // (custom mappings based on which user likes what)
    };

    // Arm control input, Modelled off of control base joysticks:
    /* axes:
    - 0: left joystick x
    - 1: left joystick y
    - 2: left joystick z
    - 3: right joystick x
    - 4: right joystick y
    - 5: right joystick z

    example: For a game controller, the below function will do its best to map the game controller inputs to sub out for the arm joysticks

    Note: the above is based on control base arm joysticks "default", we can create other 
    */

    float ee_speed_scale = 60;
    float axis_speed_scale = 10;

    struct ArmControlInput {
        // Static arrays, not vectors here
        float fk_axes[MAX_AXES] = {}; // All are standardized from -1 to 1. 
        float ik_axes[MAX_AXES] = {}; // All are standardized from -1 to 1. 
        float end_effector = 0; // Standardized from -1 to 1
        int home = 0;  // Pulse to home
        int kinematics_mode_switch = 0; // Pulse to change from IK to FK
    };

    inline static bool process_joy_input(GameController controller, const sensor_msgs::msg::Joy::SharedPtr joy_msg, ArmControlInput &arm_control_msg) 
    {
        switch (controller) {
        case GameController::PS4_JOY_LINUX:
            {
                using namespace ps4_index;
                // arm_control_msg.fk_axes[AXIS_1_INDEX] = ((joy_msg->axes[axes::L2] - joy_msg->axes[axes::R2])) / (2.0f);
                arm_control_msg.fk_axes[AXIS_1_INDEX] = joy_msg->axes[axes::LEFT_JOYSTICK_X];
                arm_control_msg.fk_axes[AXIS_2_INDEX] = joy_msg->axes[axes::LEFT_JOYSTICK_Y];
                arm_control_msg.fk_axes[AXIS_3_INDEX] = joy_msg->axes[axes::RIGHT_JOYSTICK_Y];
                arm_control_msg.fk_axes[AXIS_4_INDEX] = joy_msg->axes[axes::RIGHT_JOYSTICK_X];
                arm_control_msg.fk_axes[AXIS_5_INDEX] = joy_msg->axes[axes::DPAD_Y];
                arm_control_msg.fk_axes[AXIS_6_INDEX] = joy_msg->axes[axes::DPAD_X];

                arm_control_msg.ik_axes[IK_ANG_Z_INDEX] = ((joy_msg->axes[axes::L2] - joy_msg->axes[axes::R2])) / (2.0f);
                arm_control_msg.ik_axes[IK_LIN_X_INDEX] = joy_msg->axes[axes::LEFT_JOYSTICK_Y];
                arm_control_msg.ik_axes[IK_LIN_Z_INDEX] = joy_msg->axes[axes::RIGHT_JOYSTICK_Y];
                arm_control_msg.ik_axes[IK_LIN_Y_INDEX] = joy_msg->axes[axes::LEFT_JOYSTICK_X];
                arm_control_msg.ik_axes[IK_ANG_X_INDEX] = joy_msg->axes[axes::DPAD_X];
                arm_control_msg.ik_axes[IK_ANG_Y_INDEX] = joy_msg->axes[axes::DPAD_Y];

                arm_control_msg.end_effector = joy_msg->buttons[buttons::L1] - joy_msg->buttons[buttons::R1];
    
                arm_control_msg.home = joy_msg->buttons[buttons::SHARE];
                arm_control_msg.kinematics_mode_switch = joy_msg->buttons[buttons::CIRCLE];
            }
            break;
        case GameController::SWITCH_PRO_CONTROLLER:
            {
                using namespace switch_index;
                arm_control_msg.fk_axes[AXIS_2_INDEX] = joy_msg->axes[axes::LEFT_JOYSTICK_Y];
                arm_control_msg.fk_axes[AXIS_3_INDEX] = joy_msg->axes[axes::RIGHT_JOYSTICK_Y];
                arm_control_msg.fk_axes[AXIS_4_INDEX] = joy_msg->axes[axes::RIGHT_JOYSTICK_X];
                // dpad is buttons on Switch Pro and controls z axis
                arm_control_msg.fk_axes[AXIS_5_INDEX] = static_cast<float>(joy_msg->buttons[buttons::DPAD_UP] - joy_msg->buttons[buttons::DPAD_DOWN]);
                arm_control_msg.fk_axes[AXIS_6_INDEX] = static_cast<float>(joy_msg->buttons[buttons::DPAD_RIGHT] - joy_msg->buttons[buttons::DPAD_LEFT]);

                arm_control_msg.ik_axes[IK_LIN_X_INDEX] = joy_msg->axes[axes::LEFT_JOYSTICK_Y];
                arm_control_msg.ik_axes[IK_LIN_Z_INDEX] = joy_msg->axes[axes::RIGHT_JOYSTICK_Y];
                arm_control_msg.ik_axes[IK_LIN_Y_INDEX] = joy_msg->axes[axes::LEFT_JOYSTICK_X];
                arm_control_msg.ik_axes[IK_ANG_X_INDEX] = static_cast<float>(joy_msg->buttons[buttons::DPAD_RIGHT] - joy_msg->buttons[buttons::DPAD_LEFT]);
                arm_control_msg.ik_axes[IK_ANG_Y_INDEX] = static_cast<float>(joy_msg->buttons[buttons::DPAD_UP] - joy_msg->buttons[buttons::DPAD_DOWN]);

                arm_control_msg.end_effector = joy_msg->buttons[buttons::L1] - joy_msg->buttons[buttons::R1];
                arm_control_msg.kinematics_mode_switch = joy_msg->buttons[buttons::MINUS];
            }
            break;
        case GameController::CYBORG_JOYSTICK:
            {
            using namespace cyborg_index;
            // TODO

            
            }
            break;
        default:
            return false; // Return Error
        }
        return true; // Return Success
    }

    // Overload for drive control (later refactor)
    // inline static bool process_joy_input(GameController controller, sensor_msgs::msg::Joy joy_msg, DriveControlInput &input) 
    // {
        
    //     switch (controller)
    //     {
    //     case GameController::PS4_JOY_LINUX:
            
    //         break;
    //     case GameController::SWITCH_PRO_CONTROLLER:

    //         break;
    //     case GameController::CYBORG_JOYSTICK:
            
    //         break;
    //     default:
    //         return false;

    //     return true;
    //     }
    // }
}