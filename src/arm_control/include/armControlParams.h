#pragma once //? Ensures file isn't included more than once, which would lead to redefinition compiler errors
// Parameters for the arm. Specific to our hardware and firmware
// May want to move to rover_utils for better standardization
#include <string_view>
//* Debug levels - comment out what you don't want to see in the terminal

#define PRINTOUT_AXIS_PARAMS
#define SIM_STARTEND_MSGS
// #define NUM

#define VELOCITY_BASED 1
#define POSITION_BASED 2
#define NUM_JOINTS 6
#define PI 3.14
struct ArmConstants{
    static constexpr float axis_zero_rads[NUM_JOINTS] = {-0.9608,          //* Axis 1 Offset
                                                    -1.9390,          //* Axis 2 Offset
                                                    -1.3460,         //* Axis 3 Offset
                                                    -2.4108, //+PI      // Axis 4 Offset 
                                                     2.2060-PI/3,  //2.2060        //* Axis 5 Offset
                                                    0};        //? Axis 6 Offset
    
    static constexpr int axis_dirs[NUM_JOINTS] =          {1,
                                                     1, 
                                                     1, 
                                                     1,
                                                     -1,
                                                     1};

    // static constexpr std::string_view command_topic = "/arm/command"; //more modern way, but rclcpp uses c style chars, not cpp strings
    static constexpr char command_topic[] = "/arm/command";
    static constexpr char sim_ee_topic[] = "/arm/ee_command/sim";



    static constexpr char sim_command_topic[] = "/arm/sim_command";
    static constexpr char joint_states_topic[] = "/joint_states";
    static constexpr char joy_topic[] = "/joy";
    
    //Moveit topics
    static constexpr char servo_ik_topic[] = "/arm_moveit_control/delta_twist_cmds"; //inverse kinematics
    static constexpr char servo_fk_topic[] = "/arm_moveit_control/delta_joint_cmds"; //forward kinematics (joint space)


};


//Can use a namespace if we want:
namespace ArmParams{


}

//From moveit_control.h (the one we use)
//         //? new arm offsets
//   //? Axis 1
//   //? -0.68 -> from online app thing
//   //?  0.2808234691619873 -> read in 
//     axes[0].zero_rad = -0.9608; //? pree good
//     axes[0].dir = 1;

//   //? Axis 2 
//   //? -1.01   ISH - fack
//   //? 0.9290387630462646
//     axes[1].zero_rad = -1.9390; //? ISH
//     axes[1].dir = 1;

//   //? Axis 3
//   //? -0.60 from online app
//   //? 0.7459537386894226
//     axes[2].zero_rad = -1.3460;
//     axes[2].dir = 1;

//   //? Axis 4
//   //? 0.037 from online app
//   //? 2.447824239730835
//     axes[3].zero_rad = -2.4108; //? gear reduction probably wrong
//     axes[3].dir = -1;

//   //? Axis 5
//   //? -0.62 from online app
//   //? 1.585980772972107
//     axes[4].zero_rad = -2.2060;
//     axes[4].dir = 1;

//   //? Axis 6
//     axes[5].zero_rad = 0.0;
//     axes[5].dir = 1;

//From ArmSerialInterface.h (we also use, but is the same as above)
  //? new arm offsets
  //? Axis 1
  //? -0.68 -> from online app thing
  //?  0.2808234691619873 -> read in 
//     axes[0].zero_rad = -0.9608; //? pree good
//     axes[0].dir = 1;

//   //? Axis 2 
//   //? -1.01   ISH - fack
//   //? 0.9290387630462646
//     axes[1].zero_rad = -1.9390; //? ISH
//     axes[1].dir = 1;

//   //? Axis 3
//   //? -0.60 from online app
//   //? 0.7459537386894226
//     axes[2].zero_rad = -1.3460;
//     axes[2].dir = 1;

//   //? Axis 4
//   //? 0.037 from online app
//   //? 2.447824239730835
//     axes[3].zero_rad = -2.4108; //? gear reduction probably wrong
//     axes[3].dir = -1;

//   //? Axis 5
//   //? -0.62 from online app
//   //? 1.585980772972107
//     axes[4].zero_rad = -2.2060;
//     axes[4].dir = 1;

//   //? Axis 6
//     axes[5].zero_rad = 0.0;
//     axes[5].dir = 1;



//From armServoControl.h
    // axes[0].zero_rad = 0.984;
    // axes[0].dir = -1;

    // axes[1].zero_rad = 1.409;
    // axes[1].dir = -1;

    // axes[2].zero_rad = -0.696;
    // axes[2].dir = 1;

    // axes[3].zero_rad = 1.8067995;
    // axes[3].dir = -1;

    // axes[4].zero_rad = -1.002;
    // axes[4].dir = 1;

    // axes[5].zero_rad = -1.375;
    // axes[5].dir = 1;