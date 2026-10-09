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
#define NUM_JOINTS 7
#define NUM_JOINTS_NO_EE 6
#define EE_SPEED_SCALE 1


// Uncomment the one you want, comment the one you dont
    #define SELECT_MOTEUS_ARM
    // #define SELECT_OLD_ARM

    
#ifdef SELECT_MOTEUS_ARM
// static float max_joysticks_output_speed_deg[NUM_JOINTS] = {8, 4, 8, 8, 8, 8};
static float max_joysticks_output_speed_deg[NUM_JOINTS] = {7.2, 3.6, 3.6, 3.6, 28.8, 28.8, 30.0};

#endif

#ifdef SELECT_OLD_ARM
static float max_joysticks_output_speed_deg[NUM_JOINTS] = {80, 40, 80, 80, 80, 80};
#endif


namespace MoteusArmParams{
  static constexpr float base_max_accel = 3.0;
  static constexpr float max_accel[NUM_JOINTS] = {base_max_accel,  //A1
                                                  base_max_accel,  //A2
                                                  base_max_accel,  //A3
                                                  base_max_accel,  //A4
                                                  base_max_accel,  //A5
                                                  base_max_accel,  //A6
                                                  base_max_accel}; //A7 (End Effector)
  // No use:
  // static constexpr float base_max_velocity = 3.5;
  // static constexpr float max_velocity[NUM_JOINTS] = { base_max_velocity,  //A1
  //                                                     base_max_velocity,  //A2
  //                                                     base_max_velocity,  //A3
  //                                                     base_max_velocity,  //A4
  //                                                     base_max_velocity,  //A5
  //                                                     base_max_velocity,  //A6
  //                                                     base_max_velocity}; //A7 (End Effector)

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