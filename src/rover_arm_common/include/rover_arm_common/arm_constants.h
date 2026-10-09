
    #include "motor_addressing.h"
    #include <rover_math.h>
    namespace ArmConstants{
    inline constexpr float axis_zero_rads[NUM_AXES] = {0.0,          //* Axis 1 Offset
                                                    0.0,          //* Axis 2 Offset
                                                    0.0,         //* Axis 3 Offset
                                                    0.0, //+PI      // Axis 4 Offset 
                                                    0.0,  //2.2060        //* Axis 5 Offset
                                                    0,  //? Axis 6 Offset
                                                    0};      //? EE axis offset  
    
    inline constexpr int axis_dirs[NUM_AXES] =    {1,
                                                     1, 
                                                     1, 
                                                     1,
                                                     1,
                                                     1,
                                                     1}; //? EE dir
      // inline constexpr int ee_dir = 1;
      // inline constexpr int ee_zero_rads = 0;
      // inline constexpr std::string_view command_topic = "/arm/command"; //more modern way, but rclcpp uses c style chars, not cpp strings
      inline constexpr char command_topic[] = "/arm/command";
      inline constexpr char sim_ee_topic[] = "/arm/ee_command/sim";
      
      
      
      inline constexpr char sim_command_topic[] = "/arm/sim_command";
      inline constexpr char joint_states_topic[] = "/joint_states";
      inline constexpr char joy_topic[] = "/joy";
      
      //Moveit topics
      inline constexpr char servo_ik_topic[] = "/arm_moveit_control/delta_twist_cmds"; //inverse kinematics
      inline constexpr char servo_fk_topic[] = "/arm_moveit_control/delta_joint_cmds"; //forward kinematics (joint space)
      
      // OLD ARM
      namespace Stepper{
              inline constexpr float axis_zero_rads[NUM_AXES] = {-0.9608,          //* Axis 1 Offset
                                                          -1.9390,          //* Axis 2 Offset
                                                          -1.3460,         //* Axis 3 Offset
                                                          -2.4108, //+PI      // Axis 4 Offset 
                                                          2.2060-PI/3,  //2.2060        //* Axis 5 Offset
                                                          0,  //? Axis 6 Offset
                                                          0};      //? EE axis offset  
          
          inline constexpr int axis_dirs[NUM_AXES] =          {1,
                                                          1, 
                                                          1, 
                                                          1,
                                                          -1,
                                                          -1,
                                                          1}; //? EE dir
          };

};