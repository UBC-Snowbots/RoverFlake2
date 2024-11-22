/*
Created by: Rowan Zawadzki (Roozki)
Created on a date i dont remember
purpose: to handle moveit control, as well as servo. 
*/

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_command.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

// Servo
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <armControlParams.h>

#define JOINT_JOG 1
#define CARTESIAN_EE_FRAME 2
#define CARTESIAN_BASE_FRAME 3

class ArmMoveitControl : public rclcpp::Node {
public:
    //rclcpp::NodeOptions node_options;
    //node_options.use_intra_process_comms(false);
    ArmMoveitControl() : Node("arm_moveit_control") {
        //? new arm offsets
  //? Axis 1
  //? -0.68 -> from online app thing
  //?  0.2808234691619873 -> read in 
  //Nice and neat offset setting. Probably don't even need the axes[] structs here, and should just use directly from ArmParams struct. But also kind of nice to use axis[i]
  for(int i = 0; i < NUM_JOINTS; i++){
    axes[i].zero_rad = ArmConstants::axis_zero_rads[i]; 
    axes[i].dir = ArmConstants::axis_dirs[i];
    #ifdef PRINTOUT_AXIS_PARAMS
      RCLCPP_INFO(this->get_logger(), "Axis %i /// DIR[ %i ] /// OFFSET TO URDF's ZERO_RAD[ %f ] ", i+1, axes[i].dir, axes[i].zero_rad);
    #endif
  }
    

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        //command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        arm_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>(ArmConstants::command_topic, qos);

        // joy_vibrator = this->create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/feedback", qos);

        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        trajectory_subscriber = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
            "/dev_arm_controller/controller_state", 10, std::bind(&ArmMoveitControl::jointTrajectoryCallback, this, std::placeholders::_1));
        joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            ArmConstants::joy_topic, 10, std::bind(&ArmMoveitControl::joyCallback, this, std::placeholders::_1));
        
	joint_cmd_publisher = this->create_publisher<control_msgs::msg::JointJog>(ArmConstants::servo_fk_topic, 10);
	twist_cmd_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(ArmConstants::servo_ik_topic, 10);
        // arm_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
        //     "/arm/feedback", 10, std::bind(&ArmMoveitControl::arm_callback, this, std::placeholders::_1));
// timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ArmMoveitControl::publishCommands, this));

    }

int count_ = 0;    
void publishCommands();	
void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
int joyControlMode = CARTESIAN_EE_FRAME;

    // void test_send(){
    //     send_command(0.5, 1.0, 1.0, 0.5);
    //     // rclcpp::logger

    // }

private:
    // rclcpp::TimerBase::SharedPtr timer;
   float radToDeg(float rad);
   float moveitToFirmwareOffset(float rad, int i);
   float moveitVelocityToFirmwareOffset(float rad, int i);
//    float moveitToFirmwareOffset(float rad, int i);


    struct Axis{
        float position = 00.00;
        float velocity = 00.00;
        bool homed = 0;
        float zero_rad;
        int dir;
    };
    Axis axes[NUM_JOINTS];

    // rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr command_publisher_;
    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr arm_publisher;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_publisher;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr trajectory_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
    // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr trajectory_subscriber;

    // rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr arm_subscriber;

    // rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr joy_vibrator;


    rclcpp::TimerBase::SharedPtr timer_;
    // void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
    //     rover_msgs::msg::ArmCommand target;
    //     target.positions.resize(NUM_JOINTS);
    //     target.cmd_type = 'P';
    //     float target_positions[6];
    //     float inputs[6];

    
    //     for (int i = 0; i < NUM_JOINTS; i++){
    //         // float temp_pos = msg->position[i];
    //         target.positions[i] = moveitToFirmwareOffset(msg->position[i], i);
    //     }

    //      arm_publisher->publish(target);

    // }

    void jointTrajectoryCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg){
        rover_msgs::msg::ArmCommand target;
        //TODO position or vel
        // target.positions.resize(NUM_JOINTS);
        target.velocities.resize(NUM_JOINTS);
        target.cmd_type = 'V';
        // float target_positions[NUM_JOINTS];
        // float target_velocities[NUM_JOINTS];
        float inputs[NUM_JOINTS];

    
        for (int i = 0; i < NUM_JOINTS; i++){
            // float temp_pos = msg->position[i];
            // target.positions[i] = moveitToFirmwareOffset(msg->reference.positions[i], i);
            target.velocities[i] = moveitVelocityToFirmwareOffset(msg->reference.velocities[i], i);

        }

         arm_publisher->publish(target);

    }

    void arm_callback(const rover_msgs::msg::ArmCommand::SharedPtr msg){

        for (int i = 0; i < NUM_JOINTS; i++){
            // axes[i].position = msg->positions[i];
            // axes[i].position = msg->positions[i];

        }
    }
        

};
