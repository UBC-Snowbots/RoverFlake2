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
#include "trajectory_msgs/msg/joint_trajectory.hpp"

// Servo
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <armControlParams.h>

#define JOINT_JOG 1
#define CARTESIAN_EE_FRAME 2
#define CARTESIAN_BASE_FRAME 3

#define PI 3.14159
class ArmMoveitControl : public rclcpp::Node {
public:
    //rclcpp::NodeOptions node_options;
    //node_options.use_intra_process_comms(false);
    ArmMoveitControl() : Node("arm_moveit_control") {
        //? new arm offsets. 
  //? Axis 1
  //? -0.68 -> from online app thing
  //?  0.2808234691619873 -> read in 
    axes[0].zero_rad = -0.9608; //? pree good
    axes[0].dir = 1;

  //? Axis 2 
  //? -1.01   ISH - fack
  //? 0.9290387630462646
    axes[1].zero_rad = -1.9390; //? ISH
    axes[1].dir = 1;

  //? Axis 3
  //? -0.60 from online app
  //? 0.7459537386894226
    axes[2].zero_rad = -1.3460;
    axes[2].dir = 1;

  //? Axis 4
  //? 0.037 from online app
  //? 2.447824239730835
    axes[3].zero_rad = 2.4108 - PI; //? gear reduction probably wrong
    axes[3].dir = -1;

  //? Axis 5
  //? -0.62 from online app
  //? 1.585980772972107
    axes[4].zero_rad = 2.2060 - PI/2;
    axes[4].dir = -1;

  //? Axis 6
    axes[5].zero_rad = 0.0;
    axes[5].dir = 1;
    //?old arm offsets
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
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        //command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        arm_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/command", qos);

        // joy_vibrator = this->create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/feedback", qos);

        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        // trajectory_subscriber = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        //     "/dev_arm_controller/controller_state", 10, std::bind(&ArmMoveitControl::jointTrajectoryCallback, this, std::placeholders::_1));
         joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&ArmMoveitControl::joyCallback, this, std::placeholders::_1));
          servo_output_subscriber = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/dev_arm_controller/joint_trajectory", qos, std::bind(&ArmMoveitControl::servoCallback, this, std::placeholders::_1));
        
	joint_cmd_publisher = this->create_publisher<control_msgs::msg::JointJog>("/arm_moveit_control/delta_joint_cmds", 10);
	twist_cmd_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("/arm_moveit_control/delta_twist_cmds", 10);
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
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr servo_output_subscriber;
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
        if(msg->output.velocities.size() == NUM_JOINTS){
     for (int i = 0; i < NUM_JOINTS; i++){
            // float temp_pos = msg->position[i];
            // target.positions[i] = moveitToFirmwareOffset(msg->reference.positions[i], i);
            //target.velocities[i] = moveitVelocityToFirmwareOffset(msg->desired.velocities[i], i);

        }

        // arm_publisher->publish(target);
        }else{
          RCLCPP_ERROR(this->get_logger(), "Joint Trajrectory Controller going wack. Output does not match size of joints. Ignoring this message.");
        }
    
   

    }

    void servoCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg){
              rover_msgs::msg::ArmCommand target;
        //TODO position or vel
        // target.positions.resize(NUM_JOINTS);
        target.velocities.resize(NUM_JOINTS);
        target.cmd_type = 'V';
        // float target_positions[NUM_JOINTS];
        // float target_velocities[NUM_JOINTS];
        float inputs[NUM_JOINTS];
        if(msg->points[0].velocities.size() == NUM_JOINTS){
     for (int i = 0; i < NUM_JOINTS; i++){
            // float temp_pos = msg->position[i];
            // target.positions[i] = moveitToFirmwareOffset(msg->reference.positions[i], i);
            target.velocities[i] = moveitVelocityToFirmwareOffset(msg->points[0].velocities[i], i);
        // RCLCPP_INFO(this->get_logger(), "J%i, %lf", i, target.velocities[i]);

        }   
         arm_publisher->publish(target);
          // RCLCPP_INFO(this->get_logger(), "Joint Trajrectory Controller good. Servo is commanding arm!");

    
        }else{
          RCLCPP_ERROR(this->get_logger(), "Joint Trajrectory Controller going wack. Output does not match size of joints. Ignoring this message.");
        }
    
    }

    void arm_callback(const rover_msgs::msg::ArmCommand::SharedPtr msg){

        for (int i = 0; i < NUM_JOINTS; i++){
            // axes[i].position = msg->positions[i];
            // axes[i].position = msg->positions[i];

        }
    }
        

};
