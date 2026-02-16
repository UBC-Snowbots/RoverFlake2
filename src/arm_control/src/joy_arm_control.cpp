#include "joy_arm_control.h"

// Constructor
ArmJoy::ArmJoy() : 
Node("arm_joy_control") 
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    arm_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/command", qos);

    joy_vibrator = this->create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/feedback", qos);

    // timer_ = this->create_wall_timer(
    // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
    ps4_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&ArmJoy::joy_callback, this, std::placeholders::_1));
    arm_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
        "/arm/feedback", 10, std::bind(&ArmJoy::arm_callback, this, std::placeholders::_1));
}

// Program Entry Point
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmJoy>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}


// Member Functions

void ArmJoy::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
    rover_msgs::msg::ArmCommand target;
    target.positions.resize(NUM_JOINTS);
    target.velocities.resize(NUM_JOINTS);
    float target_positions[6];
    float inputs[6];

    // inputs[2] = msg->axes[0];
    // inputs[1] = msg->axes[1];
    // inputs[0] = msg->axes[5] - msg->axes[2];
    // inputs[3] = msg->axes[3];
    // inputs[4] = msg->axes[4];
    // inputs[5] = msg->axes[6];
    
    inputs[2] = msg->axes[0];
    inputs[1] = msg->axes[1];
    inputs[0] = msg->axes[5] - msg->axes[4];
    inputs[3] = msg->axes[3];
    inputs[4] = msg->axes[4];
    inputs[5] = msg->axes[6];

    if(CONTROL_MODE == POSITION_CONTROL){
        target.cmd_type = 'P';

    for (int i = 0; i < NUM_JOINTS; i++){
        target.positions[i] = axes[i].position + inputs[i] * 10;
    }
    }else if(CONTROL_MODE == VELOCITY_CONTROL){
        target.cmd_type = 'V';
    for (int i = 0; i < NUM_JOINTS; i++){
        target.velocities[i] = inputs[i] / 10;
    }
    }
    target.velocities[3] = 0;
    target.velocities[4] = 0;
    target.velocities[5] = 0;
        arm_publisher->publish(target);

}


void ArmJoy::arm_callback(const rover_msgs::msg::ArmCommand::SharedPtr msg){

    for (int i = 0; i < NUM_JOINTS; i++){
        axes[i].position = msg->positions[i];
        // axes[i].velocity = msg->velocities

    }
}