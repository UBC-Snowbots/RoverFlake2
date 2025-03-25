#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include "rover_msgs/msg/arm_command.hpp"

#define NUM_JOINTS 6

#define POSITION_CONTROL 1
#define VELOCITY_CONTROL 2

#define CONTROL_MODE VELOCITY_CONTROL

class ArmJoy : public rclcpp::Node {
public:
    ArmJoy() : Node("arm_joy_control") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        //command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        arm_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/command", qos);

        joy_vibrator = this->create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/feedback", qos);

        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        ps4_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "/jorsnyuorasniertsnieirsnyuoy", 10, std::bind(&ArmJoy::joy_callback, this, std::placeholders::_1));
        arm_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
            "/arm/feedback", 10, std::bind(&ArmJoy::arm_callback, this, std::placeholders::_1));
    }

    void send_command(float steering_angle, float speed, float acceleration, float jerk) {
   
    }

    void send_gear_command(int gear){
      
    }

    

    // void test_send(){
    //     send_command(0.5, 1.0, 1.0, 0.5);
    //     // rclcpp::logger

    // }

private:
    
    int current_gear = 0;
    int prev_paddleR = 0;
    int prev_paddleL = 0;

    struct Axis{
        float position = 00.00;
        float velocity = 00.00;
        bool homed = 0;
    };
    Axis axes[NUM_JOINTS];

    // rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr command_publisher_;
    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr arm_publisher;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr ps4_subscriber;
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr arm_subscriber;

    rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr joy_vibrator;


    rclcpp::TimerBase::SharedPtr timer_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        rover_msgs::msg::ArmCommand target;
        target.positions.resize(NUM_JOINTS);
        target.velocities.resize(NUM_JOINTS);
        float target_positions[6];
        float inputs[6];

        inputs[2] = msg->axes[0];
        inputs[1] = msg->axes[1];
        inputs[0] = msg->axes[5] - msg->axes[2];
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
            target.velocities[i] = inputs[i] * 100;
        }
        }

         arm_publisher->publish(target);

    }

    void arm_callback(const rover_msgs::msg::ArmCommand::SharedPtr msg){

        for (int i = 0; i < NUM_JOINTS; i++){
            axes[i].position = msg->positions[i];
            // axes[i].velocity = msg->velocities

        }
    }
        

};
