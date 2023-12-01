#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#define NUM_JOINTS 6

class ArmMoveitControl : public rclcpp::Node {
public:
    ArmMoveitControl() : Node("arm_moveit_control") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        //command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        arm_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/command", qos);

        joy_vibrator = this->create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/feedback", qos);

        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        trajectory_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ArmMoveitControl::jointStateCallback, this, std::placeholders::_1));


        // arm_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
        //     "/arm/feedback", 10, std::bind(&ArmMoveitControl::arm_callback, this, std::placeholders::_1));
    }

    

    // void test_send(){
    //     send_command(0.5, 1.0, 1.0, 0.5);
    //     // rclcpp::logger

    // }

private:
    
   
    struct Axis{
        float position = 00.00;
        float velocity = 00.00;
        bool homed = 0;
    };
    Axis axes[NUM_JOINTS];

    // rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr command_publisher_;
    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr arm_publisher;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr trajectory_subscriber;

    // rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr arm_subscriber;

    // rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr joy_vibrator;


    rclcpp::TimerBase::SharedPtr timer_;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
        rover_msgs::msg::ArmCommand target;
        target.positions.resize(NUM_JOINTS);
        target.cmd_type = 'P';
        float target_positions[6];
        float inputs[6];

    
        for (int i = 0; i < NUM_JOINTS; i++){
            target.positions[i] = msg;
        }

         arm_publisher->publish(target);

    }

    void arm_callback(const rover_msgs::msg::ArmCommand::SharedPtr msg){

        for (int i = 0; i < NUM_JOINTS; i++){
            axes[i].position = msg->positions[i];

        }
    }
        

};
