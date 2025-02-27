//this is a sample node, with a joy input
#include "sample_node.h"


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"



class SampleNode : public rclcpp::Node {
public:
    SampleNode() : Node("sample_node") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        //command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        //gear_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", qos);

        double period = 1.0/CONTROL_RATE;
        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        g29_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&SampleNode::joy_callback, this, std::placeholders::_1));
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
    int gears[4] = {GEAR_PARKING, GEAR_REVERSE, GEAR_NEUTRAL, GEAR_1};
    int current_gear = 0;
    int prev_paddleR = 0;
    int prev_paddleL = 0;

    // rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr command_publisher_;
    // rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr g29_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        float steering_angle = msg->axes[0];
        float speed = 99.0 - (msg->axes[3] + 1)*100;
        float acceleration = (msg->axes[2] + 1)*20 - (msg->axes[3] + 1)*20;
        float jerk = (msg->axes[2] + 1)*20 - (msg->axes[3] + 1)*20;
        int paddleR = msg->buttons[4];
        int paddleL = msg->buttons[5];
        
        if(paddleL == 0){
            prev_paddleL = 0;
        }
        if(paddleR == 0){
            prev_paddleR = 0;
        }

        if(paddleR){
        if(prev_paddleR != paddleR){
            prev_paddleR = paddleR;
            current_gear++;
        }
        }else if(paddleL){
        if(prev_paddleL != paddleL){
            prev_paddleL = paddleL;
            current_gear--;
        }
        }
        if(current_gear < 0){
            current_gear = 0;
        }

        if(current_gear > 4){
            current_gear = 4;
        }

        send_command(steering_angle, speed, acceleration, jerk);
        send_gear_command(gears[current_gear]);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SampleNode>();

    // Example: send a command with a steering angle of 0.5 rad and speed of 1.0 m/s

    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}