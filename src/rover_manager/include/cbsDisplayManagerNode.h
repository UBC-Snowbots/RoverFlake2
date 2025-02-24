#include "rclcpp/rclcpp.hpp"



class cbsDisplayManagerNode : public rclcpp::Node {
public:
    cbsDisplayManagerNode() : Node("cbsDisplayManagerNode") {
        // auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
        //command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        //gear_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", qos);
        std::system("notify-send 'CBS ONLINE'");
        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        // g29_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        //     "/joy", 10, std::bind(&SampleNode::joy_callback, this, std::placeholders::_1));
    }

    

    // void test_send(){
    //     send_command(0.5, 1.0, 1.0, 0.5);
    //     // rclcpp::logger

    // }

private:
  
    void moveWindow(std::string window_name, bool fullscreen);

    // rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_publisher_;
    // rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr g29_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

 
};

