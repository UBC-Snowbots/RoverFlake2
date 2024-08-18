#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
// #include "std_msgs/msg/float64_multi_array.hpp"

class ChassisJoyCtrl : public rclcpp::Node{

public:
    ChassisJoyCtrl() : Node("chassis_joy_ctrl_node"){
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&ChassisJoyCtrl::joyCallback, this, std::placeholders::_1));
  
    }

    ~ChassisJoyCtrl(){
        RCLCPP_WARN(this->get_logger(), "Chassis Joy Control Offline. Womp Womp.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);


};

