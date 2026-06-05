#include "drive_control.h"

DriveControlNode::DriveControlNode() : Node("drive_control_node") {

    RCLCPP_INFO(this->get_logger(), "DEBUG: Initializing Drive Control Node");

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", rclcpp::SensorDataQoS(),
    std::bind(&DriveControlNode::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to /joy topic");
}

void DriveControlNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    geometry_msgs::msg::Twist twist_msg;

    twist_msg.linear.x = msg->axes[1] * MAX_LINEAR_SPEED_MPS;
    twist_msg.angular.z = msg->axes[0] * -MAX_ANGULAR_SPEED_MPS;

    cmd_vel_pub_->publish(twist_msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
              
