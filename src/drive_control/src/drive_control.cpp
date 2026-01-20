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
    // Define the desired linear and angular velocities based on the joystick inputs
    geometry_msgs::msg::Twist twist_msg;

    // Get the scaling factor for the desired linear and angular
    //double scaled_speed = msg->axes[2]*MAX_LINEAR_SPEED;

    // Scale the joystick inputs to the desired linear and angular velocity
    float throttle = msg->axes[2] + 1;
    throttle = throttle *100;
    twist_msg.linear.x = msg->axes[1] * throttle;
    twist_msg.angular.z = msg->axes[0] * -throttle;

    // Publish the desired velocities to /cmd_vel
    cmd_vel_pub_->publish(twist_msg);

    // twist_msg.linear.x = msg->axes[1] * MAX_LINEAR_SPEED;
    // twist_msg.angular.z = msg->axes[3] * MAX_ANGULAR_SPEED;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
              
