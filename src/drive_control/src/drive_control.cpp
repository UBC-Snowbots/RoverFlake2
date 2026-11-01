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
    // Map the raw joy message to standardized drive inputs for the selected controller
    ArmControllerConfig::DriveControlInput control_input = {};
    if (!ArmControllerConfig::process_joy_input(this->game_controller, msg, control_input)) {
        RCLCPP_ERROR(this->get_logger(), "Failure to parse gamepad input. Disregarding joy message");
        return;
    }

    geometry_msgs::msg::Twist twist_msg;
    float linear_speed = MAX_LINEAR_SPEED_MPS;
    if (control_input.turbo)
    {
        linear_speed = linear_speed * TURBO_SPEED_MULTIPLIER;
    }
    twist_msg.linear.x = control_input.linear * linear_speed * -1;
    twist_msg.angular.z = control_input.angular * -MAX_ANGULAR_SPEED_MPS;

    cmd_vel_pub_->publish(twist_msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
