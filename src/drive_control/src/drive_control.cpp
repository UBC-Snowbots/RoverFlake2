#include "drive_control.h"


DriveControlNode::DriveControlNode() : Node("drive_control_node") {

    RCLCPP_INFO(this->get_logger(), "DEBUG: Initializing Drive Control Node");

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", rclcpp::SensorDataQoS(),
    std::bind(&DriveControlNode::joyCallback, this, std::placeholders::_1));

    button_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("button_state",10);

    RCLCPP_INFO(this->get_logger(), "Subscribed to /joy topic");


}

void DriveControlNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    geometry_msgs::msg::Twist twist_msg;
    std_msgs::msg::Bool button_state_msg;

    // Check if button 1 (brake button) is pressed
    bool brake_button_pressed = (msg->buttons[1] == 1);  // Assuming button 1 is the brake button

    // Publish the button state message
    button_state_msg.data = brake_button_pressed;  // Set the button state (true or false)
    button_state_pub_->publish(button_state_msg);

    // Check if button 1 ( brake button) is pressed
    if (brake_button_pressed) {
        // When the brake button is pressed, set the input velocities to zero
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
    } else {
        // Control rover normally when button 1 is not pressed
        twist_msg.linear.x = msg->axes[1];
        twist_msg.angular.z = msg->axes[0];
    }

    // Publish the velocity to /cmd_vel
    cmd_vel_pub_->publish(twist_msg);
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
              
