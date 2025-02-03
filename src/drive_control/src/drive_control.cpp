#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

constexpr double MAX_LINEAR_SPEED = 10.0;  // Maximum linear speed (m/s)
constexpr double MAX_ANGULAR_SPEED = 10.0; // Maximum angular speed (rad/s)

class DriveControlNode : public rclcpp::Node {
public:
    DriveControlNode();

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

DriveControlNode::DriveControlNode() : Node("drive_control_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing Drive Control Node");

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", rclcpp::SensorDataQoS(),
    std::bind(&DriveControlNode::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to /joy topic");
}

void DriveControlNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Define the desired linear and angular velocities based on the joystick inputs
    geometry_msgs::msg::Twist twist_msg;

    twist_msg.linear.x = msg->axes[1] * MAX_LINEAR_SPEED;
    twist_msg.angular.z = msg->axes[3] * MAX_ANGULAR_SPEED;

    // Publish the desired velocities to /cmd_vel
    cmd_vel_pub_->publish(twist_msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
              
