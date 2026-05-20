# include "wheel_speed.h"

// Constructor definition for WheelSpeed class
WheelSpeedNode::WheelSpeedNode() : Node("wheel_speed_node") {  // Initialize the node with the name "wheel_speed"
    RCLCPP_INFO(this->get_logger(), "Initializing Motor Controller");  // Log that the motor controller is initializing

    // Create publishers to send left and right wheel speed messages on separate topics
    left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("left_wheel_speeds", 10);
    right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("right_wheel_speeds", 10);
    
    // Create a subscription to receive Twist messages (which contain velocity commands) from the "/cmd_vel" topic
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&WheelSpeedNode::cmdVelCallback, this, std::placeholders::_1));
}

// Callback function that processes velocity commands received in the form of Twist messages
void WheelSpeedNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Extract the linear and angular velocity from the incoming message
    double linear = msg->linear.x;
    double angular = msg->angular.z;

    WheelVelocities wheel_velocities = tankDrive(linear, angular);
    wheel_velocities = preventPivotDeadzone(wheel_velocities);

    // Publish the separate left and right wheel velocity messages
    std_msgs::msg::Float64MultiArray left_msg;
    left_msg.data = wheel_velocities.left_wheel_velocities;
    left_wheel_pub_->publish(left_msg);

    std_msgs::msg::Float64MultiArray right_msg;
    right_msg.data = wheel_velocities.right_wheel_velocities;
    right_wheel_pub_->publish(right_msg);
}

WheelVelocities WheelSpeedNode::tankDrive(double linear, double angular) {
    WheelVelocities wheel_velocities;

    double l = -linear + angular;
    double r = linear + angular;

    wheel_velocities.left_wheel_velocities = { l, l, l };

    wheel_velocities.right_wheel_velocities = { r, r, r };

    return wheel_velocities;
}

WheelVelocities WheelSpeedNode::preventPivotDeadzone(WheelVelocities current_wheel_velocities) {
    WheelVelocities final_wheel_velocities;
    final_wheel_velocities.left_wheel_velocities.resize(NUM_MOTORS / 2);
    final_wheel_velocities.right_wheel_velocities.resize(NUM_MOTORS / 2);

    for (int i = 0; i < NUM_MOTORS / 2; i++) {
        double left_vel = current_wheel_velocities.left_wheel_velocities[i];
        double right_vel = current_wheel_velocities.right_wheel_velocities[i];

        if (std::abs(left_vel) < MOTOR_STOP_THRESHOLD && std::abs(right_vel) > MOTOR_START_THRESHOLD) {
            final_wheel_velocities.left_wheel_velocities[i] = right_vel > 0 ? -MOTOR_STOP_THRESHOLD : MOTOR_STOP_THRESHOLD;
        }
        else {
            final_wheel_velocities.left_wheel_velocities[i] = left_vel;
        }

        if (std::abs(right_vel) < MOTOR_STOP_THRESHOLD && std::abs(left_vel) > MOTOR_START_THRESHOLD) {
            final_wheel_velocities.right_wheel_velocities[i] = left_vel > 0 ? -MOTOR_STOP_THRESHOLD : MOTOR_STOP_THRESHOLD;
        }
        else {
            final_wheel_velocities.right_wheel_velocities[i] = right_vel;
        }
    }

    return final_wheel_velocities;
}


WheelVelocities WheelSpeedNode::ackermann(double linear, double angular) {
    WheelVelocities wheel_velocities;

    double l = -linear + angular;
    double r = linear + angular;

    wheel_velocities.left_wheel_velocities = { l, l, l };

    wheel_velocities.right_wheel_velocities = { r, r, r };

    return wheel_velocities;
}

// Main function to initialize the ROS2 node and start processing
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);  // Initialize the ROS2 system
    auto node = std::make_shared<WheelSpeedNode>();  // Create a shared pointer to an instance of the WheelSpeed node
    rclcpp::spin(node);  // Start spinning to process incoming messages and callbacks
    rclcpp::shutdown();  // Shutdown ROS2 when done
    return 0;  // Return 0 to indicate successful execution
}
