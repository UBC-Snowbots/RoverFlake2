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

    // Create a subscription to the drive/feedback topic to keep track of wheel speed history for Schmitt trigger
    drive_feedback_sub_ = this->create_subscription<rover_msgs::msg::DriveFeedback>(
        "drive/feedback", 10, std::bind(&WheelSpeedNode::driveFeedbackCallback, this, std::placeholders::_1));
}

// Callback function that processes velocity commands received in the form of Twist messages
void WheelSpeedNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Extract the linear and angular velocity from the incoming message
    double linear = msg->linear.x;
    double angular = msg->angular.z;

    WheelVelocities wheel_velocities = tankDrive(linear, angular);
    wheel_velocities = applyHysteresis(current_wheel_velocities, wheel_velocities);

    // Publish the separate left and right wheel velocity messages
    std_msgs::msg::Float64MultiArray left_msg;
    left_msg.data = wheel_velocities.left_wheel_velocities;
    left_wheel_pub_->publish(left_msg);

    std_msgs::msg::Float64MultiArray right_msg;
    right_msg.data = wheel_velocities.right_wheel_velocities;
    right_wheel_pub_->publish(right_msg);
}

void WheelSpeedNode::driveFeedbackCallback(const rover_msgs::msg::DriveFeedback::SharedPtr msg) {
    for (int i = 0; i < NUM_MOTORS / 2; i++) {
        current_wheel_velocities.left_wheel_velocities[i] = msg->velocities[i];
        current_wheel_velocities.right_wheel_velocities[i] = msg->velocities[NUM_MOTORS / 2 + i];
    }
}

WheelVelocities WheelSpeedNode::tankDrive(double linear, double angular) {
    WheelVelocities wheel_velocities;

    double l = -linear + angular;
    double r = linear + angular;

    wheel_velocities.left_wheel_velocities = { l, l, l };

    wheel_velocities.right_wheel_velocities = { r, r, r };

    return wheel_velocities;
}

WheelVelocities WheelSpeedNode::applyHysteresis(WheelVelocities current, WheelVelocities target) {
    WheelVelocities final_wheel_velocities;
    final_wheel_velocities.left_wheel_velocities.resize(NUM_MOTORS / 2);
    final_wheel_velocities.right_wheel_velocities.resize(NUM_MOTORS / 2);

    auto applyHysteresisHelper = [&](double current, double target) {
        double abs_current = std::abs(current);
        double abs_target = std::abs(target);

        if (abs_target < MOTOR_STOP_THRESHOLD) return 0.0;
        else if (abs_target < MOTOR_START_THRESHOLD) {
            if (abs_current > MOTOR_STOP_THRESHOLD) {
                // Keep moving in same direction at MOTOR_START_THRESHOLD
                return (target > 0) ? MOTOR_START_THRESHOLD : -MOTOR_START_THRESHOLD;
            }
            else return 0.0; // Keep not moving
        }
        else return target;
    };

    for (int i = 0; i < NUM_MOTORS / 2; i++) {
        final_wheel_velocities.left_wheel_velocities[i] = applyHysteresisHelper(current.left_wheel_velocities[i], target.left_wheel_velocities[i]);
        final_wheel_velocities.right_wheel_velocities[i] = applyHysteresisHelper(current.right_wheel_velocities[i], target.right_wheel_velocities[i]);
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
