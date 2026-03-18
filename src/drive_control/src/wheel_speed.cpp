# include "wheel_speed.h"

// Constructor definition for WheelSpeed class
WheelSpeedNode::WheelSpeedNode() : Node("wheel_speed_node") {  // Initialize the node with the name "wheel_speed"
    RCLCPP_INFO(this->get_logger(), "Initializing Motor Controller");  // Log that the motor controller is initializing

    // Create publishers to send left and right wheel speed messages on separate topics
    left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("left_wheel_speeds", 10);
    right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("right_wheel_speeds", 10);
    
    // Create a subscription to receive Twist messages (which contain velocity commands) from the "/cmd_vel" topic
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&WheelSpeedNode::cmdVelCallback, this, std::placeholders::_1));  // Call cmdVelCallback when a new message is received

    // Create a subscription to the drive/feedback topic to keep track of wheel speed history for Schmitt trigger
    drive_feedback_sub_ = this->create_subscription<rover_msgs::msg::DriveFeedback>(
        "drive/feedback", 10, std::bind(&WheelSpeedNode::driveFeedbackCallback, this, std::placeholders::_1));
}

// Callback function that processes velocity commands received in the form of Twist messages
void WheelSpeedNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Extract the linear and angular velocity from the incoming message
    double linear = msg->linear.x;
    double angular = msg->angular.z;
    
    // // Calculate a factor to adjust wheel speed based on the wheel radius and angular velocities
    // double wheel_speed_factor = (360.0 / WHEEL_RADIUS_METERS) * (M_PI / 2.0) * 0.0001;  // Placeholder for speed scaling
    double wheel_speed_factor =  0.1;  // Placeholder for speed scaling

    // Apply the speed factor to both linear and angular velocities
    linear *= wheel_speed_factor;
    angular *= wheel_speed_factor;

    WheelSpeeds target_wheel_speeds = tankDrive(linear, angular);
    target_wheel_speeds = applyHysteresis(current_wheel_speeds, target_wheel_speeds);
    // WheelSpeeds wheel_speeds = ackermann(linear, angular);

    // Publish the separate left and right wheel speed messages
    std_msgs::msg::Float64MultiArray left_msg;
    left_msg.data = target_wheel_speeds.left_wheel_speeds;  // Set left wheel commands data
    left_wheel_pub_->publish(left_msg);   // Publish to "/left_wheel_speeds"

    std_msgs::msg::Float64MultiArray right_msg;
    right_msg.data = target_wheel_speeds.right_wheel_speeds;  // Set right wheel commands data
    right_wheel_pub_->publish(right_msg);   // Publish to "/right_wheel_speeds"
}

void WheelSpeedNode::driveFeedbackCallback(const rover_msgs::msg::DriveFeedback::SharedPtr msg) {
    for (int i = 0; i < NUM_MOTORS / 2; i++) {
        current_wheel_speeds.left_wheel_speeds[i] = msg->velocities[i];
        current_wheel_speeds.right_wheel_speeds[i] = msg->velocities[NUM_MOTORS / 2 + i];
    }
}

WheelSpeeds WheelSpeedNode::tankDrive(double linear, double angular) {
    double l = -linear + angular;
    double r = linear + angular;

    WheelSpeeds wheel_speeds;
    wheel_speeds.left_wheel_speeds = { l, l, l };
    wheel_speeds.right_wheel_speeds = { r, r, r };

    return wheel_speeds;
}

WheelSpeeds WheelSpeedNode::applyHysteresis(WheelSpeeds current_wheel_speeds, WheelSpeeds target_wheel_speeds) {
    WheelSpeeds speeds;
    speeds.left_wheel_speeds.resize(NUM_MOTORS / 2);
    speeds.right_wheel_speeds.resize(NUM_MOTORS / 2);

    // Lambda to apply hysteresis logic to one motor, given its current and target velocity
    auto applyHysteresis = [&](double current_vel, double target_vel) {
        double abs_current_vel = std::abs(current_vel);
        double abs_target_vel = std::abs(target_vel);

        if (abs_target_vel < MOTOR_STOP_THRESHOLD) return 0.0; // Small enough target velocity that we should normalize it to 0
        else if (abs_target_vel < MOTOR_START_THRESHOLD) {
            if (abs_current_vel > MOTOR_STOP_THRESHOLD) {
                // Keep moving in same direction at MOTOR_START_THRESHOLD
                return (target_vel > 0) ? MOTOR_START_THRESHOLD : -MOTOR_START_THRESHOLD;
            }
            else return 0.0; // Keep not moving
        }
        else return target_vel; // Target velocity is high enough to use it directly
    };

    for (int i = 0; i < NUM_MOTORS / 2; i++) {
        speeds.left_wheel_speeds[i] = applyHysteresis(current_wheel_speeds.left_wheel_speeds[i], target_wheel_speeds.left_wheel_speeds[i]);
        speeds.right_wheel_speeds[i] = applyHysteresis(current_wheel_speeds.right_wheel_speeds[i], target_wheel_speeds.right_wheel_speeds[i]);
    }

    return speeds;
}

WheelSpeeds WheelSpeedNode::ackermann(double linear, double angular) {
    // Tank drive as placeholder
    double l = -linear + angular;
    double r = linear + angular;

    WheelSpeeds wheel_speeds;
    wheel_speeds.left_wheel_speeds = { l, l, l };
    wheel_speeds.right_wheel_speeds = { r, r, r };

    return wheel_speeds;
}

// Main function to initialize the ROS2 node and start processing
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);  // Initialize the ROS2 system
    auto node = std::make_shared<WheelSpeedNode>();  // Create a shared pointer to an instance of the WheelSpeed node
    rclcpp::spin(node);  // Start spinning to process incoming messages and callbacks
    rclcpp::shutdown();  // Shutdown ROS2 when done
    return 0;  // Return 0 to indicate successful execution
}
