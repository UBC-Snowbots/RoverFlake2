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

    // Create vectors of motor commands that correspond to the calculated speeds for left and right wheels
    std::vector<double> left_wheel_commands = {
        -linear + angular,   // Left Front Wheel
        -linear + angular,   // Left Mid Wheel
        -linear + angular    // Left Rear Wheel
    };
    
    std::vector<double> right_wheel_commands = {
        linear + angular,  // Right Front Wheel
        linear + angular,  // Right Mid Wheel
        linear + angular   // Right Rear Wheel
    };

    // Publish the separate left and right wheel speed messages
    std_msgs::msg::Float64MultiArray left_msg;
    left_msg.data = left_wheel_commands;  // Set left wheel commands data
    left_wheel_pub_->publish(left_msg);   // Publish to "/left_wheel_speeds"

    std_msgs::msg::Float64MultiArray right_msg;
    right_msg.data = right_wheel_commands;  // Set right wheel commands data
    right_wheel_pub_->publish(right_msg);   // Publish to "/right_wheel_speeds"
}

// Main function to initialize the ROS2 node and start processing
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);  // Initialize the ROS2 system
    auto node = std::make_shared<WheelSpeedNode>();  // Create a shared pointer to an instance of the WheelSpeed node
    rclcpp::spin(node);  // Start spinning to process incoming messages and callbacks
    rclcpp::shutdown();  // Shutdown ROS2 when done
    return 0;  // Return 0 to indicate successful execution
}
