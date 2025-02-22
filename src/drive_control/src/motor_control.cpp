#include "motor_control.h"  // Include the header file for motor control functionalities

/**
 * @brief Construct a new MotorControlNode object
 * Initializes the motor control node, sets up motor resources, and starts a timer.
 */
MotorControlNode::MotorControlNode() : Node("motor_control_node") {  // Constructor for MotorControlNode class, initializing the ROS2 node
    RCLCPP_INFO(this->get_logger(), "Motor Control Node Initiated");  // Log that the node has been initiated

    // Initialize Phidget BLDC motors
    for (int i = 0; i < NUM_MOTORS; i++) {  // Loop through each motor (assuming NUM_MOTORS is defined)
        // Create a motor object using the Phidget API
        ret = PhidgetBLDCMotor_create(&motors[i]);  
        if (ret != EPHIDGET_OK) {
            handlePhidgetError(ret, "creating motor", i);  // Handle errors if motor creation fails
            return;
        }

        // Set the hub port for the motor
        ret = Phidget_setHubPort((PhidgetHandle)motors[i], i);
        handlePhidgetError(ret, "set hub", i);  // Handle errors if setting the hub port fails

        // Wait for motor attachment
        ret = Phidget_openWaitForAttachment((PhidgetHandle)motors[i], 5000);  // Timeout set to 5000 ms
        handlePhidgetError(ret, "attachment", i);  // Handle errors if attachment fails
    }

    // Set up a timer to periodically check motor positions (every 100 ms)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MotorControlNode::check_motor_positions, this)  // Call the check_motor_positions function
    );

    // Enable failsafe for all motors to prevent motor runaway situations
    for (size_t i = 0; i < NUM_MOTORS; i++) {
        ret = PhidgetBLDCMotor_enableFailsafe(motors[i], 612);  // Set failsafe timeout to 612 ms
        handlePhidgetError(ret, "enable failsafe", i);  // Handle errors if failsafe enablement fails
    }

    // Create subscribers for left and right wheel velocity commands
    left_wheel_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "left_wheel_pub", 10, std::bind(&MotorControlNode::leftWheelCallback, this, std::placeholders::_1));

    right_wheel_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "right_wheel_pub", 10, std::bind(&MotorControlNode::rightWheelCallback, this, std::placeholders::_1));
}

/**
 * @brief Callback function for the left wheel velocity
 * @param msg The velocity message for the left wheel
 */
void MotorControlNode::leftWheelCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    // Set the left motor velocity based on the message
    float left_velocity = msg->data;  // Extract velocity value
    run_motors({0, 2, 3}, left_velocity);  // Update the velocity for the left motors (assuming motor indices for left)
}

/**
 * @brief Callback function for the right wheel velocity
 * @param msg The velocity message for the right wheel
 */
void MotorControlNode::rightWheelCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    // Set the right motor velocity based on the message
    float right_velocity = msg->data;  // Extract velocity value
    run_motors({1, 4, 5}, right_velocity);  // Update the velocity for the right motors (assuming motor indices for right)
}

/**
 * @brief Destroy the MotorControlNode object
 * Cleans up motor resources and shuts down the node.
 */
MotorControlNode::~MotorControlNode() {
    RCLCPP_WARN(this->get_logger(), "Motor Control Node shutting down.");  // Log warning when shutting down

    // Clean up motor resources if necessary
    for (size_t i = 0; i < NUM_MOTORS; i++) {
        PhidgetBLDCMotor_delete(&motors[i]);  // Delete motor object
    }
}

/**
 * @brief Handle Phidget errors and log appropriately
 * @param ret Return code from Phidget functions
 * @param action Action that was being performed (e.g., creating motor, setting velocity)
 * @param i Motor index
 * 
 * Handles and logs any errors that occur during Phidget function calls.
 */
void MotorControlNode::handlePhidgetError(PhidgetReturnCode ret, const std::string& action, int i) {
    if (ret != EPHIDGET_OK) {  // If there's an error
        // Get detailed error information
        Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
        // Log the error
        RCLCPP_ERROR(this->get_logger(), "Error at %s (%d) for motor %d: %s", action.c_str(), errorCode, i, errorString);
    } else {
        RCLCPP_INFO(this->get_logger(), "%s successful for motor %d", action.c_str(), i);  // Log success
    }
}

/**
 * @brief Run selected motors at a specified velocity
 * @param selected_motors Vector of motor indices to run
 * @param velocity Velocity to set (clamped between -1.0 and 1.0)
 * 
 * Runs the selected motors at the specified velocity, ensuring the velocity is within the valid range.
 */
void MotorControlNode::run_motors(std::vector<int> selected_motors, float velocity) {
    // Enable logging for debugging purposes
    PhidgetLog_enable(PHIDGET_LOG_INFO, "phidgetlog.log");

    // Ensure velocity is within the valid range (-1.0 to 1.0)
    velocity = std::clamp(velocity, -1.0f, 1.0f);

    // Loop through each selected motor and set its target velocity
    for (int motor_index : selected_motors) {
        ret = PhidgetBLDCMotor_setTargetVelocity(motors[motor_index], velocity);  // Set target velocity for motor
        if (ret != EPHIDGET_OK) {  // If there's an error
            // Get detailed error information
            Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
            // Log the error
            RCLCPP_ERROR(this->get_logger(), "Error at set target velocity (%d) for motor index %d: %s", errorCode, motor_index, errorString);
            return;  // Return early if an error occurs
        }
    }
}

/**
 * @brief Periodically check and log the positions of all motors and enforce velocity to 1.0
 * 
 * This function logs the position of each motor periodically and adjusts its velocity if needed.
 */
void MotorControlNode::check_motor_positions() {
    // Periodically log motor positions
    for (int i = 0; i < NUM_MOTORS; i++) {
        double position = 0.0;
        ret = PhidgetBLDCMotor_getPosition(motors[i], &position);  // Get the position of the motor
        if (ret != EPHIDGET_OK) {  // If there's an error
            // Get detailed error information
            Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
            // Log the error
            RCLCPP_ERROR(this->get_logger(), "Error getting position for motor %d: %s", i, errorString);
            return;  // Return early if an error occurs
        }

        // Log the position of the motor
        RCLCPP_INFO(this->get_logger(), "Motor %d Position: %f", i, position * 1.3666);  // Example conversion factor
    }
}

/**
 * @brief Main function to initialize and run the ROS2 node
 * 
 * Initializes the ROS2 system, creates the MotorControlNode, and starts the node execution.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // Initialize the ROS2 system
    auto node = std::make_shared<MotorControlNode>();  // Create the MotorControlNode object
    rclcpp::spin(node);  // Start processing incoming messages and callbacks
    rclcpp::shutdown();  // Shutdown ROS2 when done
    return 0;  // Return 0 to indicate successful execution
}
