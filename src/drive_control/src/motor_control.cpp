#include "motor_control.h"  // Include the header file for motor control functionalities
#include <algorithm>          // For std::clamp

/**
 * @brief Construct a new MotorControlNode object
 * Initializes the motor control node, sets up motor resources, and starts timers.
 */
MotorControlNode::MotorControlNode() : Node("motor_control_node") {
    RCLCPP_INFO(this->get_logger(), "Motor Control Node Initiated");

    // Initialize Phidget Motors and Velocity Controllers
    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetReturnCode ret;

        ret = PhidgetMotorPositionController_create(&motors[i]);
        handlePhidgetError(ret, "creating motor", i);

        ret = Phidget_setHubPort((PhidgetHandle)motors[i], i);
        handlePhidgetError(ret, "setting motor hub port", i);

        ret = Phidget_openWaitForAttachment((PhidgetHandle)motors[i], 5000);
        handlePhidgetError(ret, "opening motor connection", i);

        ret = PhidgetMotorPositionController_setNormalizePID(motors[i], 1);
        handlePhidgetError(ret, "normalizing PID controller", i);

        // Setup motor position settings so wheels are stopped by default
        double position;
        ret = PhidgetMotorPositionController_getPosition(motors[i], &position);
        handlePhidgetError(ret, "getting initial motor position", i);

        target_positions[i] = position;

        ret = PhidgetMotorPositionController_setTargetPosition(motors[i], position);
        handlePhidgetError(ret, "setting initial motor target position", i);

        PhidgetMotorPositionController_setEngaged(motors[i], 1);
        handlePhidgetError(ret, "engaging motor", i);

        ret = PhidgetMotorPositionController_setRescaleFactor(motors[i], MOTOR_RESCALE_FACTOR);
        handlePhidgetError(ret, "setting motor rescale factor", i);

        ret = PhidgetMotorPositionController_setVelocityLimit(motors[i], MAX_VELOCITY_RADS);
        handlePhidgetError(ret, "setting motor max velocity", i);
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(64));
    drive_feedback_pub_ = this->create_publisher<rover_msgs::msg::DriveFeedback>(
        "drive/feedback",
        qos
    );

    // Setup a timer to check position, velocity and target velocity of each motor
    feedback_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(DRIVE_FEEDBACK_PUBLISH_FREQUENCY_MS),
        std::bind(&MotorControlNode::publishDriveFeedback, this)
    );

    // Enable failsafe for all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetReturnCode ret = PhidgetMotorPositionController_enableFailsafe(motors[i], MOTOR_FAILSAFE_INTERVAL_MS);
        handlePhidgetError(ret, "enable failsafe", i);
    }

    // Initialize timer to reset the failsafe
    failsafe_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(MOTOR_FAILSAFE_INTERVAL_MS / 5),
        std::bind(&MotorControlNode::resetFailsafe, this)
    );

    motor_control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(MOTOR_CONTROL_LOOP_FREQUENCY_MS), 
            std::bind(&MotorControlNode::motorControlLoop, this)
    );

    // Create subscribers for left and right wheel velocity commands
    left_wheel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "left_wheel_speeds", rclcpp::QoS(10), std::bind(&MotorControlNode::leftWheelCallback, this, std::placeholders::_1));

    right_wheel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "right_wheel_speeds", rclcpp::QoS(10), std::bind(&MotorControlNode::rightWheelCallback, this, std::placeholders::_1));
}

MotorControlNode::~MotorControlNode() {
    RCLCPP_WARN(this->get_logger(), "Motor Control Node shutting down.");
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motors[i] != nullptr) {
            PhidgetMotorPositionController_delete(&motors[i]);
        }
    }
}

void MotorControlNode::leftWheelCallback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
    float left_velocity = msg->data.empty() ? 0.0f : msg->data[0];  // Extract velocity safely
    setVelocity({3, 4, 5}, left_velocity);
    // runMotors({3, 4, 5}, left_velocity);  // Corrected function name
}

void MotorControlNode::rightWheelCallback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
    float right_velocity = msg->data.empty() ? 0.0f : msg->data[0];
    setVelocity({0, 1, 2}, right_velocity);
    // runMotors({0, 1, 2}, right_velocity);  // Corrected function name
}

void MotorControlNode::handlePhidgetError(PhidgetReturnCode ret, const std::string& action, int i) {
    if (ret != EPHIDGET_OK) {
        const char* errorString;
        char errorDetail[100];
        size_t errorDetailLen = sizeof(errorDetail);
        PhidgetReturnCode errorCode;
        Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
        RCLCPP_ERROR(this->get_logger(), "Error at %s (%d) for motor %d: %s", action.c_str(), errorCode, i, errorString);
    } else {
        RCLCPP_INFO(this->get_logger(), "%s successful for motor %d", action.c_str(), i);
    }
}

void MotorControlNode::motorControlLoop() {
    PhidgetReturnCode ret;
    for (int i = 0; i < NUM_MOTORS; i++) {
        double current_position;
        ret = PhidgetMotorPositionController_getPosition(motors[i], &current_position);

        target_positions[i] = current_position + (velocities[i] * MOTOR_CONTROL_LOOP_FREQUENCY_MS);
        ret = PhidgetMotorPositionController_setTargetPosition(motors[i], target_positions[i]);
        handlePhidgetError(ret, "setting motor position", i);
    }
}

void MotorControlNode::setVelocity(const std::vector<int>& selected_motors, float velocity) {
    double velocity_rads;
    if (std::abs(velocity) < MIN_VELOCITY_MS) {
        velocity_rads = 0.0;
    }
    else {
        velocity_rads = std::clamp(velocity / WHEEL_RADIUS_METERS, -MAX_VELOCITY_RADS, MAX_VELOCITY_RADS);
    }

    for (int i : selected_motors) {
        velocities[i] = velocity_rads;
    }
}

void MotorControlNode::publishDriveFeedback() {
    rover_msgs::msg::DriveFeedback message;

    message.valid_data.resize(NUM_MOTORS, true);
    message.velocities.resize(NUM_MOTORS);
    message.positions.resize(NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetReturnCode ret;

        ret = PhidgetMotorPositionController_getPosition(motors[i], &message.positions[i]);
        if (ret != EPHIDGET_OK) {
            handlePhidgetError(ret, "getting motor position", i);
            message.valid_data[i] = false;
        }
        message.velocities[i] = velocities[i];
    }
    drive_feedback_pub_->publish(message);
}

void MotorControlNode::resetFailsafe() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetReturnCode ret = PhidgetMotorPositionController_resetFailsafe(motors[i]);
        handlePhidgetError(ret, "failsafe", i);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
