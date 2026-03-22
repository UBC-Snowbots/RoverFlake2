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

        ret = PhidgetBLDCMotor_create(&motors[i]);
        handlePhidgetError(ret, "creating motor", i);
        ret = PhidgetMotorVelocityController_create(&motor_velocity_controllers[i]);
        handlePhidgetError(ret, "creating velocity controller", i);

        ret = Phidget_setHubPort((PhidgetHandle)motors[i], i);
        handlePhidgetError(ret, "setting motor hub port", i);
        ret = Phidget_setHubPort((PhidgetHandle)motor_velocity_controllers[i], i);
        handlePhidgetError(ret, "setting velocity controller hub port", i);

        ret = Phidget_openWaitForAttachment((PhidgetHandle)motors[i], 5000);
        handlePhidgetError(ret, "opening motor", i);
        ret = Phidget_openWaitForAttachment((PhidgetHandle)motor_velocity_controllers[i], 5000);
        handlePhidgetError(ret, "opening velocity controller", i);

        ret = PhidgetBLDCMotor_setRescaleFactor(motors[i], MOTOR_RESCALE_FACTOR);
        handlePhidgetError(ret, "set motor rescale factor", i);
        ret = PhidgetMotorVelocityController_setRescaleFactor(motor_velocity_controllers[i], MOTOR_RESCALE_FACTOR);
        handlePhidgetError(ret, "set rescale factor", i);

        PhidgetMotorVelocityController_setEngaged(motor_velocity_controllers[i], 1);
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
        PhidgetReturnCode ret = PhidgetBLDCMotor_enableFailsafe(motors[i], MOTOR_FAILSAFE_INTERVAL_MS);
        handlePhidgetError(ret, "enable failsafe", i);
    }

    // Initialize timer to reset the failsafe
    failsafe_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(MOTOR_FAILSAFE_INTERVAL_MS / 5),
        std::bind(&MotorControlNode::resetFailsafe, this)
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
            PhidgetBLDCMotor_delete(&motors[i]);
        }
    }
}

void MotorControlNode::leftWheelCallback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
    float left_velocity = msg->data.empty() ? 0.0f : msg->data[0];  // Extract velocity safely
    runMotors({3, 4, 5}, left_velocity);  // Corrected function name
}

void MotorControlNode::rightWheelCallback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
    float right_velocity = msg->data.empty() ? 0.0f : msg->data[0];
    runMotors({0, 1, 2}, right_velocity);  // Corrected function name
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

void MotorControlNode::runMotors(const std::vector<int>& selected_motors, float velocity) {
    PhidgetLog_enable(PHIDGET_LOG_INFO, "phidgetlog.log");

    double velocity_rad_s = velocity / WHEEL_RADIUS_METERS;
    velocity_rad_s = std::clamp(velocity_rad_s, -MOTOR_MAX_VELOCITY_RADIANS, MOTOR_MAX_VELOCITY_RADIANS);
    
    for (int motor_index : selected_motors) {
        PhidgetReturnCode ret = PhidgetMotorVelocityController_setTargetVelocity(motor_velocity_controllers[motor_index], (double)velocity_rad_s);
        handlePhidgetError(ret, "set target velocity", motor_index);
    }
}

void MotorControlNode::publishDriveFeedback() {
    rover_msgs::msg::DriveFeedback message;

    message.valid_data.resize(NUM_MOTORS, true);
    message.velocities.resize(NUM_MOTORS);
    message.target_velocities.resize(NUM_MOTORS);
    message.positions.resize(NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetReturnCode ret;

        ret = PhidgetMotorVelocityController_getVelocity(motor_velocity_controllers[i], &message.velocities[i]);
        if (ret != EPHIDGET_OK) {
            handlePhidgetError(ret, "get velocity", i);
            message.valid_data[i] = false;
        }

        ret = PhidgetMotorVelocityController_getTargetVelocity(motor_velocity_controllers[i], &message.target_velocities[i]);
        if (ret != EPHIDGET_OK) {
            handlePhidgetError(ret, "get target velocity", i);
            message.valid_data[i] = false;
        }

        ret = PhidgetBLDCMotor_getPosition(motors[i], &message.positions[i]);
        if (ret != EPHIDGET_OK) {
            handlePhidgetError(ret, "get position", i);
            message.valid_data[i] = false;
        }
    }

    drive_feedback_pub_->publish(message);
}

void MotorControlNode::resetFailsafe() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetReturnCode ret = PhidgetBLDCMotor_resetFailsafe(motors[i]);
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
