#include "motor_control.h"  // Include the header file for motor control functionalities


#ifdef REVERT 

#include <algorithm>          // For std::clamp

/**
 * @brief Construct a new MotorControlNode object
 * Initializes the motor control node, sets up motor resources, and starts a timer.
 */
MotorControlNode::MotorControlNode() : Node("motor_control_node") {
    RCLCPP_INFO(this->get_logger(), "Motor Control Node Initiated");

    // Initialize Phidget BLDC motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetReturnCode ret = PhidgetBLDCMotor_create(&motors[i]);
        if (ret != EPHIDGET_OK) {
            handlePhidgetError(ret, "creating motor", i);
            continue;
        }

        ret = Phidget_setHubPort((PhidgetHandle)motors[i], i);
        handlePhidgetError(ret, "set hub port", i);

        ret = Phidget_openWaitForAttachment((PhidgetHandle)motors[i], 900);
        handlePhidgetError(ret, "attachment", i);
    }

    // // Set up a timer to check motor positions every 100 ms
    // timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(100),
    //     std::bind(&MotorControlNode::checkMotorPositions, this)
    // );

    // Create a timer to call printTargetVelocity every 100 ms
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&MotorControlNode::printTargetVelocity, this)  // For motor 0, for example
    );

    // Enable failsafe for all motors
    // for (int i = 0; i < NUM_MOTORS; i++) {
    //     PhidgetReturnCode ret = PhidgetBLDCMotor_enableFailsafe(motors[i], 5000);
    //     handlePhidgetError(ret, "enable failsafe", i);
    // }

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
    velocity = std::clamp(velocity, -1.0f, 1.0f);
    
    for (int motor_index : selected_motors) {
        PhidgetReturnCode ret = PhidgetBLDCMotor_setTargetVelocity(motors[motor_index], velocity);
        if (ret != EPHIDGET_OK) {
            handlePhidgetError(ret, "set target velocity", motor_index);
        }
    }
}

// void MotorControlNode::checkMotorPositions() {
//     for (int i = 0; i < NUM_MOTORS; i++) {
//         double position = 0.0;
//         PhidgetReturnCode ret = PhidgetBLDCMotor_getPosition(motors[i], &position);
//         if (ret != EPHIDGET_OK) {
//             handlePhidgetError(ret, "get position", i);
//             continue;
//         }
//         RCLCPP_INFO(this->get_logger(), "Motor %d Position: %f", i, position * 1.3666);
//     }
// }

void MotorControlNode::printTargetVelocity() {
    
        double targetVelocity = 0.0;
        PhidgetReturnCode ret = PhidgetBLDCMotor_getTargetVelocity(motors[5], &targetVelocity);

        if (ret != EPHIDGET_OK) {
            handlePhidgetError(ret, "get target velocity", 5);  // Use i instead of motor_index
        } else {
            RCLCPP_INFO(this->get_logger(), "Motor %d Target Velocity: %f", 5, targetVelocity);
        }

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#else

#include <algorithm>          // For std::clamp

/**
 * @brief Construct a new MotorControlNode object
 * Initializes the motor control node, sets up motor resources, and starts timers.
 */
MotorControlNode::MotorControlNode() : Node("motor_control_node") {
    RCLCPP_INFO(this->get_logger(), "Motor Control Node Initiated");

    // Initialize Phidget Motors Position Controllers
    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetReturnCode ret;

        ret = PhidgetMotorPositionController_create(&motors[i]);
        handlePhidgetError(ret, "creating motor", i);

        ret = Phidget_setHubPort((PhidgetHandle)motors[i], i);
        handlePhidgetError(ret, "setting motor hub port", i);

        ret = Phidget_openWaitForAttachment((PhidgetHandle)motors[i], 900);
        handlePhidgetError(ret, "opening motor connection", i);

        ret = PhidgetMotorPositionController_setNormalizePID(motors[i], 1);
        handlePhidgetError(ret, "normalizing PID controller", i);

        ret = PhidgetMotorPositionController_setRescaleFactor(motors[i], MOTOR_RESCALE_FACTOR);
        handlePhidgetError(ret, "setting motor rescale factor", i);

        ret = PhidgetMotorPositionController_setVelocityLimit(motors[i], MAX_VELOCITY_RADS);
        handlePhidgetError(ret, "setting motor max velocity", i);

        ret = PhidgetMotorPositionController_setAcceleration(motors[i], MAX_ACCEL_RADS);
        handlePhidgetError(ret, "setting motor max acceleration", i);

        // Initialize motor position so wheels are stationary on startup
        ret = PhidgetMotorPositionController_getPosition(motors[i], &current_positions[i]);
        if (ret == EPHIDGET_OK) {
            target_positions[i] = current_positions[i];
        }
        else {
            handlePhidgetError(ret, "getting initial motor position", i);
            target_positions[i] = 0;
        }

        ret = PhidgetMotorPositionController_setTargetPosition(motors[i], current_positions[i]);
        handlePhidgetError(ret, "setting initial motor target position", i);

        ret = PhidgetMotorPositionController_setEngaged(motors[i], 1);
        handlePhidgetError(ret, "engaging motor", i);
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(64));
    drive_feedback_pub_ = this->create_publisher<rover_msgs::msg::DriveFeedback>(
        "drive/feedback",
        qos
    );

    // Initialize timer for publishing odometry
    feedback_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(DRIVE_FEEDBACK_PUBLISH_FREQUENCY_MS),
        std::bind(&MotorControlNode::publishDriveFeedback, this)
    );

    // Enable failsafe for all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetReturnCode ret = PhidgetMotorPositionController_enableFailsafe(motors[i], MOTOR_FAILSAFE_INTERVAL_MS);
        handlePhidgetError(ret, "enable failsafe", i);
    }

    // Initialize timer for motor control loop
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
}

void MotorControlNode::rightWheelCallback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
    float right_velocity = msg->data.empty() ? 0.0f : msg->data[0];
    setVelocity({0, 1, 2}, right_velocity);
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
    // static uint32_t 
    for (int i = 0; i < NUM_MOTORS; i++) {
        target_positions[i] += (target_velocities[i] * (MOTOR_CONTROL_LOOP_FREQUENCY_MS / 1000.0));

        float target_position_diff = 0.0;
        const float diff_limit = 5;
        double position;

       PhidgetReturnCode ret = PhidgetMotorPositionController_getPosition(motors[i], &position);
        if (ret != EPHIDGET_OK) {

            position = current_positions[i];
            target_position_diff = 0.0;
        }

        target_position_diff = target_positions[i] - position;

        if(abs(target_position_diff) >= diff_limit)
        {
            target_position_diff = diff_limit;

            target_positions[i] = position + target_position_diff;
        }
        RCLCPP_WARN(this->get_logger(), "MOTOR: %i TARGET POSITION DIFF: %f", i+1, target_position_diff);


        PhidgetMotorPositionController_setTargetPosition(motors[i], target_positions[i]);
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
        target_velocities[i] = velocity_rads;
    }
}

void MotorControlNode::publishDriveFeedback() {
    rover_msgs::msg::DriveFeedback message;

    message.valid_data.resize(NUM_MOTORS, true);
    message.target_velocities.resize(NUM_MOTORS);
    message.actual_velocities.resize(NUM_MOTORS);
    message.target_positions.resize(NUM_MOTORS);
    message.actual_positions.resize(NUM_MOTORS);

    double position;
    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetReturnCode ret;

        message.target_velocities[i] = target_velocities[i] * WHEEL_RADIUS_METERS;

        ret = PhidgetMotorPositionController_getPosition(motors[i], &position);
        if (ret != EPHIDGET_OK) {
            message.valid_data[i] = false;
            position = current_positions[i];
        }
        message.actual_velocities[i] = ((position - current_positions[i]) / (DRIVE_FEEDBACK_PUBLISH_FREQUENCY_MS / 1000.0)) * WHEEL_RADIUS_METERS * ODOMETRY_RESCALE_FACTOR;
        current_positions[i] = position;

        message.target_positions[i] = target_positions[i];
        message.actual_positions[i] = position;
    }
    drive_feedback_pub_->publish(message);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


#endif 
