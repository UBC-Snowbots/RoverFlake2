#include "motor_control.h"  // Include the header file for motor control functionalities
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

        ret = Phidget_openWaitForAttachment((PhidgetHandle)motors[i], 5000);
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
