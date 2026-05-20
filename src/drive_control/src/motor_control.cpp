#include "motor_control.h"  // Include the header file for motor control functionalities
#include <algorithm>          // For std::clamp
#include <cmath>              // For std::abs (float)

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

        ret = PhidgetMotorPositionController_setRescaleFactor(motors[i], MOTOR_RESCALE_FACTOR);
        handlePhidgetError(ret, "setting motor rescale factor", i);

        ret = PhidgetMotorPositionController_setVelocityLimit(motors[i], MAX_VELOCITY_RADS);
        handlePhidgetError(ret, "setting motor max velocity", i);

        // Setup motor position settings so wheels are stopped by default
        double position;
        ret = PhidgetMotorPositionController_getPosition(motors[i], &position);
        if (ret == EPHIDGET_OK) {
            target_positions[i] = position;
        }
        else {
            handlePhidgetError(ret, "getting initial motor position", i);
            target_positions[i] = 0;
        }

        ret = PhidgetMotorPositionController_setTargetPosition(motors[i], position);
        handlePhidgetError(ret, "setting initial motor target position", i);

        ret = PhidgetMotorPositionController_setEngaged(motors[i], 1);
        handlePhidgetError(ret, "engaging motor", i);
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(64));
    drive_feedback_pub_ = this->create_publisher<rover_msgs::msg::DriveFeedback>(
        "drive/feedback",
        qos
    );

    wheel_states_pub_ = this->create_publisher<rover_msgs::msg::WheelStates>(
        "/drivetrain/wheel_states", rclcpp::QoS(10).reliable());

    auto stop_qos = rclcpp::QoS(10).reliable().transient_local();
    stop_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/drivetrain/remote_stop_status", stop_qos);

    stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/drivetrain/remote_stop", rclcpp::QoS(10).reliable(),
        std::bind(&MotorControlNode::onRemoteStop, this, std::placeholders::_1));

    stop_heartbeat_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(STATUS_HEARTBEAT_INTERVAL_MS),
        std::bind(&MotorControlNode::publishStopStatus, this));

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

    publishStopStatus();
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
    PhidgetReturnCode ret;
    for (int i = 0; i < NUM_MOTORS; i++) {
        double current_position;
        ret = PhidgetMotorPositionController_getPosition(motors[i], &current_position);
        if (ret != EPHIDGET_OK) {
            handlePhidgetError(ret, "getting motor position", i);
            continue;
        }

        double dv = target_velocities[i] - applied_velocities[i];
        dv = std::clamp(dv, -MAX_DV, MAX_DV);
        applied_velocities[i] = applied_velocities[i] + dv;

        target_positions[i] = current_position + (applied_velocities[i] * (MOTOR_CONTROL_LOOP_FREQUENCY_MS / 1000.0));
        ret = PhidgetMotorPositionController_setTargetPosition(motors[i], target_positions[i]);
        handlePhidgetError(ret, "setting motor position", i);

        // Reset the failsafe
        PhidgetReturnCode ret = PhidgetMotorPositionController_resetFailsafe(motors[i]);
        handlePhidgetError(ret, "failsafe", i);
    }
}

void MotorControlNode::setVelocity(const std::vector<int>& selected_motors, float velocity) {
    if (remote_stop_.load()) return;
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
    message.velocities.resize(NUM_MOTORS);
    message.positions.resize(NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetReturnCode ret;

        ret = PhidgetMotorPositionController_getPosition(motors[i], &message.positions[i]);
        if (ret != EPHIDGET_OK) {
            handlePhidgetError(ret, "getting motor position", i);
            message.valid_data[i] = false;
        }
        
        message.velocities[i] = applied_velocities[i];
    }
    drive_feedback_pub_->publish(message);
    publishWheelStates();
}

void MotorControlNode::publishWheelStates() {
    rover_msgs::msg::WheelStates msg;
    const float nan = std::numeric_limits<float>::quiet_NaN();

    for (int w = 0; w < NUM_WHEELS; w++) {
        int port = WHEEL_TO_PORT[w];
        WheelState& ws = wheel_state_[w];

        int attached_int = 0;
        PhidgetReturnCode att_ret = Phidget_getAttached(
            (PhidgetHandle)motors[port], &attached_int);
        ws.engaged = (att_ret == EPHIDGET_OK) && (attached_int != 0);
        ws.valid = ws.engaged;

        float v_rad = static_cast<float>(applied_velocities[port]);
        float rpm = v_rad * 60.0f / (2.0f * static_cast<float>(M_PI));

        msg.speed_rpm[w]     = ws.valid ? rpm : nan;
        float torque = v_rad * TORQUE_PROXY_SCALE;
        msg.torque_nm[w]     = ws.valid ? torque : nan;
        msg.power_w[w]       = ws.valid ? (torque * std::abs(v_rad)) : nan;
        msg.temperature_c[w] = ws.valid ? (ws.overheat ? 90.0f : 25.0f) : nan;
        msg.enabled[w]       = ws.valid && !remote_stop_.load();
    }

    wheel_states_pub_->publish(msg);
}

void MotorControlNode::onRemoteStop(const std_msgs::msg::Bool::SharedPtr msg) {
    bool requested = msg->data;
    bool prev = remote_stop_.exchange(requested);

    if (requested && !prev) {
        // false → true: zero velocity intent, disengage so motors coast.
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (!motors[i]) continue;
            target_velocities[i] = 0.0;
            applied_velocities[i] = 0.0;
            PhidgetMotorPositionController_setEngaged(motors[i], 0);
        }
        RCLCPP_WARN(this->get_logger(), "Drivetrain remote-stop ENGAGED");
    } else if (!requested && prev) {
        // true → false: re-engage. target/applied velocities stay at 0 from
        // the stop transition — operator must move the joystick to re-issue
        // intent.
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (!motors[i]) continue;
            PhidgetMotorPositionController_setEngaged(motors[i], 1);
        }
        RCLCPP_INFO(this->get_logger(), "Drivetrain remote-stop RELEASED");
    }

    publishStopStatus();
}

void MotorControlNode::publishStopStatus() {
    if (!stop_status_pub_) return;
    std_msgs::msg::Bool msg;
    msg.data = remote_stop_.load();
    stop_status_pub_->publish(msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
