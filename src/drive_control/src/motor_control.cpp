#include "motor_control.h"

/**
 * @brief Construct a new MotorControlNode object
 *
 * Opens the CAN transport, creates one moteus controller handle per drive
 * motor, clears any latched faults, and starts the control/feedback timers.
 *
 * Protocol notes (see also arm_hardware_interface/include/moteus_protocol.h):
 *  - All positions/velocities on the wire are OUTPUT-SHAFT revolutions and
 *    rev/s; the gear reduction is applied by the moteus firmware
 *    (motor_position.rotor_to_output_ratio set with moteus_tool).
 *  - A position frame with position = NaN is a pure velocity command.
 *  - Commands do not latch: they must be re-sent every cycle, and the moteus
 *    watchdog (servo.default_timeout_s) stops the motor if they stop arriving.
 */
MotorControlNode::MotorControlNode() : Node("motor_control_node") {
    RCLCPP_INFO(this->get_logger(), "Motor Control Node Initiated");

    // Same transport settings as the arm driver: disable bit-rate switching
    // so the bus stays at 1 Mb/s.
    transport_ = mot::TransportRegistry::singleton().make({"--can-disable-brs"}).first;

    for (int i = 0; i < NUM_MOTORS; i++) {
        mot::Controller::Options opts;
        opts.id        = DRIVE_MOTOR_CAN_ID_BASE + i;
        opts.transport = transport_;
        // Make velocity/accel limits writable per-frame so the motion profile
        // enforces MAX_VELOCITY_REVS / MAX_ACCEL_REVS on every command.
        opts.position_format.velocity_limit = mot::kFloat;
        opts.position_format.accel_limit    = mot::kFloat;
        controllers_.push_back(std::make_shared<mot::Controller>(opts));
    }

    // Send a one-shot stop to every motor: clears any latched fault from a
    // previous run and leaves the motors limp until the first velocity command.
    {
        std::vector<mot::CanFdFrame> frames;
        for (auto& ctrl : controllers_) {
            frames.push_back(ctrl->MakeStop());
        }
        std::vector<mot::CanFdFrame> replies;
        transport_->BlockingCycle(frames.data(), frames.size(), &replies);
        RCLCPP_INFO(this->get_logger(), "Stop sent to %d motors, %zu replied",
                    NUM_MOTORS, replies.size());
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
    // Best-effort stop so the wheels are not left holding a velocity target
    // (the moteus watchdog would stop them anyway once frames cease).
    if (transport_) {
        std::vector<mot::CanFdFrame> frames;
        for (auto& ctrl : controllers_) {
            frames.push_back(ctrl->MakeStop());
        }
        std::vector<mot::CanFdFrame> replies;
        transport_->BlockingCycle(frames.data(), frames.size(), &replies);
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

/**
 * One bus cycle: send a velocity command frame to every motor, then decode
 * the telemetry replies.
 *
 * Unlike the old Phidget position controller, moteus supports velocity mode
 * natively (position = NaN), so no host-side position integration or
 * target-diff clamping is needed — the accel_limit in the frame performs the
 * ramping that MAX_DV used to approximate.
 */
void MotorControlNode::motorControlLoop() {
    std::vector<mot::CanFdFrame> frames;
    frames.reserve(NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++) {
        mot::PositionMode::Command cmd;
        cmd.position       = NAN; // pure velocity mode
        cmd.velocity       = target_velocities_revs_[i];
        cmd.velocity_limit = MAX_VELOCITY_REVS;
        cmd.accel_limit    = MAX_ACCEL_REVS;
        frames.push_back(controllers_[i]->MakePosition(cmd));

        // Integrate the commanded velocity for DriveFeedback.target_positions
        target_positions_rad_[i] +=
            target_velocities_revs_[i] * REVS_TO_RADS * (MOTOR_CONTROL_LOOP_FREQUENCY_MS / 1000.0);
    }

    std::vector<mot::CanFdFrame> replies;
    transport_->BlockingCycle(frames.data(), frames.size(), &replies);

    for (int i = 0; i < NUM_MOTORS; i++) {
        connected_[i] = false;
    }

    for (const auto& frame : replies) {
        int idx = frame.source - DRIVE_MOTOR_CAN_ID_BASE;
        if (idx < 0 || idx >= NUM_MOTORS) continue;

        telem_[idx] = mot::Query::Parse(frame.data, frame.size);
        connected_[idx] = true;

        if (telem_[idx].fault != 0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Motor %d (CAN ID %d) fault %d (mode %d)",
                idx, idx + DRIVE_MOTOR_CAN_ID_BASE,
                telem_[idx].fault, static_cast<int>(telem_[idx].mode));
        }
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!connected_[i]) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Motor %d (CAN ID %d) did not reply", i, i + DRIVE_MOTOR_CAN_ID_BASE);
        }
    }
}

void MotorControlNode::setVelocity(const std::vector<int>& selected_motors, float velocity) {
    double velocity_revs;
    if (std::abs(velocity) < MIN_VELOCITY_MS) {
        velocity_revs = 0.0;
    }
    else {
        velocity_revs = std::clamp(velocity * MS_TO_REVS, -MAX_VELOCITY_REVS, MAX_VELOCITY_REVS);
    }

    for (int i : selected_motors) {
        target_velocities_revs_[i] = velocity_revs;
    }
}

/**
 * Publish target/actual velocity (m/s) and position (rad) per motor.
 * Actual values come straight from the moteus telemetry replies — no more
 * host-side differentiation or ODOMETRY_RESCALE_FACTOR fudge.
 */
void MotorControlNode::publishDriveFeedback() {
    rover_msgs::msg::DriveFeedback message;

    message.valid_data.resize(NUM_MOTORS);
    message.target_velocities.resize(NUM_MOTORS);
    message.actual_velocities.resize(NUM_MOTORS);
    message.target_positions.resize(NUM_MOTORS);
    message.actual_positions.resize(NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++) {
        message.valid_data[i] = connected_[i];

        message.target_velocities[i] = target_velocities_revs_[i] * WHEEL_CIRCUMFERENCE_METERS;
        message.actual_velocities[i] = telem_[i].velocity * WHEEL_CIRCUMFERENCE_METERS;

        message.target_positions[i] = target_positions_rad_[i];
        message.actual_positions[i] = telem_[i].position * REVS_TO_RADS;
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
