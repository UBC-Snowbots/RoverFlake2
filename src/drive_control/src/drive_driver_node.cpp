#include "drive_driver_node.h"

namespace {
    std::vector<MotorConfig> get_drive_configuration() {
        std::vector<MotorConfig> motors(NUM_MOTORS);
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].position_min = NAN;
            motors[i].position_max = NAN;
            motors[i].max_velocity = MAX_VELOCITY_REVS;
            motors[i].max_acceleration = MAX_ACCEL_REVS;
        }
        return motors;
    }
    std::vector<std::string> get_drive_motor_names() {
        // ID 1-6
        /*
            Right now:
            ID 1 left_front - isnt connected
            ID 2 left_mid - connected but doesnt move
            ID 3 - isnt connected
            ID 4 right_rear - connected 
            ID 5 left_rear - connected
            ID 6 right_front - connected
        */
        return {"left_front", "left_mid", "left_rear", "right_front", "right_mid", "right_rear"};
    }

    MoteusBusOptions get_drive_bus_options() {
        std::vector<int> ids;
        for (int i = 0; i < NUM_MOTORS; i++) {
            ids.push_back(i+DRIVE_MOTOR_CAN_ID_BASE);
        }
        std::vector<bool> enabled(NUM_MOTORS, true);


        MoteusBusOptions opts;
        opts.can_ids = ids;
        opts.enabled = enabled;
        opts.query_aux2 = false;
        return opts;
    }
}
DriveDriverNode::DriveDriverNode() : MoteusNode("motor_control_node", "drive", get_drive_configuration(), get_drive_motor_names(), get_drive_bus_options()) {
    RCLCPP_INFO(this->get_logger(), "Motor Control Node Initiated");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(64));
    drive_feedback_pub_ = this->create_publisher<rover_msgs::msg::DriveFeedback>(
        "drive/feedback",
        qos
    );

    // Initialize timer for publishing odometry
    feedback_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(DRIVE_FEEDBACK_PUBLISH_FREQUENCY_MS),
        std::bind(&DriveDriverNode::publishDriveFeedback, this)
    );

    // Create subscribers for left and right wheel velocity commands
    left_wheel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "left_wheel_speeds", rclcpp::QoS(10), std::bind(&DriveDriverNode::leftWheelCallback, this, std::placeholders::_1));

    right_wheel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "right_wheel_speeds", rclcpp::QoS(10), std::bind(&DriveDriverNode::rightWheelCallback, this, std::placeholders::_1));
    
    last_cycle_time_ = this->now();
    last_cmd_time_ = this->now();
}

void DriveDriverNode::prepareCommands() {
    if (first_cycle_) {
        for (auto& c : motor_cmds_) {
            c.active = true;
            c.is_stop = true;
        }
        first_cycle_ = false;
        return;
    }

    

    rclcpp::Time now = this->now();
    double dt = (now-last_cycle_time_).seconds();

    if ((now - last_cmd_time_).seconds() > CMD_TIMEOUT) {
        // command timeout
        for (int i = 0; i < NUM_MOTORS; i++) target_velocities_revs_[i] = 0;
    }
    
    last_cycle_time_ = now;
    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& c = motor_cmds_[i];
        
        c.active         = true;
        c.is_stop        = false;
        c.position       = NAN; // pure velocity mode
        c.velocity       = target_velocities_revs_[i];
        c.max_velocity   = MAX_VELOCITY_REVS;
        c.max_acceleration = MAX_ACCEL_REVS;
        
        // Integrate the commanded velocity for DriveFeedback.target_positions
        target_positions_rad_[i] +=
            c.velocity * REVS_TO_RADS * dt;
    }
}

void DriveDriverNode::handleTelemetry() {
    
}

void DriveDriverNode::setVelocity(const std::vector<int>& selected_motors, float velocity) {
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

void DriveDriverNode::leftWheelCallback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
    float left_velocity = msg->data.empty() ? 0.0f : msg->data[0];  // Extract velocity safely
    last_cmd_time_ = this->now();
    setVelocity({0, 1, 2}, left_velocity);
}

void DriveDriverNode::rightWheelCallback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
    float right_velocity = msg->data.empty() ? 0.0f : msg->data[0];
    last_cmd_time_ = this->now();
    setVelocity({3, 4, 5}, right_velocity);
}

/**
 * Publish target/actual velocity (m/s) and position (rad) per motor.
 * Actual values come straight from the moteus telemetry replies — no more
 * host-side differentiation or ODOMETRY_RESCALE_FACTOR fudge.
 */
void DriveDriverNode::publishDriveFeedback() {
    rover_msgs::msg::DriveFeedback message;

    message.valid_data.resize(NUM_MOTORS);
    message.target_velocities.resize(NUM_MOTORS);
    message.actual_velocities.resize(NUM_MOTORS);
    message.target_positions.resize(NUM_MOTORS);
    message.actual_positions.resize(NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++) {
        message.valid_data[i] = telem_[i].connected;

        message.target_velocities[i] = target_velocities_revs_[i] * WHEEL_CIRCUMFERENCE_METERS;
        message.actual_velocities[i] = telem_[i].velocity * WHEEL_CIRCUMFERENCE_METERS;

        message.target_positions[i] = target_positions_rad_[i];
        message.actual_positions[i] = telem_[i].position * REVS_TO_RADS;
    }
    drive_feedback_pub_->publish(message);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}