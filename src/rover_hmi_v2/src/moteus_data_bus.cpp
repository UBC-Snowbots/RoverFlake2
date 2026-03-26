#include "moteus_data_bus.h"
#include <cmath>

MoteusDataBus::MoteusDataBus(rclcpp::Node::SharedPtr node, QObject* parent)
    : QObject(parent), node_(node) {

    auto qos = rclcpp::QoS(1).reliable().durability_volatile();

    feedback_sub_ = node_->create_subscription<rover_msgs::msg::MoteusArmStatus>(
        "/arm/moteus_feedback", qos,
        std::bind(&MoteusDataBus::onFeedback, this, std::placeholders::_1));

    config_log_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/arm/config_log", qos,
        std::bind(&MoteusDataBus::onConfigLog, this, std::placeholders::_1));

    command_pub_ = node_->create_publisher<rover_msgs::msg::ArmCommand>(
        "/arm/command", qos);

    elapsed_.start();
}

void MoteusDataBus::start() {
    spin_timer_ = new QTimer(this);
    connect(spin_timer_, &QTimer::timeout, this, &MoteusDataBus::spinOnce);
    spin_timer_->start(20);

    logCmd("# connected to /arm/moteus_feedback and /arm/command");
}

void MoteusDataBus::stop() {
    if (spin_timer_) spin_timer_->stop();
    logCmd("# disconnected");
}

void MoteusDataBus::spinOnce() {
    rclcpp::spin_some(node_);
}

void MoteusDataBus::onConfigLog(const std_msgs::msg::String::SharedPtr msg) {
    logCmd(QString::fromStdString(msg->data));
}

void MoteusDataBus::onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
    double now = elapsed_.elapsed() / 1000.0;
    std::array<MotorState, NUM_MOTORS> states{};

    for (int i = 0; i < NUM_MOTORS && i < (int)msg->status.size(); i++) {
        const auto& s = msg->status[i];
        states[i].id = i + 1;
        states[i].mode = s.moteus_mode;
        states[i].fault = s.moteus_fault;
        states[i].position = s.curr_position;
        states[i].velocity = s.curr_velocity;
        states[i].torque = s.curr_torque;
        states[i].voltage = s.curr_voltage_volts;
        states[i].temperature = s.driver_temp_degreesc;
        states[i].timestamp = now;
    }

    // Extract config if present (driver includes it in every message)
    if (!config_received_ && (int)msg->config.size() >= NUM_MOTORS) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            const auto& c = msg->config[i];
            configs_[i].kp = c.kp;
            configs_[i].ki = c.ki;
            configs_[i].kd = c.kd;
            configs_[i].max_current = c.max_current_amps;
            configs_[i].max_velocity = c.max_velocity;
            configs_[i].max_acceleration = c.max_acceleration;
            configs_[i].position_min = c.min_position;
            configs_[i].position_max = c.max_position;
            configs_[i].max_voltage = c.max_voltage_volts;
            configs_[i].max_power = c.max_power_watts;
            configs_[i].gear_reduction = c.gear_reduction;
        }
        config_received_ = true;
        emit configUpdated(configs_);
    }

    emit telemetryUpdated(states);
}

void MoteusDataBus::logCmd(const QString& cmd) {
    emit commandLogged(cmd);
}

void MoteusDataBus::sendPosition(int motor_id, double position,
                                  double velocity, double max_torque) {
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ABS_POS;
    msg.positions.resize(NUM_MOTORS, NAN);
    msg.velocities.resize(NUM_MOTORS, NAN);

    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
        msg.positions[motor_id - 1] = position;
        msg.velocities[motor_id - 1] = velocity;
    }

    command_pub_->publish(msg);

    auto fmt = [](double v) -> QString {
        return std::isnan(v) ? "nan" : QString::number(v, 'f', 3);
    };
    logCmd(QString("%1> d pos %2 %3 %4")
           .arg(motor_id).arg(fmt(position)).arg(fmt(velocity)).arg(fmt(max_torque)));
}

void MoteusDataBus::sendVelocity(int motor_id, double velocity) {
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ABS_VEL;
    msg.velocities.resize(NUM_MOTORS, NAN);
    if (motor_id >= 1 && motor_id <= NUM_MOTORS)
        msg.velocities[motor_id - 1] = velocity;

    command_pub_->publish(msg);

    logCmd(QString("%1> d vel %2")
           .arg(motor_id).arg(QString::number(velocity, 'f', 3)));
}

void MoteusDataBus::sendStop(int motor_id) {
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ABS_VEL;
    msg.velocities.resize(NUM_MOTORS, NAN);
    if (motor_id >= 1 && motor_id <= NUM_MOTORS)
        msg.velocities[motor_id - 1] = 0.0;

    command_pub_->publish(msg);
    logCmd(QString("%1> d stop").arg(motor_id));
}

void MoteusDataBus::sendStopAll() {
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_STOP;
    command_pub_->publish(msg);
    logCmd("A> d stop");
}

void MoteusDataBus::sendZero(int motor_id) {
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ZERO;
    // Non-NaN at [motor_id-1] flags that motor for zeroing; all others skipped.
    msg.positions.resize(NUM_MOTORS, NAN);
    if (motor_id >= 1 && motor_id <= NUM_MOTORS)
        msg.positions[motor_id - 1] = 1.0;
    command_pub_->publish(msg);
    logCmd(QString("%1> d exact 0").arg(motor_id));
}
