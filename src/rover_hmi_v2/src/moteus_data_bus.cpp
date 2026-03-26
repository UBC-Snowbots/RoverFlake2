#include "moteus_data_bus.h"
#include <QElapsedTimer>
#include <cmath>

MoteusDataBus::MoteusDataBus(rclcpp::Node::SharedPtr node, QObject* parent)
    : QObject(parent), node_(node) {

    auto qos = rclcpp::QoS(1).reliable().durability_volatile();

    feedback_sub_ = node_->create_subscription<rover_msgs::msg::MoteusArmStatus>(
        "/arm/moteus_feedback", qos,
        std::bind(&MoteusDataBus::onFeedback, this, std::placeholders::_1));

    command_pub_ = node_->create_publisher<rover_msgs::msg::ArmCommand>(
        "/arm/command", qos);

    elapsed_.start();
}

void MoteusDataBus::start() {
    // Periodically call rclcpp::spin_some so ROS callbacks fire within the Qt event loop
    spin_timer_ = new QTimer(this);
    connect(spin_timer_, &QTimer::timeout, this, &MoteusDataBus::spinOnce);
    spin_timer_->start(20);  // 50 Hz spin rate

    logCmd("# connected to /arm/moteus_feedback and /arm/command");
}

void MoteusDataBus::stop() {
    if (spin_timer_) {
        spin_timer_->stop();
    }
    logCmd("# disconnected");
}

void MoteusDataBus::spinOnce() {
    rclcpp::spin_some(node_);
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

    emit telemetryUpdated(states);
}

void MoteusDataBus::logCmd(const QString& cmd) {
    emit commandLogged(cmd);
}

void MoteusDataBus::sendPosition(int motor_id, double position,
                                  double velocity, double max_torque) {
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ABS_POS;

    // Fill arrays with NaN (= don't command) except for the target motor
    msg.positions.resize(NUM_MOTORS, NAN);
    msg.velocities.resize(NUM_MOTORS, NAN);

    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
        msg.positions[motor_id - 1] = position;
        msg.velocities[motor_id - 1] = velocity;
    }

    command_pub_->publish(msg);

    // Log tview-style command
    auto fmt = [](double v) -> QString {
        return std::isnan(v) ? "nan" : QString::number(v, 'f', 3);
    };
    logCmd(QString("%1> d pos %2 %3 %4")
           .arg(motor_id).arg(fmt(position)).arg(fmt(velocity)).arg(fmt(max_torque)));
}

void MoteusDataBus::sendStop(int motor_id) {
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ABS_VEL;

    // Velocity of 0 = stop for that motor, NaN = skip others
    msg.velocities.resize(NUM_MOTORS, NAN);
    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
        msg.velocities[motor_id - 1] = 0.0;
    }

    command_pub_->publish(msg);
    logCmd(QString("%1> d stop").arg(motor_id));
}

void MoteusDataBus::sendStopAll() {
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_STOP;
    command_pub_->publish(msg);
    logCmd("A> d stop");
}
