#pragma once

#include <QObject>
#include <QTimer>
#include <QElapsedTimer>
#include <QString>

#include <array>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"

constexpr int NUM_MOTORS = 6;

// Command codes (matching driver convention)
constexpr char CMD_ABS_POS = 'P';
constexpr char CMD_ABS_VEL = 'V';
constexpr char CMD_STOP    = 'S';

struct MotorState {
    int id = 0;
    int mode = 0;
    int fault = 0;
    double position = 0.0;
    double velocity = 0.0;
    double torque = 0.0;
    double voltage = 0.0;
    double temperature = 0.0;
    double timestamp = 0.0;
};

class MoteusDataBus : public QObject {
    Q_OBJECT
public:
    explicit MoteusDataBus(rclcpp::Node::SharedPtr node, QObject* parent = nullptr);

    void start();
    void stop();

    // Queue commands — these publish to /arm/command
    void sendPosition(int motor_id, double position, double velocity = 0.0,
                      double max_torque = NAN);
    void sendStop(int motor_id);
    void sendStopAll();

signals:
    void telemetryUpdated(const std::array<MotorState, NUM_MOTORS>& states);
    void commandLogged(const QString& cmd);

private:
    void spinOnce();
    void onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg);
    void logCmd(const QString& cmd);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rover_msgs::msg::MoteusArmStatus>::SharedPtr feedback_sub_;
    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr command_pub_;

    QTimer* spin_timer_ = nullptr;
    QElapsedTimer elapsed_;
};
