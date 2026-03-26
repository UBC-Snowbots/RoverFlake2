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
#include "rover_msgs/msg/bldc_servo_config.hpp"
#include "std_msgs/msg/string.hpp"

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

struct MotorConfigInfo {
    float kp = 0, ki = 0, kd = 0;
    float max_current = 0;
    float max_velocity = 0;
    float max_acceleration = 0;
    float position_min = 0, position_max = 0;
    float max_voltage = 0, max_power = 0;
    float gear_reduction = 0;
};

class MoteusDataBus : public QObject {
    Q_OBJECT
public:
    explicit MoteusDataBus(rclcpp::Node::SharedPtr node, QObject* parent = nullptr);

    void start();
    void stop();

    void sendPosition(int motor_id, double position, double velocity = 0.0,
                      double max_torque = NAN);
    void sendStop(int motor_id);
    void sendStopAll();

    const std::array<MotorConfigInfo, NUM_MOTORS>& configs() const { return configs_; }

signals:
    void telemetryUpdated(const std::array<MotorState, NUM_MOTORS>& states);
    void configUpdated(const std::array<MotorConfigInfo, NUM_MOTORS>& configs);
    void commandLogged(const QString& cmd);

private:
    void spinOnce();
    void onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg);
    void onConfigLog(const std_msgs::msg::String::SharedPtr msg);
    void logCmd(const QString& cmd);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rover_msgs::msg::MoteusArmStatus>::SharedPtr feedback_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_log_sub_;
    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr command_pub_;

    QTimer* spin_timer_ = nullptr;
    QElapsedTimer elapsed_;
    std::array<MotorConfigInfo, NUM_MOTORS> configs_{};
    bool config_received_ = false;
};
