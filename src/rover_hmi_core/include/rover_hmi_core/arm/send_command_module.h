// send_command_module.h
// Motor command panel — send position/stop commands, hold-to-jog, and
// multi-axis zero.
//
// Lives in arm_hardware_interface because commands are published to
// /arm/command, which is subscribed by moteus_driver_node in this same package.
//
// JogButton implements a hold-to-move pattern: jogPressed fires when the mouse
// button goes down and triggers a continuous velocity command; jogReleased fires
// on mouse-up and sends velocity 0 to stop the motor. Standard QPushButton
// clicked() is not used because it only fires on release.
//
// Commands are also published to /arm/hmi_log as formatted strings so
// CommandLogModule can display them without any direct coupling between modules.

#pragma once

#include <rover_hmi_core/gui_module.h>
#include <rover_arm_common/motor_addressing.h>
#include <rover_arm_common/arm_commands.h>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QMessageBox>
#include <QMouseEvent>
#include <array>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class QTimer;

constexpr int NUM_ZERO_AXES = NUM_MOTORS;

// JogButton and KeyJogFilter are defined in send_command_module.cpp (Q_OBJECT
// in the .cpp avoids AUTOMOC header-scanning issues with private include
// directories).
class JogButton;
class KeyJogFilter;

class SendCommandModule : public rover_hmi_core::GuiModule {
public:
    std::string name() const override { return "Send Command"; }
    std::string layoutHint() const override { return "right"; }
    std::string sectionName() const override { return "Arm"; }
    QWidget* createWidget(QWidget* parent) override;
    void setNode(rclcpp::Node::SharedPtr node) override;
    void start() override {}
    void stop() override {}

private:
    void sendPosition(int motor_id, double pos, double vel);
    void sendVelocity(int motor_id, double velocity);
    // Keyboard jog: one CMD_ABS_VEL carrying every mapped axis at once, so
    // several held keys move several axes simultaneously. Capture is an
    // application-wide event filter gated by the ARM toggle — the toggle is
    // the safety, so the keys work from any panel, not just a focused widget.
    void buildKeyJogSection(class QVBoxLayout* layout, QWidget* owner);
    void publishKeyJog();          // reads the filter's held keys, publishes once
    void setKeyJogArmed(bool on);
    void restyleKeyJogChips();     // held-key readout
    void applyKeyJogMode();        // relabels rows, starts/stops the IK timer
    bool keyJogIkMode() const;
    // IK mode: the same keys become a Cartesian twist for MoveIt Servo, which
    // needs a continuous stream (it halts after incoming_command_timeout), so
    // this is timer-driven rather than published on key change.
    void publishKeyJogTwist();
    void sendStop(int motor_id);   // masked CMD_STOP: real "d stop" for one target
    void sendStopAll();
    void sendZeroChecked();
    void homeChecked();   // safe-pose prompt -> zero selected -> group home
    void logCmd(const QString& cmd);

    // Target dropdown holds axis entries (A1..A6, EE; data = id) plus raw
    // motor entries M5/M6 (data = 100+id → cmd_value CMD_SPACE_MOTOR, which
    // bypasses the driver's differential-wrist transform). Picking the entry
    // IS the space selection — no separate mode state.
    int  targetId() const;
    bool targetMotorSpace() const;

    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

    QComboBox* motor_select_ = nullptr;
    QDoubleSpinBox* position_spin_ = nullptr;
    QDoubleSpinBox* velocity_spin_ = nullptr;
    QDoubleSpinBox* jog_speed_spin_ = nullptr;
    QCheckBox* pos_enable_ = nullptr;
    QCheckBox* vel_enable_ = nullptr;
    std::array<QCheckBox*, NUM_ZERO_AXES> zero_checks_{};

    KeyJogFilter*   key_jog_filter_  = nullptr;
    QPushButton*    key_jog_arm_btn_ = nullptr;
    QDoubleSpinBox* key_jog_scale_   = nullptr;
    QLabel*         key_jog_status_  = nullptr;
    QComboBox*      key_jog_mode_    = nullptr;  // joint (FK) vs Cartesian (IK)
    QComboBox*      ik_frame_        = nullptr;  // twist header.frame_id
    QTimer*         ik_timer_        = nullptr;
    std::vector<QLabel*> key_jog_chips_;
    std::vector<QLabel*> key_jog_row_names_;     // relabelled per mode
    bool            key_jog_moving_  = false;  // true → an all-zero stop is owed
    bool            ik_twist_moving_ = false;  // log twist transitions only
    rclcpp::Node::SharedPtr node_;             // for twist header stamps
};
