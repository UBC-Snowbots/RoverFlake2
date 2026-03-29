// motor_config_module.h  —  "Motor Params"
//
// Editable per-motor configuration table with local persistence.
//
// Persistence model:
//   • The moteus driver applies hardcoded motor_config.h defaults on every
//     startup.  This module sits on top and applies user overrides afterwards.
//   • When the user edits a value:  it is published to /arm/config_update
//     (driver applies immediately) AND saved to QSettings.
//   • On start():  all saved overrides are re-published so they survive
//     driver and HMI restarts.
//   • Per-row ↺ Reset button:  clears QSettings for that motor and
//     re-publishes the motor_config.h defaults.
//
// Columns: Kp | Ki | Kd | Max I | Max Vel | Max Accel | Pos Min | Pos Max |
//          Max V | Max Power | Timeout | Gear (RO) | [↺ Reset]

#pragma once

#include <rover_hmi_core/gui_module.h>
#include "motor_addressing.h"

#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QString>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"
#include "rover_msgs/msg/moteus_config_update.hpp"

class MotorConfigModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Motor Params"; }
    std::string layoutHint()  const override { return "right"; }
    std::string sectionName() const override { return "Arm"; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;

    // Re-applies any user-saved overrides from QSettings to the driver.
    // Called after the window is shown so the driver is ready to receive.
    void start() override;
    void stop() override {}

    std::vector<std::pair<std::string,std::string>> keybindings() const override {
        return {
            { "Click cell",  "Edit value"               },
            { "Enter",       "Apply to motor + save"    },
            { "↺  (button)", "Reset motor to defaults"  },
        };
    }

private:
    void onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg);

    // Publish one config update AND save it to QSettings.
    void publishUpdate(int motor_idx, int col, float value);

    // Clear QSettings for motor_idx and re-publish motor_config.h defaults.
    void resetMotorToDefaults(int motor_idx);

    // Publish a single register update without saving to QSettings.
    // Used by resetMotorToDefaults() to push defaults without recording them.
    void publishRaw(int motor_idx, const char* reg, float value);

    // 12 data columns (0..10 editable, 11 gear read-only) + reset button column
    static constexpr int NUM_EDITABLE = 11;
    static constexpr int NUM_FIELDS   = 12;

    QLineEdit*  edits_[NUM_MOTORS][NUM_FIELDS] = {};
    QPushButton* reset_btns_[NUM_MOTORS] = {};
    QLabel*     status_ = nullptr;

    rclcpp::Subscription<rover_msgs::msg::MoteusArmStatus>::SharedPtr          sub_;
    rclcpp::Publisher<rover_msgs::msg::MoteusConfigUpdate>::SharedPtr           pub_;
};
