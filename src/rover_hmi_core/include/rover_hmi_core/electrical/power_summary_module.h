// power_summary_module.h — "Power Summary"
//
// Displays per-battery voltage/current/power and per-module (ARM/CHASSIS/SCIENCE)
// power bars with on/off toggles. Optionally records battery current to CSV.
// Section: General (main page). layoutHint: large.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QFile>
#include <QTextStream>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/power_status.hpp"

class PowerSummaryModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Power Summary"; }
    std::string layoutHint()  const override { return "large"; }
    std::string sectionName() const override { return "General"; }
    bool        defaultVisible() const override { return true; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override {}
    void     stop() override;

private:
    void onPowerStatus(const rover_msgs::msg::PowerStatus::SharedPtr msg);
    void publishModuleEnable(const rover_msgs::msg::PowerStatus& state);
    void toggleRecording();

    // Battery row widgets (3 batteries)
    QLabel* bat_voltage_[3]   = {};
    QLabel* bat_current_[3]   = {};
    QLabel* bat_power_[3]     = {};
    QLabel* bat_indicator_[3] = {};

    // Module row widgets
    QLabel*       mod_power_[3]  = {};
    QProgressBar* mod_bar_[3]    = {};
    QPushButton*  mod_btn_[3]    = {};

    // Record button + status
    QPushButton* record_btn_ = nullptr;
    QLabel*      status_lbl_ = nullptr;

    // Current power state (for enable publish)
    rover_msgs::msg::PowerStatus last_state_{};
    bool state_received_ = false;

    // CSV recording
    bool       recording_     = false;
    QFile*     csv_file_      = nullptr;
    QTextStream* csv_stream_  = nullptr;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rover_msgs::msg::PowerStatus>::SharedPtr sub_;
    rclcpp::Publisher<rover_msgs::msg::PowerStatus>::SharedPtr    pub_;
};
