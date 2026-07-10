// power_timeline_module.h — "Power Timeline"
//
// Rolling canvas showing up to 120 samples of battery voltages,
// total current, and total power over ~60 seconds.
// Section: Electricals. defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QWidget>
#include <QPushButton>
#include <QTimer>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/power_status.hpp"

// TimelineCanvas defined in power_timeline_module.cpp (Q_OBJECT in .cpp avoids
// AUTOMOC header-scanning issues with private include directories).
class TimelineCanvas;

// ---------------------------------------------------------------------------
// PowerTimelineModule
// ---------------------------------------------------------------------------
class PowerTimelineModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Power Timeline"; }
    std::string sectionName() const override { return "Electricals"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override;
    void     stop() override;

private:
    void onPowerStatus(const rover_msgs::msg::PowerStatus::SharedPtr msg);

    TimelineCanvas*              canvas_   = nullptr;
    QTimer*                      timer_    = nullptr;
    std::array<QPushButton*, 5>  tog_btns_ = {};

    rclcpp::Subscription<rover_msgs::msg::PowerStatus>::SharedPtr sub_;
};
