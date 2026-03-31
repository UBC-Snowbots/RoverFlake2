// lighting_module.h — "Lighting"
//
// 4 lighting zone controls: Front, Back, Left, Right.
// Publishes LightControl, subscribes LightStatus.
// Section: Electricals. defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QLabel>
#include <QPushButton>
#include <QSlider>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/light_control.hpp"
#include "rover_msgs/msg/light_status.hpp"

class LightingModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Lighting"; }
    std::string sectionName() const override { return "Electricals"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override {}
    void     stop() override {}

private:
    void onLightStatus(const rover_msgs::msg::LightStatus::SharedPtr msg);
    void publishControl(int zone, bool enabled, int brightness);

    static constexpr int NUM_ZONES = 4;

    QPushButton* toggle_btns_[NUM_ZONES] = {};
    QSlider*     sliders_[NUM_ZONES]     = {};
    QLabel*      value_lbls_[NUM_ZONES]  = {};
    QLabel*      swatch_lbls_[NUM_ZONES] = {};
    QLabel*      auto_lbls_[NUM_ZONES]   = {};
    QLabel*      status_                 = nullptr;

    bool zone_enabled_[NUM_ZONES] = {};

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<rover_msgs::msg::LightControl>::SharedPtr    pub_;
    rclcpp::Subscription<rover_msgs::msg::LightStatus>::SharedPtr  sub_;
};
