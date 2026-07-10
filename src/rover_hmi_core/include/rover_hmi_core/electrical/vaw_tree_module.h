// vaw_tree_module.h — "VAW Telemetry"
//
// QTreeWidget showing power system and drivetrain hierarchy.
// Subscribes to /power/status and /drivetrain/wheel_states.
// Section: Electricals. defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QTreeWidget>
#include <QTreeWidgetItem>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/power_status.hpp"
#include "rover_msgs/msg/wheel_states.hpp"

class VawTreeModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "VAW Telemetry"; }
    std::string sectionName() const override { return "Electricals"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override {}
    void     stop() override {}

private:
    void onPowerStatus(const rover_msgs::msg::PowerStatus::SharedPtr msg);
    void onWheelStates(const rover_msgs::msg::WheelStates::SharedPtr msg);

    QTreeWidget* tree_ = nullptr;

    // Power system items
    QTreeWidgetItem* power_root_        = nullptr;
    QTreeWidgetItem* bat_items_[3]      = {};
    QTreeWidgetItem* bat_sep_           = nullptr;
    QTreeWidgetItem* arm_item_          = nullptr;
    QTreeWidgetItem* chassis_item_      = nullptr;
    QTreeWidgetItem* science_item_      = nullptr;

    // Drivetrain items
    QTreeWidgetItem* drive_root_        = nullptr;
    QTreeWidgetItem* wheel_items_[6]    = {};

    rclcpp::Subscription<rover_msgs::msg::PowerStatus>::SharedPtr  power_sub_;
    rclcpp::Subscription<rover_msgs::msg::WheelStates>::SharedPtr  wheel_sub_;
};
