// plotting_module.h
// Real-time position and velocity plots for all arm joints.
//
// Lives in arm_hardware_interface because the data comes directly from
// moteus_driver_node in this same package via /arm/moteus_feedback.
// Uses motor_addressing.h for NUM_MOTORS and joint names.
//
// NOTE: elapsed_ is started in createWidget, not setNode. setNode is called
// before the widget exists, so starting the timer there would produce a large
// time offset on the first plotted point once the widget is finally created.

#pragma once

#include <rover_hmi_core/gui_module.h>
#include <rover_hmi_core/plot_widget.h>
#include "motor_addressing.h"

#include <QCheckBox>
#include <QPushButton>
#include <QComboBox>
#include <QElapsedTimer>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"

class PlottingModule : public rover_hmi_core::GuiModule {
public:
    std::string name() const override { return "Plots"; }
    std::string layoutHint() const override { return "right"; }
    std::string sectionName() const override { return "Arm"; }
    QWidget* createWidget(QWidget* parent) override;
    void setNode(rclcpp::Node::SharedPtr node) override;
    void start() override {}
    void stop() override {}

private:
    void onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg);

    PlotWidget* pos_plot_ = nullptr;
    PlotWidget* vel_plot_ = nullptr;
    QCheckBox* checks_[NUM_MOTORS] = {};
    QPushButton* pause_btn_ = nullptr;
    bool paused_ = false;

    QElapsedTimer elapsed_;
    rclcpp::Subscription<rover_msgs::msg::MoteusArmStatus>::SharedPtr sub_;
};
