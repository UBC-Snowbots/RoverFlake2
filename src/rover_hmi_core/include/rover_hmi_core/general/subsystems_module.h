// Heart control panel: observed per-subsystem state from each per-computer
// heart (/heart/status), start/stop/restart via /heart/command. Host liveness
// is arrival-time on OUR steady clock — sender stamps are never trusted.
#pragma once
#include <QWidget>
#include <QGroupBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <map>
#include "rover_hmi_core/gui_module.h"
#include "rover_msgs/msg/heart_status.hpp"
#include "rover_msgs/msg/subsystem_command.hpp"

class SubsystemsModule : public rover_hmi_core::GuiModule {
public:
    std::string name() const override { return "Subsystems"; }
    std::string sectionName() const override { return "General"; }
    std::string layoutHint() const override { return "right"; }
    QWidget* createWidget(QWidget* parent) override;
    void setNode(rclcpp::Node::SharedPtr node) override;

private:
    struct Row {
        QLabel *chip = nullptr, *uptime = nullptr;
    };
    struct HostGroup {
        QGroupBox* box = nullptr;
        QGridLayout* grid = nullptr;
        std::map<std::string, Row> rows;
        qint64 last_arrival_ms = 0;   // steady clock, stamped on message arrival
    };
    void onStatus(rover_msgs::msg::HeartStatus::SharedPtr msg);
    void sendCommand(const std::string& name, uint8_t action);
    HostGroup& hostGroup(const std::string& host);
    Row& row(HostGroup& g, const std::string& subsystem);
    void applyState(Row& r, const rover_msgs::msg::SubsystemState& s);
    void checkHostsAlive();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rover_msgs::msg::HeartStatus>::SharedPtr sub_;
    rclcpp::Publisher<rover_msgs::msg::SubsystemCommand>::SharedPtr pub_;
    QWidget* root_ = nullptr;
    QVBoxLayout* hosts_layout_ = nullptr;
    QLabel* waiting_ = nullptr;
    std::map<std::string, HostGroup> hosts_;
};
