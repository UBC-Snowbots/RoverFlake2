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
        QLabel* chip = nullptr;
    };
    // NeverSeen: pre-rendered, no beat yet. Fresh: beat within HOST_TIMEOUT_MS.
    // Lost: seen-then-lost. Tracked so the 250ms tick only restyles the
    // beacon/readout on an actual state change (see checkHostsAlive).
    enum class HeartPhase { NeverSeen, Fresh, Lost };
    struct HostGroup {
        QGroupBox* box = nullptr;
        QGridLayout* grid = nullptr;
        QLabel* beacon = nullptr;     // flashes bright green per beat, dims next tick
        QLabel* readout = nullptr;    // live status text, updated every tick
        std::map<std::string, Row> rows;
        qint64 last_arrival_ms = 0;   // steady clock, stamped on message arrival
        HeartPhase phase = HeartPhase::NeverSeen;
        bool pending_dim = false;     // beacon flashed bright; dim on next tick
    };
    void onStatus(rover_msgs::msg::HeartStatus::SharedPtr msg);
    void sendCommand(const std::string& name, uint8_t action);
    HostGroup& hostGroup(const std::string& host);
    Row& row(HostGroup& g, const std::string& subsystem);
    void applyState(Row& r, const rover_msgs::msg::SubsystemState& s);
    void checkHostsAlive();
    // Pre-creates host/subsystem rows from rover_manager's heart.yaml so
    // controls exist (and work) before any heartbeat arrives. Warns and
    // no-ops on missing/unparsable file — falls back to dynamic-only.
    void loadExpectedHosts();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rover_msgs::msg::HeartStatus>::SharedPtr sub_;
    rclcpp::Publisher<rover_msgs::msg::SubsystemCommand>::SharedPtr pub_;
    QWidget* root_ = nullptr;
    QVBoxLayout* hosts_layout_ = nullptr;
    QLabel* waiting_ = nullptr;
    std::map<std::string, HostGroup> hosts_;
};
