// Heart control panel: per-subsystem running state from each per-computer
// heart (/heart/running_subsystems), start/stop via /heart/request — the same
// HeartRequest protocol the old GTK dashboard (dashboard_bringup) spoke. The
// old dashboard's backend block in heart.yaml (/dashboard_hmi_node: topics,
// watchdog_timeout_ms) is read directly, so retuning the backend retunes this
// panel without a rebuild.
// Host liveness is arrival-time on OUR steady clock — sender stamps are never trusted.
#pragma once
#include <QWidget>
#include <QGroupBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <map>
#include "rover_hmi_core/gui_module.h"
#include "rover_msgs/msg/heart_request.hpp"

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
    void onFeedback(rover_msgs::msg::HeartRequest::SharedPtr msg);
    void sendRequest(const std::string& name, bool running);
    HostGroup& hostGroup(const std::string& host);
    Row& row(HostGroup& g, const std::string& subsystem);
    void applyRunning(Row& r, bool running);
    void checkHostsAlive();
    // Pre-creates host/subsystem rows from rover_manager's heart.yaml so
    // controls exist (and work) before any heartbeat arrives. Warns and
    // no-ops on missing/unparsable file — falls back to dynamic-only.
    void loadExpectedHosts();
    // Maps the old dashboard's /dashboard_hmi_node block (topics, watchdog)
    // onto this panel; keeps the built-in defaults on any parse failure.
    void loadBackendParams();
    std::string heartYamlPath() const;      // "" (with a warn) when not found

    std::string request_topic_  = "/heart/request";
    std::string feedback_topic_ = "/heart/running_subsystems";
    qint64 host_timeout_ms_ = 2500;         // fallback: 2.5 beats at 1 Hz

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rover_msgs::msg::HeartRequest>::SharedPtr sub_;
    rclcpp::Publisher<rover_msgs::msg::HeartRequest>::SharedPtr pub_;
    QWidget* root_ = nullptr;
    QVBoxLayout* hosts_layout_ = nullptr;
    QLabel* waiting_ = nullptr;
    std::map<std::string, HostGroup> hosts_;
};
