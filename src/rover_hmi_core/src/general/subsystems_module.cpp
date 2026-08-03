#include "rover_hmi_core/general/subsystems_module.h"
#include "rover_hmi_core/catppuccin.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QElapsedTimer>
#include <pluginlib/class_list_macros.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace {
QElapsedTimer g_steady;  // steady/monotonic arrival clock
qint64 steadyMs() { if (!g_steady.isValid()) g_steady.start(); return g_steady.elapsed(); }
constexpr qint64 HOST_TIMEOUT_MS = 2500;  // 2.5 beats at 1 Hz
constexpr const char* kBeaconDimGreen = "#145c34";  // dim phase between beats (beacon logic untouched)

// Literal colors lifted from the old GTK dashboard (src/rover_hmi/css_files/*.css,
// src/rover_hmi/glade_files/dashboard.glade) — see task-7-report.md "derived from"
// table for the exact class/line each one came from.
constexpr const char* kOnlineBg   = "#3dfe3d";  // .subsys_ONLINE background: rgb(61,254,61)
constexpr const char* kOnlineFg   = "#000000";  // .subsys_ONLINE color: black
constexpr const char* kOfflineBg  = "#ff0000";  // .subsys_OFFLINE background: red
constexpr const char* kOfflineFg  = "#f5f5f5";  // .subsys_OFFLINE color: whitesmoke
constexpr const char* kUnknownBg  = "#808080";  // .dash_status_unknown background: grey
constexpr const char* kUnknownFg  = "#000000";  // .dash_status_unknown color: black
constexpr const char* kKillBg     = "#a52a2a";  // .subsys_button_KILL_OFFLINE background: brown
constexpr const char* kKillFg     = "#f5f5f5";  // .subsys_button_KILL_OFFLINE color: whitesmoke
constexpr const char* kRunBg      = "#adff2f";  // .subsys_button_RUN_ONLINE background: greenyellow
constexpr const char* kRunFg      = "#000000";  // .subsys_button_RUN_ONLINE color: black
constexpr const char* kHeaderBg   = "#f5f5f5";  // .monitored_node_name background: whitesmoke
constexpr const char* kHeaderFg   = "#000000";  // .monitored_node_name color: black

// Beacon: small solid-fill circle. Color alone carries the state so it reads
// at a glance; only called on an actual state/phase change.
void styleBeacon(QLabel* beacon, const char* color) {
    beacon->setStyleSheet(QString("background:%1; border-radius:8px;").arg(color));
}

// Subsystem chip: solid color block. Width is fixed by the widget itself
// (see row()); min-width here is a fallback.
QString chipStyle(const char* bg, const char* fg) {
    return QString("background:%1; color:%2; font-weight:bold; "
                    "padding:3px 10px; border-radius:4px; min-width:140px;").arg(bg, fg);
}

// KILL/RUN/RESTART: colored like the old dash's action columns. No fixed width —
// buttons size to their own text (padding + font) so "RESTART" is never clipped.
QString buttonStyle(const char* bg, const char* fg) {
    return QString("QPushButton{background:%1;color:%2;font-weight:bold;"
                    "border:1px solid %1;border-radius:4px;padding:4px 12px;font-size:%3px;}"
                    "QPushButton:hover{border-color:%4;}")
        .arg(bg).arg(fg).arg(theme::FontSizeSm).arg(theme::Border);
}
}

void SubsystemsModule::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    sub_ = node_->create_subscription<rover_msgs::msg::HeartStatus>(
        "/heart/status", 10,
        std::bind(&SubsystemsModule::onStatus, this, std::placeholders::_1));
    pub_ = node_->create_publisher<rover_msgs::msg::SubsystemCommand>("/heart/command", 10);
}

QWidget* SubsystemsModule::createWidget(QWidget* parent) {
    root_ = new QWidget(parent);
    auto* outer = new QVBoxLayout(root_);
    waiting_ = new QLabel("⏳ waiting for hearts on /heart/status");
    outer->addWidget(waiting_);
    hosts_layout_ = new QVBoxLayout();
    outer->addLayout(hosts_layout_);
    outer->addStretch();
    loadExpectedHosts();
    auto* alive = new QTimer(root_);
    QObject::connect(alive, &QTimer::timeout, [this]() { checkHostsAlive(); });
    alive->start(250);  // drives beacon dim-phase + live readout; beat rate is 1Hz
    return root_;
}

SubsystemsModule::HostGroup& SubsystemsModule::hostGroup(const std::string& host) {
    auto it = hosts_.find(host);
    if (it != hosts_.end()) return it->second;
    waiting_->hide();
    HostGroup g;
    g.box = new QGroupBox();  // no title text — GtkGroupBox title styling is unreliable under the app QSS
    g.grid = new QGridLayout(g.box);
    // Host name title bar — mirrors the old dash's "<HOST> subsystem health"
    // header, styled like .monitored_node_name (whitesmoke block, black text).
    auto* host_label = new QLabel(QString::fromStdString(host).toUpper());
    host_label->setStyleSheet(
        QString("background:%1; color:%2; font-weight:bold; padding:4px 12px; "
                "border-radius:4px; font-size:%3px;")
            .arg(kHeaderBg, kHeaderFg).arg(theme::FontSizeLg));
    g.beacon = new QLabel();
    g.beacon->setFixedSize(16, 16);
    styleBeacon(g.beacon, theme::BorderDim);  // grey: never seen
    g.readout = new QLabel("NO HEARTBEAT DETECTED");
    g.readout->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
    auto* header = new QHBoxLayout();
    header->addWidget(host_label);
    header->addWidget(g.beacon);
    header->addWidget(g.readout);
    header->addStretch();
    g.grid->addLayout(header, 0, 0, 1, 6);  // row 0: title + beacon + readout, spans all columns
    g.grid->setColumnStretch(5, 1);         // trailing column absorbs leftover width, not the chip
    hosts_layout_->addWidget(g.box);
    return hosts_.emplace(host, g).first->second;
}

SubsystemsModule::Row& SubsystemsModule::row(HostGroup& g, const std::string& name) {
    auto it = g.rows.find(name);
    if (it != g.rows.end()) return it->second;
    const int r = static_cast<int>(g.rows.size()) + 1;  // +1: row 0 is the header
    Row rw;
    g.grid->addWidget(new QLabel(QString::fromStdString(name)), r, 0);
    rw.chip = new QLabel("—");
    rw.chip->setAlignment(Qt::AlignCenter);
    rw.chip->setFixedWidth(140);  // comfortable fixed width so the chip doesn't stretch into a bar
    rw.chip->setStyleSheet(chipStyle(kUnknownBg, kUnknownFg));
    g.grid->addWidget(rw.chip, r, 1);
    // Order matches the old dash's glade layout: KILL before RUN; RESTART (no
    // old equivalent) appended last.
    const struct { const char* label; uint8_t action; const char* bg; const char* fg; } btns[] = {
        {"KILL", rover_msgs::msg::SubsystemCommand::ACTION_STOP, kKillBg, kKillFg},
        {"RUN", rover_msgs::msg::SubsystemCommand::ACTION_START, kRunBg, kRunFg},
        {"RESTART", rover_msgs::msg::SubsystemCommand::ACTION_RESTART, theme::Yellow, theme::Bg},
    };
    for (int i = 0; i < 3; i++) {
        auto* b = new QPushButton(btns[i].label);
        b->setStyleSheet(buttonStyle(btns[i].bg, btns[i].fg));
        const uint8_t action = btns[i].action;
        QObject::connect(b, &QPushButton::clicked,
                         [this, name, action]() { sendCommand(name, action); });
        g.grid->addWidget(b, r, 2 + i);
    }
    return g.rows.emplace(name, rw).first->second;
}

void SubsystemsModule::applyState(Row& rw, const rover_msgs::msg::SubsystemState& s) {
    using St = rover_msgs::msg::SubsystemState;
    switch (s.state) {
    case St::RUNNING:
        rw.chip->setText(QString("ONLINE %1s").arg(s.uptime_s));
        rw.chip->setStyleSheet(chipStyle(kOnlineBg, kOnlineFg));
        break;
    case St::STOPPING:
        rw.chip->setText("STOPPING");
        rw.chip->setStyleSheet(chipStyle(theme::Yellow, theme::Bg));
        break;
    case St::CRASHED:
        rw.chip->setText(QString("CRASHED (%1)").arg(s.exit_code));
        rw.chip->setStyleSheet(chipStyle(theme::Red, theme::Bg));
        break;
    case St::STUCK:
        rw.chip->setText("STUCK");
        rw.chip->setStyleSheet(chipStyle(theme::Red, theme::Bg));
        break;
    default:  // STOPPED
        rw.chip->setText("OFFLINE");
        rw.chip->setStyleSheet(chipStyle(kOfflineBg, kOfflineFg));
    }
}

void SubsystemsModule::onStatus(rover_msgs::msg::HeartStatus::SharedPtr msg) {
    HostGroup& g = hostGroup(msg->host);
    g.last_arrival_ms = steadyMs();
    styleBeacon(g.beacon, theme::Green);  // bright flash: a beat was just read
    g.pending_dim = true;                 // next 250ms tick dims it, unless another beat wins first
    for (const auto& s : msg->subsystems) applyState(row(g, s.name), s);
}

void SubsystemsModule::checkHostsAlive() {
    const qint64 now = steadyMs();
    for (auto& [host, g] : hosts_) {
        const bool never_seen = (g.last_arrival_ms == 0);
        const bool lost = !never_seen && (now - g.last_arrival_ms > HOST_TIMEOUT_MS);
        const HeartPhase new_phase = never_seen ? HeartPhase::NeverSeen
                                    : lost       ? HeartPhase::Lost
                                                 : HeartPhase::Fresh;
        if (new_phase != g.phase) {
            g.phase = new_phase;
            switch (new_phase) {
            case HeartPhase::NeverSeen:
                styleBeacon(g.beacon, theme::BorderDim);
                g.readout->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
                break;
            case HeartPhase::Lost:
                styleBeacon(g.beacon, theme::Red);
                g.readout->setStyleSheet(QString("color:%1;font-weight:bold;").arg(theme::Red));
                break;
            case HeartPhase::Fresh:
                // beacon is already bright from onStatus; just recolor the text
                g.readout->setStyleSheet(QString("color:%1;font-weight:bold;").arg(theme::Green));
                break;
            }
        }
        // Text-only update every tick is cheap; widget restyle above only runs on change.
        switch (new_phase) {
        case HeartPhase::NeverSeen:
            g.readout->setText("NO HEARTBEAT DETECTED");
            break;
        case HeartPhase::Fresh:
            if (g.pending_dim) {
                styleBeacon(g.beacon, kBeaconDimGreen);
                g.pending_dim = false;
            }
            g.readout->setText(QString("HEALTHY! %1 ms").arg(now - g.last_arrival_ms));
            break;
        case HeartPhase::Lost:
            g.readout->setText(QString("WATCHDOG EXCEEDED — %1 s silent").arg((now - g.last_arrival_ms) / 1000));
            break;
        }
    }
}

void SubsystemsModule::loadExpectedHosts() {
    std::string path;
    try {
        path = ament_index_cpp::get_package_share_directory("rover_manager") + "/config/heart.yaml";
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Subsystems: rover_manager share dir not found (%s); no pre-rendered hosts", e.what());
        return;
    }
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Subsystems: failed to parse %s (%s); no pre-rendered hosts", path.c_str(), e.what());
        return;
    }
    // heart.yaml is hand-edited config: a structurally-wrong entry (e.g. a
    // "/heart_x" key whose value isn't a map) must not be able to crash the
    // HMI. Guard each entry individually; warn once and skip the bad entry
    // so the rest of the file still pre-renders.
    bool warned = false;
    for (const auto& top : root) {
        try {
            const std::string key = top.first.as<std::string>();
            if (key.rfind("/heart_", 0) != 0) continue;  // only heart nodes, e.g. skip /dashboard_hmi_node
            const std::string host = key.substr(7);      // strip "/heart_"
            const YAML::Node subsystems = top.second["ros__parameters"]["subsystems"];
            if (!subsystems || !subsystems.IsMap()) continue;
            HostGroup& g = hostGroup(host);
            for (const auto& sub : subsystems) row(g, sub.first.as<std::string>());
        } catch (const std::exception& e) {
            if (!warned) {
                RCLCPP_WARN(node_->get_logger(), "Subsystems: malformed entry in %s (%s); skipping bad entries", path.c_str(), e.what());
                warned = true;
            }
        }
    }
}

void SubsystemsModule::sendCommand(const std::string& name, uint8_t action) {
    rover_msgs::msg::SubsystemCommand cmd;
    cmd.subsystem_name = name;
    cmd.action = action;
    pub_->publish(cmd);
}

PLUGINLIB_EXPORT_CLASS(SubsystemsModule, rover_hmi_core::GuiModule)
