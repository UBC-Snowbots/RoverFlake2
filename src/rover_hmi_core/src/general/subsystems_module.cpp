#include "rover_hmi_core/general/subsystems_module.h"
#include "rover_hmi_core/catppuccin.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
#include <QTimer>
#include <QElapsedTimer>
#include <pluginlib/class_list_macros.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

// Styled entirely in the app's own idiom (catppuccin.h tokens + the e-stop
// button's dark-tinted-bg/colored-border/colored-text pattern from
// drivetrain_stop_module.cpp), not the old GTK dashboard's literal palette —
// see task-7-report.md for why that palette was reverted.
namespace {
QElapsedTimer g_steady;  // steady/monotonic arrival clock
qint64 steadyMs() { if (!g_steady.isValid()) g_steady.start(); return g_steady.elapsed(); }
constexpr qint64 HOST_TIMEOUT_MS = 2500;  // 2.5 beats at 1 Hz
constexpr const char* kBeaconDim      = "#0d5533";  // dim phase between beats (beacon logic untouched)
constexpr const char* kBeaconNeverSeen = "#444444";
constexpr const char* kOnlineTint      = "#0d2a1a";

// Beacon: small solid-fill circle. Color alone carries the state so it reads
// at a glance; only called on an actual state/phase change.
void styleBeacon(QLabel* beacon, const char* color) {
    beacon->setStyleSheet(QString("background:%1; border-radius:8px;").arg(color));
}

// Subsystem status cell — the e-stop idiom (dark tinted bg + matching colored
// border + colored text) scaled down. Width is fixed by the widget itself
// (see row()).
QString chipStyle(const char* bg, const char* fg, const char* border) {
    return QString("background:%1; color:%2; border:1px solid %3; "
                    "border-radius:6px; font-weight:bold; padding:3px 6px;").arg(bg, fg).arg(border);
}

// KILL/RUN/RESTART: native quiet buttons — background/border/radius/font all
// inherit the global QPushButton style; only the accent color and a uniform
// min-width (so "RESTART" never clips, and all three align) are set here.
QString buttonStyle(const char* accent) {
    return QString("QPushButton{color:%1;padding:6px 14px;min-width:130px;}"
                    "QPushButton:hover{border-color:%1;}").arg(accent);
}
}

void SubsystemsModule::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    sub_ = node_->create_subscription<rover_msgs::msg::HeartRequest>(
        "/heart/running_subsystems", 10,
        std::bind(&SubsystemsModule::onFeedback, this, std::placeholders::_1));
    pub_ = node_->create_publisher<rover_msgs::msg::HeartRequest>("/heart/request", 10);
}

QWidget* SubsystemsModule::createWidget(QWidget* parent) {
    root_ = new QWidget(parent);
    auto* outer = new QVBoxLayout(root_);
    waiting_ = new QLabel("⏳ waiting for hearts on /heart/running_subsystems");
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
    g.box->setStyleSheet(QString("QGroupBox{background:%1; border:1px solid %2; "
                                  "border-radius:6px; margin-top:0px;}").arg(theme::Bg, theme::BorderDim));
    auto* box_layout = new QVBoxLayout(g.box);
    box_layout->setContentsMargins(0, 0, 0, 0);  // header bar sits flush against the box's top edge
    box_layout->setSpacing(0);

    // Header bar: a real table-header row (own background + bottom rule),
    // not a chip — host name on the left, beacon + live readout on the right.
    auto* header_bar = new QFrame();
    header_bar->setFrameShape(QFrame::NoFrame);
    header_bar->setStyleSheet(QString("background:%1; border-bottom:1px solid %2;")
                                   .arg(theme::HeaderBg, theme::BorderDim));
    auto* header = new QHBoxLayout(header_bar);
    header->setContentsMargins(10, 6, 10, 6);
    auto* host_label = new QLabel(QString::fromStdString(host).toUpper());
    host_label->setStyleSheet(QString("color:%1; font-weight:bold; font-size:%2px;")
                                   .arg(theme::Text).arg(theme::FontSizeLg));
    g.beacon = new QLabel();
    g.beacon->setFixedSize(16, 16);
    styleBeacon(g.beacon, kBeaconNeverSeen);  // never seen
    g.readout = new QLabel("NO HEARTBEAT DETECTED");
    g.readout->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
    header->addWidget(host_label);
    header->addWidget(g.beacon);
    header->addWidget(g.readout);
    header->addStretch();
    box_layout->addWidget(header_bar);

    // Body: per-subsystem rows, with normal padding (the header bar above is
    // the only thing that needs to be flush).
    auto* body = new QWidget();
    g.grid = new QGridLayout(body);
    g.grid->setContentsMargins(8, 8, 8, 8);
    g.grid->setVerticalSpacing(6);
    g.grid->setHorizontalSpacing(6);
    g.grid->setColumnStretch(5, 1);  // trailing column absorbs leftover width, not the chip
    box_layout->addWidget(body);

    hosts_layout_->addWidget(g.box);
    return hosts_.emplace(host, g).first->second;
}

SubsystemsModule::Row& SubsystemsModule::row(HostGroup& g, const std::string& name) {
    auto it = g.rows.find(name);
    if (it != g.rows.end()) return it->second;
    const int r = static_cast<int>(g.rows.size());
    Row rw;
    auto* name_lbl = new QLabel(QString::fromStdString(name));
    name_lbl->setMinimumWidth(220);
    g.grid->addWidget(name_lbl, r, 0);
    rw.chip = new QLabel("—");
    rw.chip->setAlignment(Qt::AlignCenter);
    rw.chip->setFixedWidth(150);
    rw.chip->setStyleSheet(chipStyle(theme::BgPanel, theme::TextDim, theme::BorderDim));
    g.grid->addWidget(rw.chip, r, 1);
    // The heart's protocol is a bool: running true/false. RESTART is a UI
    // convenience — kill then run, same as the two clicks it replaces (the
    // heart marks a subsystem offline synchronously in its kill handler, so
    // in-order delivery makes the follow-up run valid).
    const struct { const char* label; int mode; const char* accent; } btns[] = {
        {"KILL", 0, theme::Red},
        {"RUN", 1, theme::Green},
        {"RESTART", 2, theme::Yellow},
    };
    for (int i = 0; i < 3; i++) {
        auto* b = new QPushButton(btns[i].label);
        b->setStyleSheet(buttonStyle(btns[i].accent));
        const int mode = btns[i].mode;
        QObject::connect(b, &QPushButton::clicked, [this, name, mode]() {
            if (mode != 1) sendRequest(name, false);
            if (mode != 0) sendRequest(name, true);
        });
        g.grid->addWidget(b, r, 2 + i);
    }
    return g.rows.emplace(name, rw).first->second;
}

void SubsystemsModule::applyRunning(Row& rw, bool running) {
    if (running) {
        rw.chip->setText("ONLINE");
        rw.chip->setStyleSheet(chipStyle(kOnlineTint, theme::Green, theme::Green));
    } else {
        rw.chip->setText("OFFLINE");
        rw.chip->setStyleSheet(chipStyle(theme::BgPanel, theme::TextDim, theme::BorderDim));
    }
}

// Old-protocol beat: each heart publishes one HeartRequest per subsystem per
// beat on the shared feedback topic; the host name rides in subsystem_host.
void SubsystemsModule::onFeedback(rover_msgs::msg::HeartRequest::SharedPtr msg) {
    HostGroup& g = hostGroup(msg->subsystem_host);
    g.last_arrival_ms = steadyMs();
    styleBeacon(g.beacon, theme::Green);  // bright flash: a beat was just read
    g.pending_dim = true;                 // next 250ms tick dims it, unless another beat wins first
    applyRunning(row(g, msg->subsystem_name), msg->running);
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
                styleBeacon(g.beacon, kBeaconNeverSeen);
                g.readout->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
                break;
            case HeartPhase::Lost:
                styleBeacon(g.beacon, theme::Red);
                g.readout->setStyleSheet(QString("color:%1;font-weight:bold;").arg(theme::Red));
                break;
            case HeartPhase::Fresh:
                // beacon is already bright from onFeedback; just recolor the text
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
                styleBeacon(g.beacon, kBeaconDim);
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

void SubsystemsModule::sendRequest(const std::string& name, bool running) {
    rover_msgs::msg::HeartRequest msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.subsystem_name = name;
    msg.running = running;
    pub_->publish(msg);
}

PLUGINLIB_EXPORT_CLASS(SubsystemsModule, rover_hmi_core::GuiModule)
