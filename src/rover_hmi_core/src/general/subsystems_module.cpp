#include "rover_hmi_core/general/subsystems_module.h"
#include "rover_hmi_core/catppuccin.h"
#include <QVBoxLayout>
#include <QTimer>
#include <QElapsedTimer>
#include <pluginlib/class_list_macros.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace {
QElapsedTimer g_steady;  // steady/monotonic arrival clock
qint64 steadyMs() { if (!g_steady.isValid()) g_steady.start(); return g_steady.elapsed(); }
constexpr qint64 HOST_TIMEOUT_MS = 2500;  // 2.5 beats at 1 Hz
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
    alive->start(500);
    return root_;
}

SubsystemsModule::HostGroup& SubsystemsModule::hostGroup(const std::string& host) {
    auto it = hosts_.find(host);
    if (it != hosts_.end()) return it->second;
    waiting_->hide();
    HostGroup g;
    g.box = new QGroupBox(QString("%1 — ⏳ no heartbeat yet").arg(QString::fromStdString(host)));
    g.box->setStyleSheet(QString("QGroupBox{color:%1;}").arg(theme::TextDim));
    g.grid = new QGridLayout(g.box);
    hosts_layout_->addWidget(g.box);
    return hosts_.emplace(host, g).first->second;
}

SubsystemsModule::Row& SubsystemsModule::row(HostGroup& g, const std::string& name) {
    auto it = g.rows.find(name);
    if (it != g.rows.end()) return it->second;
    const int r = g.rows.size();
    Row rw;
    g.grid->addWidget(new QLabel(QString::fromStdString(name)), r, 0);
    rw.chip = new QLabel("—");
    rw.chip->setAlignment(Qt::AlignCenter);
    rw.chip->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
    g.grid->addWidget(rw.chip, r, 1);
    rw.uptime = new QLabel("");
    g.grid->addWidget(rw.uptime, r, 2);
    const struct { const char* label; uint8_t action; } btns[] = {
        {"▶", rover_msgs::msg::SubsystemCommand::ACTION_START},
        {"■", rover_msgs::msg::SubsystemCommand::ACTION_STOP},
        {"↻", rover_msgs::msg::SubsystemCommand::ACTION_RESTART},
    };
    for (int i = 0; i < 3; i++) {
        auto* b = new QPushButton(btns[i].label);
        b->setFixedWidth(32);
        const uint8_t action = btns[i].action;
        QObject::connect(b, &QPushButton::clicked,
                         [this, name, action]() { sendCommand(name, action); });
        g.grid->addWidget(b, r, 3 + i);
    }
    return g.rows.emplace(name, rw).first->second;
}

void SubsystemsModule::applyState(Row& rw, const rover_msgs::msg::SubsystemState& s) {
    using St = rover_msgs::msg::SubsystemState;
    switch (s.state) {
    case St::RUNNING:
        rw.chip->setText("RUNNING");
        rw.chip->setStyleSheet(QString("color:%1;font-weight:bold;").arg(theme::Green));
        rw.uptime->setText(QString("%1 s").arg(s.uptime_s));
        break;
    case St::STOPPING:
        rw.chip->setText("STOPPING");
        rw.chip->setStyleSheet(QString("color:%1;font-weight:bold;").arg(theme::Yellow));
        rw.uptime->setText("");
        break;
    case St::CRASHED:
        rw.chip->setText(QString("CRASHED (%1)").arg(s.exit_code));
        rw.chip->setStyleSheet(QString("color:%1;font-weight:bold;").arg(theme::Red));
        rw.uptime->setText("");
        break;
    case St::STUCK:
        rw.chip->setText("STUCK");
        rw.chip->setStyleSheet(QString("color:%1;font-weight:bold;").arg(theme::Red));
        rw.uptime->setText("");
        break;
    default:
        rw.chip->setText("STOPPED");
        rw.chip->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
        rw.uptime->setText("");
    }
}

void SubsystemsModule::onStatus(rover_msgs::msg::HeartStatus::SharedPtr msg) {
    HostGroup& g = hostGroup(msg->host);
    g.last_arrival_ms = steadyMs();
    g.offline = false;
    g.box->setTitle(QString::fromStdString(msg->host));
    g.box->setStyleSheet("");
    for (const auto& s : msg->subsystems) applyState(row(g, s.name), s);
}

void SubsystemsModule::checkHostsAlive() {
    for (auto& [host, g] : hosts_) {
        if (g.last_arrival_ms == 0) continue;  // pre-rendered, never seen: not offline, just waiting
        if (steadyMs() - g.last_arrival_ms <= HOST_TIMEOUT_MS) continue;
        if (!g.offline) {
            g.offline = true;
            g.box->setTitle(QString("%1 — ✖ HEART OFFLINE").arg(QString::fromStdString(host)));
            g.box->setStyleSheet(QString("QGroupBox{color:%1;}").arg(theme::Red));
            for (auto& [n, rw] : g.rows) {
                rw.chip->setText("?");
                rw.chip->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
            }
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
