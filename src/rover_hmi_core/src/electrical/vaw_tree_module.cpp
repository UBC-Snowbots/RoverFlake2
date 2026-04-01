// vaw_tree_module.cpp — "VAW Telemetry"
//
// Two top-level nodes: "Power System" and "Drivetrain".
// Updates items in-place without collapsing expanded nodes.

#include "vaw_tree_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QScrollArea>
#include <QVBoxLayout>
#include <QFont>
#include <QSizePolicy>
#include <QColor>

#include <pluginlib/class_list_macros.hpp>

static const char* WHEEL_NAMES[] = { "FL", "FR", "ML", "MR", "RL", "RR" };

QWidget* VawTreeModule::createWidget(QWidget* parent) {
    auto* scroll = new QScrollArea(parent);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setStyleSheet(
        QString("QScrollArea { background: %1; }"
                "QScrollArea > QWidget > QWidget { background: %1; }")
        .arg(theme::Bg));

    auto* container = new QWidget();
    container->setStyleSheet(QString("background: %1;").arg(theme::Bg));
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* vl = new QVBoxLayout(container);
    vl->setContentsMargins(4, 4, 4, 4);
    vl->setSpacing(0);

    QFont mono("monospace", theme::FontSize);
    QFont monoBold("monospace", theme::FontSize, QFont::Bold);

    tree_ = new QTreeWidget();
    tree_->setColumnCount(2);
    tree_->setHeaderHidden(true);
    tree_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    tree_->setFont(mono);
    tree_->setStyleSheet(QString(
        "QTreeWidget { background: #000000; color: #cdd6f4; border: none; }"
        "QTreeWidget::item { padding: 3px 4px; }"
        "QTreeWidget::item:selected { background: #1a1a2e; color: #ffffff; }"
        "QTreeWidget::branch { background: #000000; }"
        "QTreeWidget::branch:has-siblings:!adjoins-item {"
        "    border-image: none; }"
        "QTreeWidget::branch:has-siblings:adjoins-item {"
        "    border-image: none; }"
        "QTreeWidget::branch:!has-children:!has-siblings:adjoins-item {"
        "    border-image: none; }"
        "QTreeWidget::branch:has-children:!has-siblings:closed,"
        "QTreeWidget::branch:closed:has-children:has-siblings {"
        "    border-image: none; image: none; }"
        "QTreeWidget::branch:open:has-children:!has-siblings,"
        "QTreeWidget::branch:open:has-children:has-siblings {"
        "    border-image: none; image: none; }"
    ));

    // ── Power System root ─────────────────────────────────────────────────────
    power_root_ = new QTreeWidgetItem(tree_);
    power_root_->setText(0, "\u25b6 Power System");
    power_root_->setText(1, "-- V / -- A / -- W");
    power_root_->setFont(0, monoBold);
    power_root_->setForeground(0, QColor(theme::Cyan));
    power_root_->setForeground(1, QColor(theme::TextDim));

    // Battery sub-items
    for (int b = 0; b < 3; b++) {
        bat_items_[b] = new QTreeWidgetItem(power_root_);
        bat_items_[b]->setText(0, QString("  Battery %1 \u25cf").arg(b + 1));
        bat_items_[b]->setText(1, "[--]");
        bat_items_[b]->setFont(0, mono);
        bat_items_[b]->setForeground(0, QColor(theme::TextDim));
        bat_items_[b]->setForeground(1, QColor(theme::TextDim));
    }

    bat_sep_ = new QTreeWidgetItem(power_root_);
    bat_sep_->setText(0, "  ──────────────────────────");
    bat_sep_->setFlags(Qt::NoItemFlags);
    bat_sep_->setForeground(0, QColor(theme::BorderDim));

    arm_item_ = new QTreeWidgetItem(power_root_);
    arm_item_->setText(0, "  ARM");
    arm_item_->setText(1, "-- W  \u25cf ON");
    arm_item_->setFont(0, mono);
    arm_item_->setForeground(0, QColor(theme::MotorColors[0]));

    chassis_item_ = new QTreeWidgetItem(power_root_);
    chassis_item_->setText(0, "  Chassis");
    chassis_item_->setText(1, "-- W  \u25cf ON");
    chassis_item_->setFont(0, mono);
    chassis_item_->setForeground(0, QColor(theme::MotorColors[2]));

    science_item_ = new QTreeWidgetItem(power_root_);
    science_item_->setText(0, "  Science");
    science_item_->setText(1, "-- W  \u25cf ON");
    science_item_->setFont(0, mono);
    science_item_->setForeground(0, QColor(theme::MotorColors[4]));

    // ── Drivetrain root ───────────────────────────────────────────────────────
    drive_root_ = new QTreeWidgetItem(tree_);
    drive_root_->setText(0, "\u25b6 Drivetrain");
    drive_root_->setText(1, "-- W total");
    drive_root_->setFont(0, monoBold);
    drive_root_->setForeground(0, QColor(theme::Yellow));
    drive_root_->setForeground(1, QColor(theme::TextDim));

    for (int w = 0; w < 6; w++) {
        wheel_items_[w] = new QTreeWidgetItem(drive_root_);
        wheel_items_[w]->setText(0, QString("  %1").arg(WHEEL_NAMES[w]));
        wheel_items_[w]->setText(1, "-- rpm | -- Nm | --\xc2\xb0""C | -- W");
        wheel_items_[w]->setFont(0, mono);
        wheel_items_[w]->setForeground(0, QColor(theme::MotorColors[w]));
        wheel_items_[w]->setForeground(1, QColor(theme::Text));
    }

    // Expand all
    tree_->expandAll();
    tree_->setColumnWidth(0, 260);

    vl->addWidget(tree_);

    scroll->setWidget(container);
    return scroll;
}

void VawTreeModule::setNode(rclcpp::Node::SharedPtr node) {
    auto qos = rclcpp::QoS(10).reliable();
    power_sub_ = node->create_subscription<rover_msgs::msg::PowerStatus>(
        "/power/status", qos,
        [this](const rover_msgs::msg::PowerStatus::SharedPtr msg) {
            onPowerStatus(msg);
        });
    wheel_sub_ = node->create_subscription<rover_msgs::msg::WheelStates>(
        "/drivetrain/wheel_states", qos,
        [this](const rover_msgs::msg::WheelStates::SharedPtr msg) {
            onWheelStates(msg);
        });
}

void VawTreeModule::onPowerStatus(const rover_msgs::msg::PowerStatus::SharedPtr msg) {
    if (!power_root_) return;

    power_root_->setText(1,
        QString("%1 V / %2 A / %3 W")
        .arg(msg->total_voltage_v, 0, 'f', 1)
        .arg(msg->total_current_a, 0, 'f', 1)
        .arg(msg->total_power_w, 0, 'f', 0));

    for (int b = 0; b < 3; b++) {
        if (!bat_items_[b]) continue;
        bool connected = msg->battery_connected[b];
        float v = msg->battery_voltage_v[b];
        float a = msg->battery_current_a[b];
        float w = msg->battery_power_w[b];

        QString indicator = connected ? "\u25cf" : "\u2715";
        bat_items_[b]->setText(0, QString("  Battery %1 %2").arg(b + 1).arg(indicator));
        bat_items_[b]->setForeground(0, QColor(connected ? theme::Green : theme::Red));

        if (connected) {
            QString vcol;
            if (v > 22.0f)      vcol = theme::Green;
            else if (v > 20.0f) vcol = theme::Yellow;
            else                vcol = theme::Red;
            bat_items_[b]->setText(1,
                QString("[%1 V | %2 A | %3 W]")
                .arg(v, 0, 'f', 1)
                .arg(a, 0, 'f', 1)
                .arg(w, 0, 'f', 0));
            bat_items_[b]->setForeground(1, QColor(vcol));
        } else {
            bat_items_[b]->setText(1, "[--]");
            bat_items_[b]->setForeground(1, QColor(theme::TextDim));
        }
    }

    auto setModItem = [](QTreeWidgetItem* item, const char* name, float power, bool enabled) {
        item->setText(0, QString("  %1").arg(name));
        item->setText(1, QString("%1 W  %2 %3")
            .arg(power, 0, 'f', 0)
            .arg(enabled ? "\u25cf" : "\u25cf")
            .arg(enabled ? "ON" : "OFF"));
        item->setForeground(1, QColor(enabled ? theme::Green : theme::Red));
    };

    if (arm_item_)     setModItem(arm_item_,     "ARM",     msg->arm_power_w,     msg->arm_enabled);
    if (chassis_item_) setModItem(chassis_item_,  "Chassis", msg->chassis_power_w, msg->chassis_enabled);
    if (science_item_) setModItem(science_item_,  "Science", msg->science_power_w, msg->science_enabled);
}

void VawTreeModule::onWheelStates(const rover_msgs::msg::WheelStates::SharedPtr msg) {
    if (!drive_root_) return;

    float total_w = 0.0f;
    for (int w = 0; w < 6; w++) total_w += msg->power_w[w];

    drive_root_->setText(1, QString("%1 W total").arg(total_w, 0, 'f', 0));

    for (int w = 0; w < 6; w++) {
        if (!wheel_items_[w]) continue;
        float temp = msg->temperature_c[w];
        QString tcol;
        if (temp >= 70.0f)      tcol = theme::Red;
        else if (temp >= 50.0f) tcol = theme::Yellow;
        else                    tcol = theme::Green;

        wheel_items_[w]->setText(1,
            QString("%1 rpm | %2 Nm | %3\xc2\xb0""C | %4 W")
            .arg(msg->speed_rpm[w], 0, 'f', 1)
            .arg(msg->torque_nm[w], 0, 'f', 2)
            .arg(temp, 0, 'f', 1)
            .arg(msg->power_w[w], 0, 'f', 0));
        wheel_items_[w]->setForeground(1, QColor(tcol));
    }
}

PLUGINLIB_EXPORT_CLASS(VawTreeModule, rover_hmi_core::GuiModule)
