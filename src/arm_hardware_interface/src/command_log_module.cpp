// command_log_module.cpp
// See command_log_module.h for module overview.

#include "command_log_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QVBoxLayout>
#include <QFont>
#include <QScrollBar>
#include <QDateTime>

#include <pluginlib/class_list_macros.hpp>

QWidget* CommandLogModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    auto* layout = new QVBoxLayout(widget);
    layout->setContentsMargins(0, 0, 0, 0);

    log_ = new QTextEdit();
    log_->setReadOnly(true);
    log_->setFont(QFont("monospace", theme::FontSizeSm));
    log_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    log_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    log_->setStyleSheet(
        QString("QTextEdit { background: %1; color: %2; border: none; padding: 8px; }"
                "QScrollBar:vertical { background: %1; width: 8px; }"
                "QScrollBar::handle:vertical { background: %3; border-radius: 4px; min-height: 20px; }"
                "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }")
        .arg(theme::BgPanel).arg(theme::TextDim).arg(theme::BorderDim));

    layout->addWidget(log_);
    return widget;
}

void CommandLogModule::setNode(rclcpp::Node::SharedPtr node) {
    // Depth 500 so burst output during calibration (many lines quickly) isn't dropped.
    auto qos = rclcpp::QoS(500).reliable().durability_volatile();

    // Commands sent by HMI modules (published to /arm/hmi_log as formatted strings)
    hmi_log_sub_ = node->create_subscription<std_msgs::msg::String>(
        "/arm/hmi_log", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QString cmd = QString::fromStdString(msg->data);
            // Color-code by command type so the operator can scan the log quickly:
            //   red   — stop/e-stop (safety-critical, stands out)
            //   green — position moves (normal commanded motion)
            //   cyan  — comments or annotations (lines starting with #)
            //   dim   — everything else (velocity, zero, etc.)
            QString color = theme::TextDim;
            if (cmd.contains("d stop"))    color = theme::Red;
            else if (cmd.contains("d pos")) color = theme::Green;
            else if (cmd.startsWith("#"))   color = theme::Cyan;
            appendLog(cmd, color);
        });

    // Messages from the arm driver (e.g. config applied, errors)
    config_log_sub_ = node->create_subscription<std_msgs::msg::String>(
        "/arm/config_log", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            appendLog(QString::fromStdString(msg->data), theme::Cyan);
        });
}

void CommandLogModule::appendLog(const QString& text, const QString& color) {
    if (!log_) return;
    QString time = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    log_->append(
        QString("<span style='color:%1'>%2</span>  <span style='color:%3'>%4</span>")
        .arg(theme::BorderDim).arg(time).arg(color).arg(text.toHtmlEscaped()));

    auto* sb = log_->verticalScrollBar();
    sb->setValue(sb->maximum());
}

PLUGINLIB_EXPORT_CLASS(CommandLogModule, rover_hmi_core::GuiModule)
