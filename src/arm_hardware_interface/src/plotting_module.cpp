// plotting_module.cpp
// See plotting_module.h for module overview.

#include "plotting_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTabWidget>
#include <QFont>

#include <pluginlib/class_list_macros.hpp>

QWidget* PlottingModule::createWidget(QWidget* parent) {
    // Start the timer here, not in setNode, because setNode runs before the
    // widget is constructed. Starting early would cause a large artificial time
    // offset on the first data point after the widget appears.
    elapsed_.start();

    auto* widget = new QWidget(parent);
    auto* layout = new QVBoxLayout(widget);
    layout->setSpacing(6);

    auto* tabs = new QTabWidget();

    pos_plot_ = new PlotWidget("Position (rev)");
    vel_plot_ = new PlotWidget("Velocity (rev/s)");

    for (int i = 0; i < NUM_MOTORS; i++) {
        pos_plot_->addSeries(ARM_JOINTS[i].hardware_name, theme::MotorColors[i]);
        vel_plot_->addSeries(ARM_JOINTS[i].hardware_name, theme::MotorColors[i]);
    }

    tabs->addTab(pos_plot_, "Position");
    tabs->addTab(vel_plot_, "Velocity");
    layout->addWidget(tabs, 1);

    auto* controls = new QHBoxLayout();
    controls->setSpacing(10);

    QFont font("monospace", theme::FontSize);

    for (int i = 0; i < NUM_MOTORS; i++) {
        checks_[i] = new QCheckBox(ARM_JOINTS[i].hardware_name);
        checks_[i]->setChecked(true);
        checks_[i]->setFont(font);
        checks_[i]->setStyleSheet(
            QString("QCheckBox { color: %1; } "
                    "QCheckBox::indicator:checked { background: %1; border-color: %1; }")
            .arg(theme::MotorColors[i]));

        QObject::connect(checks_[i], &QCheckBox::toggled, [this, i](bool on) {
            pos_plot_->setSeriesVisible(i, on);
            vel_plot_->setSeriesVisible(i, on);
        });
        controls->addWidget(checks_[i]);
    }

    controls->addStretch();

    auto* time_label = new QLabel("Window:");
    time_label->setFont(font);
    controls->addWidget(time_label);

    auto* time_combo = new QComboBox();
    time_combo->setFont(font);
    time_combo->addItem("10s", 10.0);
    time_combo->addItem("30s", 30.0);
    time_combo->addItem("60s", 60.0);
    time_combo->setCurrentIndex(1);
    QObject::connect(time_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     [this, time_combo](int idx) {
        double w = time_combo->itemData(idx).toDouble();
        pos_plot_->setTimeWindow(w);
        vel_plot_->setTimeWindow(w);
    });
    controls->addWidget(time_combo);

    pause_btn_ = new QPushButton("Pause");
    pause_btn_->setFont(font);
    QObject::connect(pause_btn_, &QPushButton::clicked, [this]() {
        paused_ = !paused_;
        pause_btn_->setText(paused_ ? "Resume" : "Pause");
        pos_plot_->setPaused(paused_);
        vel_plot_->setPaused(paused_);
    });
    controls->addWidget(pause_btn_);

    auto* clear_btn = new QPushButton("Clear");
    clear_btn->setFont(font);
    QObject::connect(clear_btn, &QPushButton::clicked, [this]() {
        pos_plot_->clear();
        vel_plot_->clear();
    });
    controls->addWidget(clear_btn);

    layout->addLayout(controls);

    return widget;
}

void PlottingModule::setNode(rclcpp::Node::SharedPtr node) {
    auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    sub_ = node->create_subscription<rover_msgs::msg::MoteusArmStatus>(
        "/arm/moteus_feedback", qos,
        [this](const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
            onFeedback(msg);
        });
}

void PlottingModule::onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
    if (!pos_plot_) return;
    double now = elapsed_.elapsed() / 1000.0;

    for (int i = 0; i < NUM_MOTORS && i < (int)msg->status.size(); i++) {
        pos_plot_->addPoint(i, now, msg->status[i].curr_position);
        vel_plot_->addPoint(i, now, msg->status[i].curr_velocity);
    }
}

PLUGINLIB_EXPORT_CLASS(PlottingModule, rover_hmi_core::GuiModule)
