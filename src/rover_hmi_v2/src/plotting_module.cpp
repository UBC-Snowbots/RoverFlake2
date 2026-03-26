#include "plotting_module.h"
#include "catppuccin.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTabWidget>
#include <QFont>

static const char* JOINT_NAMES[] = {
    "Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "End Effector"
};

QWidget* PlottingModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    auto* layout = new QVBoxLayout(widget);
    layout->setSpacing(6);

    // Plots in tabs
    auto* tabs = new QTabWidget();

    pos_plot_ = new PlotWidget("Position (rev)");
    vel_plot_ = new PlotWidget("Velocity (rev/s)");

    for (int i = 0; i < NUM_MOTORS; i++) {
        pos_plot_->addSeries(JOINT_NAMES[i], theme::MotorColors[i]);
        vel_plot_->addSeries(JOINT_NAMES[i], theme::MotorColors[i]);
    }

    tabs->addTab(pos_plot_, "Position");
    tabs->addTab(vel_plot_, "Velocity");
    layout->addWidget(tabs, 1);

    // Legend + controls row
    auto* controls = new QHBoxLayout();
    controls->setSpacing(10);

    QFont font("monospace", theme::FontSize);

    for (int i = 0; i < NUM_MOTORS; i++) {
        checks_[i] = new QCheckBox(JOINT_NAMES[i]);
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

    // Time window selector
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

    // Pause button
    pause_btn_ = new QPushButton("Pause");
    pause_btn_->setFont(font);
    QObject::connect(pause_btn_, &QPushButton::clicked, [this]() {
        paused_ = !paused_;
        pause_btn_->setText(paused_ ? "Resume" : "Pause");
        pos_plot_->setPaused(paused_);
        vel_plot_->setPaused(paused_);
    });
    controls->addWidget(pause_btn_);

    // Clear button
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

void PlottingModule::setDataBus(MoteusDataBus* bus) {
    QObject::connect(bus, &MoteusDataBus::telemetryUpdated,
                     [this](const std::array<MotorState, NUM_MOTORS>& s) {
                         onTelemetry(s);
                     });
}

void PlottingModule::onTelemetry(const std::array<MotorState, NUM_MOTORS>& states) {
    for (const auto& s : states) {
        if (s.id < 1 || s.id > NUM_MOTORS) continue;
        int idx = s.id - 1;
        pos_plot_->addPoint(idx, s.timestamp, s.position);
        vel_plot_->addPoint(idx, s.timestamp, s.velocity);
    }
}
