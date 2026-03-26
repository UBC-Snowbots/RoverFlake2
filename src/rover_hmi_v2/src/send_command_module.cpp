#include "send_command_module.h"
#include "catppuccin.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QFont>

static const char* JOINT_NAMES[] = {
    "1 - Base", "2 - Shoulder", "3 - Elbow",
    "4 - Wrist Pitch", "5 - Wrist Roll", "6 - End Effector"
};

QWidget* SendCommandModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    auto* layout = new QVBoxLayout(widget);
    layout->setSpacing(10);

    QFont font("monospace", theme::FontSize);
    QFont fontBold("monospace", theme::FontSize, QFont::Bold);

    auto* grid = new QGridLayout();
    grid->setSpacing(8);

    // Motor select
    auto* motor_lbl = new QLabel("Motor:");
    motor_lbl->setFont(fontBold);
    grid->addWidget(motor_lbl, 0, 0);
    motor_select_ = new QComboBox();
    motor_select_->setFont(font);
    for (int i = 0; i < NUM_MOTORS; i++)
        motor_select_->addItem(JOINT_NAMES[i], i + 1);
    grid->addWidget(motor_select_, 0, 1);

    // Command type
    auto* cmd_lbl = new QLabel("Command:");
    cmd_lbl->setFont(fontBold);
    grid->addWidget(cmd_lbl, 1, 0);
    cmd_type_ = new QComboBox();
    cmd_type_->setFont(font);
    cmd_type_->addItem("Stop");
    cmd_type_->addItem("Position");
    grid->addWidget(cmd_type_, 1, 1);

    // Position
    auto* pos_lbl = new QLabel("Position (rev):");
    pos_lbl->setFont(font);
    grid->addWidget(pos_lbl, 2, 0);
    position_spin_ = new QDoubleSpinBox();
    position_spin_->setFont(font);
    position_spin_->setRange(-100.0, 100.0);
    position_spin_->setDecimals(3);
    position_spin_->setSingleStep(0.01);
    grid->addWidget(position_spin_, 2, 1);

    // Velocity
    auto* vel_lbl = new QLabel("Velocity (rev/s):");
    vel_lbl->setFont(font);
    grid->addWidget(vel_lbl, 3, 0);
    velocity_spin_ = new QDoubleSpinBox();
    velocity_spin_->setFont(font);
    velocity_spin_->setRange(-50.0, 50.0);
    velocity_spin_->setDecimals(3);
    velocity_spin_->setSingleStep(0.1);
    grid->addWidget(velocity_spin_, 3, 1);

    // Max torque
    auto* torq_lbl = new QLabel("Max Torque (Nm):");
    torq_lbl->setFont(font);
    grid->addWidget(torq_lbl, 4, 0);
    torque_spin_ = new QDoubleSpinBox();
    torque_spin_->setFont(font);
    torque_spin_->setRange(0.0, 10.0);
    torque_spin_->setDecimals(2);
    torque_spin_->setSingleStep(0.1);
    torque_spin_->setValue(0.5);
    grid->addWidget(torque_spin_, 4, 1);

    layout->addLayout(grid);

    // Buttons
    auto* btns = new QHBoxLayout();

    auto* send_btn = new QPushButton("Send");
    send_btn->setFont(fontBold);
    QObject::connect(send_btn, &QPushButton::clicked, [this]() {
        if (!bus_) return;
        int id = motor_select_->currentData().toInt();
        if (cmd_type_->currentText() == "Stop") {
            bus_->sendStop(id);
        } else {
            bus_->sendPosition(id, position_spin_->value(),
                               velocity_spin_->value(),
                               torque_spin_->value());
        }
    });
    btns->addWidget(send_btn);

    auto* estop_btn = new QPushButton("E-STOP ALL");
    estop_btn->setFont(fontBold);
    estop_btn->setStyleSheet(
        QString("QPushButton { background: %1; color: #000000; border: 2px solid %1; padding: 10px 18px; }"
                "QPushButton:hover { background: #ff6688; }")
        .arg(theme::Red));
    QObject::connect(estop_btn, &QPushButton::clicked, [this]() {
        if (bus_) bus_->sendStopAll();
    });
    btns->addWidget(estop_btn);

    btns->addStretch();
    layout->addLayout(btns);
    layout->addStretch();

    return widget;
}

void SendCommandModule::setDataBus(MoteusDataBus* bus) {
    bus_ = bus;
}
