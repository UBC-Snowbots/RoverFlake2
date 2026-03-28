// send_command_module.cpp
// See send_command_module.h for module overview.

#include "send_command_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QFont>

#include <pluginlib/class_list_macros.hpp>

// JogButton overrides the mouse press/release events to emit dedicated signals.
// The standard clicked() signal only fires on release, which makes it
// unsuitable for jog control. Instead:
//   jogPressed  → fires immediately on mouse-down → sendVelocity(speed)
//   jogReleased → fires on mouse-up              → sendVelocity(0)
JogButton::JogButton(const QString& text, QWidget* parent) : QPushButton(text, parent) {}

void JogButton::mousePressEvent(QMouseEvent* e) {
    QPushButton::mousePressEvent(e);
    emit jogPressed();
}

void JogButton::mouseReleaseEvent(QMouseEvent* e) {
    QPushButton::mouseReleaseEvent(e);
    emit jogReleased();
}

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

    auto* motor_lbl = new QLabel("Motor:");
    motor_lbl->setFont(fontBold);
    grid->addWidget(motor_lbl, 0, 0);
    motor_select_ = new QComboBox();
    motor_select_->setFont(font);
    for (int i = 0; i < NUM_MOTORS; i++)
        motor_select_->addItem(JOINT_NAMES[i], i + 1);
    grid->addWidget(motor_select_, 0, 1, 1, 2);

    auto* cmd_lbl = new QLabel("Command:");
    cmd_lbl->setFont(fontBold);
    grid->addWidget(cmd_lbl, 1, 0);
    cmd_type_ = new QComboBox();
    cmd_type_->setFont(font);
    cmd_type_->addItem("Stop");
    cmd_type_->addItem("Position");
    grid->addWidget(cmd_type_, 1, 1, 1, 2);

    pos_enable_ = new QCheckBox();
    pos_enable_->setChecked(true);
    pos_enable_->setToolTip("Uncheck for NaN (hold current position)");
    position_spin_ = new QDoubleSpinBox();
    position_spin_->setFont(font);
    position_spin_->setRange(-100.0, 100.0);
    position_spin_->setDecimals(3);
    position_spin_->setSingleStep(0.01);
    auto* pos_lbl = new QLabel("Position (rev):");
    pos_lbl->setFont(font);
    grid->addWidget(pos_lbl, 2, 0);
    grid->addWidget(position_spin_, 2, 1);
    grid->addWidget(pos_enable_, 2, 2);

    QObject::connect(pos_enable_, &QCheckBox::toggled, [this](bool on) {
        position_spin_->setEnabled(on);
        position_spin_->setStyleSheet(on ? "" :
            QString("QDoubleSpinBox { color: %1; }").arg(theme::TextDim));
    });

    vel_enable_ = new QCheckBox();
    vel_enable_->setChecked(true);
    vel_enable_->setToolTip("Uncheck for NaN");
    velocity_spin_ = new QDoubleSpinBox();
    velocity_spin_->setFont(font);
    velocity_spin_->setRange(-50.0, 50.0);
    velocity_spin_->setDecimals(3);
    velocity_spin_->setSingleStep(0.1);
    auto* vel_lbl = new QLabel("Velocity (rev/s):");
    vel_lbl->setFont(font);
    grid->addWidget(vel_lbl, 3, 0);
    grid->addWidget(velocity_spin_, 3, 1);
    grid->addWidget(vel_enable_, 3, 2);

    QObject::connect(vel_enable_, &QCheckBox::toggled, [this](bool on) {
        velocity_spin_->setEnabled(on);
        velocity_spin_->setStyleSheet(on ? "" :
            QString("QDoubleSpinBox { color: %1; }").arg(theme::TextDim));
    });

    torque_enable_ = new QCheckBox();
    torque_enable_->setChecked(false);
    torque_enable_->setToolTip("Uncheck for NaN (no torque limit)");
    torque_spin_ = new QDoubleSpinBox();
    torque_spin_->setFont(font);
    torque_spin_->setRange(0.0, 10.0);
    torque_spin_->setDecimals(2);
    torque_spin_->setSingleStep(0.1);
    torque_spin_->setValue(0.5);
    torque_spin_->setEnabled(false);
    torque_spin_->setStyleSheet(
        QString("QDoubleSpinBox { color: %1; }").arg(theme::TextDim));
    auto* torq_lbl = new QLabel("Max Torque (Nm):");
    torq_lbl->setFont(font);
    grid->addWidget(torq_lbl, 4, 0);
    grid->addWidget(torque_spin_, 4, 1);
    grid->addWidget(torque_enable_, 4, 2);

    QObject::connect(torque_enable_, &QCheckBox::toggled, [this](bool on) {
        torque_spin_->setEnabled(on);
        torque_spin_->setStyleSheet(on ? "" :
            QString("QDoubleSpinBox { color: %1; }").arg(theme::TextDim));
    });

    auto* nan_hint = new QLabel("Uncheck = NaN (e.g. d pos nan 0 nan)");
    nan_hint->setFont(QFont("monospace", theme::FontSizeSm));
    nan_hint->setStyleSheet(QString("color: %1;").arg(theme::TextDim));
    grid->addWidget(nan_hint, 5, 0, 1, 3);

    layout->addLayout(grid);

    auto* btns = new QHBoxLayout();

    auto* send_btn = new QPushButton("Send");
    send_btn->setFont(fontBold);
    QObject::connect(send_btn, &QPushButton::clicked, [this]() {
        int id = motor_select_->currentData().toInt();
        if (cmd_type_->currentText() == "Stop") {
            sendStop(id);
        } else {
            double pos    = pos_enable_->isChecked()    ? position_spin_->value() : NAN;
            double vel    = vel_enable_->isChecked()    ? velocity_spin_->value() : NAN;
            double torque = torque_enable_->isChecked() ? torque_spin_->value()   : NAN;
            sendPosition(id, pos, vel, torque);
        }
    });
    btns->addWidget(send_btn);

    auto* estop_btn = new QPushButton("E-STOP ALL");
    estop_btn->setFont(fontBold);
    estop_btn->setStyleSheet(
        QString("QPushButton { background: %1; color: #000000; border: 2px solid %1; padding: 10px 18px; }"
                "QPushButton:hover { background: #ff6688; }")
        .arg(theme::Red));
    QObject::connect(estop_btn, &QPushButton::clicked, [this]() { sendStopAll(); });
    btns->addWidget(estop_btn);

    btns->addStretch();
    layout->addLayout(btns);

    auto* jog_sep = new QWidget();
    jog_sep->setFixedHeight(1);
    jog_sep->setStyleSheet(QString("background: %1;").arg(theme::BorderDim));
    layout->addWidget(jog_sep);

    auto* jog_title = new QLabel("Jog (hold to move)");
    jog_title->setFont(fontBold);
    jog_title->setStyleSheet(QString("color: %1;").arg(theme::Text));
    layout->addWidget(jog_title);

    auto* jog_row = new QHBoxLayout();
    jog_row->setSpacing(8);

    auto* jog_speed_lbl = new QLabel("Speed:");
    jog_speed_lbl->setFont(font);
    jog_row->addWidget(jog_speed_lbl);

    jog_speed_spin_ = new QDoubleSpinBox();
    jog_speed_spin_->setFont(font);
    jog_speed_spin_->setRange(0.001, 10.0);
    jog_speed_spin_->setDecimals(3);
    jog_speed_spin_->setSingleStep(0.01);
    jog_speed_spin_->setValue(0.05);
    jog_speed_spin_->setSuffix(" rev/s");
    jog_row->addWidget(jog_speed_spin_);

    auto btnStyle = QString(
        "QPushButton { background: %1; color: %2; border: 1px solid %3; "
        "padding: 8px 16px; font-weight: bold; }"
        "QPushButton:pressed { background: %4; }")
        .arg(theme::Bg).arg(theme::Text).arg(theme::Border).arg(theme::Green);

    auto* jog_minus = new JogButton("-");
    jog_minus->setFont(QFont("monospace", theme::FontSizeLg, QFont::Bold));
    jog_minus->setStyleSheet(btnStyle);
    jog_row->addWidget(jog_minus);

    auto* jog_plus = new JogButton("+");
    jog_plus->setFont(QFont("monospace", theme::FontSizeLg, QFont::Bold));
    jog_plus->setStyleSheet(btnStyle);
    jog_row->addWidget(jog_plus);

    layout->addLayout(jog_row);

    QObject::connect(jog_plus, &JogButton::jogPressed, [this]() {
        sendVelocity(motor_select_->currentData().toInt(), jog_speed_spin_->value());
    });
    QObject::connect(jog_plus, &JogButton::jogReleased, [this]() {
        sendVelocity(motor_select_->currentData().toInt(), 0.0);
    });
    QObject::connect(jog_minus, &JogButton::jogPressed, [this]() {
        sendVelocity(motor_select_->currentData().toInt(), -jog_speed_spin_->value());
    });
    QObject::connect(jog_minus, &JogButton::jogReleased, [this]() {
        sendVelocity(motor_select_->currentData().toInt(), 0.0);
    });

    auto* zero_sep = new QWidget();
    zero_sep->setFixedHeight(1);
    zero_sep->setStyleSheet(QString("background: %1;").arg(theme::BorderDim));
    layout->addWidget(zero_sep);

    auto* zero_header = new QHBoxLayout();
    auto* zero_title = new QLabel("Zero Axes");
    zero_title->setFont(fontBold);
    zero_title->setStyleSheet(QString("color: %1;").arg(theme::Text));
    zero_header->addWidget(zero_title);
    zero_header->addStretch();

    auto* all_none_btn = new QPushButton("All / None");
    all_none_btn->setFont(QFont("monospace", theme::FontSizeSm));
    all_none_btn->setStyleSheet(
        QString("QPushButton { background: %1; color: %2; border: 1px solid %3; padding: 2px 8px; }"
                "QPushButton:hover { border-color: %4; }")
        .arg(theme::Bg).arg(theme::TextDim).arg(theme::BorderDim).arg(theme::Border));
    zero_header->addWidget(all_none_btn);
    layout->addLayout(zero_header);

    auto* zero_checks_row = new QHBoxLayout();
    zero_checks_row->setSpacing(6);

    static const char* ZERO_LABELS[] = { "1", "2", "3", "4", "5", "6" };
    for (int i = 0; i < NUM_ZERO_AXES; i++) {
        auto* col = new QVBoxLayout();
        col->setSpacing(2);
        col->setAlignment(Qt::AlignHCenter);

        auto* name_lbl = new QLabel(ZERO_LABELS[i]);
        name_lbl->setFont(QFont("monospace", theme::FontSizeSm));
        name_lbl->setStyleSheet(QString("color: %1;").arg(theme::TextDim));
        name_lbl->setAlignment(Qt::AlignHCenter);
        col->addWidget(name_lbl);

        zero_checks_[i] = new QCheckBox();
        zero_checks_[i]->setChecked(true);
        zero_checks_[i]->setStyleSheet(
            QString("QCheckBox::indicator { width: 16px; height: 16px; }"
                    "QCheckBox::indicator:checked { background: %1; border: 1px solid %2; border-radius: 3px; }"
                    "QCheckBox::indicator:unchecked { background: %3; border: 1px solid %4; border-radius: 3px; }")
            .arg(theme::Green).arg(theme::Border).arg(theme::Bg).arg(theme::BorderDim));
        col->addWidget(zero_checks_[i], 0, Qt::AlignHCenter);

        zero_checks_row->addLayout(col);
    }
    zero_checks_row->addStretch();
    layout->addLayout(zero_checks_row);

    auto* zero_btn = new QPushButton("Zero Selected");
    zero_btn->setFont(fontBold);
    zero_btn->setStyleSheet(
        QString("QPushButton { background: %1; color: %2; border: 1px solid %3; padding: 6px 14px; }"
                "QPushButton:hover { border-color: %4; }")
        .arg(theme::Bg).arg(theme::Yellow).arg(theme::Yellow).arg(theme::Text));
    layout->addWidget(zero_btn);

    QObject::connect(zero_btn, &QPushButton::clicked, [this]() {
        for (int i = 0; i < NUM_ZERO_AXES; i++)
            if (zero_checks_[i]->isChecked())
                sendZero(i + 1);
    });

    QObject::connect(all_none_btn, &QPushButton::clicked, [this]() {
        bool any_unchecked = false;
        for (int i = 0; i < NUM_ZERO_AXES; i++)
            if (!zero_checks_[i]->isChecked()) { any_unchecked = true; break; }
        for (int i = 0; i < NUM_ZERO_AXES; i++)
            zero_checks_[i]->setChecked(any_unchecked);
    });

    layout->addStretch();
    return widget;
}

void SendCommandModule::setNode(rclcpp::Node::SharedPtr node) {
    auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    cmd_pub_ = node->create_publisher<rover_msgs::msg::ArmCommand>("/arm/command", qos);
    log_pub_ = node->create_publisher<std_msgs::msg::String>("/arm/hmi_log", qos);
}

// Publish a human-readable description of a command to /arm/hmi_log so
// CommandLogModule can display it. Using a topic (rather than a direct call)
// keeps the two modules decoupled — CommandLogModule doesn't need to know
// about SendCommandModule and vice versa.
void SendCommandModule::logCmd(const QString& cmd) {
    if (!log_pub_) return;
    std_msgs::msg::String msg;
    msg.data = cmd.toStdString();
    log_pub_->publish(msg);
}

void SendCommandModule::sendPosition(int motor_id, double pos, double vel, double max_torque) {
    if (!cmd_pub_) return;
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ABS_POS;
    msg.positions.resize(NUM_MOTORS, NAN);
    msg.velocities.resize(NUM_MOTORS, NAN);
    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
        msg.positions[motor_id - 1] = pos;
        msg.velocities[motor_id - 1] = vel;
    }
    cmd_pub_->publish(msg);

    auto fmt = [](double v) -> QString {
        return std::isnan(v) ? "nan" : QString::number(v, 'f', 3);
    };
    logCmd(QString("%1> d pos %2 %3 %4")
           .arg(motor_id).arg(fmt(pos)).arg(fmt(vel)).arg(fmt(max_torque)));
}

void SendCommandModule::sendVelocity(int motor_id, double velocity) {
    if (!cmd_pub_) return;
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ABS_VEL;
    msg.velocities.resize(NUM_MOTORS, NAN);
    if (motor_id >= 1 && motor_id <= NUM_MOTORS)
        msg.velocities[motor_id - 1] = velocity;
    cmd_pub_->publish(msg);
    logCmd(QString("%1> d vel %2").arg(motor_id).arg(QString::number(velocity, 'f', 3)));
}

void SendCommandModule::sendStop(int motor_id) {
    if (!cmd_pub_) return;
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ABS_VEL;
    msg.velocities.resize(NUM_MOTORS, NAN);
    if (motor_id >= 1 && motor_id <= NUM_MOTORS)
        msg.velocities[motor_id - 1] = 0.0;
    cmd_pub_->publish(msg);
    logCmd(QString("%1> d stop").arg(motor_id));
}

void SendCommandModule::sendStopAll() {
    if (!cmd_pub_) return;
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_STOP;
    cmd_pub_->publish(msg);
    logCmd("A> d stop");
}

void SendCommandModule::sendZero(int motor_id) {
    if (!cmd_pub_) return;
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ZERO;
    msg.positions.resize(NUM_MOTORS, NAN);
    if (motor_id >= 1 && motor_id <= NUM_MOTORS)
        msg.positions[motor_id - 1] = 1.0;
    cmd_pub_->publish(msg);
    logCmd(QString("%1> d exact 0").arg(motor_id));
}

PLUGINLIB_EXPORT_CLASS(SendCommandModule, rover_hmi_core::GuiModule)
