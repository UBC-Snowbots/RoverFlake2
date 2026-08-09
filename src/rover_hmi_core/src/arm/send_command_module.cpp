// send_command_module.cpp
// See send_command_module.h for module overview.

#include "send_command_module.h"
#include <rover_arm_common/motor_config.h>  // HmiDefaults
#include <rover_hmi_core/catppuccin.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QFont>

#include <pluginlib/class_list_macros.hpp>

// JogButton — defined here so Q_OBJECT is in the .cpp (avoids AUTOMOC
// header-scanning issues with private include directories).
class JogButton : public QPushButton {
    Q_OBJECT
public:
    JogButton(const QString& text, QWidget* parent = nullptr);
signals:
    void jogPressed();
    void jogReleased();
protected:
    void mousePressEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
};

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

QWidget* SendCommandModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    auto* layout = new QVBoxLayout(widget);
    layout->setSpacing(10);

    QFont font("monospace", theme::FontSize);
    QFont fontBold("monospace", theme::FontSize, QFont::Bold);

    auto* grid = new QGridLayout();
    grid->setSpacing(8);

    auto* motor_lbl = new QLabel("Target:");
    motor_lbl->setFont(fontBold);
    grid->addWidget(motor_lbl, 0, 0);
    motor_select_ = new QComboBox();
    motor_select_->setFont(font);
    // Motors are the real targets (commands address CAN ids); A5/A6 are
    // differential combinations of M5+M6, appended after. Picking an entry
    // IS the motor/axis-space selection — no mode state.
    for (int i = 0; i < NUM_MOTORS; i++) {
        const bool wrist = (i == AXIS_5_INDEX || i == AXIS_6_INDEX);
        const QString hint = (i == AXIS_EE_INDEX) ? QStringLiteral(" – EE")
                           : (wrist ? QStringLiteral(" – wrist raw") : QString());
        motor_select_->addItem(QString("M%1%2").arg(i + 1).arg(hint), 100 + i + 1);
    }
    motor_select_->insertSeparator(motor_select_->count());
    motor_select_->addItem("A5 – wrist (M5+M6)", AXIS_5_INDEX + 1);
    motor_select_->addItem("A6 – wrist (M5−M6)", AXIS_6_INDEX + 1);
    grid->addWidget(motor_select_, 0, 1, 1, 2);

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
    grid->addWidget(pos_lbl, 1, 0);
    grid->addWidget(position_spin_, 1, 1);
    grid->addWidget(pos_enable_, 1, 2);

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
    grid->addWidget(vel_lbl, 2, 0);
    grid->addWidget(velocity_spin_, 2, 1);
    grid->addWidget(vel_enable_, 2, 2);

    QObject::connect(vel_enable_, &QCheckBox::toggled, [this](bool on) {
        velocity_spin_->setEnabled(on);
        velocity_spin_->setStyleSheet(on ? "" :
            QString("QDoubleSpinBox { color: %1; }").arg(theme::TextDim));
    });

    auto* nan_hint = new QLabel("Uncheck = NaN (skip that field, e.g. velocity-only jog)");
    nan_hint->setFont(QFont("monospace", theme::FontSizeSm));
    nan_hint->setStyleSheet(QString("color: %1;").arg(theme::TextDim));
    grid->addWidget(nan_hint, 3, 0, 1, 3);

    layout->addLayout(grid);

    auto* btns = new QHBoxLayout();

    auto* send_btn = new QPushButton("Send");
    send_btn->setFont(fontBold);
    QObject::connect(send_btn, &QPushButton::clicked, [this]() {
        // The checked boxes ARE the command: pos+vel, pos-only, or vel-only.
        double pos = pos_enable_->isChecked() ? position_spin_->value() : NAN;
        double vel = vel_enable_->isChecked() ? velocity_spin_->value() : NAN;
        sendPosition(targetId(), pos, vel);
    });
    btns->addWidget(send_btn);

    // Per-target "d stop": limp + clears a latched fault on just this target.
    auto* stop_btn = new QPushButton("D-STOP");
    stop_btn->setFont(fontBold);
    stop_btn->setStyleSheet(
        QString("QPushButton { background: %1; color: %2; border: 2px solid %2; padding: 10px 18px; }"
                "QPushButton:hover { border-color: %3; }")
        .arg(theme::Bg).arg(theme::Yellow).arg(theme::Text));
    QObject::connect(stop_btn, &QPushButton::clicked, [this]() { sendStop(targetId()); });
    auto updateStopLabel = [this, stop_btn]() {
        stop_btn->setText("D-STOP " + motor_select_->currentText().section(QChar(' '), 0, 0));
    };
    QObject::connect(motor_select_, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     [updateStopLabel](int) { updateStopLabel(); });
    updateStopLabel();
    btns->addWidget(stop_btn);

    // "d stop" to every motor (goes limp, clears faults). NOT an e-stop —
    // there is no software e-stop; kill power for that.
    auto* estop_btn = new QPushButton("D-STOP ALL");
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
        sendVelocity(targetId(), jog_speed_spin_->value());
    });
    QObject::connect(jog_plus, &JogButton::jogReleased, [this]() {
        sendVelocity(targetId(), 0.0);
    });
    QObject::connect(jog_minus, &JogButton::jogPressed, [this]() {
        sendVelocity(targetId(), -jog_speed_spin_->value());
    });
    QObject::connect(jog_minus, &JogButton::jogReleased, [this]() {
        sendVelocity(targetId(), 0.0);
    });

    // Picking a target loads its default velocity (HmiDefaults in
    // motor_config.h) into both speed boxes; 0 in the table = jog disabled.
    auto applyTargetDefaults = [this, jog_minus, jog_plus]() {
        const int idx = targetId() - 1;
        if (idx < 0 || idx >= NUM_MOTORS) return;
        const float def = targetMotorSpace() ? HmiDefaults::motor_velocity_revps[idx]
                                             : HmiDefaults::axis_velocity_revps[idx];
        const bool on = def > 0.0f;
        velocity_spin_->setValue(on ? def : 0.0);
        if (on) jog_speed_spin_->setValue(def);
        jog_speed_spin_->setEnabled(on);
        for (JogButton* b : {jog_minus, jog_plus}) {
            b->setEnabled(on);
            b->setToolTip(on ? QString()
                             : QStringLiteral("Disabled in motor_config.h (HmiDefaults)"));
        }
    };
    QObject::connect(motor_select_, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     [applyTargetDefaults](int) { applyTargetDefaults(); });
    applyTargetDefaults();

    auto* zero_sep = new QWidget();
    zero_sep->setFixedHeight(1);
    zero_sep->setStyleSheet(QString("background: %1;").arg(theme::BorderDim));
    layout->addWidget(zero_sep);

    auto* zero_header = new QHBoxLayout();
    auto* zero_title = new QLabel("Axis Select (zero / home)");
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

    for (int i = 0; i < NUM_ZERO_AXES; i++) {
        auto* col = new QVBoxLayout();
        col->setSpacing(2);
        col->setAlignment(Qt::AlignHCenter);

        auto* name_lbl = new QLabel(QString::number(i + 1));
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

    auto* zero_actions = new QHBoxLayout();
    auto* zero_btn = new QPushButton("Zero Selected");
    zero_btn->setFont(fontBold);
    zero_btn->setStyleSheet(
        QString("QPushButton { background: %1; color: %2; border: 1px solid %3; padding: 6px 14px; }"
                "QPushButton:hover { border-color: %4; }")
        .arg(theme::Bg).arg(theme::Yellow).arg(theme::Yellow).arg(theme::Text));
    zero_actions->addWidget(zero_btn);

    auto* home_btn = new QPushButton("Home Selected…");
    home_btn->setFont(fontBold);
    home_btn->setStyleSheet(
        QString("QPushButton { background: %1; color: %2; border: 1px solid %2; padding: 6px 14px; }"
                "QPushButton:hover { border-color: %3; }")
        .arg(theme::Bg).arg(theme::Green).arg(theme::Text));
    zero_actions->addWidget(home_btn);
    layout->addLayout(zero_actions);

    QObject::connect(zero_btn, &QPushButton::clicked, [this]() { sendZeroChecked(); });
    QObject::connect(home_btn, &QPushButton::clicked, [this]() { homeChecked(); });

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

int SendCommandModule::targetId() const {
    const int d = motor_select_ ? motor_select_->currentData().toInt() : 0;
    return d > 100 ? d - 100 : d;
}

bool SendCommandModule::targetMotorSpace() const {
    return motor_select_ && motor_select_->currentData().toInt() > 100;
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

void SendCommandModule::sendPosition(int motor_id, double pos, double vel) {
    if (!cmd_pub_) return;
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ABS_POS;
    msg.cmd_value = targetMotorSpace() ? CMD_SPACE_MOTOR : CMD_SPACE_AXIS;
    msg.positions.resize(NUM_MOTORS, NAN);
    msg.velocities.resize(NUM_MOTORS, NAN);
    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
        msg.positions[motor_id - 1] = pos;
        msg.velocities[motor_id - 1] = vel * 360.0;  // wire contract is deg/s, UI is rev/s
    }
    cmd_pub_->publish(msg);

    auto fmt = [](double v) -> QString {
        return std::isnan(v) ? "nan" : QString::number(v, 'f', 3);
    };
    logCmd(QString("%1%2> d pos %3 %4").arg(targetMotorSpace() ? 'M' : 'A')
           .arg(motor_id).arg(fmt(pos)).arg(fmt(vel)));
}

void SendCommandModule::sendVelocity(int motor_id, double velocity) {
    if (!cmd_pub_) return;
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ABS_VEL;
    msg.cmd_value = targetMotorSpace() ? CMD_SPACE_MOTOR : CMD_SPACE_AXIS;
    msg.velocities.resize(NUM_MOTORS, NAN);
    if (motor_id >= 1 && motor_id <= NUM_MOTORS)
        msg.velocities[motor_id - 1] = velocity * 360.0;  // wire contract is deg/s, UI is rev/s
    cmd_pub_->publish(msg);
    logCmd(QString("%1%2> d pos nan %3 nan").arg(targetMotorSpace() ? 'M' : 'A')
           .arg(motor_id).arg(QString::number(velocity, 'f', 3)));
}

// Real "d stop" for one target (motor goes limp, clears a latched fault) —
// unlike jog-release, which is velocity-0 hold.
void SendCommandModule::sendStop(int motor_id) {
    if (!cmd_pub_) return;
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_STOP;
    msg.cmd_value = targetMotorSpace() ? CMD_SPACE_MOTOR : CMD_SPACE_AXIS;
    msg.positions.resize(NUM_MOTORS, NAN);   // mask: non-NaN = stop that one
    if (motor_id >= 1 && motor_id <= NUM_MOTORS)
        msg.positions[motor_id - 1] = 1.0;
    cmd_pub_->publish(msg);
    logCmd(QString("%1%2> d stop").arg(targetMotorSpace() ? 'M' : 'A').arg(motor_id));
}

void SendCommandModule::sendStopAll() {
    if (!cmd_pub_) return;
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_STOP;
    cmd_pub_->publish(msg);
    logCmd("A> d stop");
}

// Switch homing: the Zero Axes checkboxes select which axes home (switch-
// equipped axes 1-6; EE has no switch and the driver denies it anyway).
// Each axis creeps to its switch, zeroes THERE, and parks just off it —
// independent per axis; the wrist pair (A5/A6) homes one at a time.
void SendCommandModule::homeChecked() {
    if (!cmd_pub_) return;
    QStringList names;
    std::vector<double> selected;
    for (int i = 0; i < NUM_ZERO_AXES && i <= AXIS_6_INDEX; i++) {
        if (!zero_checks_[i]->isChecked()) continue;
        selected.push_back((double)i);
        names << QString("A%1").arg(i + 1);
    }
    if (selected.empty()) return;

    QMessageBox box;
    box.setWindowTitle("Home arm");
    box.setText(QString("Make sure the path to each limit switch is clear.\n\n"
                        "On confirm, %1 will each creep to their limit switch, "
                        "zero there, and park just off the switch. A5/A6 home "
                        "one at a time.")
                    .arg(names.join(", ")));
    box.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    box.setDefaultButton(QMessageBox::Cancel);
    box.setFont(QFont("monospace", theme::FontSize));
    box.setStyleSheet(QString(
        "QMessageBox { background: %1; }"
        "QLabel { color: %2; background: transparent; }"
        "QPushButton { background: %1; color: %2; border: 1px solid %3; padding: 6px 14px; min-width: 70px; }"
        "QPushButton:hover { border-color: %2; }")
        .arg(theme::Bg).arg(theme::Text).arg(theme::Border));
    if (box.exec() != QMessageBox::Ok) return;

    rover_msgs::msg::ArmCommand home;
    home.cmd_type = CMD_HOME;
    home.cmd_value = HOME_VALUE_SELECTED;
    home.positions = selected;
    cmd_pub_->publish(home);

    logCmd(QString("%1> home (switch = 0)").arg(names.join(",")));
}

// One message for all selected axes: per-axis publishes on this KeepLast(1)
// topic overwrite each other in flight, so only the last axis ever zeroed.
void SendCommandModule::sendZeroChecked() {
    if (!cmd_pub_) return;
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ZERO;
    msg.positions.resize(NUM_MOTORS, NAN);
    QStringList ids;
    for (int i = 0; i < NUM_ZERO_AXES && i < NUM_MOTORS; i++) {
        if (!zero_checks_[i]->isChecked()) continue;
        msg.positions[i] = 1.0;
        ids << QString::number(i + 1);
    }
    if (ids.isEmpty()) return;
    cmd_pub_->publish(msg);
    logCmd(QString("%1> d exact 0").arg(ids.join(",")));
}

PLUGINLIB_EXPORT_CLASS(SendCommandModule, rover_hmi_core::GuiModule)
#include "send_command_module.moc"
