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
#include <QKeyEvent>
#include <QApplication>
#include <QSet>
#include <QTimer>

#include <algorithm>

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

// ---------------------------------------------------------------------------
// Keyboard jog
// ---------------------------------------------------------------------------
// One key per direction per axis. Several keys can be down at once, and the
// resulting velocities go out in a SINGLE CMD_ABS_VEL message — the driver's
// commandCallback loops over velocities[] and applies every non-NaN entry, so
// N held keys means N axes moving together. (Publishing one message per axis
// would not work: /arm/command is KeepLast(1), so all but the last would be
// dropped in flight.)
//
// Bindings are laid out as a left-hand cluster for the arm proper and a
// right-hand cluster for the wrist + end effector:
//
//   Q/E  A3 elbow        I/K  A5 wrist pitch
//   W/S  A2 shoulder     J/L  A6 wrist roll
//   A/D  A1 base         U/O  EE
//   Z/C  A4 elbow twist
// Each binding carries both meanings of the key: which firmware axis it jogs in
// joint mode, and which Cartesian twist component it drives in IK mode. Rows of
// the readout grid are indexed by `axis` in both modes — the axis→twist map is
// a bijection, so only the row labels change.
struct KeyJogBinding {
    int         key;
    const char* label;
    int         axis;   // firmware axis (joint mode)
    double      dir;    // +1 / -1
    int         twist;  // IkIndex (IK mode), or -1 for none (EE)
};

static const KeyJogBinding KEY_JOG_BINDINGS[] = {
    { Qt::Key_A, "A", AXIS_1_INDEX,  +1.0, IK_LIN_Y_INDEX }, { Qt::Key_D, "D", AXIS_1_INDEX,  -1.0, IK_LIN_Y_INDEX },
    { Qt::Key_W, "W", AXIS_2_INDEX,  +1.0, IK_LIN_X_INDEX }, { Qt::Key_S, "S", AXIS_2_INDEX,  -1.0, IK_LIN_X_INDEX },
    { Qt::Key_Q, "Q", AXIS_3_INDEX,  +1.0, IK_LIN_Z_INDEX }, { Qt::Key_E, "E", AXIS_3_INDEX,  -1.0, IK_LIN_Z_INDEX },
    { Qt::Key_Z, "Z", AXIS_4_INDEX,  +1.0, IK_ANG_X_INDEX }, { Qt::Key_C, "C", AXIS_4_INDEX,  -1.0, IK_ANG_X_INDEX },
    { Qt::Key_I, "I", AXIS_5_INDEX,  +1.0, IK_ANG_Y_INDEX }, { Qt::Key_K, "K", AXIS_5_INDEX,  -1.0, IK_ANG_Y_INDEX },
    { Qt::Key_J, "J", AXIS_6_INDEX,  +1.0, IK_ANG_Z_INDEX }, { Qt::Key_L, "L", AXIS_6_INDEX,  -1.0, IK_ANG_Z_INDEX },
    { Qt::Key_U, "U", AXIS_EE_INDEX, +1.0, -1             }, { Qt::Key_O, "O", AXIS_EE_INDEX, -1.0, -1             },
};
constexpr int NUM_KEY_JOG_BINDINGS = (int)(sizeof(KEY_JOG_BINDINGS) / sizeof(KEY_JOG_BINDINGS[0]));

static const char* const KEY_JOG_AXIS_NAMES[NUM_AXES] = {
    "A1 base", "A2 shoulder", "A3 elbow", "A4 twist",
    "A5 wrist pitch", "A6 wrist roll", "EE",
};

// Same rows, Cartesian meaning. EE keeps jogging the EE axis directly in IK
// mode — it is not part of the Servo planning group.
static const char* const KEY_JOG_TWIST_NAMES[NUM_AXES] = {
    "lin Y (left/right)", "lin X (fwd/back)", "lin Z (up/down)", "ang X (roll)",
    "ang Y (pitch)", "ang Z (yaw)", "EE (direct)",
};

// MoveIt Servo's Cartesian input. Mirrors ArmConstants::servo_ik_topic in
// arm_control/include/armControlParams.h — duplicated rather than included
// because rover_hmi_core does not depend on arm_control.
static constexpr const char* SERVO_TWIST_TOPIC = "/arm_moveit_control/delta_twist_cmds";

// ---------------------------------------------------------------------------
// PS4 pad jog
// ---------------------------------------------------------------------------
// Same axes as the keyboard, driven by sticks instead of keys. joy_linux
// already owns the device and publishes /joy, so there is nothing to detect or
// open here — the panel just subscribes and reads the indices below.
//
// Mirrors ps4_index and the PS4_JOY_LINUX case of
// ArmControllerConfig::process_joy_input in arm_control/include/
// controller_config.h — duplicated rather than included for the same reason as
// SERVO_TWIST_TOPIC: rover_hmi_core does not depend on arm_control. Keep the
// two in step.
namespace ps4_index {
    namespace axes {
        constexpr int LEFT_JOYSTICK_X  = 0;
        constexpr int LEFT_JOYSTICK_Y  = 1;
        constexpr int L2               = 2;   // left trigger
        constexpr int RIGHT_JOYSTICK_X = 3;
        constexpr int RIGHT_JOYSTICK_Y = 4;
        constexpr int R2               = 5;   // right trigger
        constexpr int DPAD_X           = 6;
        constexpr int DPAD_Y           = 7;
    }
    namespace buttons {
        constexpr int L1    = 4;   // left bumper
        constexpr int R1    = 5;   // right bumper
        constexpr int SHARE = 8;   // left of the trackpad
    }
}

// Which pad control drives which firmware axis in joint mode. Rows match
// KEY_JOG_AXIS_NAMES, so the readout grid lights up the same way it does for
// keys. EE is the L1/R1 pair, not an axis, so it is handled separately.
static const struct { int axis; int joy_axis; } PS4_FK_BINDINGS[] = {
    { AXIS_1_INDEX, ps4_index::axes::LEFT_JOYSTICK_X  },
    { AXIS_2_INDEX, ps4_index::axes::LEFT_JOYSTICK_Y  },
    { AXIS_3_INDEX, ps4_index::axes::RIGHT_JOYSTICK_Y },
    { AXIS_4_INDEX, ps4_index::axes::RIGHT_JOYSTICK_X },
    { AXIS_5_INDEX, ps4_index::axes::DPAD_Y           },
    { AXIS_6_INDEX, ps4_index::axes::DPAD_X           },
};

// Cartesian mode. Not the same pairing as the joint map — the triggers take
// over yaw and the right stick X drops out, exactly as in controller_config.h.
static const struct { int twist; int joy_axis; } PS4_IK_BINDINGS[] = {
    { IK_LIN_X_INDEX, ps4_index::axes::LEFT_JOYSTICK_Y  },
    { IK_LIN_Y_INDEX, ps4_index::axes::LEFT_JOYSTICK_X  },
    { IK_LIN_Z_INDEX, ps4_index::axes::RIGHT_JOYSTICK_Y },
    { IK_ANG_X_INDEX, ps4_index::axes::DPAD_X           },
    { IK_ANG_Y_INDEX, ps4_index::axes::DPAD_Y           },
    // IK_ANG_Z is the trigger pair (L2 − R2) / 2, applied in activeTwist().
};

// /joy goes quiet for this long while the pad is armed → treat the pad as gone
// and stop. joy_linux runs at autorepeat_rate 100 Hz (game_controller.launch.py),
// so it republishes even when a stick is held still; silence is a real fault,
// not a still hand. Without this the driver re-sends its last command forever.
static constexpr qint64 JOY_TIMEOUT_MS = 500;

// Reads that fall off the end of a short Joy message return 0 rather than
// running past the vector — a pad that reports fewer axes must not crash the HMI.
static double joyAxis(const sensor_msgs::msg::Joy& m, int i) {
    return (i >= 0 && i < (int)m.axes.size()) ? (double)m.axes[i] : 0.0;
}
static int joyButton(const sensor_msgs::msg::Joy& m, int i) {
    return (i >= 0 && i < (int)m.buttons.size()) ? m.buttons[i] : 0;
}

// An axis with a 0 default in HmiDefaults is disabled in the config table; its
// keys are shown greyed and never contribute a velocity (same rule the +/- jog
// buttons already follow).
static bool keyJogAxisEnabled(int axis) {
    return axis >= 0 && axis < NUM_AXES && HmiDefaults::axis_velocity_revps[axis] > 0.0f;
}

// KeyJogFilter — an application-wide key grab, gated by the ARM toggle.
//
// Installed on qApp, so it sees every key event in the HMI before any widget
// does. That is the point: the arm toggle IS the safety, not widget focus.
// While armed the bound keys drive the arm no matter which panel is focused,
// and get consumed so nothing else reacts to them; while disarmed the filter
// is transparent and the spin boxes type normally.
//
// Modified keys are deliberately passed through, so the host's Alt+... tiling
// shortcuts and Ctrl+= zoom keep working even while armed.
class KeyJogFilter : public QObject {
    Q_OBJECT
public:
    explicit KeyJogFilter(QObject* parent = nullptr);

    void setArmed(bool on);
    bool armed() const { return armed_; }
    bool isHeld(int key) const { return held_.contains(key); }

    // Sum of the held directions per axis, -1/0/+1. Holding both keys of a
    // pair cancels to 0 rather than picking a winner.
    std::array<double, NUM_AXES> directions() const;
    void releaseAll();          // clears held keys and emits keysChanged()

signals:
    void keysChanged();
    void stopAllRequested();    // Space
    void windowLeft();          // HMI window deactivated → auto-released

protected:
    bool eventFilter(QObject* obj, QEvent* ev) override;

private:
    bool armed_ = false;
    QSet<int> held_;
};

KeyJogFilter::KeyJogFilter(QObject* parent) : QObject(parent) {
    if (qApp) qApp->installEventFilter(this);
}

void KeyJogFilter::setArmed(bool on) {
    if (armed_ == on) return;
    armed_ = on;
    if (on) {
        // Arming steals the keyboard, so drop the caret out of whatever spin
        // box holds it — otherwise it would sit there looking editable while
        // every keystroke goes to the arm instead.
        if (QWidget* f = QApplication::focusWidget()) f->clearFocus();
    } else {
        releaseAll();
    }
}

std::array<double, NUM_AXES> KeyJogFilter::directions() const {
    std::array<double, NUM_AXES> dirs{};
    dirs.fill(0.0);
    for (const auto& bind : KEY_JOG_BINDINGS) {
        if (!keyJogAxisEnabled(bind.axis)) continue;
        if (held_.contains(bind.key)) dirs[bind.axis] += bind.dir;
    }
    for (double& d : dirs) d = std::clamp(d, -1.0, 1.0);
    return dirs;
}

void KeyJogFilter::releaseAll() {
    if (held_.isEmpty()) return;
    held_.clear();
    emit keysChanged();
}

bool KeyJogFilter::eventFilter(QObject* obj, QEvent* ev) {
    const QEvent::Type type = ev->type();

    // Leaving the HMI window never delivers the key-up events, so the held set
    // would stay stale and the arm would keep running. Release on the way out.
    // This is the only involuntary disarm — clicking between panels inside the
    // window does not interrupt jogging.
    if (type == QEvent::WindowDeactivate || type == QEvent::ApplicationDeactivate) {
        if (armed_ && !held_.isEmpty()) { releaseAll(); emit windowLeft(); }
        return QObject::eventFilter(obj, ev);
    }

    if (!armed_ || (type != QEvent::KeyPress && type != QEvent::KeyRelease))
        return QObject::eventFilter(obj, ev);

    auto* ke = static_cast<QKeyEvent*>(ev);

    // Alt/Ctrl/Meta chords belong to the host (tiling, zoom, overlays).
    const auto mods = ke->modifiers() & ~Qt::ShiftModifier;
    if (mods != Qt::NoModifier) return QObject::eventFilter(obj, ev);

    // isAutoRepeat() is the crux of hold-to-move on X11/Wayland: a held key
    // generates a continuous release/press stream, and treating those as real
    // releases would make the axis stutter to a stop between repeats.
    const int key = ke->key();

    if (key == Qt::Key_Space) {
        if (type == QEvent::KeyPress && !ke->isAutoRepeat()) {
            releaseAll();
            emit stopAllRequested();
        }
        return true;
    }
    if (key == Qt::Key_Escape) {
        if (type == QEvent::KeyPress && !ke->isAutoRepeat()) releaseAll();
        return true;
    }

    bool bound = false;
    for (const auto& bind : KEY_JOG_BINDINGS) {
        if (bind.key == key) { bound = true; break; }
    }
    if (!bound) return QObject::eventFilter(obj, ev);

    if (ke->isAutoRepeat()) return true;   // swallow, but do not re-trigger

    if (type == QEvent::KeyPress) {
        if (!held_.contains(key)) { held_.insert(key); emit keysChanged(); }
    } else {
        if (held_.remove(key)) emit keysChanged();
    }
    return true;
}

QWidget* SendCommandModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    auto* layout = new QVBoxLayout(widget);
    layout->setSpacing(10);

    QFont font("monospace", theme::FontSize);
    QFont fontBold("monospace", theme::FontSize, QFont::Bold);

    buildKeyJogSection(layout, widget);

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

// Built at the TOP of the panel: the toggle has to stay visible, and panels
// here are not wrapped in a scroll area (hmi_host.cpp drops the module widget
// straight into the tile), so anything below the fold is unreachable.
void SendCommandModule::buildKeyJogSection(QVBoxLayout* layout, QWidget* owner) {
    QFont font("monospace", theme::FontSize);
    QFont fontBold("monospace", theme::FontSize, QFont::Bold);

    // Parented to the panel, so the filter is uninstalled and destroyed with it.
    key_jog_filter_ = new KeyJogFilter(owner);

    auto* ctl = new QHBoxLayout();
    ctl->setSpacing(8);

    // Input source: exactly one of these owns the arm at a time, and the lit
    // button says which. Three plain buttons rather than a combo — this is the
    // safety control, so what is live has to be readable across the room.
    static const char* const SRC_LABELS[3] = { "OFF", "ARM KEYBOARD", "ARM PS4" };
    for (int s = 0; s < 3; s++) {
        auto* b = new QPushButton(SRC_LABELS[s]);
        b->setFont(QFont("monospace", theme::FontSizeLg, QFont::Bold));
        // No focus: a button must never eat a keystroke itself (Space would
        // otherwise re-press it once it had been clicked).
        b->setFocusPolicy(Qt::NoFocus);
        QObject::connect(b, &QPushButton::clicked, [this, s]() { setInputSource(s); });
        src_btns_[s] = b;
        ctl->addWidget(b);
    }

    auto* scale_lbl = new QLabel("Speed x");
    scale_lbl->setFont(font);
    ctl->addWidget(scale_lbl);

    // Scale on top of the per-axis HmiDefaults velocity, so one control trims
    // the whole arm without flattening the (very different) per-axis limits.
    key_jog_scale_ = new QDoubleSpinBox();
    key_jog_scale_->setFont(font);
    key_jog_scale_->setRange(0.05, 2.0);
    key_jog_scale_->setDecimals(2);
    key_jog_scale_->setSingleStep(0.05);
    key_jog_scale_->setValue(1.0);
    ctl->addWidget(key_jog_scale_);

    key_jog_status_ = new QLabel();
    key_jog_status_->setFont(font);
    ctl->addWidget(key_jog_status_);
    ctl->addStretch();
    layout->addLayout(ctl);

    // Mode swap: the same keys either jog joints directly (ArmCommand) or feed
    // a Cartesian twist to MoveIt Servo, which solves IK and emits joint
    // velocities of its own.
    auto* mode_row = new QHBoxLayout();
    mode_row->setSpacing(8);
    auto* mode_lbl = new QLabel("Mode:");
    mode_lbl->setFont(font);
    mode_row->addWidget(mode_lbl);

    key_jog_mode_ = new QComboBox();
    key_jog_mode_->setFont(font);
    key_jog_mode_->addItem("Joint jog (FK)", 0);
    key_jog_mode_->addItem("Cartesian (IK — needs servo)", 1);
    mode_row->addWidget(key_jog_mode_);

    auto* frame_lbl = new QLabel("Frame:");
    frame_lbl->setFont(font);
    mode_row->addWidget(frame_lbl);

    // Servo transforms the twist from whatever frame this names, so it has to
    // be a real link in the loaded URDF — which differs between the v2 and v3
    // arm descriptions. Editable so it can be corrected without a rebuild.
    ik_frame_ = new QComboBox();
    ik_frame_->setFont(font);
    ik_frame_->setEditable(true);
    ik_frame_->addItems({"base_link", "link_0", "ee_base_link", "link_tt"});
    ik_frame_->setToolTip("header.frame_id of the twist — must exist in TF");
    mode_row->addWidget(ik_frame_);
    mode_row->addStretch();
    layout->addLayout(mode_row);

    // Live readout of what is held — display only, nothing to click.
    auto* chips = new QWidget();
    auto* grid = new QGridLayout(chips);
    grid->setSpacing(6);
    grid->setContentsMargins(0, 0, 0, 0);
    key_jog_row_names_.assign(NUM_AXES, nullptr);
    for (int a = 0; a < NUM_AXES; a++) {
        auto* name = new QLabel(KEY_JOG_AXIS_NAMES[a]);
        name->setFont(QFont("monospace", theme::FontSizeSm));
        key_jog_row_names_[a] = name;
        grid->addWidget(name, a, 2);
    }
    key_jog_chips_.assign(NUM_KEY_JOG_BINDINGS, nullptr);
    for (int b = 0; b < NUM_KEY_JOG_BINDINGS; b++) {
        const auto& bind = KEY_JOG_BINDINGS[b];
        auto* chip = new QLabel(QString("%1 %2").arg(bind.dir > 0 ? "+" : "−").arg(bind.label));
        chip->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
        chip->setAlignment(Qt::AlignCenter);
        chip->setMinimumWidth(64);
        key_jog_chips_[b] = chip;
        grid->addWidget(chip, bind.axis, bind.dir > 0 ? 1 : 0);
    }
    grid->setColumnStretch(2, 1);
    layout->addWidget(chips);

    auto* hint = new QLabel("Keyboard: keys drive the arm from any panel · "
                            "Space = D-STOP ALL · Esc = release keys\n"
                            "PS4: sticks + D-pad jog · L1/R1 = EE · SHARE = home (disarms, then asks)");
    hint->setFont(QFont("monospace", theme::FontSizeSm));
    hint->setStyleSheet(QString("color: %1;").arg(theme::TextDim));
    layout->addWidget(hint);

    auto* sep = new QWidget();
    sep->setFixedHeight(1);
    sep->setStyleSheet(QString("background: %1;").arg(theme::BorderDim));
    layout->addWidget(sep);

    // Two streaming jobs share this timer:
    //   IK  — Servo halts on incoming_command_timeout (1 s in rover_servo_params)
    //         and its publish_period is 0.034 s, so the twist has to be streamed
    //         rather than sent on change like the joint-mode command is.
    //   PS4 — sticks are analog and /joy arrives at 100 Hz; republishing here
    //         instead of per message keeps /arm/command (and the command log)
    //         at a sane rate, and gives the pad timeout somewhere to run.
    jog_timer_ = new QTimer(owner);
    jog_timer_->setInterval(30);
    QObject::connect(jog_timer_, &QTimer::timeout, [this]() {
        if (input_source_ == SRC_PS4) {
            // Notice a dead pad even while nothing is being commanded.
            const bool lost = !joyLive();
            if (lost != joy_lost_) {
                joy_lost_ = lost;
                styleSourceButtons();
                restyleKeyJogChips();
            }
        }
        // Both, in IK mode: Servo owns the joints, but the EE is outside its
        // planning group and still rides the joint path.
        if (keyJogIkMode()) publishKeyJogTwist();
        publishKeyJog(true);
    });

    QObject::connect(key_jog_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     [this](int) { applyKeyJogMode(); });
    QObject::connect(key_jog_filter_, &KeyJogFilter::keysChanged, [this]() {
        restyleKeyJogChips();
        publishKeyJog();
    });
    QObject::connect(key_jog_filter_, &KeyJogFilter::stopAllRequested, [this]() {
        restyleKeyJogChips();
        publishKeyJog();     // filter already dropped the keys → stops the arm
        sendStopAll();
    });
    // Leaving the window auto-releases but stays armed, so coming back resumes
    // without another click.
    QObject::connect(key_jog_filter_, &KeyJogFilter::windowLeft, [this]() {
        restyleKeyJogChips();
        publishKeyJog();
    });

    setInputSource(SRC_OFF);
    applyKeyJogMode();
}

bool SendCommandModule::keyJogIkMode() const {
    return key_jog_mode_ && key_jog_mode_->currentData().toInt() == 1;
}

// Switching mode mid-jog would leave the other pipeline latched at its last
// command, so release everything and let both paths publish their stop.
void SendCommandModule::applyKeyJogMode() {
    if (!key_jog_filter_) return;
    const bool ik = keyJogIkMode();

    key_jog_filter_->releaseAll();
    // Not publishKeyJog(): a pad stick stays deflected across the switch, and
    // in IK mode that path would leave the joints NaN — still latched.
    publishJogStop();
    if (twist_pub_) publishKeyJogTwist();   // zero twist

    for (int a = 0; a < NUM_AXES && a < (int)key_jog_row_names_.size(); a++)
        key_jog_row_names_[a]->setText(ik ? KEY_JOG_TWIST_NAMES[a] : KEY_JOG_AXIS_NAMES[a]);

    if (ik_frame_) ik_frame_->setEnabled(ik);
    // Streaming is needed for IK (Servo) in either source, and for the pad in
    // either mode (analog sticks). Keyboard + joint mode publishes on change.
    if (jog_timer_) {
        if (jogArmed() && (ik || input_source_ == SRC_PS4)) jog_timer_->start();
        else                                                jog_timer_->stop();
    }
    restyleKeyJogChips();
}

void SendCommandModule::restyleKeyJogChips() {
    const bool armed = jogArmed();
    const bool ik = keyJogIkMode();
    // For the pad there are no keys to be "held" — a chip lights when its axis
    // is actually being commanded in that direction. Above the pad's own
    // deadzone so a resting stick doesn't make the readout flicker.
    const auto dirs = activeDirections();
    const auto tw   = activeTwist();
    for (int b = 0; b < (int)key_jog_chips_.size() && b < NUM_KEY_JOG_BINDINGS; b++) {
        const auto& bind = KEY_JOG_BINDINGS[b];
        // In IK mode the twist keys are always live — Servo decides which
        // joints move, so a per-axis HmiDefaults of 0 doesn't disable them.
        // The EE keys stay direct-drive in both modes, so they keep that gate.
        const bool enabled = (ik && bind.twist >= 0) ? true : keyJogAxisEnabled(bind.axis);
        const double level = (ik && bind.twist >= 0) ? tw[bind.twist] : dirs[bind.axis];
        const bool active = (input_source_ == SRC_KEYBOARD)
                                ? key_jog_filter_ && key_jog_filter_->isHeld(bind.key)
                                : level * bind.dir > 0.15;
        const bool down = enabled && armed && active;
        const char* fg = !enabled ? theme::TextDim
                                  : (down ? theme::Bg : (armed ? theme::Text : theme::TextDim));
        const char* bg = down ? theme::Green : theme::Bg;
        const char* border = !enabled ? theme::BorderDim
                                      : (down ? theme::Green
                                              : (armed ? theme::Border : theme::BorderDim));
        key_jog_chips_[b]->setStyleSheet(
            QString("QLabel { color: %1; background: %2; border: 1px solid %3; "
                    "border-radius: 6px; padding: 4px 8px; }")
                .arg(fg).arg(bg).arg(border));
        key_jog_chips_[b]->setToolTip(
            enabled ? QString() : QStringLiteral("Disabled in motor_config.h (HmiDefaults)"));
    }
}

// The one place the armed source changes. Whatever was live is stopped first —
// switching straight from one input to another must not leave the old one
// latched at its last command.
void SendCommandModule::setInputSource(int src) {
    if (!key_jog_filter_) return;
    input_source_ = src;

    // Only the keyboard grabs keys. Disarming releases → keysChanged → stop.
    key_jog_filter_->setArmed(src == SRC_KEYBOARD);
    // Unconditional: a pad stick does not un-deflect just because it stopped
    // being the armed source, so the outgoing source has to be zeroed here.
    publishJogStop();

    if (jog_timer_) {
        if (src != SRC_OFF && (keyJogIkMode() || src == SRC_PS4)) {
            jog_timer_->start();
        } else {
            jog_timer_->stop();
            if (twist_pub_) publishKeyJogTwist();   // one zero twist on the way out
        }
    }

    if (src == SRC_PS4) {
        // Re-arming after a dropout should not inherit the old verdict; the
        // timer re-evaluates on its next tick.
        joy_lost_ = !joyLive();
    } else {
        joy_lost_ = false;
    }

    styleSourceButtons();
    restyleKeyJogChips();
}

void SendCommandModule::styleSourceButtons() {
    static const char* const LIT_BY_SRC[3] = { theme::Text, theme::Green, theme::Cyan };
    for (int s = 0; s < 3; s++) {
        if (!src_btns_[s]) continue;
        const bool on = (input_source_ == s);
        const char* lit = LIT_BY_SRC[s];
        src_btns_[s]->setStyleSheet(
            on ? QString("QPushButton { background: %1; color: %2; border: 2px solid %1; "
                         "padding: 10px 18px; font-weight: bold; }")
                     .arg(lit).arg(theme::Bg)
               : QString("QPushButton { background: %1; color: %2; border: 2px solid %3; "
                         "padding: 10px 18px; font-weight: bold; }")
                     .arg(theme::Bg).arg(theme::TextDim).arg(theme::BorderDim));
    }
    if (!key_jog_status_) return;
    const char* text = "disarmed";
    const char* color = theme::TextDim;
    if (input_source_ == SRC_KEYBOARD) { text = "keys → arm";  color = theme::Green; }
    else if (input_source_ == SRC_PS4) {
        if (joy_lost_) { text = joy_seen_ ? "PS4 LOST — /joy silent" : "waiting for /joy";
                         color = theme::Red; }
        else           { text = "PS4 → arm"; color = theme::Cyan; }
    }
    key_jog_status_->setText(text);
    key_jog_status_->setStyleSheet(QString("color: %1;").arg(color));
}

// One message, every mapped axis, whichever source is armed. Idle axes are
// commanded to 0 rather than left NaN, so releasing one key of a multi-key hold
// (or centring one stick of two) stops just that axis while the others keep
// going.
//
// Once everything is idle we publish a final all-zero message and then go quiet
// — the driver re-sends its active command every poll tick to feed the moteus
// watchdog, so the keyboard never has to stream. The pad does stream, but only
// because analog sticks change value, not to keep the arm alive.
void SendCommandModule::publishKeyJog(bool streaming) {
    if (!cmd_pub_ || !key_jog_filter_ || !key_jog_scale_) return;

    const auto dirs = activeDirections();
    const double scale = key_jog_scale_->value();

    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ABS_VEL;
    msg.cmd_value = CMD_SPACE_AXIS;   // wrist stays differential; never motor space
    msg.velocities.assign(NUM_MOTORS, NAN);

    // In IK mode the arm joints belong to Servo — this path only carries the
    // end effector, which is outside the planning group. Touching the other
    // axes here would fight Servo's own joint velocities.
    const bool ik = keyJogIkMode();

    QStringList active;
    bool moving = false;
    for (int a = 0; a < NUM_AXES && a < NUM_MOTORS; a++) {
        if (!keyJogAxisEnabled(a)) continue;
        if (ik && a != AXIS_EE_INDEX) continue;
        const double rev_s = dirs[a] * HmiDefaults::axis_velocity_revps[a] * scale;
        msg.velocities[a] = rev_s * 360.0;   // wire contract is deg/s, UI is rev/s
        if (rev_s != 0.0) {
            moving = true;
            active << QString("A%1 %2%3").arg(a + 1)
                          .arg(rev_s > 0 ? "+" : "").arg(rev_s, 0, 'f', 3);
        }
    }

    // Nothing held and nothing was moving → don't spam a stop on every stray
    // keystroke (Esc, an unbound key, re-arming) or every idle pad tick.
    if (!moving && !key_jog_moving_) return;
    const bool was_moving = key_jog_moving_;
    key_jog_moving_ = moving;

    cmd_pub_->publish(msg);
    // The streaming path fires 33x/s, so log its transitions only or the
    // command log drowns. The change-driven keyboard path logs in full.
    const char* tag = (input_source_ == SRC_PS4) ? "ps4jog" : "keyjog";
    if (!streaming || moving != was_moving) {
        logCmd(moving ? QString("%1> %2").arg(tag).arg(active.join(", "))
                      : QString("%1> release (all axes 0)").arg(tag));
    }
}

// Command every enabled axis to 0 regardless of mode or source. publishKeyJog()
// deliberately leaves the arm joints NaN in IK mode (they belong to Servo) —
// but NaN means "skip", so a joint that was jogging in FK stays latched in the
// driver, which re-sends it every poll tick. Anything that ends a jog outright
// has to go through here instead.
void SendCommandModule::publishJogStop() {
    if (!cmd_pub_) return;
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = CMD_ABS_VEL;
    msg.cmd_value = CMD_SPACE_AXIS;
    msg.velocities.assign(NUM_MOTORS, NAN);
    for (int a = 0; a < NUM_AXES && a < NUM_MOTORS; a++)
        if (keyJogAxisEnabled(a)) msg.velocities[a] = 0.0;
    cmd_pub_->publish(msg);
    key_jog_moving_ = false;
}

// -1..+1 per firmware axis, from whichever source is armed. Axes disabled in
// HmiDefaults contribute nothing, so a stick pushed toward A4 does nothing for
// the same reason its keys are greyed out.
std::array<double, NUM_AXES> SendCommandModule::activeDirections() const {
    std::array<double, NUM_AXES> dirs{};
    dirs.fill(0.0);
    if (input_source_ == SRC_KEYBOARD && key_jog_filter_) return key_jog_filter_->directions();
    if (input_source_ != SRC_PS4 || !joyLive()) return dirs;

    for (const auto& bind : PS4_FK_BINDINGS) {
        if (!keyJogAxisEnabled(bind.axis)) continue;
        dirs[bind.axis] = std::clamp(joyAxis(joy_, bind.joy_axis), -1.0, 1.0);
    }
    // EE is a bumper pair, not a stick: L1 opens, R1 closes.
    if (keyJogAxisEnabled(AXIS_EE_INDEX)) {
        dirs[AXIS_EE_INDEX] = joyButton(joy_, ps4_index::buttons::L1)
                            - joyButton(joy_, ps4_index::buttons::R1);
    }
    return dirs;
}

// The Cartesian twist, same deal.
std::array<double, 6> SendCommandModule::activeTwist() const {
    std::array<double, 6> t{};
    t.fill(0.0);
    if (input_source_ == SRC_KEYBOARD && key_jog_filter_) {
        for (const auto& bind : KEY_JOG_BINDINGS) {
            if (bind.twist < 0 || bind.twist >= 6) continue;
            if (key_jog_filter_->isHeld(bind.key)) t[bind.twist] += bind.dir;
        }
        return t;
    }
    if (input_source_ != SRC_PS4 || !joyLive()) return t;

    for (const auto& bind : PS4_IK_BINDINGS)
        t[bind.twist] = joyAxis(joy_, bind.joy_axis);
    // Yaw is the trigger pair. The difference cancels any rest value the two
    // share — but joydev reports a trigger as 0.0 until it is first pulled, so
    // pull both once after connecting or the untouched one biases the yaw.
    t[IK_ANG_Z_INDEX] = (joyAxis(joy_, ps4_index::axes::L2)
                       - joyAxis(joy_, ps4_index::axes::R2)) / 2.0;
    return t;
}

// The pad counts as live only while /joy is actually arriving.
bool SendCommandModule::joyLive() const {
    if (!joy_seen_ || !joy_clock_.isValid()) return false;
    return (joy_clock_.elapsed() - joy_last_ms_) <= JOY_TIMEOUT_MS;
}

// joy_linux owns the device; this just caches the latest state. Runs on the Qt
// thread (the host pumps spin_some from a QTimer), so widgets are safe to touch.
void SendCommandModule::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (!msg) return;
    joy_ = *msg;
    if (!joy_clock_.isValid()) joy_clock_.start();
    joy_last_ms_ = joy_clock_.elapsed();
    joy_seen_ = true;
    if (!key_jog_status_) return;   // setNode() runs before createWidget()

    // SHARE homes, edge-triggered. Homing is a driver-side state machine that
    // jog commands would fight, so drop to OFF first — that publishes the stop
    // and stops the jog timer, which also keeps the modal box below from being
    // re-entered when it pumps the event loop (and with it, spin_some).
    const int home_btn = joyButton(joy_, ps4_index::buttons::SHARE);
    const bool home_edge = home_btn && !prev_joy_home_ && input_source_ == SRC_PS4;
    prev_joy_home_ = home_btn;

    if (home_edge && !home_prompt_open_) {
        setInputSource(SRC_OFF);
        home_prompt_open_ = true;
        homeChecked();               // same confirm dialog as the panel button
        home_prompt_open_ = false;
        return;
    }

    // Motion itself goes out on the jog timer, not per message — /joy arrives
    // at 100 Hz. Only the readout follows the sticks directly.
    if (input_source_ == SRC_PS4) {
        if (joy_lost_) { joy_lost_ = false; styleSourceButtons(); }
        restyleKeyJogChips();
    }
}

// Cartesian mode: publish the armed source as a twist for MoveIt Servo. Servo's
// command_in_type is "unitless" ([-1:1] scaled by its own linear/rotational
// scale params), so the speed spin box is clamped into that range rather than
// treated as rev/s.
//
// Unlike the joint path this runs on a timer while armed, including when
// nothing is held: a steady zero twist keeps Servo alive without motion, and
// stopping the stream entirely is what triggers its halt behaviour.
void SendCommandModule::publishKeyJogTwist() {
    if (!twist_pub_ || !key_jog_filter_ || !key_jog_scale_) return;

    std::array<double, 6> t = activeTwist();
    const double scale = std::clamp(key_jog_scale_->value(), 0.0, 1.0);
    for (double& v : t) v = std::clamp(v, -1.0, 1.0) * scale;

    geometry_msgs::msg::TwistStamped msg;
    msg.header.frame_id = ik_frame_ ? ik_frame_->currentText().toStdString() : "base_link";
    if (node_) msg.header.stamp = node_->now();
    msg.twist.linear.x  = t[IK_LIN_X_INDEX];
    msg.twist.linear.y  = t[IK_LIN_Y_INDEX];
    msg.twist.linear.z  = t[IK_LIN_Z_INDEX];
    msg.twist.angular.x = t[IK_ANG_X_INDEX];
    msg.twist.angular.y = t[IK_ANG_Y_INDEX];
    msg.twist.angular.z = t[IK_ANG_Z_INDEX];
    twist_pub_->publish(msg);

    // Log only on transitions — this fires 33x/s and would drown the log.
    const bool moving = std::any_of(t.begin(), t.end(), [](double v) { return v != 0.0; });
    if (moving != ik_twist_moving_) {
        ik_twist_moving_ = moving;
        logCmd(moving ? QString("ik> twist %1 [%2 %3 %4 | %5 %6 %7]")
                            .arg(QString::fromStdString(msg.header.frame_id))
                            .arg(t[0], 0, 'f', 2).arg(t[1], 0, 'f', 2).arg(t[2], 0, 'f', 2)
                            .arg(t[3], 0, 'f', 2).arg(t[4], 0, 'f', 2).arg(t[5], 0, 'f', 2)
                      : QStringLiteral("ik> twist 0"));
    }
}

int SendCommandModule::targetId() const {
    const int d = motor_select_ ? motor_select_->currentData().toInt() : 0;
    return d > 100 ? d - 100 : d;
}

bool SendCommandModule::targetMotorSpace() const {
    return motor_select_ && motor_select_->currentData().toInt() > 100;
}

void SendCommandModule::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    cmd_pub_ = node->create_publisher<rover_msgs::msg::ArmCommand>("/arm/command", qos);
    log_pub_ = node->create_publisher<std_msgs::msg::String>("/arm/hmi_log", qos);
    // Servo's own subscriber is plain KeepLast(10); a stream, not latched state.
    twist_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(
        SERVO_TWIST_TOPIC, rclcpp::QoS(10));

    // joy_linux publishes here as soon as the pad is plugged in — nothing to
    // detect or open on this side. Depth 1: only the newest stick position
    // matters, a backlog of stale ones does not.
    joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::QoS(1),
        std::bind(&SendCommandModule::joyCallback, this, std::placeholders::_1));
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
