// motor_config_module.cpp  —  "Motor Params"
//
// Edit flow:
//   Click cell → cyan border.  Type value.  Enter (or blur) →
//     1. Published to /arm/config_update  (driver applies to RAM immediately)
//     2. Saved to QSettings               (survives HMI and driver restarts)
//     3. Status bar shows what was sent.
//
// Load flow (start()):
//   For every (motor, register) that has been saved, publish a
//   MoteusConfigUpdate.  This re-applies user customisations on top of the
//   driver's motor_config.h defaults each time the HMI starts.
//
// Reset flow (↺ button):
//   Clears all QSettings for that motor and publishes motor_config.h defaults
//   for every register.  Does NOT save the defaults (so the slot stays clean).

#include "motor_config_module.h"
#include "motor_config.h"           // get_arm_configuration(), MotorConfig
#include <rover_hmi_core/catppuccin.h>

#include <QApplication>
#include <QGridLayout>
#include <QScrollArea>
#include <QFont>
#include <QSettings>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

// ---------------------------------------------------------------------------
// Column definitions
// ---------------------------------------------------------------------------

static const char* FIELD_HEADERS[] = {
    "Kp", "Ki", "Kd",
    "Max I (A)", "Max Vel (rev/s)", "Max Accel (rev/s²)",
    "Pos Min (rev)", "Pos Max (rev)",
    "Max V (V)", "Max Power (W)", "Timeout (s)",
    "Gear Ratio",
};

// Moteus register for each column; nullptr = read-only (Gear).
static const char* REGISTERS[] = {
    "servo.pid_position.kp",        // 0
    "servo.pid_position.ki",        // 1
    "servo.pid_position.kd",        // 2
    "servo.max_current_A",          // 3
    "servo.max_velocity",           // 4  (driver mirrors → default_velocity_limit)
    "servo.default_accel_limit",    // 5
    "servopos.position_min",        // 6
    "servopos.position_max",        // 7
    "servo.max_voltage",            // 8
    "servo.max_power_W",            // 9
    "servo.default_timeout_s",      // 10
    nullptr,                        // 11  Gear — display only
};
static constexpr int NUM_COLS = 12;

static const int PRECISION[] = { 0, 3, 0, 1, 4, 3, 3, 3, 1, 1, 3, 0 };

static const char* JOINT_NAMES[] = {
    "Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "End Effector"
};

// QSettings group/key helpers
static QString settingsKey(int motor_idx, const char* reg) {
    return QString("motor_params/motor_%1/%2").arg(motor_idx).arg(reg);
}

// ---------------------------------------------------------------------------
// Styles
// ---------------------------------------------------------------------------

static QString roStyle(const char* bg, const char* fg) {
    return QString(
        "QLineEdit { background: %1; color: %2;"
        "  border: 1px solid transparent; padding: 5px 8px; }")
        .arg(bg).arg(fg);
}

static QString activeStyle() {
    return QString(
        "QLineEdit { background: #1a2a3a; color: %1;"
        "  border: 1px solid %2; padding: 5px 8px; }")
        .arg(theme::Text).arg(theme::Cyan);
}

static QString savedStyle() {
    // Cells that have a user override stored in QSettings get a subtle cyan tint
    return QString(
        "QLineEdit { background: #0d1f2d; color: %1;"
        "  border: 1px solid #1a3a4a; padding: 5px 8px; }")
        .arg(theme::Cyan);
}

static QString gearStyle() {
    return QString(
        "QLineEdit { background: %1; color: %2;"
        "  border: 1px solid transparent; padding: 5px 8px; }")
        .arg(theme::BgPanel).arg(theme::TextDim);
}

// ---------------------------------------------------------------------------
// createWidget
// ---------------------------------------------------------------------------

QWidget* MotorConfigModule::createWidget(QWidget* parent) {
    auto* scroll = new QScrollArea(parent);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll->setStyleSheet(
        QString("QScrollArea { background: %1; }").arg(theme::Bg));

    auto* widget = new QWidget();
    auto* grid = new QGridLayout(widget);
    grid->setSpacing(2);
    grid->setContentsMargins(4, 4, 4, 4);

    QFont mono("monospace", theme::FontSize);
    QFont monoBold("monospace", theme::FontSize, QFont::Bold);

    auto headerStyle = QString(
        "background: %1; color: %2; padding: 6px 10px;")
        .arg(theme::HeaderBg).arg(theme::Text);

    // ── Column headers ────────────────────────────────────────────────────────
    auto* corner = new QLabel("Joint");
    corner->setFont(monoBold);
    corner->setStyleSheet(headerStyle);
    corner->setAlignment(Qt::AlignCenter);
    grid->addWidget(corner, 0, 0);

    for (int c = 0; c < NUM_COLS; c++) {
        bool is_ro = (REGISTERS[c] == nullptr);
        auto* lbl = new QLabel(FIELD_HEADERS[c]);
        lbl->setFont(monoBold);
        lbl->setStyleSheet(headerStyle
            + (is_ro ? " color: " + QString(theme::TextDim) + ";" : ""));
        lbl->setAlignment(Qt::AlignCenter);
        grid->addWidget(lbl, 0, c + 1);
    }

    // Reset column header
    auto* rst_hdr = new QLabel("Reset");
    rst_hdr->setFont(monoBold);
    rst_hdr->setStyleSheet(headerStyle + " color: " + QString(theme::TextDim) + ";");
    rst_hdr->setAlignment(Qt::AlignCenter);
    grid->addWidget(rst_hdr, 0, NUM_COLS + 1);

    // ── Motor rows ────────────────────────────────────────────────────────────
    QSettings settings("RoverTeam", "RoverHMI");
    auto arm_defaults = get_arm_configuration();

    for (int r = 0; r < NUM_MOTORS; r++) {
        auto* jlbl = new QLabel(JOINT_NAMES[r]);
        jlbl->setFont(monoBold);
        jlbl->setStyleSheet(
            QString("background: %1; color: %2; padding: 6px 10px;")
            .arg(theme::Bg).arg(theme::MotorColors[r]));
        jlbl->setAlignment(Qt::AlignCenter);
        grid->addWidget(jlbl, r + 1, 0);

        // Per-motor defaults from motor_config.h
        const auto& def = arm_defaults[r];
        float motor_defaults[NUM_COLS] = {
            def.kp, def.ki, def.kd,
            def.max_current_A,
            def.max_velocity,
            def.max_acceleration,
            def.position_min,
            def.position_max,
            def.max_voltage,
            def.max_power_W,
            def.def_timeout,
            def.gear_reduction,  // col 11 — gear, formatted separately
        };

        for (int c = 0; c < NUM_COLS; c++) {
            auto* edit = new QLineEdit();
            edit->setFont(mono);
            edit->setAlignment(Qt::AlignCenter);
            edit->setMinimumWidth(90);

            if (REGISTERS[c] == nullptr) {
                // Gear ratio — read-only, format as 1/N
                edit->setReadOnly(true);
                edit->setStyleSheet(gearStyle());
                edit->setText(def.gear_reduction > 0.0f
                    ? QString("1/%1").arg(qRound(1.0f / def.gear_reduction))
                    : "--");
            } else {
                // Populate from QSettings override if present, otherwise use default
                bool has_override = settings.contains(settingsKey(r, REGISTERS[c]));
                float display_val = has_override
                    ? settings.value(settingsKey(r, REGISTERS[c])).toFloat()
                    : motor_defaults[c];

                edit->setText(std::isnan(display_val) ? "nan"
                    : QString::number(display_val, 'f', PRECISION[c]));
                edit->setStyleSheet(has_override
                    ? savedStyle()
                    : roStyle(theme::Bg, theme::Text));

                // Focus visual feedback
                QObject::connect(
                    qApp, &QApplication::focusChanged,
                    edit, [edit](QWidget* old_w, QWidget* new_w) {
                        if (new_w == edit)
                            edit->setStyleSheet(activeStyle());
                        else if (old_w == edit)
                            edit->setStyleSheet(roStyle(theme::Bg, theme::Text));
                    });

                int motor_row = r;
                int col       = c;
                QObject::connect(edit, &QLineEdit::editingFinished,
                    [this, motor_row, col, edit]() {
                        bool ok;
                        float val = edit->text().toFloat(&ok);
                        if (!ok || std::isnan(val)) {
                            edit->setStyleSheet(roStyle(theme::Bg, theme::Text));
                            return;
                        }
                        publishUpdate(motor_row, col, val);
                        // savedStyle is applied by publishUpdate
                    });
            }

            edits_[r][c] = edit;
            grid->addWidget(edit, r + 1, c + 1);
        }

        // ↺ Reset button
        auto* rst_btn = new QPushButton("↺");
        rst_btn->setFont(monoBold);
        rst_btn->setToolTip(QString("Reset %1 to motor_config.h defaults").arg(JOINT_NAMES[r]));
        rst_btn->setStyleSheet(
            QString("QPushButton { background: %1; color: %2;"
                    "  border: 1px solid %3; border-radius: 4px; padding: 5px 10px; }"
                    "QPushButton:hover   { background: #1a1a1a; border-color: %2; }"
                    "QPushButton:pressed { background: #222222; }")
            .arg(theme::Bg).arg(theme::Yellow).arg(theme::BorderDim));
        grid->addWidget(rst_btn, r + 1, NUM_COLS + 1);
        reset_btns_[r] = rst_btn;

        int motor_row = r;
        QObject::connect(rst_btn, &QPushButton::clicked,
            [this, motor_row]() { resetMotorToDefaults(motor_row); });
    }

    // Status bar
    status_ = new QLabel(
        "Click any cell to edit  ·  Enter to apply + save  ·  ↺ to reset row to defaults"
        "  ·  cyan tint = user override active");
    status_->setFont(mono);
    status_->setStyleSheet(
        QString("background: %1; color: %2; padding: 6px 10px;")
        .arg(theme::Bg).arg(theme::TextDim));
    grid->addWidget(status_, NUM_MOTORS + 1, 0, 1, NUM_COLS + 2);

    for (int c = 0; c <= NUM_COLS + 1; c++)
        grid->setColumnStretch(c, 1);

    scroll->setWidget(widget);
    return scroll;
}

// ---------------------------------------------------------------------------
// setNode
// ---------------------------------------------------------------------------

void MotorConfigModule::setNode(rclcpp::Node::SharedPtr node) {
    auto qos = rclcpp::QoS(1).reliable().durability_volatile();

    sub_ = node->create_subscription<rover_msgs::msg::MoteusArmStatus>(
        "/arm/moteus_feedback", qos,
        [this](const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
            onFeedback(msg);
        });

    pub_ = node->create_publisher<rover_msgs::msg::MoteusConfigUpdate>(
        "/arm/config_update", qos);
}

// ---------------------------------------------------------------------------
// start()  — re-apply saved overrides after the window is shown
// ---------------------------------------------------------------------------

void MotorConfigModule::start() {
    if (!pub_) return;

    QSettings settings("RoverTeam", "RoverHMI");
    int n_applied = 0;

    for (int r = 0; r < NUM_MOTORS; r++) {
        for (int c = 0; c < NUM_EDITABLE; c++) {
            if (REGISTERS[c] == nullptr) continue;
            QString key = settingsKey(r, REGISTERS[c]);
            if (!settings.contains(key)) continue;

            float val = settings.value(key).toFloat();
            publishRaw(r, REGISTERS[c], val);
            ++n_applied;
        }
    }

    if (status_ && n_applied > 0) {
        status_->setText(
            QString("Restored %1 saved override%2 from previous session  ·  "
                    "cyan tint = user override active")
            .arg(n_applied).arg(n_applied == 1 ? "" : "s"));
    }
}

// ---------------------------------------------------------------------------
// onFeedback  — update cells not currently being edited
// ---------------------------------------------------------------------------

void MotorConfigModule::onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
    if (!edits_[0][0]) return;
    if ((int)msg->config.size() < NUM_MOTORS) return;

    QSettings settings("RoverTeam", "RoverHMI");

    for (int r = 0; r < NUM_MOTORS; r++) {
        if (r >= (int)msg->config.size()) break;
        const auto& c = msg->config[r];

        float vals[NUM_COLS] = {
            c.kp, c.ki, c.kd,
            c.max_current_amps,
            c.max_velocity,
            c.max_acceleration,
            c.min_position,
            c.max_position,
            c.max_voltage_volts,
            c.max_power_watts,
            c.cmd_timeout_s,
            c.gear_reduction,
        };

        for (int col = 0; col < NUM_COLS; col++) {
            auto* edit = edits_[r][col];
            if (!edit || edit->hasFocus()) continue;

            if (col == 11) {
                edit->setText(c.gear_reduction > 0.0f
                    ? QString("1/%1").arg(qRound(1.0f / c.gear_reduction))
                    : "--");
            } else {
                float v = vals[col];
                edit->setText(std::isnan(v) ? "nan"
                    : QString::number(v, 'f', PRECISION[col]));

                // Keep the saved-override tint correct
                if (REGISTERS[col]) {
                    bool saved = settings.contains(settingsKey(r, REGISTERS[col]));
                    if (!edit->hasFocus())
                        edit->setStyleSheet(saved ? savedStyle()
                                                  : roStyle(theme::Bg, theme::Text));
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// publishUpdate  — send update AND save to QSettings
// ---------------------------------------------------------------------------

void MotorConfigModule::publishUpdate(int motor_idx, int col, float value) {
    if (!pub_ || col < 0 || col >= NUM_COLS || REGISTERS[col] == nullptr) return;

    publishRaw(motor_idx, REGISTERS[col], value);

    // Persist
    QSettings settings("RoverTeam", "RoverHMI");
    settings.setValue(settingsKey(motor_idx, REGISTERS[col]), value);

    // Update cell tint
    if (edits_[motor_idx][col])
        edits_[motor_idx][col]->setStyleSheet(savedStyle());

    if (status_) {
        status_->setText(
            QString("Saved + applied:  motor %1  %2 = %3")
            .arg(motor_idx + 1)
            .arg(REGISTERS[col])
            .arg(value, 0, 'f', PRECISION[col]));
    }
}

// ---------------------------------------------------------------------------
// publishRaw  — send update WITHOUT saving to QSettings
// ---------------------------------------------------------------------------

void MotorConfigModule::publishRaw(int motor_idx, const char* reg, float value) {
    if (!pub_) return;
    rover_msgs::msg::MoteusConfigUpdate msg;
    msg.motor_id      = motor_idx;
    msg.register_name = reg;
    msg.value         = value;
    pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// resetMotorToDefaults  — clear saves and re-publish motor_config.h values
// ---------------------------------------------------------------------------

void MotorConfigModule::resetMotorToDefaults(int motor_idx) {
    auto defaults = get_arm_configuration();
    if (motor_idx < 0 || motor_idx >= (int)defaults.size()) return;
    const auto& def = defaults[motor_idx];

    // Clear all saved overrides for this motor
    QSettings settings("RoverTeam", "RoverHMI");
    settings.beginGroup(QString("motor_params/motor_%1").arg(motor_idx));
    settings.remove("");  // removes every key under this group
    settings.endGroup();

    // Publish defaults (not saved — QSettings slot is now empty)
    publishRaw(motor_idx, "servo.pid_position.kp",        def.kp);
    publishRaw(motor_idx, "servo.pid_position.ki",        def.ki);
    publishRaw(motor_idx, "servo.pid_position.kd",        def.kd);
    publishRaw(motor_idx, "servo.max_current_A",          def.max_current_A);
    publishRaw(motor_idx, "servo.max_velocity",           def.max_velocity);
    publishRaw(motor_idx, "servo.default_accel_limit",    def.max_acceleration);
    publishRaw(motor_idx, "servopos.position_min",        def.position_min);
    publishRaw(motor_idx, "servopos.position_max",        def.position_max);
    publishRaw(motor_idx, "servo.max_voltage",            def.max_voltage);
    publishRaw(motor_idx, "servo.max_power_W",            def.max_power_W);
    publishRaw(motor_idx, "servo.default_timeout_s",      def.def_timeout);

    // Revert cell tint to default (no override)
    for (int col = 0; col < NUM_EDITABLE; col++) {
        if (edits_[motor_idx][col] && !edits_[motor_idx][col]->hasFocus())
            edits_[motor_idx][col]->setStyleSheet(roStyle(theme::Bg, theme::Text));
    }

    if (status_) {
        status_->setText(
            QString("Motor %1 (%2) reset to defaults — saved overrides cleared")
            .arg(motor_idx + 1).arg(JOINT_NAMES[motor_idx]));
    }
}

PLUGINLIB_EXPORT_CLASS(MotorConfigModule, rover_hmi_core::GuiModule)
