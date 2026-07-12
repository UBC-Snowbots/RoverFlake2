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
#include <rover_arm_common/motor_config.h>           // get_arm_configuration(), MotorConfig
#include <rover_hmi_core/catppuccin.h>

#include <QApplication>
#include <QGridLayout>
#include <QScrollArea>
#include <QFont>
#include <QSettings>
#include <cmath>
#include <algorithm>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

// ---------------------------------------------------------------------------
// TableScaleFilter — updates all cell fonts proportionally on resize.
// ---------------------------------------------------------------------------
class TableScaleFilter : public QObject {
public:
    TableScaleFilter(QObject* parent, int num_rows,
                     std::vector<QWidget*> normal_ws,
                     std::vector<QWidget*> bold_ws)
        : QObject(parent), num_rows_(num_rows),
          normal_ws_(std::move(normal_ws)), bold_ws_(std::move(bold_ws)) {}

    bool eventFilter(QObject* obj, QEvent* ev) override {
        if (ev->type() == QEvent::Resize)
            updateFonts(static_cast<QWidget*>(obj)->height());
        return QObject::eventFilter(obj, ev);
    }

    void updateFonts(int height) {
        if (num_rows_ == 0) return;
        int row_h = height / num_rows_;
        int fs = std::max(8, std::min(24, row_h * 38 / 100));
        QFont fn("monospace", fs);
        QFont fb("monospace", fs, QFont::Bold);
        for (auto* w : normal_ws_) w->setFont(fn);
        for (auto* w : bold_ws_)   w->setFont(fb);
    }

private:
    int num_rows_;
    std::vector<QWidget*> normal_ws_;
    std::vector<QWidget*> bold_ws_;
};

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
    // Cells with a user QSettings override: cyan text on black — no blue background
    return QString(
        "QLineEdit { background: %1; color: %2;"
        "  border: 1px solid %3; padding: 5px 8px; }")
        .arg(theme::Bg).arg(theme::Cyan).arg(theme::BorderDim);
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
    scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setStyleSheet(
        QString("QScrollArea { background: %1; }"
                "QScrollArea > QWidget > QWidget { background: %1; }")
        .arg(theme::Bg));

    auto* widget = new QWidget();
    auto* grid = new QGridLayout(widget);
    grid->setSpacing(1);
    grid->setContentsMargins(2, 2, 2, 2);

    QFont mono("monospace", theme::FontSize);
    QFont monoBold("monospace", theme::FontSize, QFont::Bold);

    auto headerStyle = QString(
        "background: %1; color: %2; padding: 4px 6px;"
        " border-bottom: 1px solid %3;")
        .arg(theme::Bg).arg(theme::Text).arg(theme::BorderDim);

    // Widget lists for font scaling
    std::vector<QWidget*> bold_ws, normal_ws;

    // ── Column headers ────────────────────────────────────────────────────────
    auto* corner = new QLabel("Joint");
    corner->setFont(monoBold);
    corner->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    corner->setStyleSheet(headerStyle);
    corner->setAlignment(Qt::AlignCenter);
    grid->addWidget(corner, 0, 0);
    bold_ws.push_back(corner);

    for (int c = 0; c < NUM_COLS; c++) {
        bool is_ro = (REGISTERS[c] == nullptr);
        auto* lbl = new QLabel(FIELD_HEADERS[c]);
        lbl->setFont(monoBold);
        lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        lbl->setStyleSheet(headerStyle
            + (is_ro ? " color: " + QString(theme::TextDim) + ";" : ""));
        lbl->setAlignment(Qt::AlignCenter);
        grid->addWidget(lbl, 0, c + 1);
        bold_ws.push_back(lbl);
    }

    // No header labels for Reset / Calibrate columns — the buttons are self-explanatory.

    // ── Motor rows ────────────────────────────────────────────────────────────
    QSettings settings("RoverTeam", "RoverHMI");
    auto arm_defaults = get_arm_configuration();

    for (int r = 0; r < NUM_MOTORS; r++) {
        auto* jlbl = new QLabel(JOINT_NAMES[r]);
        jlbl->setFont(monoBold);
        jlbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        jlbl->setStyleSheet(
            QString("background: %1; color: %2; padding: 4px 6px;")
            .arg(theme::Bg).arg(theme::MotorColors[r]));
        jlbl->setAlignment(Qt::AlignCenter);
        grid->addWidget(jlbl, r + 1, 0);
        bold_ws.push_back(jlbl);

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
            // QLineEdit defaults to Fixed vertical — override so rows expand with the panel.
            edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

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

                // Focus visual feedback.
                // Guard old_w != nullptr: Qt auto-focuses the first widget on startup
                // with old_w=nullptr — we don't want that to trigger activeStyle.
                QObject::connect(
                    qApp, &QApplication::focusChanged,
                    edit, [edit](QWidget* old_w, QWidget* new_w) {
                        if (new_w == edit && old_w != nullptr)
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
            normal_ws.push_back(edit);
        }

        // ↺ Reset button
        auto* rst_btn = new QPushButton("↺");
        rst_btn->setFont(monoBold);
        rst_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        rst_btn->setToolTip(QString("Reset %1 to motor_config.h defaults").arg(JOINT_NAMES[r]));
        rst_btn->setStyleSheet(
            QString("QPushButton { background: %1; color: %2;"
                    "  border: 1px solid %3; border-radius: 4px; padding: 5px 10px; }"
                    "QPushButton:hover   { background: #1a1a1a; border-color: %2; }"
                    "QPushButton:pressed { background: #222222; }")
            .arg(theme::Bg).arg(theme::Yellow).arg(theme::BorderDim));
        grid->addWidget(rst_btn, r + 1, NUM_COLS + 1);
        reset_btns_[r] = rst_btn;
        bold_ws.push_back(rst_btn);

        {
            int motor_row = r;
            QObject::connect(rst_btn, &QPushButton::clicked,
                [this, motor_row]() { resetMotorToDefaults(motor_row); });
        }

        // Calibrate button (CAN ID = r+1)
        auto* cal_btn = new QPushButton("Calibrate");
        cal_btn->setFont(monoBold);
        cal_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        cal_btn->setToolTip(
            QString("Run Hall calibration for %1\n"
                    "  python3 -m moteus.moteus_tool -t %2\n"
                    "    --calibrate --cal-motor-poles 16 --cal-force-kv 265 --cal-hal\n"
                    "then sets motor_position.sources.0.type = type.hall:4")
            .arg(JOINT_NAMES[r]).arg(r + 1));
        cal_btn->setStyleSheet(
            QString("QPushButton { background: %1; color: %2;"
                    "  border: 1px solid %3; border-radius: 4px; padding: 5px 10px; }"
                    "QPushButton:hover   { background: #1a1a1a; border-color: %2; }"
                    "QPushButton:pressed { background: #222222; }"
                    "QPushButton:disabled { color: %4; border-color: %4; }")
            .arg(theme::Bg).arg(theme::Yellow).arg(theme::BorderDim).arg(theme::TextDim));
        grid->addWidget(cal_btn, r + 1, NUM_COLS + 2);
        calib_btns_[r] = cal_btn;
        bold_ws.push_back(cal_btn);

        {
            int motor_row = r;
            QObject::connect(cal_btn, &QPushButton::clicked,
                [this, motor_row]() { requestCalibration(motor_row + 1); });
        }
    }

    // Calibrate All button (spans the last column, bottom row)
    calib_all_btn_ = new QPushButton("Calibrate All (Sequential)");
    calib_all_btn_->setFont(monoBold);
    calib_all_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    bold_ws.push_back(calib_all_btn_);
    calib_all_btn_->setToolTip(
        "Calibrate all 6 motors one by one (motor 1 → 6).\n"
        "Each takes ~30 s.  The arm driver will pause during calibration.");
    calib_all_btn_->setStyleSheet(
        QString("QPushButton { background: #1a1400; color: %1;"
                "  border: 1px solid %1; border-radius: 4px; padding: 6px 14px; }"
                "QPushButton:hover   { background: #2a2000; }"
                "QPushButton:pressed { background: #333000; }"
                "QPushButton:disabled { color: %2; border-color: %2; }")
        .arg(theme::Yellow).arg(theme::TextDim));
    grid->addWidget(calib_all_btn_, NUM_MOTORS + 1, NUM_COLS + 2);

    QObject::connect(calib_all_btn_, &QPushButton::clicked,
        [this]() { requestCalibration(0); });

    // Status bar
    status_ = new QLabel(
        "Click any cell to edit  ·  Enter to apply + save  ·  ↺ to reset row to defaults"
        "  ·  cyan = user override active");
    status_->setFont(mono);
    status_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    status_->setStyleSheet(
        QString("background: %1; color: %2; padding: 4px 6px;")
        .arg(theme::Bg).arg(theme::TextDim));
    grid->addWidget(status_, NUM_MOTORS + 1, 0, 1, NUM_COLS + 2);
    normal_ws.push_back(status_);

    // All columns equal stretch
    for (int c = 0; c <= NUM_COLS + 2; c++)
        grid->setColumnStretch(c, 1);

    // All rows equal stretch so every row grows proportionally with the panel
    for (int r = 0; r <= NUM_MOTORS + 1; r++)
        grid->setRowStretch(r, 1);

    // Font scaler: recalculates font size on every resize
    // NUM_MOTORS + 2 = header row + motor rows + status row
    auto* scaler = new TableScaleFilter(
        scroll, NUM_MOTORS + 2, std::move(normal_ws), std::move(bold_ws));
    scroll->installEventFilter(scaler);

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

    calib_pub_ = node->create_publisher<rover_msgs::msg::MoteusCalibrationRequest>(
        "/arm/calibration_request", qos);

    calib_status_sub_ = node->create_subscription<rover_msgs::msg::MoteusCalibrationStatus>(
        "/arm/calibration_status", qos,
        [this](const rover_msgs::msg::MoteusCalibrationStatus::SharedPtr msg) {
            onCalibStatus(msg);
        });
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

// ---------------------------------------------------------------------------
// requestCalibration  — publish to /arm/calibration_request
// motor_id: 1-based CAN ID; 0 = all sequential
// ---------------------------------------------------------------------------

void MotorConfigModule::requestCalibration(int motor_id) {
    if (!calib_pub_) return;

    rover_msgs::msg::MoteusCalibrationRequest msg;
    msg.motor_id = motor_id;
    calib_pub_->publish(msg);

    if (motor_id == 0) {
        // Disable all buttons while sequential calibration runs
        for (int r = 0; r < NUM_MOTORS; r++) {
            if (calib_btns_[r]) calib_btns_[r]->setEnabled(false);
        }
        if (calib_all_btn_) {
            calib_all_btn_->setText("Calibrating All...");
            calib_all_btn_->setEnabled(false);
        }
        if (status_)
            status_->setText("Calibrating all motors sequentially (motor 1 → 6)...  ~3 min total");
    } else {
        int r = motor_id - 1;
        if (r >= 0 && r < NUM_MOTORS && calib_btns_[r]) {
            calib_btns_[r]->setText("Running...");
            calib_btns_[r]->setEnabled(false);
        }
        if (status_)
            status_->setText(
                QString("Calibrating motor %1 (%2)...  ~30 s")
                .arg(motor_id).arg(JOINT_NAMES[motor_id - 1]));
    }
}

// ---------------------------------------------------------------------------
// onCalibStatus  — update button state from driver feedback
// ---------------------------------------------------------------------------

void MotorConfigModule::onCalibStatus(
        const rover_msgs::msg::MoteusCalibrationStatus::SharedPtr msg)
{
    int r = msg->motor_id - 1;  // 0-based index

    auto setBtn = [&](QPushButton* btn, const QString& text,
                      const char* fg, bool enabled) {
        if (!btn) return;
        btn->setText(text);
        btn->setEnabled(enabled);
        btn->setStyleSheet(
            QString("QPushButton { background: %1; color: %2;"
                    "  border: 1px solid %3; border-radius: 4px; padding: 5px 10px; }"
                    "QPushButton:hover   { background: #1a1a1a; border-color: %3; }"
                    "QPushButton:pressed { background: #222222; }"
                    "QPushButton:disabled { color: %4; border-color: %4; }")
            .arg(theme::Bg).arg(fg).arg(fg).arg(theme::TextDim));
    };

    using S = rover_msgs::msg::MoteusCalibrationStatus;

    if (r >= 0 && r < NUM_MOTORS) {
        switch (msg->state) {
            case S::CALIB_RUNNING:
                setBtn(calib_btns_[r], "Running...", theme::Yellow, false);
                break;
            case S::CALIB_SUCCESS:
                setBtn(calib_btns_[r], "Done ✓", theme::Green, true);
                break;
            case S::CALIB_FAILED:
                setBtn(calib_btns_[r], "FAILED", theme::Red, true);
                break;
            case S::CALIB_IDLE:
            default:
                setBtn(calib_btns_[r], "Calibrate", theme::Yellow, true);
                break;
        }
    }

    // Re-enable "Calibrate All" when all motors are done (state != running)
    bool any_running = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (calib_btns_[i] && !calib_btns_[i]->isEnabled())
            any_running = true;
    }
    if (!any_running && calib_all_btn_) {
        calib_all_btn_->setText("Calibrate All (Sequential)");
        calib_all_btn_->setEnabled(true);
    }

    if (status_) {
        const char* state_str =
            (msg->state == S::CALIB_RUNNING) ? "RUNNING" :
            (msg->state == S::CALIB_SUCCESS) ? "DONE" :
            (msg->state == S::CALIB_FAILED)  ? "FAILED" : "idle";
        status_->setText(
            QString("[Calib motor %1]  %2  —  %3")
            .arg(msg->motor_id)
            .arg(state_str)
            .arg(QString::fromStdString(msg->message)));
    }
}

PLUGINLIB_EXPORT_CLASS(MotorConfigModule, rover_hmi_core::GuiModule)
