// motor_config_module.cpp  —  "Motor Params"
//
// Edit flow:
//   Click cell → cyan border.  Type value.  Enter (or blur) →
//     1. Published to /arm/config_update  (driver applies to RAM immediately)
//     2. Written to arm_hardware_interface/config/motor_config.yaml — the
//        repo-tracked file the driver also loads at startup, so the edit
//        survives HMI and driver restarts AND shows up in git diff.
//     3. Status bar shows what was sent (or that the file write failed).
//
// Reset flow (↺ button):
//   Writes the compiled-in fallback defaults for that motor back into
//   motor_config.yaml and publishes them so the driver applies immediately.

#include "motor_config_module.h"
#include "motor_config.h"           // get_arm_configuration(), yaml accessors
#include <rover_hmi_core/catppuccin.h>

#include <QApplication>
#include <QGridLayout>
#include <QScrollArea>
#include <QFont>
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

// motor_config.yaml key for each column (parallel to REGISTERS).
static const char* YAML_KEYS[] = {
    "kp", "ki", "kd",
    "max_current_a", "max_velocity", "max_acceleration",
    "position_min", "position_max",
    "max_voltage", "max_power_w", "timeout_s",
    nullptr,
};

// The per-motor calibration blocks in motor_config.yaml are still identical
// placeholders — running calibration would apply the same (possibly wrong)
// values to every motor. Flip to true once the real per-motor values land.
static constexpr bool CALIBRATION_UI_ENABLED = false;
static const char* CALIBRATION_DISABLED_NOTE =
    "Disabled: per-motor calibration values in motor_config.yaml are still\n"
    "identical placeholders — fill in real values per motor first.";

static const int PRECISION[] = { 0, 3, 0, 1, 4, 3, 3, 3, 1, 1, 3, 0 };

static const char* JOINT_NAMES[] = {
    "Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "End Effector"
};

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
    // Cells edited this session (saved to motor_config.yaml): cyan on black
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
    // Values come from motor_config.yaml (the repo file the driver also loads).
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
                float display_val = motor_defaults[c];
                edit->setText(std::isnan(display_val) ? "nan"
                    : QString::number(display_val, 'f', PRECISION[c]));
                edit->setStyleSheet(roStyle(theme::Bg, theme::Text));

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
        rst_btn->setToolTip(
            QString("Reset %1 to built-in defaults (writes motor_config.yaml)")
            .arg(JOINT_NAMES[r]));
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
        cal_btn->setEnabled(CALIBRATION_UI_ENABLED);
        cal_btn->setToolTip(CALIBRATION_UI_ENABLED
            ? QString("Run Hall calibration for %1 with the calibration values\n"
                      "from motor_config.yaml, then set\n"
                      "motor_position.sources.0.type = type.hall:4")
              .arg(JOINT_NAMES[r])
            : QString(CALIBRATION_DISABLED_NOTE));
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
    calib_all_btn_->setEnabled(CALIBRATION_UI_ENABLED);
    calib_all_btn_->setToolTip(CALIBRATION_UI_ENABLED
        ? "Calibrate all 6 motors one by one (motor 1 → 6).\n"
          "Each takes ~30 s.  The arm driver will pause during calibration."
        : CALIBRATION_DISABLED_NOTE);
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
        "Click any cell to edit  ·  Enter to apply + write motor_config.yaml"
        "  ·  ↺ to reset row to defaults  ·  cyan = edited this session");
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
// start()  — warn if edits won't reach the repository
// ---------------------------------------------------------------------------
// The driver reads motor_config.yaml itself at startup, so there is nothing to
// re-apply here; edits published while both run keep them in sync live.

void MotorConfigModule::start() {
    if (!status_) return;
    if (!motor_config_yaml_in_repo()) {
        status_->setText(
            QString("⚠ %1 is not a symlink into the repo — edits will NOT be "
                    "git-tracked (rebuild with --symlink-install)")
            .arg(QString::fromStdString(motor_config_yaml_path())));
        status_->setStyleSheet(
            QString("background: %1; color: %2; padding: 4px 6px;")
            .arg(theme::Bg).arg(theme::Yellow));
    }
}

// ---------------------------------------------------------------------------
// onFeedback  — update cells not currently being edited
// ---------------------------------------------------------------------------

void MotorConfigModule::onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
    if (!edits_[0][0]) return;
    if ((int)msg->config.size() < NUM_MOTORS) return;

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
            }
        }
    }
}

// ---------------------------------------------------------------------------
// publishUpdate  — send update AND write it to motor_config.yaml
// ---------------------------------------------------------------------------

void MotorConfigModule::publishUpdate(int motor_idx, int col, float value) {
    if (!pub_ || col < 0 || col >= NUM_COLS || REGISTERS[col] == nullptr) return;

    publishRaw(motor_idx, REGISTERS[col], value);

    // Persist to the repo-tracked yaml (also read by the driver at startup)
    bool saved = save_motor_config_value(motor_idx, YAML_KEYS[col], value);

    // Update cell tint
    if (edits_[motor_idx][col])
        edits_[motor_idx][col]->setStyleSheet(savedStyle());

    if (status_) {
        if (saved) {
            status_->setText(
                QString("Applied + written to motor_config.yaml:  motor %1  %2 = %3")
                .arg(motor_idx + 1)
                .arg(REGISTERS[col])
                .arg(value, 0, 'f', PRECISION[col]));
        } else {
            status_->setText(
                QString("⚠ Applied to driver but FAILED to write %1 (motor %2  %3)")
                .arg(QString::fromStdString(motor_config_yaml_path()))
                .arg(motor_idx + 1)
                .arg(REGISTERS[col]));
        }
    }
}

// ---------------------------------------------------------------------------
// publishRaw  — send update WITHOUT touching motor_config.yaml
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
// resetMotorToDefaults  — write built-in defaults to motor_config.yaml and
// publish them so the driver applies immediately
// ---------------------------------------------------------------------------

void MotorConfigModule::resetMotorToDefaults(int motor_idx) {
    auto defaults = fallback_arm_configuration();
    if (motor_idx < 0 || motor_idx >= (int)defaults.size()) return;
    const auto& def = defaults[motor_idx];

    const float def_vals[NUM_COLS] = {
        def.kp, def.ki, def.kd,
        def.max_current_A,
        def.max_velocity,
        def.max_acceleration,
        def.position_min,
        def.position_max,
        def.max_voltage,
        def.max_power_W,
        def.def_timeout,
        def.gear_reduction,  // col 11 — display only, never written
    };

    bool all_saved = true;
    for (int col = 0; col < NUM_COLS; col++) {
        if (REGISTERS[col] == nullptr) continue;
        publishRaw(motor_idx, REGISTERS[col], def_vals[col]);
        if (!save_motor_config_value(motor_idx, YAML_KEYS[col], def_vals[col]))
            all_saved = false;
    }

    // Revert cell tint
    for (int col = 0; col < NUM_EDITABLE; col++) {
        if (edits_[motor_idx][col] && !edits_[motor_idx][col]->hasFocus())
            edits_[motor_idx][col]->setStyleSheet(roStyle(theme::Bg, theme::Text));
    }

    if (status_) {
        status_->setText(all_saved
            ? QString("Motor %1 (%2) reset to built-in defaults — written to motor_config.yaml")
              .arg(motor_idx + 1).arg(JOINT_NAMES[motor_idx])
            : QString("⚠ Motor %1 reset published, but writing %2 failed")
              .arg(motor_idx + 1)
              .arg(QString::fromStdString(motor_config_yaml_path())));
    }
}

// ---------------------------------------------------------------------------
// requestCalibration  — publish to /arm/calibration_request
// motor_id: 1-based CAN ID; 0 = all sequential
// ---------------------------------------------------------------------------

void MotorConfigModule::requestCalibration(int motor_id) {
    if (!CALIBRATION_UI_ENABLED) return;  // buttons are disabled; belt-and-braces
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
        btn->setEnabled(enabled && CALIBRATION_UI_ENABLED);
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
