// motor_status_module.cpp  —  "Motor Telemetry"
//
// Live read-only table showing every hardware-reported value per motor.
//
// Row colors:
//   Fault (code ≠ 0) → red     (moteus_fault field)
//   Mode 1 (Fault)   → red
//   Mode 10 (Pos)    → green   (actively positioning)
//   Mode 11 (Timeout)→ yellow
//   Mode 12 (ZeroVel)→ cyan
//   Mode 0 (Stopped) → dim grey
//   Other            → default
//
// Temperature cell is additionally color-coded regardless of row state:
//   > 60°C  → red  (overheating warning)
//   40–60°C → yellow (warm)

#include "motor_status_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QGridLayout>
#include <QScrollArea>
#include <QFont>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

static const char* JOINT_NAMES[] = {
    "Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "End Effector"
};

// Column indices
enum Col {
    COL_MODE = 0,
    COL_FAULT,
    COL_POS_CURR,
    COL_POS_DES,
    COL_VEL_CURR,
    COL_VEL_DES,
    COL_TORQUE,
    COL_CURRENT,
    COL_POWER,
    COL_VOLTAGE,
    COL_TEMP,
    NUM_COLS
};

static const char* FIELD_HEADERS[] = {
    "Mode",
    "Fault",
    "Pos (rev)",
    "Des Pos (rev)",
    "Vel (rev/s)",
    "Des Vel",
    "Torque (Nm)",
    "Current (A)",
    "Power (W)",
    "Voltage (V)",
    "Temp (°C)",
};
static_assert(sizeof(FIELD_HEADERS) / sizeof(FIELD_HEADERS[0]) == NUM_COLS, "column count mismatch");

// Human-readable moteus mode names (index == mode number)
static const char* MODE_NAMES[] = {
    "Stopped",        // 0
    "FAULT",          // 1
    "Enabling",       // 2
    "Calibrating",    // 3
    "Cal Complete",   // 4
    "PWM",            // 5
    "Voltage",        // 6
    "VoltageFOC",     // 7
    "VoltageDQ",      // 8
    "Current",        // 9
    "Position",       // 10
    "TIMEOUT",        // 11
    "ZeroVelocity",   // 12
};
static constexpr int NUM_MODES = 13;

QWidget* MotorStatusModule::createWidget(QWidget* parent) {
    // Scroll area so the table doesn't get squished on small screens
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

    auto headerStyle = QString("background: %1; color: %2; padding: 6px 10px;")
        .arg(theme::HeaderBg).arg(theme::Text);

    // Corner header
    auto* corner = new QLabel("Joint");
    corner->setFont(monoBold);
    corner->setStyleSheet(headerStyle);
    corner->setAlignment(Qt::AlignCenter);
    grid->addWidget(corner, 0, 0);

    for (int c = 0; c < NUM_COLS; c++) {
        auto* lbl = new QLabel(FIELD_HEADERS[c]);
        lbl->setFont(monoBold);
        lbl->setStyleSheet(headerStyle);
        lbl->setAlignment(Qt::AlignCenter);
        grid->addWidget(lbl, 0, c + 1);
    }

    auto cellStyle = QString("background: %1; color: %2; padding: 6px 10px;")
        .arg(theme::Bg).arg(theme::Text);

    for (int r = 0; r < NUM_MOTORS; r++) {
        row_labels_[r] = new QLabel(JOINT_NAMES[r]);
        row_labels_[r]->setFont(monoBold);
        row_labels_[r]->setStyleSheet(
            QString("background: %1; color: %2; padding: 6px 10px;")
            .arg(theme::Bg).arg(theme::MotorColors[r]));
        row_labels_[r]->setAlignment(Qt::AlignCenter);
        grid->addWidget(row_labels_[r], r + 1, 0);

        for (int c = 0; c < NUM_COLS; c++) {
            auto* lbl = new QLabel("--");
            lbl->setFont(mono);
            lbl->setStyleSheet(cellStyle);
            lbl->setAlignment(Qt::AlignCenter);
            lbl->setMinimumWidth(80);
            grid->addWidget(lbl, r + 1, c + 1);
            cells_[r][c] = lbl;
        }
    }

    status_ = new QLabel("Waiting for telemetry...");
    status_->setFont(mono);
    status_->setStyleSheet(
        QString("background: %1; color: %2; padding: 6px 10px;")
        .arg(theme::Bg).arg(theme::TextDim));
    grid->addWidget(status_, NUM_MOTORS + 1, 0, 1, NUM_COLS + 1);

    for (int c = 0; c <= NUM_COLS; c++)
        grid->setColumnStretch(c, 1);

    scroll->setWidget(widget);
    return scroll;
}

void MotorStatusModule::setNode(rclcpp::Node::SharedPtr node) {
    auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    sub_ = node->create_subscription<rover_msgs::msg::MoteusArmStatus>(
        "/arm/moteus_feedback", qos,
        [this](const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
            onFeedback(msg);
        });
}

static inline QString fmtF(float v, int prec, const char* nan_str = "--") {
    return std::isnan(v) ? nan_str : QString::number(v, 'f', prec);
}

void MotorStatusModule::onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
    if (!cells_[0][0]) return;

    int active = 0;
    for (int i = 0; i < NUM_MOTORS && i < (int)msg->status.size(); i++) {
        const auto& s = msg->status[i];
        active++;

        // ── Row tint ──────────────────────────────────────────────────────────
        QString rowBg, rowFg;
        bool bold_row = false;

        if (s.moteus_fault != 0) {
            rowBg = "#3d1b1b"; rowFg = theme::Red; bold_row = true;
        } else {
            switch (s.moteus_mode) {
                case 0:   rowBg = "#1a1a2e"; rowFg = theme::TextDim; break;
                case 1:   rowBg = "#3d1b1b"; rowFg = theme::Red;    bold_row = true; break;
                case 10:  rowBg = "#1b3d2a"; rowFg = theme::Green;  break;
                case 11:  rowBg = "#3d3520"; rowFg = theme::Yellow; bold_row = true; break;
                case 12:  rowBg = "#1b2d3d"; rowFg = theme::Cyan;   break;
                default:  rowBg = theme::Bg; rowFg = theme::Text;   break;
            }
        }

        QString cellStyle = QString("background: %1; color: %2; padding: 6px 10px;%3")
            .arg(rowBg).arg(rowFg)
            .arg(bold_row ? " font-weight: bold;" : "");

        // ── Mode ──────────────────────────────────────────────────────────────
        int mode = s.moteus_mode;
        cells_[i][COL_MODE]->setText(
            (mode >= 0 && mode < NUM_MODES) ? MODE_NAMES[mode]
                                            : QString("Mode %1").arg(mode));
        cells_[i][COL_MODE]->setStyleSheet(cellStyle);

        // ── Fault ─────────────────────────────────────────────────────────────
        if (s.moteus_fault == 0) {
            cells_[i][COL_FAULT]->setText("OK");
            cells_[i][COL_FAULT]->setStyleSheet(
                QString("background: %1; color: %2; padding: 6px 10px;")
                .arg(rowBg).arg(theme::Green));
        } else {
            cells_[i][COL_FAULT]->setText(QString("ERR %1").arg(s.moteus_fault));
            cells_[i][COL_FAULT]->setStyleSheet(
                QString("background: %1; color: %2; padding: 6px 10px; font-weight: bold;")
                .arg(rowBg).arg(theme::Red));
        }

        // ── Position ──────────────────────────────────────────────────────────
        cells_[i][COL_POS_CURR]->setText(fmtF(s.curr_position, 3));
        cells_[i][COL_POS_CURR]->setStyleSheet(cellStyle);

        cells_[i][COL_POS_DES]->setText(fmtF(s.des_position, 3));
        cells_[i][COL_POS_DES]->setStyleSheet(cellStyle);

        // ── Velocity ──────────────────────────────────────────────────────────
        cells_[i][COL_VEL_CURR]->setText(fmtF(s.curr_velocity, 3));
        cells_[i][COL_VEL_CURR]->setStyleSheet(cellStyle);

        cells_[i][COL_VEL_DES]->setText(fmtF(s.des_velocity, 3));
        cells_[i][COL_VEL_DES]->setStyleSheet(cellStyle);

        // ── Torque ────────────────────────────────────────────────────────────
        cells_[i][COL_TORQUE]->setText(fmtF(s.curr_torque, 3));
        cells_[i][COL_TORQUE]->setStyleSheet(cellStyle);

        // ── Current (Q-axis amps) ─────────────────────────────────────────────
        cells_[i][COL_CURRENT]->setText(fmtF(s.curr_current_amps, 2));
        cells_[i][COL_CURRENT]->setStyleSheet(cellStyle);

        // ── Power ────────────────────────────────────────────────────────────
        cells_[i][COL_POWER]->setText(fmtF(s.curr_power_watts, 1));
        cells_[i][COL_POWER]->setStyleSheet(cellStyle);

        // ── Voltage ──────────────────────────────────────────────────────────
        cells_[i][COL_VOLTAGE]->setText(fmtF(s.curr_voltage_volts, 1));
        cells_[i][COL_VOLTAGE]->setStyleSheet(cellStyle);

        // ── Temperature (extra color-coding) ──────────────────────────────────
        float temp = s.driver_temp_degreesc;
        cells_[i][COL_TEMP]->setText(fmtF(temp, 1));
        if (temp >= 60.0f) {
            cells_[i][COL_TEMP]->setStyleSheet(
                QString("background: %1; color: %2; padding: 6px 10px; font-weight: bold;")
                .arg(rowBg).arg(theme::Red));
        } else if (temp >= 40.0f) {
            cells_[i][COL_TEMP]->setStyleSheet(
                QString("background: %1; color: %2; padding: 6px 10px;")
                .arg(rowBg).arg(theme::Yellow));
        } else {
            cells_[i][COL_TEMP]->setStyleSheet(cellStyle);
        }

        // ── Row label (keep motor accent color, update background tint) ───────
        if (row_labels_[i]) {
            row_labels_[i]->setStyleSheet(
                QString("background: %1; color: %2; padding: 6px 10px; font-weight: bold;")
                .arg(rowBg).arg(theme::MotorColors[i]));
        }
    }

    if (status_)
        status_->setText(QString("Live  ·  %1 motor%2 reporting")
            .arg(active).arg(active == 1 ? "" : "s"));
}

PLUGINLIB_EXPORT_CLASS(MotorStatusModule, rover_hmi_core::GuiModule)
