// motor_status_module.cpp  —  "Motor Telemetry"
//
// Live read-only table showing every hardware-reported value per motor.
//
// Row colors:
//   Fault (code ≠ 0) → red     (moteus_fault field)
//   Mode 1 (Fault)   → red
//   Mode 10 (Pos)    → green   (actively positioning)
//   Mode 11 (Timeout)→ yellow
//   Mode 12 (ZeroVel)→ dim grey (idle on connect)
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
#include <QTimer>
#include <QDateTime>
#include <cmath>
#include <algorithm>
#include <map>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

// ---------------------------------------------------------------------------
// TableScaleFilter — updates all cell fonts when the widget is resized.
// No Q_OBJECT needed: we only override the virtual eventFilter(), no signals.
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

// Column indices
enum Col {
    COL_MODE = 0,
    COL_FAULT,
    COL_LIMIT,
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
    "Lim",
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
static_assert(MotorStatusModule::NUM_FIELDS == NUM_COLS, "cells_ storage must match column count");

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
    "Within",         // 13 stay_within_bounds
    "MeasureInd",     // 14
    "Brake",          // 15
};
static constexpr int NUM_MODES = 16;

QWidget* MotorStatusModule::createWidget(QWidget* parent) {
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
    widget->setStyleSheet(QString("background: %1;").arg(theme::Bg));
    dim_ = new QGraphicsOpacityEffect(widget);   // dims table when feedback stops
    dim_->setOpacity(1.0);
    widget->setGraphicsEffect(dim_);
    auto* grid = new QGridLayout(widget);
    grid->setSpacing(1);
    grid->setContentsMargins(2, 2, 2, 2);

    QFont mono("monospace", theme::FontSize);
    QFont monoBold("monospace", theme::FontSize, QFont::Bold);

    auto headerStyle = QString(
        "background: %1; color: %2; padding: 4px 6px;"
        " border-bottom: 1px solid %3;")
        .arg(theme::Bg).arg(theme::Text).arg(theme::BorderDim);
    auto cellStyle = QString(
        "background: %1; color: %2; padding: 4px 6px; border: 1px solid %3;")
        .arg(theme::Bg).arg(theme::Text).arg(theme::BorderDim);

    // Widget lists for font scaling
    std::vector<QWidget*> bold_ws, normal_ws;

    // Corner + column headers
    auto* corner = new QLabel("Joint");
    corner->setFont(monoBold);
    corner->setStyleSheet(headerStyle);
    corner->setAlignment(Qt::AlignCenter);
    grid->addWidget(corner, 0, 0);
    bold_ws.push_back(corner);

    for (int c = 0; c < NUM_COLS; c++) {
        auto* lbl = new QLabel(FIELD_HEADERS[c]);
        lbl->setFont(monoBold);
        lbl->setStyleSheet(headerStyle);
        lbl->setAlignment(Qt::AlignCenter);
        grid->addWidget(lbl, 0, c + 1);
        bold_ws.push_back(lbl);
    }

    for (int r = 0; r < NUM_MOTORS; r++) {
        // Telemetry rows are MOTORS (one CAN id each), not axes.
        row_labels_[r] = new QLabel(QString("M%1").arg(r + 1));
        row_labels_[r]->setFont(monoBold);
        row_labels_[r]->setStyleSheet(
            QString("background: %1; color: %2; padding: 4px 6px; border: 1px solid %3;")
            .arg(theme::Bg).arg(theme::MotorColors[r]).arg(theme::BorderDim));
        row_labels_[r]->setAlignment(Qt::AlignCenter);
        grid->addWidget(row_labels_[r], r + 1, 0);
        bold_ws.push_back(row_labels_[r]);

        for (int c = 0; c < NUM_COLS; c++) {
            auto* lbl = new QLabel("--");
            lbl->setFont(mono);
            lbl->setStyleSheet(cellStyle);
            lbl->setAlignment(Qt::AlignCenter);
            grid->addWidget(lbl, r + 1, c + 1);
            cells_[r][c] = lbl;
            normal_ws.push_back(lbl);
        }
    }

    // Bench-only local driver control; disabled when the fdcanusb lives on
    // the rover (remote driver is bringup/systemd's job, not the HMI's).
    drv_btn_ = new QPushButton();
    drv_btn_->setFont(mono);
    QObject::connect(drv_btn_, &QPushButton::clicked, [this]() {
        if (drv_.running()) drv_.stop();
        else if (!drv_.start())
            setBanner("✖ FAILED TO START DRIVER — is arm_hardware_interface built?",
                      theme::Red, "#2a0d0d");
        updateDriverBtn();
    });
    grid->addWidget(drv_btn_, NUM_MOTORS + 1, 0);

    status_ = new QLabel();
    status_->setFont(mono);
    grid->addWidget(status_, NUM_MOTORS + 1, 1, 1, NUM_COLS);
    normal_ws.push_back(status_);
    setBanner("⏳ WAITING FOR ARM DRIVER — no telemetry yet", theme::TextDim, "#1a1a1a");

    // Staleness watchdog: values on screen must never look fresher than they
    // are — after 600 ms of silence the whole table dims and the banner says
    // so; after 1.5 s the driver itself is presumed down.
    auto* stale = new QTimer(status_);
    QObject::connect(stale, &QTimer::timeout, [this]() {
        const qint64 age = QDateTime::currentMSecsSinceEpoch() - last_msg_ms_;
        if (last_msg_ms_ != 0 && age > 1500) {
            if (dim_) dim_->setOpacity(0.35);
            setBanner("✖ ARM DRIVER OFFLINE — /arm/moteus_feedback stopped",
                      theme::Red, "#2a0d0d");
        } else if (last_msg_ms_ != 0 && age > 600) {
            if (dim_) dim_->setOpacity(0.35);
            setBanner(QString("⚠ TELEMETRY STALE — last update %1 s ago, values frozen")
                          .arg(age / 1000.0, 0, 'f', 1),
                      theme::Yellow, "#2a2208");
        }
        updateDriverBtn();
    });
    stale->start(250);
    updateDriverBtn();

    // All columns equal stretch
    for (int c = 0; c <= NUM_COLS; c++)
        grid->setColumnStretch(c, 1);

    // All rows equal stretch so every row grows proportionally
    for (int r = 0; r <= NUM_MOTORS + 1; r++)
        grid->setRowStretch(r, 1);

    // Font scaler: updates font size whenever the scroll area is resized
    // NUM_MOTORS + 2 = header row + motor rows + status row
    auto* scaler = new TableScaleFilter(
        scroll, NUM_MOTORS + 2, std::move(normal_ws), std::move(bold_ws));
    scroll->installEventFilter(scaler);

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
    if (dim_) dim_->setOpacity(1.0);   // fresh data — undim

    int active = 0;
    for (int i = 0; i < NUM_MOTORS && i < (int)msg->status.size(); i++) {
        const auto& s = msg->status[i];
        active++;

        // ── Row tint ──────────────────────────────────────────────────────────
        QString rowBg, rowFg;
        bool bold_row = false;

        // Codes ≥96 are output-limit flags (current/power/temp caps active),
        // not faults — see moteus fw/error.h.
        const bool limit_flag = s.moteus_fault >= 96;
        if (!s.connected) {
            rowBg = "#0a0a0a"; rowFg = "#444444";   // stale — last-known values
        } else if (s.moteus_fault != 0 && !limit_flag) {
            rowBg = "#3d1b1b"; rowFg = theme::Red; bold_row = true;
        } else {
            switch (s.moteus_mode) {
                case 0:   rowBg = "#111111"; rowFg = theme::TextDim; break;  // Stopped — dark grey
                case 1:   rowBg = "#3d1b1b"; rowFg = theme::Red;    bold_row = true; break;
                case 10:  rowBg = "#1b3d2a"; rowFg = theme::Green;  break;
                case 11:  rowBg = "#3d3520"; rowFg = theme::Yellow; bold_row = true; break;
                case 12:  rowBg = "#111111"; rowFg = theme::TextDim; break;  // ZeroVelocity — same as Stopped
                default:  rowBg = theme::Bg; rowFg = theme::Text;   break;
            }
        }

        QString cellStyle = QString(
            "background: %1; color: %2; padding: 6px 10px;"
            " border: 1px solid %3;%4")
            .arg(rowBg).arg(rowFg).arg(theme::BorderDim)
            .arg(bold_row ? " font-weight: bold;" : "");

        // ── Mode ──────────────────────────────────────────────────────────────
        int mode = s.moteus_mode;
        cells_[i][COL_MODE]->setText(
            !s.connected ? "NO REPLY"
            : (mode >= 0 && mode < NUM_MODES) ? MODE_NAMES[mode]
                                              : QString("Mode %1").arg(mode));
        cells_[i][COL_MODE]->setStyleSheet(cellStyle);

        // ── Fault ─────────────────────────────────────────────────────────────
        static const std::map<int, const char*> LIMIT_NAMES = {
            {96, "VEL CAP"}, {97, "PWR CAP"}, {98, "VOLT CAP"}, {99, "CUR CAP"},
            {100, "FET TEMP"}, {101, "MTR TEMP"}, {102, "TRQ CAP"},
            {103, "POS BOUND"}, {104, "FLUX BRK"},
        };
        // Stale rows keep their last-known text but never vivid colors — a
        // dead motor must not display a bright green "OK".
        const QString ok_fg  = s.connected ? QString(theme::Green)  : QString("#444444");
        const QString lim_fg = s.connected ? QString(theme::Yellow) : QString("#444444");
        const QString err_fg = s.connected ? QString(theme::Red)    : QString("#444444");
        if (s.moteus_fault == 0) {
            cells_[i][COL_FAULT]->setText("OK");
            cells_[i][COL_FAULT]->setStyleSheet(
                QString("background: %1; color: %2; padding: 6px 10px; border: 1px solid %3;")
                .arg(rowBg).arg(ok_fg).arg(theme::BorderDim));
        } else if (limit_flag) {
            auto it = LIMIT_NAMES.find(s.moteus_fault);
            cells_[i][COL_FAULT]->setText(
                it != LIMIT_NAMES.end() ? QString(it->second)
                                        : QString("LIM %1").arg(s.moteus_fault));
            cells_[i][COL_FAULT]->setStyleSheet(
                QString("background: %1; color: %2; padding: 6px 10px; border: 1px solid %3;")
                .arg(rowBg).arg(lim_fg).arg(theme::BorderDim));
        } else {
            cells_[i][COL_FAULT]->setText(QString("ERR %1").arg(s.moteus_fault));
            cells_[i][COL_FAULT]->setStyleSheet(
                QString("background: %1; color: %2; padding: 6px 10px;"
                        " border: 1px solid %3; font-weight: bold;")
                .arg(rowBg).arg(err_fg).arg(theme::BorderDim));
        }

        // ── Limit switch ─────────────────────────────────────────────────────
        bool lim = i < (int)msg->limit_switches.size() && msg->limit_switches[i];
        cells_[i][COL_LIMIT]->setText(lim ? "LIM" : "—");
        cells_[i][COL_LIMIT]->setStyleSheet(lim && s.connected
            ? QString("background: %1; color: %2; padding: 6px 10px;"
                      " border: 1px solid %3; font-weight: bold;")
                .arg(rowBg).arg(theme::Red).arg(theme::BorderDim)
            : cellStyle);

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
                QString("background: %1; color: %2; padding: 6px 10px;"
                        " border: 1px solid %3; font-weight: bold;")
                .arg(rowBg).arg(theme::Red).arg(theme::BorderDim));
        } else if (temp >= 40.0f) {
            cells_[i][COL_TEMP]->setStyleSheet(
                QString("background: %1; color: %2; padding: 6px 10px; border: 1px solid %3;")
                .arg(rowBg).arg(theme::Yellow).arg(theme::BorderDim));
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

    // ── CAN link banner ───────────────────────────────────────────────────────
    // Distinguishes the three failure layers the raw table can't: adapter
    // missing, adapter present but bus silent, and partial replies.
    last_msg_ms_ = QDateTime::currentMSecsSinceEpoch();
    const QString dev = QString::fromStdString(msg->can_device);
    const int expected = NUM_MOTORS;   // all 7: M1-M6 + EE (stale "A4 disabled" -1 removed)
    (void)active;
    if (dev.isEmpty()) {
        setBanner("✖ NO FDCANUSB — USB adapter not found (check cable/port)",
                  theme::Red, "#2a0d0d");
    } else if (msg->motors_replying == 0) {
        setBanner(QString("⚠ FDCANUSB OK on %1 — BUS SILENT: no motor replies "
                          "(servo power? CAN wiring?)").arg(dev),
                  theme::Yellow, "#2a2208");
    } else if (msg->motors_replying < expected) {
        setBanner(QString("⚠ PARTIAL: %1/%2 motors replying on %3")
                  .arg(msg->motors_replying).arg(expected).arg(dev),
                  theme::Yellow, "#2a2208");
    } else {
        setBanner(QString("● CAN LIVE on %1 — %2/%3 motors")
                  .arg(dev).arg(msg->motors_replying).arg(expected),
                  theme::Green, "#0d2a17", false);
    }
}

void MotorStatusModule::updateDriverBtn()
{
    if (!drv_btn_) return;
    const bool fresh = last_msg_ms_ != 0 &&
        QDateTime::currentMSecsSinceEpoch() - last_msg_ms_ <= 1500;
    if (drv_.running()) {
        drv_btn_->setText("■ Stop Driver");
        drv_btn_->setEnabled(true);
        drv_btn_->setToolTip("Stops the HMI-owned driver (frees the CAN port for tview)");
    } else if (fresh) {
        drv_btn_->setText("Driver: external");
        drv_btn_->setEnabled(false);
        drv_btn_->setToolTip("Telemetry is coming from a driver this HMI didn't start "
                             "(terminal or rover bringup) — stop it where it was started");
    } else if (DriverProcess::localAdapterPresent()) {
        drv_btn_->setText("▶ Start Driver");
        drv_btn_->setEnabled(true);
        drv_btn_->setToolTip("Starts moteus_driver on this machine, tied to the HMI's "
                             "lifetime — it dies with the HMI, even on a crash-close");
    } else {
        drv_btn_->setText("▶ Start Driver");
        drv_btn_->setEnabled(false);
        drv_btn_->setToolTip("No local fdcanusb — on the rover the driver runs on the "
                             "NUC via bringup, not from the HMI");
    }
}

void MotorStatusModule::setBanner(const QString& text, const char* fg,
                                  const char* bg, bool bold)
{
    if (!status_) return;
    status_->setText(text);
    status_->setStyleSheet(
        QString("background: %1; color: %2; padding: 6px 8px;%3")
        .arg(bg).arg(fg).arg(bold ? " font-weight: bold;" : ""));
}

PLUGINLIB_EXPORT_CLASS(MotorStatusModule, rover_hmi_core::GuiModule)
