// motor_config_module.cpp  —  "Motor Params"  (see header for the model)

#include "motor_config_module.h"
#include <rover_arm_common/motor_config.h>
#include <rover_hmi_core/catppuccin.h>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QTimer>
#include <QFont>
#include <cmath>
#include <cstdlib>

#include <pluginlib/class_list_macros.hpp>

// ── Register metadata ────────────────────────────────────────────────────────

namespace {

struct RegDef {
    const char* label;
    const char* reg;                      // moteus register ("" = read-only row)
    float (*def)(const MotorConfig&);     // motor_config.h default
    float (*act)(const rover_msgs::msg::BldcServoConfig&);  // driver readback
};

using Cfg = MotorConfig;
using Fc  = rover_msgs::msg::BldcServoConfig;

const RegDef REGS[MotorConfigModule::NUM_REGS] = {
    {"Kp",           "servo.pid_position.kp",
        [](const Cfg& c){ return c.kp; },              [](const Fc& f){ return f.kp; }},
    {"Ki",           "servo.pid_position.ki",
        [](const Cfg& c){ return c.ki; },              [](const Fc& f){ return f.ki; }},
    {"Kd",           "servo.pid_position.kd",
        [](const Cfg& c){ return c.kd; },              [](const Fc& f){ return f.kd; }},
    {"Max I (A)",    "servo.max_current_A",
        [](const Cfg& c){ return c.max_current_A; },   [](const Fc& f){ return f.max_current_amps; }},
    {"Max Vel",      "servo.default_velocity_limit",
        [](const Cfg& c){ return c.max_velocity; },    [](const Fc& f){ return f.max_velocity; }},
    {"Max Accel",    "servo.default_accel_limit",
        [](const Cfg& c){ return c.max_acceleration; },[](const Fc& f){ return f.max_acceleration; }},
    {"Pos Min",      "servopos.position_min",
        [](const Cfg& c){ return c.position_min; },    [](const Fc& f){ return f.min_position; }},
    {"Pos Max",      "servopos.position_max",
        [](const Cfg& c){ return c.position_max; },    [](const Fc& f){ return f.max_position; }},
    {"Max V",        "servo.max_voltage",
        [](const Cfg& c){ return c.max_voltage; },     [](const Fc& f){ return f.max_voltage_volts; }},
    {"Max P (W)",    "servo.max_power_W",
        [](const Cfg& c){ return c.max_power_W; },     [](const Fc& f){ return f.max_power_watts; }},
    {"Timeout (s)",  "servo.default_timeout_s",
        [](const Cfg& c){ return c.def_timeout; },     [](const Fc& f){ return f.cmd_timeout_s; }},
    {"Gear",         "",
        [](const Cfg& c){ return c.gear_reduction; },  [](const Fc& f){ return f.gear_reduction; }},
};

// Repo-tracked record of flash-written changes (config/arm_params.ini),
// resolved like config/layout: env override first, then the build machine's
// source tree.
QString armParamsPath() {
    if (const char* root = std::getenv("ROVERFLAKE_ROOT"))
        return QString("%1/src/rover_hmi_core/config/arm_params.ini").arg(root);
#ifdef ROVER_HMI_CORE_SOURCE_DIR
    return QString(ROVER_HMI_CORE_SOURCE_DIR "/config/arm_params.ini");
#else
    return {};
#endif
}

QString fmtVal(float v) {
    return std::isnan(v) ? QStringLiteral("?") : QString::number(v, 'g', 5);
}

bool nearlyEqual(float a, float b) {
    if (std::isnan(a) || std::isnan(b)) return false;
    return std::fabs(a - b) <= 0.001f * std::max({std::fabs(a), std::fabs(b), 1e-3f});
}

} // namespace

// ── UI ───────────────────────────────────────────────────────────────────────

QWidget* MotorConfigModule::createWidget(QWidget* parent) {
    auto* scroll = new QScrollArea(parent);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setStyleSheet(QString(
        "QScrollArea { background: %1; }"
        "QScrollArea > QWidget > QWidget { background: %1; }").arg(theme::Bg));

    auto* w = new QWidget();
    auto* root = new QVBoxLayout(w);
    root->setContentsMargins(8, 6, 8, 8);
    root->setSpacing(6);
    QFont mono("monospace", theme::FontSize);
    QFont monoBold("monospace", theme::FontSize, QFont::Bold);

    // Motor selector + refresh
    auto* top = new QHBoxLayout();
    motor_sel_ = new QComboBox();
    motor_sel_->setFont(monoBold);
    for (int m = 0; m < NUM_MOTORS; m++)
        motor_sel_->addItem(QString::fromUtf8(ARM_JOINTS[m].hardware_name));
    motor_sel_->setStyleSheet(QString(
        "QComboBox { background: %1; color: %2; border: 1px solid %3; padding: 4px 8px; }")
        .arg(theme::BgPanel).arg(theme::Text).arg(theme::Border));
    QObject::connect(motor_sel_, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     [this](int) { refreshDisplay(); });
    top->addWidget(motor_sel_, 1);

    auto* refresh_btn = new QPushButton("⟳ Refresh");
    refresh_btn->setFont(mono);
    refresh_btn->setToolTip("Driver re-reads every register from the controllers (conf get)");
    refresh_btn->setStyleSheet(QString(
        "QPushButton { background: %1; color: %2; border: 1px solid %3; padding: 4px 10px; }"
        "QPushButton:hover { border-color: %4; }")
        .arg(theme::Bg).arg(theme::TextDim).arg(theme::BorderDim).arg(theme::Border));
    QObject::connect(refresh_btn, &QPushButton::clicked, [this]() {
        if (refresh_pub_) refresh_pub_->publish(std_msgs::msg::Empty());
        if (status_) status_->setText("Refresh requested — Actual updates as controllers answer");
    });
    top->addWidget(refresh_btn);
    root->addLayout(top);

    // Table
    auto* grid = new QGridLayout();
    grid->setSpacing(2);
    const char* headers[] = {"Register", "Default", "Actual", "New"};
    for (int c = 0; c < 4; c++) {
        auto* h = new QLabel(headers[c]);
        h->setFont(monoBold);
        h->setAlignment(Qt::AlignCenter);
        h->setStyleSheet(QString("color: %1; padding: 4px; border-bottom: 1px solid %2;")
            .arg(theme::Text).arg(theme::BorderDim));
        grid->addWidget(h, 0, c);
    }
    for (int r = 0; r < NUM_REGS; r++) {
        auto* name = new QLabel(REGS[r].label);
        name->setFont(mono);
        name->setStyleSheet(QString("color: %1; padding: 3px 6px;").arg(theme::TextDim));
        name->setToolTip(REGS[r].reg[0] ? REGS[r].reg : "informational");
        grid->addWidget(name, r + 1, 0);

        defaults_[r] = new QLabel("--");
        defaults_[r]->setFont(mono);
        defaults_[r]->setAlignment(Qt::AlignCenter);
        defaults_[r]->setStyleSheet(QString("color: %1;").arg(theme::TextDim));
        grid->addWidget(defaults_[r], r + 1, 1);

        actuals_[r] = new QLabel("?");
        actuals_[r]->setFont(monoBold);
        actuals_[r]->setAlignment(Qt::AlignCenter);
        grid->addWidget(actuals_[r], r + 1, 2);

        edits_[r] = new QLineEdit();
        edits_[r]->setFont(mono);
        edits_[r]->setAlignment(Qt::AlignCenter);
        edits_[r]->setPlaceholderText("·");
        edits_[r]->setEnabled(REGS[r].reg[0] != '\0');
        edits_[r]->setStyleSheet(QString(
            "QLineEdit { background: %1; color: %2; border: 1px solid %3; padding: 3px; }"
            "QLineEdit:focus { border-color: %4; }"
            "QLineEdit:disabled { background: transparent; border: none; }")
            .arg(theme::BgPanel).arg(theme::Text).arg(theme::BorderDim).arg(theme::Cyan));
        QObject::connect(edits_[r], &QLineEdit::returnPressed,
                         [this, r]() { confirmAndApply(r); });
        grid->addWidget(edits_[r], r + 1, 3);
    }
    grid->setColumnStretch(0, 2);
    for (int c = 1; c < 4; c++) grid->setColumnStretch(c, 1);
    root->addLayout(grid);

    // Calibration (same request topic/flow as before)
    auto* cal_row = new QHBoxLayout();
    calib_btn_ = new QPushButton("Calibrate");
    calib_all_btn_ = new QPushButton("Calibrate All (Sequential)");
    for (auto* b : { calib_btn_, calib_all_btn_ }) {
        b->setFont(mono);
        b->setStyleSheet(QString(
            "QPushButton { background: #1a1400; color: %1; border: 1px solid %1; padding: 5px 10px; }"
            "QPushButton:hover { background: #2a2000; }").arg(theme::Yellow));
        cal_row->addWidget(b);
    }
    QObject::connect(calib_btn_, &QPushButton::clicked, [this]() {
        rover_msgs::msg::MoteusCalibrationRequest req;
        req.motor_id = motor_sel_->currentIndex() + 1;
        if (calib_pub_) calib_pub_->publish(req);
    });
    QObject::connect(calib_all_btn_, &QPushButton::clicked, [this]() {
        rover_msgs::msg::MoteusCalibrationRequest req;
        req.motor_id = 0;
        if (calib_pub_) calib_pub_->publish(req);
    });
    root->addLayout(cal_row);

    status_ = new QLabel("Actual column is read from the controllers — '?' means not read yet");
    status_->setFont(mono);
    status_->setWordWrap(true);
    status_->setStyleSheet(QString("color: %1; padding: 4px 0;").arg(theme::TextDim));
    root->addWidget(status_);
    root->addStretch();

    // Repaint at 2 Hz from the cached message — avoids painting at feedback rate.
    auto* tick = new QTimer(w);
    QObject::connect(tick, &QTimer::timeout, [this]() { refreshDisplay(); });
    tick->start(500);

    refreshDisplay();
    scroll->setWidget(w);
    return scroll;
}

// ── ROS ──────────────────────────────────────────────────────────────────────

void MotorConfigModule::setNode(rclcpp::Node::SharedPtr node) {
    auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    pub_ = node->create_publisher<rover_msgs::msg::MoteusConfigUpdate>("/arm/config_update", qos);
    refresh_pub_ = node->create_publisher<std_msgs::msg::Empty>("/arm/config_refresh", qos);
    calib_pub_ = node->create_publisher<rover_msgs::msg::MoteusCalibrationRequest>(
        "/arm/calibration_request", qos);
    sub_ = node->create_subscription<rover_msgs::msg::MoteusArmStatus>(
        "/arm/moteus_feedback", qos,
        [this](const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) { onFeedback(msg); });
}

void MotorConfigModule::onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
    latest_ = *msg;
    have_msg_ = true;
}

// ── Display ──────────────────────────────────────────────────────────────────

void MotorConfigModule::refreshDisplay() {
    if (!motor_sel_) return;
    const int m = motor_sel_->currentIndex();
    static const auto cfgs = get_arm_configuration();
    if (m < 0 || m >= (int)cfgs.size()) return;

    QSettings repo(armParamsPath(), QSettings::IniFormat);

    for (int r = 0; r < NUM_REGS; r++) {
        const float dv = REGS[r].def(cfgs[m]);
        defaults_[r]->setText(fmtVal(dv));

        float av = NAN;
        if (have_msg_ && m < (int)latest_.config.size())
            av = REGS[r].act(latest_.config[m]);
        actuals_[r]->setText(fmtVal(av));

        // gray '?' unread · dim == default · yellow diverged from default ·
        // orange bg additionally drifted from arm_params.ini
        QString style;
        if (std::isnan(av))            style = "color: #555555;";
        else if (nearlyEqual(av, dv))  style = QString("color: %1;").arg(theme::TextDim);
        else                           style = QString("color: %1;").arg(theme::Yellow);

        if (REGS[r].reg[0]) {
            const QString key = QString("A%1/%2").arg(m + 1).arg(REGS[r].reg);
            if (repo.contains(key)) {
                const float rv = repo.value(key).toFloat();
                if (!std::isnan(av) && !nearlyEqual(av, rv)) {
                    style += " background: #2a1a00;";
                    actuals_[r]->setToolTip(QString("arm_params.ini expects %1").arg(fmtVal(rv)));
                } else {
                    actuals_[r]->setToolTip("matches arm_params.ini");
                }
            } else {
                actuals_[r]->setToolTip("");
            }
        }
        actuals_[r]->setStyleSheet(style + " padding: 3px;");
    }
}

// ── Guarded mutation ─────────────────────────────────────────────────────────

void MotorConfigModule::confirmAndApply(int r) {
    const int m = motor_sel_->currentIndex();
    bool ok = false;
    const float nv = edits_[r]->text().toFloat(&ok);
    if (!ok) {
        status_->setText(QString("'%1' is not a number").arg(edits_[r]->text()));
        return;
    }
    if ((r == 3 || r == 8 || r == 9) && nv <= 0.0f) {   // current/voltage/power
        status_->setText("Safety limits must be positive");
        return;
    }

    float av = NAN;
    if (have_msg_ && m < (int)latest_.config.size())
        av = REGS[r].act(latest_.config[m]);

    QMessageBox box(edits_[r]->window());
    box.setWindowTitle("Apply parameter change");
    box.setText(QString("%1 · %2\n\n    %3  →  %4")
        .arg(ARM_JOINTS[m].hardware_name).arg(REGS[r].reg)
        .arg(fmtVal(av)).arg(fmtVal(nv)));
    auto* ram   = box.addButton("Apply (RAM — reverts on power-off)", QMessageBox::AcceptRole);
    auto* flash = box.addButton("Apply && Write Flash", QMessageBox::DestructiveRole);
    box.addButton(QMessageBox::Cancel);
    box.exec();

    const bool persist = box.clickedButton() == flash;
    if (box.clickedButton() != ram && !persist) { edits_[r]->clear(); return; }

    rover_msgs::msg::MoteusConfigUpdate msg;
    msg.motor_id = m;
    msg.register_name = REGS[r].reg;
    msg.value = nv;
    msg.write_flash = persist;
    if (pub_) pub_->publish(msg);

    if (persist) {
        QSettings repo(armParamsPath(), QSettings::IniFormat);
        repo.setValue(QString("A%1/%2").arg(m + 1).arg(REGS[r].reg), nv);
        repo.sync();
    }
    status_->setText(QString("%1 %2 = %3 sent (%4) — watch Actual for confirmation")
        .arg(ARM_JOINTS[m].hardware_name).arg(REGS[r].reg).arg(fmtVal(nv))
        .arg(persist ? "flash" : "RAM only"));
    edits_[r]->clear();
}

PLUGINLIB_EXPORT_CLASS(MotorConfigModule, rover_hmi_core::GuiModule)
