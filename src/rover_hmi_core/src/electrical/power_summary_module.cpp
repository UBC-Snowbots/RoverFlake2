// power_summary_module.cpp — "Power Summary"
//
// Row 0: Battery columns (3) + Total column
// Row 1: ARM / CHASSIS / SCIENCE power bars + on/off toggles
// Row 2: CSV record button
// Row 3: Status bar

#include "power_summary_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QFont>
#include <QSizePolicy>
#include <QDateTime>
#include <QDir>
#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

// ---------------------------------------------------------------------------
// ResizeScaler — scales fonts proportionally on widget resize
// ---------------------------------------------------------------------------
class ResizeScaler : public QObject {
public:
    ResizeScaler(QObject* parent, int num_rows,
                 std::vector<QWidget*> normal_ws,
                 std::vector<QWidget*> bold_ws)
        : QObject(parent), num_rows_(num_rows),
          normal_ws_(std::move(normal_ws)),
          bold_ws_(std::move(bold_ws)) {}

    bool eventFilter(QObject* obj, QEvent* ev) override {
        if (ev->type() == QEvent::Resize)
            updateFonts(static_cast<QWidget*>(obj)->height());
        return QObject::eventFilter(obj, ev);
    }

    void updateFonts(int height) {
        if (num_rows_ == 0) return;
        int row_h = height / num_rows_;
        int fs = std::max(8, std::min(28, row_h * 30 / 100));
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
// createWidget
// ---------------------------------------------------------------------------
QWidget* PowerSummaryModule::createWidget(QWidget* parent) {
    auto* scroll = new QScrollArea(parent);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll->setStyleSheet(
        QString("QScrollArea { background: %1; }"
                "QScrollArea > QWidget > QWidget { background: %1; }")
        .arg(theme::Bg));

    auto* widget = new QWidget();
    widget->setStyleSheet(QString("background: %1;").arg(theme::Bg));
    auto* grid = new QGridLayout(widget);
    grid->setSpacing(2);
    grid->setContentsMargins(4, 4, 4, 4);

    QFont mono("monospace", theme::FontSize);
    QFont monoBold("monospace", theme::FontSize, QFont::Bold);
    QFont monoBig("monospace", theme::FontSizeLg, QFont::Bold);

    auto headerStyle = QString(
        "background: %1; color: %2; padding: 4px 6px;"
        " border-bottom: 1px solid %3; font-weight: bold;")
        .arg(theme::HeaderBg).arg(theme::Text).arg(theme::BorderDim);
    auto cellStyle = QString(
        "background: %1; color: %2; padding: 4px 6px; border: 1px solid %3;")
        .arg(theme::Bg).arg(theme::Text).arg(theme::BorderDim);

    std::vector<QWidget*> normal_ws, bold_ws;

    // ── Column headers (Battery 1, 2, 3, Total) ──────────────────────────────
    const char* colHeaders[] = { "Battery 1", "Battery 2", "Battery 3", "Total" };
    for (int c = 0; c < 4; c++) {
        auto* lbl = new QLabel(colHeaders[c]);
        lbl->setFont(monoBold);
        lbl->setStyleSheet(headerStyle);
        lbl->setAlignment(Qt::AlignCenter);
        lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        grid->addWidget(lbl, 0, c);
        bold_ws.push_back(lbl);
    }

    // ── Row 1: Battery data panels (voltage / current / power / indicator) ───
    static const char* BAT_LABELS[] = { "Battery 1", "Battery 2", "Battery 3" };
    for (int b = 0; b < 3; b++) {
        auto* cell = new QWidget();
        cell->setStyleSheet(
            QString("background: %1; border: 1px solid %2; border-radius: 4px;")
            .arg(theme::Bg).arg(theme::BorderDim));
        cell->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        auto* vl = new QVBoxLayout(cell);
        vl->setSpacing(2);
        vl->setContentsMargins(6, 6, 6, 6);

        bat_voltage_[b] = new QLabel("--.- V");
        bat_voltage_[b]->setFont(monoBig);
        bat_voltage_[b]->setAlignment(Qt::AlignCenter);
        bat_voltage_[b]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        bat_voltage_[b]->setStyleSheet(QString("color: %1;").arg(theme::Text));
        vl->addWidget(bat_voltage_[b]);
        bold_ws.push_back(bat_voltage_[b]);

        bat_current_[b] = new QLabel("--.- A");
        bat_current_[b]->setFont(mono);
        bat_current_[b]->setAlignment(Qt::AlignCenter);
        bat_current_[b]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        bat_current_[b]->setStyleSheet(QString("color: %1;").arg(theme::Text));
        vl->addWidget(bat_current_[b]);
        normal_ws.push_back(bat_current_[b]);

        bat_power_[b] = new QLabel("---- W");
        bat_power_[b]->setFont(mono);
        bat_power_[b]->setAlignment(Qt::AlignCenter);
        bat_power_[b]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        bat_power_[b]->setStyleSheet(QString("color: %1;").arg(theme::TextDim));
        vl->addWidget(bat_power_[b]);
        normal_ws.push_back(bat_power_[b]);

        bat_indicator_[b] = new QLabel("●");
        bat_indicator_[b]->setFont(monoBold);
        bat_indicator_[b]->setAlignment(Qt::AlignCenter);
        bat_indicator_[b]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        bat_indicator_[b]->setStyleSheet(QString("color: %1;").arg(theme::TextDim));
        vl->addWidget(bat_indicator_[b]);
        bold_ws.push_back(bat_indicator_[b]);

        grid->addWidget(cell, 1, b);
        (void)BAT_LABELS[b];
    }

    // Total column
    auto* total_cell = new QWidget();
    total_cell->setStyleSheet(
        QString("background: %1; border: 1px solid %2; border-radius: 4px;")
        .arg(theme::Bg).arg(theme::BorderDim));
    total_cell->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* total_vl = new QVBoxLayout(total_cell);
    total_vl->setSpacing(2);
    total_vl->setContentsMargins(6, 6, 6, 6);

    // We'll reuse bat_voltage_[0] slot for total (stored separately)
    auto* total_v_lbl = new QLabel("TOTAL");
    total_v_lbl->setFont(monoBold);
    total_v_lbl->setAlignment(Qt::AlignCenter);
    total_v_lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    total_v_lbl->setStyleSheet(QString("color: %1;").arg(theme::Cyan));
    total_vl->addWidget(total_v_lbl);
    bold_ws.push_back(total_v_lbl);

    // Store total labels using a member — we'll use status for total voltage display
    // We allocate extra labels for total — stored in status_lbl_ subtitles
    // Simple approach: use QLabel pointers stored locally and updated via lambda capture
    // We'll store the total widgets in fixed positions inside the grid as named members
    // For simplicity, embed total voltage/current/power as three labels
    auto* tot_v = new QLabel("--.- V");
    tot_v->setFont(monoBig);
    tot_v->setAlignment(Qt::AlignCenter);
    tot_v->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    tot_v->setStyleSheet(QString("color: %1;").arg(theme::Cyan));
    total_vl->addWidget(tot_v);
    bold_ws.push_back(tot_v);

    auto* tot_a = new QLabel("--.- A");
    tot_a->setFont(mono);
    tot_a->setAlignment(Qt::AlignCenter);
    tot_a->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    tot_a->setStyleSheet(QString("color: %1;").arg(theme::Text));
    total_vl->addWidget(tot_a);
    normal_ws.push_back(tot_a);

    auto* tot_w = new QLabel("---- W");
    tot_w->setFont(mono);
    tot_w->setAlignment(Qt::AlignCenter);
    tot_w->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    tot_w->setStyleSheet(QString("color: %1;").arg(theme::TextDim));
    total_vl->addWidget(tot_w);
    normal_ws.push_back(tot_w);

    grid->addWidget(total_cell, 1, 3);

    // Store total labels by tagging them with object names so onPowerStatus finds them
    tot_v->setObjectName("total_v");
    tot_a->setObjectName("total_a");
    tot_w->setObjectName("total_w");

    // ── Row 2: Module power bars (ARM / CHASSIS / SCIENCE) ───────────────────
    static const char* MOD_NAMES[] = { "ARM", "CHASSIS", "SCIENCE" };
    auto* mod_row_widget = new QWidget();
    mod_row_widget->setStyleSheet(QString("background: %1;").arg(theme::Bg));
    mod_row_widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* mod_layout = new QHBoxLayout(mod_row_widget);
    mod_layout->setSpacing(4);
    mod_layout->setContentsMargins(2, 2, 2, 2);

    for (int m = 0; m < 3; m++) {
        auto* panel = new QWidget();
        panel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        panel->setStyleSheet(
            QString("background: %1; border: 1px solid %2; border-radius: 4px;")
            .arg(theme::Bg).arg(theme::BorderDim));
        auto* pl = new QVBoxLayout(panel);
        pl->setSpacing(2);
        pl->setContentsMargins(4, 4, 4, 4);

        auto* name_lbl = new QLabel(MOD_NAMES[m]);
        name_lbl->setFont(monoBold);
        name_lbl->setAlignment(Qt::AlignCenter);
        name_lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        name_lbl->setStyleSheet(QString("color: %1;").arg(theme::Cyan));
        pl->addWidget(name_lbl);
        bold_ws.push_back(name_lbl);

        mod_power_[m] = new QLabel("---- W");
        mod_power_[m]->setFont(mono);
        mod_power_[m]->setAlignment(Qt::AlignCenter);
        mod_power_[m]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        mod_power_[m]->setStyleSheet(QString("color: %1;").arg(theme::Text));
        pl->addWidget(mod_power_[m]);
        normal_ws.push_back(mod_power_[m]);

        mod_bar_[m] = new QProgressBar();
        mod_bar_[m]->setRange(0, 7200);
        mod_bar_[m]->setValue(0);
        mod_bar_[m]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        mod_bar_[m]->setTextVisible(false);
        mod_bar_[m]->setStyleSheet(QString(
            "QProgressBar { background: %1; border: 1px solid %2; border-radius: 3px; }"
            "QProgressBar::chunk { background: %3; border-radius: 2px; }")
            .arg(theme::BgPanel).arg(theme::BorderDim).arg(theme::Green));
        pl->addWidget(mod_bar_[m]);

        mod_btn_[m] = new QPushButton("ON");
        mod_btn_[m]->setFont(monoBold);
        mod_btn_[m]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        mod_btn_[m]->setStyleSheet(
            QString("background: #1b3d2a; color: %1; border: 1px solid %1;"
                    " border-radius: 4px; padding: 4px;").arg(theme::Green));
        pl->addWidget(mod_btn_[m]);
        bold_ws.push_back(mod_btn_[m]);

        int idx = m;
        QObject::connect(mod_btn_[m], &QPushButton::clicked, [this, idx]() {
            if (!state_received_) return;
            if (idx == 0) last_state_.arm_enabled     = !last_state_.arm_enabled;
            if (idx == 1) last_state_.chassis_enabled  = !last_state_.chassis_enabled;
            if (idx == 2) last_state_.science_enabled  = !last_state_.science_enabled;
            publishModuleEnable(last_state_);
        });

        mod_layout->addWidget(panel);
    }

    grid->addWidget(mod_row_widget, 2, 0, 1, 4);

    // ── Row 3: Record button ──────────────────────────────────────────────────
    record_btn_ = new QPushButton("\u23fa Record Battery Current");
    record_btn_->setFont(monoBold);
    record_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    record_btn_->setStyleSheet(
        QString("background: %1; color: %2; border: 1px solid %3;"
                " border-radius: 6px; padding: 8px;")
        .arg(theme::BgPanel).arg(theme::Text).arg(theme::BorderDim));
    QObject::connect(record_btn_, &QPushButton::clicked, [this]() {
        toggleRecording();
    });
    grid->addWidget(record_btn_, 3, 0, 1, 4);
    bold_ws.push_back(record_btn_);

    // ── Row 4: Status bar ─────────────────────────────────────────────────────
    status_lbl_ = new QLabel("Waiting for power data...");
    status_lbl_->setFont(mono);
    status_lbl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    status_lbl_->setStyleSheet(
        QString("background: %1; color: %2; padding: 4px 6px; border: 1px solid %3;")
        .arg(theme::Bg).arg(theme::TextDim).arg(theme::BorderDim));
    grid->addWidget(status_lbl_, 4, 0, 1, 4);
    normal_ws.push_back(status_lbl_);

    // Equal column/row stretch
    for (int c = 0; c < 4; c++) grid->setColumnStretch(c, 1);
    for (int r = 0; r <= 4; r++) grid->setRowStretch(r, 1);

    // Font scaler (5 rows total: header, battery, modules, record, status)
    auto* scaler = new ResizeScaler(
        scroll, 5, std::move(normal_ws), std::move(bold_ws));
    scroll->installEventFilter(scaler);

    scroll->setWidget(widget);
    return scroll;
}

void PowerSummaryModule::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    auto qos = rclcpp::QoS(10).reliable();
    sub_ = node->create_subscription<rover_msgs::msg::PowerStatus>(
        "/power/status", qos,
        [this](const rover_msgs::msg::PowerStatus::SharedPtr msg) {
            onPowerStatus(msg);
        });
    pub_ = node->create_publisher<rover_msgs::msg::PowerStatus>(
        "/power/module_enable", qos);
}

void PowerSummaryModule::stop() {
    if (recording_) toggleRecording();
}

void PowerSummaryModule::onPowerStatus(const rover_msgs::msg::PowerStatus::SharedPtr msg) {
    last_state_     = *msg;
    state_received_ = true;

    for (int b = 0; b < 3; b++) {
        float v = msg->battery_voltage_v[b];
        float a = msg->battery_current_a[b];
        float w = msg->battery_power_w[b];
        bool  c = msg->battery_connected[b];

        // Voltage color
        QString vcol;
        if (v > 22.0f)      vcol = theme::Green;
        else if (v > 20.0f) vcol = theme::Yellow;
        else                 vcol = theme::Red;

        if (bat_voltage_[b]) {
            bat_voltage_[b]->setText(QString("%1 V").arg(v, 0, 'f', 1));
            bat_voltage_[b]->setStyleSheet(
                QString("color: %1; font-weight: bold;").arg(vcol));
        }
        if (bat_current_[b])
            bat_current_[b]->setText(QString("%1 A").arg(a, 0, 'f', 1));
        if (bat_power_[b])
            bat_power_[b]->setText(QString("%1 W").arg(w, 0, 'f', 0));
        if (bat_indicator_[b]) {
            bat_indicator_[b]->setText(c ? "\u25cf" : "\u25cf");
            bat_indicator_[b]->setStyleSheet(
                QString("color: %1; font-weight: bold;")
                .arg(c ? theme::Green : theme::Red));
        }
    }

    // Total labels (found by objectName)
    if (auto* w = qobject_cast<QLabel*>(
            bat_voltage_[0] ? bat_voltage_[0]->window()->findChild<QLabel*>("total_v") : nullptr)) {
        (void)w;
    }
    // Walk the widget tree for total labels
    {
        auto findLabel = [&](QWidget* root, const QString& name) -> QLabel* {
            if (!root) return nullptr;
            return root->findChild<QLabel*>(name);
        };
        QWidget* root = bat_voltage_[0] ? bat_voltage_[0]->window() : nullptr;
        if (root) {
            if (auto* tv = findLabel(root, "total_v"))
                tv->setText(QString("%1 V").arg(msg->total_voltage_v, 0, 'f', 1));
            if (auto* ta = findLabel(root, "total_a"))
                ta->setText(QString("%1 A").arg(msg->total_current_a, 0, 'f', 1));
            if (auto* tw = findLabel(root, "total_w"))
                tw->setText(QString("%1 W").arg(msg->total_power_w, 0, 'f', 0));
        }
    }

    // Module power
    float pows[3] = { msg->arm_power_w, msg->chassis_power_w, msg->science_power_w };
    bool  ens[3]  = { msg->arm_enabled,  msg->chassis_enabled,  msg->science_enabled };
    for (int m = 0; m < 3; m++) {
        if (mod_power_[m]) mod_power_[m]->setText(QString("%1 W").arg(pows[m], 0, 'f', 0));
        if (mod_bar_[m])   mod_bar_[m]->setValue(static_cast<int>(pows[m]));
        if (mod_btn_[m]) {
            if (ens[m]) {
                mod_btn_[m]->setText("ON");
                mod_btn_[m]->setStyleSheet(
                    QString("background: #1b3d2a; color: %1; border: 1px solid %1;"
                            " border-radius: 4px; padding: 4px; font-weight: bold;")
                    .arg(theme::Green));
            } else {
                mod_btn_[m]->setText("OFF");
                mod_btn_[m]->setStyleSheet(
                    QString("background: %1; color: %2; border: 1px solid %2;"
                            " border-radius: 4px; padding: 4px; font-weight: bold;")
                    .arg(theme::BgPanel).arg(theme::TextDim));
            }
        }
    }

    if (status_lbl_)
        status_lbl_->setText(
            QString("Live  \xc2\xb7  Total: %1 V / %2 A / %3 W")
            .arg(msg->total_voltage_v, 0, 'f', 1)
            .arg(msg->total_current_a, 0, 'f', 1)
            .arg(msg->total_power_w, 0, 'f', 0));

    // CSV recording
    if (recording_ && csv_stream_) {
        QString ts = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz");
        *csv_stream_ << ts
                     << "," << msg->battery_current_a[0]
                     << "," << msg->battery_current_a[1]
                     << "," << msg->battery_current_a[2]
                     << "," << msg->total_current_a
                     << "\n";
        csv_stream_->flush();
    }
}

void PowerSummaryModule::publishModuleEnable(const rover_msgs::msg::PowerStatus& state) {
    if (pub_) pub_->publish(state);
}

void PowerSummaryModule::toggleRecording() {
    if (!recording_) {
        QString fname = QDir::homePath() + "/battery_log_"
            + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".csv";
        csv_file_   = new QFile(fname);
        if (csv_file_->open(QIODevice::WriteOnly | QIODevice::Text)) {
            csv_stream_ = new QTextStream(csv_file_);
            *csv_stream_ << "timestamp,bat1_current_a,bat2_current_a,bat3_current_a,total_current_a\n";
            recording_ = true;
            if (record_btn_) {
                record_btn_->setText("\u23f9 Recording\u2026");
                record_btn_->setStyleSheet(
                    QString("background: #3d1b1b; color: %1; border: 1px solid %1;"
                            " border-radius: 6px; padding: 8px; font-weight: bold;")
                    .arg(theme::Red));
            }
        } else {
            delete csv_file_;
            csv_file_ = nullptr;
        }
    } else {
        recording_ = false;
        if (csv_stream_) { delete csv_stream_; csv_stream_ = nullptr; }
        if (csv_file_)   { csv_file_->close(); delete csv_file_; csv_file_ = nullptr; }
        if (record_btn_) {
            record_btn_->setText("\u23fa Record Battery Current");
            record_btn_->setStyleSheet(
                QString("background: %1; color: %2; border: 1px solid %3;"
                        " border-radius: 6px; padding: 8px;")
                .arg(theme::BgPanel).arg(theme::Text).arg(theme::BorderDim));
        }
    }
}

PLUGINLIB_EXPORT_CLASS(PowerSummaryModule, rover_hmi_core::GuiModule)
