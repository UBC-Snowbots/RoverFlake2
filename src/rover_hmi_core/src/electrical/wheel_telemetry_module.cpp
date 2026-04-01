// wheel_telemetry_module.cpp — "Wheel Telemetry"
//
// Table: header + 6 wheel rows + status row.
// Temp color: green <50, yellow <70, red >=70.
// Row background dim when wheel disabled.

#include "wheel_telemetry_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QGridLayout>
#include <QScrollArea>
#include <QFont>
#include <QSizePolicy>
#include <QEvent>
#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

// ---------------------------------------------------------------------------
// TableScaleFilter
// ---------------------------------------------------------------------------
class WheelTableScaleFilter : public QObject {
public:
    WheelTableScaleFilter(QObject* parent, int num_rows,
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

static const char* WHEEL_NAMES[] = { "FL", "FR", "ML", "MR", "RL", "RR" };
static const char* COL_HEADERS[] = { "Speed (rpm)", "Torque (Nm)", "Temp (\xc2\xb0""C)", "Power (W)", "Enabled" };

QWidget* WheelTelemetryModule::createWidget(QWidget* parent) {
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
    grid->setSpacing(1);
    grid->setContentsMargins(2, 2, 2, 2);

    QFont mono("monospace", theme::FontSize);
    QFont monoBold("monospace", theme::FontSize, QFont::Bold);

    auto headerStyle = QString(
        "background: %1; color: %2; padding: 4px 6px; border-bottom: 1px solid %3; font-weight: bold;")
        .arg(theme::HeaderBg).arg(theme::Text).arg(theme::BorderDim);
    auto cellStyle = QString(
        "background: %1; color: %2; padding: 4px 6px; border: 1px solid %3;")
        .arg(theme::Bg).arg(theme::Text).arg(theme::BorderDim);

    std::vector<QWidget*> bold_ws, normal_ws;

    // Corner
    auto* corner = new QLabel("Wheel");
    corner->setFont(monoBold);
    corner->setStyleSheet(headerStyle);
    corner->setAlignment(Qt::AlignCenter);
    corner->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    grid->addWidget(corner, 0, 0);
    bold_ws.push_back(corner);

    // Column headers
    for (int c = 0; c < NUM_COLS; c++) {
        auto* lbl = new QLabel(COL_HEADERS[c]);
        lbl->setFont(monoBold);
        lbl->setStyleSheet(headerStyle);
        lbl->setAlignment(Qt::AlignCenter);
        lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        grid->addWidget(lbl, 0, c + 1);
        bold_ws.push_back(lbl);
    }

    // Wheel rows
    for (int r = 0; r < NUM_WHEELS; r++) {
        row_labels_[r] = new QLabel(WHEEL_NAMES[r]);
        row_labels_[r]->setFont(monoBold);
        row_labels_[r]->setStyleSheet(
            QString("background: %1; color: %2; padding: 4px 6px; border: 1px solid %3; font-weight: bold;")
            .arg(theme::Bg).arg(theme::MotorColors[r]).arg(theme::BorderDim));
        row_labels_[r]->setAlignment(Qt::AlignCenter);
        row_labels_[r]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        grid->addWidget(row_labels_[r], r + 1, 0);
        bold_ws.push_back(row_labels_[r]);

        for (int c = 0; c < NUM_COLS; c++) {
            auto* lbl = new QLabel("--");
            lbl->setFont(mono);
            lbl->setStyleSheet(cellStyle);
            lbl->setAlignment(Qt::AlignCenter);
            lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
            grid->addWidget(lbl, r + 1, c + 1);
            cells_[r][c] = lbl;
            normal_ws.push_back(lbl);
        }
    }

    // Status row
    status_ = new QLabel("Waiting for wheel data...");
    status_->setFont(mono);
    status_->setStyleSheet(
        QString("background: %1; color: %2; padding: 4px 6px;")
        .arg(theme::Bg).arg(theme::TextDim));
    status_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    grid->addWidget(status_, NUM_WHEELS + 1, 0, 1, NUM_COLS + 1);
    normal_ws.push_back(status_);

    // Equal stretches
    for (int c = 0; c <= NUM_COLS; c++) grid->setColumnStretch(c, 1);
    for (int r = 0; r <= NUM_WHEELS + 1; r++) grid->setRowStretch(r, 1);

    auto* scaler = new WheelTableScaleFilter(
        scroll, NUM_WHEELS + 2, std::move(normal_ws), std::move(bold_ws));
    scroll->installEventFilter(scaler);

    scroll->setWidget(widget);
    return scroll;
}

void WheelTelemetryModule::setNode(rclcpp::Node::SharedPtr node) {
    auto qos = rclcpp::QoS(10).reliable();
    sub_ = node->create_subscription<rover_msgs::msg::WheelStates>(
        "/drivetrain/wheel_states", qos,
        [this](const rover_msgs::msg::WheelStates::SharedPtr msg) {
            onWheelStates(msg);
        });
}

void WheelTelemetryModule::onWheelStates(const rover_msgs::msg::WheelStates::SharedPtr msg) {
    if (!cells_[0][0]) return;

    for (int i = 0; i < NUM_WHEELS; i++) {
        bool enabled = msg->enabled[i];
        float temp   = msg->temperature_c[i];

        QString rowBg = enabled ? theme::Bg : "#111111";
        QString rowFg = enabled ? theme::Text : theme::TextDim;

        auto cs = QString("background: %1; color: %2; padding: 4px 6px; border: 1px solid %3;")
                  .arg(rowBg).arg(rowFg).arg(theme::BorderDim);

        // Speed
        cells_[i][0]->setText(QString("%1").arg(msg->speed_rpm[i], 0, 'f', 1));
        cells_[i][0]->setStyleSheet(cs);

        // Torque
        cells_[i][1]->setText(QString("%1").arg(msg->torque_nm[i], 0, 'f', 2));
        cells_[i][1]->setStyleSheet(cs);

        // Temp (color coded)
        cells_[i][2]->setText(QString("%1").arg(temp, 0, 'f', 1));
        QString tempCol;
        if (temp >= 70.0f)      tempCol = theme::Red;
        else if (temp >= 50.0f) tempCol = theme::Yellow;
        else                    tempCol = theme::Green;
        cells_[i][2]->setStyleSheet(
            QString("background: %1; color: %2; padding: 4px 6px; border: 1px solid %3;")
            .arg(rowBg).arg(tempCol).arg(theme::BorderDim));

        // Power
        cells_[i][3]->setText(QString("%1").arg(msg->power_w[i], 0, 'f', 0));
        cells_[i][3]->setStyleSheet(cs);

        // Enabled indicator
        cells_[i][4]->setText(enabled ? "\u25cf ON" : "\u25cf OFF");
        cells_[i][4]->setStyleSheet(
            QString("background: %1; color: %2; padding: 4px 6px; border: 1px solid %3; font-weight: bold;")
            .arg(rowBg).arg(enabled ? theme::Green : theme::Red).arg(theme::BorderDim));

        // Row label background update
        if (row_labels_[i]) {
            row_labels_[i]->setStyleSheet(
                QString("background: %1; color: %2; padding: 4px 6px; border: 1px solid %3; font-weight: bold;")
                .arg(rowBg).arg(theme::MotorColors[i]).arg(theme::BorderDim));
        }
    }

    if (status_)
        status_->setText(QString("Live  \xc2\xb7  Wheel states received"));
}

PLUGINLIB_EXPORT_CLASS(WheelTelemetryModule, rover_hmi_core::GuiModule)
