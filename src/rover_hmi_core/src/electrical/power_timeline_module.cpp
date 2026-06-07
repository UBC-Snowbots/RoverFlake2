// power_timeline_module.cpp — "Power Timeline"
//
// Series (index):
//   0 = Bat1 voltage (18–26 V)
//   1 = Bat2 voltage (18–26 V)
//   2 = Bat3 voltage (18–26 V)
//   3 = Total current (0–300 A)
//   4 = Total power  (0–7200 W)
//
// Each series has its own Y-axis range so they can all be displayed overlaid
// on a [0,1] normalised canvas, then drawn with distinct colors.

#include "power_timeline_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFont>
#include <QSizePolicy>
#include <QFontMetrics>
#include <QColor>
#include <cmath>
#include <algorithm>
#include <deque>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

// ===========================================================================
// TimelineCanvas — defined here so Q_OBJECT is in the .cpp (avoids AUTOMOC
// header-scanning issues with private include directories).
// ===========================================================================
class TimelineCanvas : public QWidget {
    Q_OBJECT
public:
    static constexpr int MAX_SAMPLES = 120;

    struct Series {
        QString label;
        QString color;
        std::deque<float> samples;
        float y_min;
        float y_max;
        bool  visible = true;
    };

    explicit TimelineCanvas(QWidget* parent = nullptr);

    void pushSample(int series_idx, float value);
    void setSeriesVisible(int idx, bool v);
    bool seriesVisible(int idx) const;
    int  numSeries() const { return static_cast<int>(series_.size()); }
    QString seriesLabel(int idx) const;
    QString seriesColor(int idx) const;
    float   seriesLatest(int idx) const;

protected:
    void paintEvent(QPaintEvent* ev) override;

private:
    std::vector<Series> series_;
};

// ===========================================================================
// TimelineCanvas implementation
// ===========================================================================

TimelineCanvas::TimelineCanvas(QWidget* parent)
    : QWidget(parent)
{
    setStyleSheet(QString("background: %1;").arg(theme::Bg));
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // Series definitions
    series_ = {
        { "Bat1 V",  theme::MotorColors[0],  {}, 18.0f, 26.0f, true },
        { "Bat2 V",  theme::MotorColors[1],  {}, 18.0f, 26.0f, true },
        { "Bat3 V",  theme::MotorColors[2],  {}, 18.0f, 26.0f, true },
        { "Total A", theme::Cyan,             {}, 0.0f,  300.0f, true },
        { "Total kW",theme::Yellow,           {}, 0.0f,  7200.0f, true },
    };
}

void TimelineCanvas::pushSample(int series_idx, float value) {
    if (series_idx < 0 || series_idx >= (int)series_.size()) return;
    auto& s = series_[series_idx];
    s.samples.push_back(value);
    while ((int)s.samples.size() > MAX_SAMPLES)
        s.samples.pop_front();
}

void TimelineCanvas::setSeriesVisible(int idx, bool v) {
    if (idx >= 0 && idx < (int)series_.size())
        series_[idx].visible = v;
}

bool TimelineCanvas::seriesVisible(int idx) const {
    if (idx < 0 || idx >= (int)series_.size()) return false;
    return series_[idx].visible;
}

QString TimelineCanvas::seriesLabel(int idx) const {
    if (idx < 0 || idx >= (int)series_.size()) return {};
    return series_[idx].label;
}

QString TimelineCanvas::seriesColor(int idx) const {
    if (idx < 0 || idx >= (int)series_.size()) return "#ffffff";
    return series_[idx].color;
}

float TimelineCanvas::seriesLatest(int idx) const {
    if (idx < 0 || idx >= (int)series_.size()) return 0.0f;
    const auto& s = series_[idx];
    if (s.samples.empty()) return 0.0f;
    return s.samples.back();
}

void TimelineCanvas::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const int W = width();
    const int H = height();
    const int pad_l = 50, pad_r = 170, pad_t = 20, pad_b = 30;

    p.fillRect(0, 0, W, H, QColor(theme::Bg));

    const int cx = W - pad_r - pad_l;
    const int cy = H - pad_t - pad_b;

    if (cx <= 0 || cy <= 0) return;

    // Grid lines
    p.setPen(QPen(QColor(theme::BorderDim), 1, Qt::DotLine));
    const int num_h_lines = 5;
    for (int i = 0; i <= num_h_lines; i++) {
        int y = pad_t + cy * i / num_h_lines;
        p.drawLine(pad_l, y, pad_l + cx, y);
    }
    const int num_v_lines = 6;
    for (int i = 0; i <= num_v_lines; i++) {
        int x = pad_l + cx * i / num_v_lines;
        p.drawLine(x, pad_t, x, pad_t + cy);
    }

    // Axes
    p.setPen(QPen(QColor(theme::BorderDim), 1));
    p.drawRect(pad_l, pad_t, cx, cy);

    // X-axis label
    QFont smFont("monospace", theme::FontSizeSm);
    p.setFont(smFont);
    p.setPen(QColor(theme::TextDim));
    p.drawText(pad_l, H - 4, "older");
    p.drawText(pad_l + cx - 40, H - 4, "now");

    // Draw each series
    for (const auto& s : series_) {
        if (!s.visible || s.samples.empty()) continue;
        QColor col(s.color);
        p.setPen(QPen(col, 2));

        float range = s.y_max - s.y_min;
        if (range < 1e-6f) range = 1.0f;

        int n = (int)s.samples.size();
        QPointF prev;
        bool has_prev = false;

        for (int i = 0; i < n; i++) {
            float norm_x = (float)i / (float)(MAX_SAMPLES - 1);
            float norm_y = (s.samples[i] - s.y_min) / range;
            norm_y = std::max(0.0f, std::min(1.0f, norm_y));

            float px = pad_l + norm_x * cx;
            float py = pad_t + cy - norm_y * cy;

            QPointF pt(px, py);
            if (has_prev)
                p.drawLine(prev, pt);
            prev     = pt;
            has_prev = true;
        }
    }

    // Legend (top-right)
    int leg_x = pad_l + cx + 8;
    int leg_y = pad_t + 4;
    int leg_h = 18;
    QFont legFont("monospace", theme::FontSizeSm);
    p.setFont(legFont);

    for (const auto& s : series_) {
        QColor col(s.color);
        p.fillRect(leg_x, leg_y, 12, 12, col);
        p.setPen(QColor(s.visible ? theme::Text : theme::TextDim));

        QString cur_val;
        if (!s.samples.empty()) {
            float v = s.samples.back();
            // Format based on range
            if (s.y_max <= 30.0f)
                cur_val = QString::number(v, 'f', 1);
            else if (s.y_max <= 350.0f)
                cur_val = QString::number(v, 'f', 0) + " A";
            else
                cur_val = QString::number(v / 1000.0f, 'f', 2) + " kW";
        } else {
            cur_val = "--";
        }

        p.drawText(leg_x + 16, leg_y + 12,
                   QString("%1  %2").arg(s.label).arg(cur_val));
        leg_y += leg_h;
    }
}

// ===========================================================================
// PowerTimelineModule
// ===========================================================================

QWidget* PowerTimelineModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    widget->setStyleSheet(QString("background: %1;").arg(theme::Bg));

    auto* vl = new QVBoxLayout(widget);
    vl->setSpacing(4);
    vl->setContentsMargins(4, 4, 4, 4);

    QFont mono("monospace", theme::FontSizeSm, QFont::Bold);

    // Series toggle buttons
    auto* btn_row = new QWidget();
    btn_row->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    auto* hl = new QHBoxLayout(btn_row);
    hl->setSpacing(4);
    hl->setContentsMargins(0, 0, 0, 0);

    canvas_ = new TimelineCanvas();

    for (int i = 0; i < 5; i++) {
        tog_btns_[i] = new QPushButton(canvas_->seriesLabel(i));
        tog_btns_[i]->setFont(mono);
        tog_btns_[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        QString col = canvas_->seriesColor(i);
        tog_btns_[i]->setStyleSheet(
            QString("background: %1; color: %2; border: 1px solid %2;"
                    " border-radius: 4px; padding: 4px 8px;")
            .arg(theme::BgPanel).arg(col));

        int idx = i;
        QObject::connect(tog_btns_[i], &QPushButton::clicked, [this, idx]() {
            bool now = !canvas_->seriesVisible(idx);
            canvas_->setSeriesVisible(idx, now);
            QString col = canvas_->seriesColor(idx);
            if (now) {
                tog_btns_[idx]->setStyleSheet(
                    QString("background: %1; color: %2; border: 1px solid %2;"
                            " border-radius: 4px; padding: 4px 8px;")
                    .arg(theme::BgPanel).arg(col));
            } else {
                tog_btns_[idx]->setStyleSheet(
                    QString("background: %1; color: %2; border: 1px solid %2;"
                            " border-radius: 4px; padding: 4px 8px;")
                    .arg(theme::BgPanel).arg(theme::TextDim));
            }
        });
        hl->addWidget(tog_btns_[i]);
    }

    vl->addWidget(btn_row);
    vl->addWidget(canvas_, 1);

    timer_ = new QTimer(widget);
    timer_->setInterval(100);
    QObject::connect(timer_, &QTimer::timeout, [this]() {
        if (canvas_) canvas_->update();
    });

    return widget;
}

void PowerTimelineModule::setNode(rclcpp::Node::SharedPtr node) {
    auto qos = rclcpp::QoS(10).reliable();
    sub_ = node->create_subscription<rover_msgs::msg::PowerStatus>(
        "/power/status", qos,
        [this](const rover_msgs::msg::PowerStatus::SharedPtr msg) {
            onPowerStatus(msg);
        });
}

void PowerTimelineModule::start() {
    if (timer_) timer_->start();
}

void PowerTimelineModule::stop() {
    if (timer_) timer_->stop();
}

void PowerTimelineModule::onPowerStatus(const rover_msgs::msg::PowerStatus::SharedPtr msg) {
    if (!canvas_) return;
    canvas_->pushSample(0, msg->battery_voltage_v[0]);
    canvas_->pushSample(1, msg->battery_voltage_v[1]);
    canvas_->pushSample(2, msg->battery_voltage_v[2]);
    canvas_->pushSample(3, msg->total_current_a);
    canvas_->pushSample(4, msg->total_power_w);
}

PLUGINLIB_EXPORT_CLASS(PowerTimelineModule, rover_hmi_core::GuiModule)
#include "power_timeline_module.moc"
