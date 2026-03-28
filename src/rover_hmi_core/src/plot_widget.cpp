#include "rover_hmi_core/plot_widget.h"
#include "rover_hmi_core/catppuccin.h"

#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

PlotWidget::PlotWidget(const QString& ylabel, QWidget* parent)
    : QWidget(parent), ylabel_(ylabel) {
    setMinimumSize(300, 200);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void PlotWidget::addSeries(const std::string& name, const char* color) {
    Series s;
    s.name = name;
    s.color = QColor(color);
    series_.push_back(std::move(s));
}

void PlotWidget::addPoint(int series, double time, double value) {
    if (paused_) return;
    if (series < 0 || series >= (int)series_.size()) return;

    auto& data = series_[series].data;
    data.push_back({time, value});
    while ((int)data.size() > MAX_POINTS)
        data.pop_front();

    update();
}

void PlotWidget::setSeriesVisible(int series, bool visible) {
    if (series >= 0 && series < (int)series_.size()) {
        series_[series].visible = visible;
        update();
    }
}

void PlotWidget::setTimeWindow(double seconds) {
    time_window_ = seconds;
    update();
}

void PlotWidget::clear() {
    for (auto& s : series_)
        s.data.clear();
    update();
}

void PlotWidget::setPaused(bool paused) {
    if (paused && !paused_) {
        double max_t = 0;
        for (const auto& s : series_)
            if (!s.data.empty())
                max_t = std::max(max_t, s.data.back().first);
        pause_time_ = max_t;
    }
    paused_ = paused;
    update();
}

void PlotWidget::wheelEvent(QWheelEvent* event) {
    double delta = event->angleDelta().y() > 0 ? -5.0 : 5.0;
    time_window_ = std::clamp(time_window_ + delta, 5.0, 120.0);
    update();
}

void PlotWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const int W = width();
    const int H = height();
    const int margin_l = 70;
    const int margin_r = 14;
    const int margin_t = 14;
    const int margin_b = 36;

    const int plot_w = W - margin_l - margin_r;
    const int plot_h = H - margin_t - margin_b;

    p.fillRect(rect(), QColor(theme::Bg));

    if (plot_w <= 0 || plot_h <= 0) return;

    double t_max = 0;
    if (paused_) {
        t_max = pause_time_;
    } else {
        for (const auto& s : series_)
            if (!s.data.empty())
                t_max = std::max(t_max, s.data.back().first);
    }
    double t_min = t_max - time_window_;

    double v_min = 1e9, v_max = -1e9;
    bool has_data = false;
    for (const auto& s : series_) {
        if (!s.visible) continue;
        for (const auto& [t, v] : s.data) {
            if (t < t_min) continue;
            v_min = std::min(v_min, v);
            v_max = std::max(v_max, v);
            has_data = true;
        }
    }

    if (!has_data) { v_min = -1.0; v_max = 1.0; }

    double v_range = v_max - v_min;
    if (v_range < 0.001) v_range = 1.0;
    v_min -= v_range * 0.1;
    v_max += v_range * 0.1;

    auto mapX = [&](double t) -> double {
        return margin_l + (t - t_min) / time_window_ * plot_w;
    };
    auto mapY = [&](double v) -> double {
        return margin_t + plot_h - (v - v_min) / (v_max - v_min) * plot_h;
    };

    p.setPen(QPen(QColor("#1a1a1a"), 1));
    int num_grid_h = 5;
    for (int i = 0; i <= num_grid_h; i++) {
        double y = margin_t + plot_h * i / num_grid_h;
        p.drawLine(QPointF(margin_l, y), QPointF(W - margin_r, y));
    }
    int num_grid_v = 6;
    for (int i = 0; i <= num_grid_v; i++) {
        double x = margin_l + plot_w * i / num_grid_v;
        p.drawLine(QPointF(x, margin_t), QPointF(x, margin_t + plot_h));
    }

    p.setPen(QPen(QColor(theme::BorderDim), 1));
    p.drawRect(margin_l, margin_t, plot_w, plot_h);

    QFont font("monospace", theme::FontSize - 3);
    p.setFont(font);
    p.setPen(QColor(theme::TextDim));

    for (int i = 0; i <= num_grid_h; i++) {
        double v = v_max - (v_max - v_min) * i / num_grid_h;
        double y = margin_t + plot_h * i / num_grid_h;
        p.drawText(QRectF(0, y - 10, margin_l - 6, 20),
                   Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(v, 'f', 2));
    }

    for (int i = 0; i <= num_grid_v; i++) {
        double t = t_min + time_window_ * i / num_grid_v;
        double x = margin_l + plot_w * i / num_grid_v;
        p.drawText(QRectF(x - 30, margin_t + plot_h + 6, 60, 24),
                   Qt::AlignCenter, QString::number(t, 'f', 1) + "s");
    }

    p.save();
    p.translate(14, margin_t + plot_h / 2);
    p.rotate(-90);
    p.drawText(QRectF(-60, -10, 120, 20), Qt::AlignCenter, ylabel_);
    p.restore();

    p.setClipRect(margin_l, margin_t, plot_w, plot_h);
    for (const auto& s : series_) {
        if (!s.visible || s.data.empty()) continue;

        QPainterPath path;
        bool started = false;

        for (const auto& [t, v] : s.data) {
            if (t < t_min) continue;
            double x = mapX(t);
            double y = mapY(v);
            if (!started) {
                path.moveTo(x, y);
                started = true;
            } else {
                path.lineTo(x, y);
            }
        }

        p.setPen(QPen(s.color, 2));
        p.setBrush(Qt::NoBrush);
        p.drawPath(path);
    }
}
