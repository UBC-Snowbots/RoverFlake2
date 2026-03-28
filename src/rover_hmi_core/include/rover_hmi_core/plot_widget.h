#pragma once

#include <QWidget>
#include <deque>
#include <vector>
#include <string>
#include <utility>

// Scrolling time-series plot widget.
//
// Supports multiple named series, each with its own color and independent
// visibility toggle.  The displayed time window defaults to 30 s and can be
// changed programmatically via setTimeWindow() or interactively with the mouse
// wheel.  The Y axis auto-scales on every paint pass to the range of the
// currently visible, currently visible data — no manual range setting needed.
//
// Each series caps its history at MAX_POINTS samples; the oldest point is
// dropped when the cap is reached, which bounds memory regardless of how long
// the widget runs.
//
// Typical usage:
//   auto* plot = new PlotWidget("Velocity (rad/s)");
//   int s = plot->addSeries("joint_0", "#cba6f7");
//   // ... on each ROS callback:
//   plot->addPoint(s, rclcpp::Clock().now().seconds(), value);
class PlotWidget : public QWidget {
    Q_OBJECT
public:
    explicit PlotWidget(const QString& ylabel = "Value",
                        QWidget* parent = nullptr);

    void addSeries(const std::string& name, const char* color);
    void addPoint(int series, double time, double value);
    void setSeriesVisible(int series, bool visible);
    void setTimeWindow(double seconds);
    void clear();
    void setPaused(bool paused);

protected:
    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    struct Series {
        std::string name;
        QColor color;
        bool visible = true;
        std::deque<std::pair<double, double>> data;  // (time, value)
    };

    static constexpr int MAX_POINTS = 600;

    std::vector<Series> series_;
    double time_window_ = 30.0;
    QString ylabel_;
    bool paused_ = false;
    double pause_time_ = 0.0;
};
