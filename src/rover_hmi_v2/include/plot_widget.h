#pragma once

#include <QWidget>
#include <deque>
#include <vector>
#include <string>
#include <utility>

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
