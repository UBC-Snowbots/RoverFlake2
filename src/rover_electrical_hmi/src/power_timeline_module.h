// power_timeline_module.h — "Power Timeline"
//
// Rolling canvas showing up to 120 samples of battery voltages,
// total current, and total power over ~60 seconds.
// Section: Electricals. defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QWidget>
#include <QPushButton>
#include <QTimer>
#include <array>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/power_status.hpp"

// ---------------------------------------------------------------------------
// TimelineCanvas — custom widget that paints the rolling series graph
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// PowerTimelineModule
// ---------------------------------------------------------------------------
class PowerTimelineModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Power Timeline"; }
    std::string sectionName() const override { return "Electricals"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override;
    void     stop() override;

private:
    void onPowerStatus(const rover_msgs::msg::PowerStatus::SharedPtr msg);

    TimelineCanvas*              canvas_   = nullptr;
    QTimer*                      timer_    = nullptr;
    std::array<QPushButton*, 5>  tog_btns_ = {};

    rclcpp::Subscription<rover_msgs::msg::PowerStatus>::SharedPtr sub_;
};
