// camera_viewport.h — single-feed display: zero-copy QImage wrap of the ROS
// buffer in paintEvent, FNAF-style animated static when there is no signal.
#pragma once

#include <QElapsedTimer>
#include <QTimer>
#include <QWidget>

#include "sensor_msgs/msg/image.hpp"

class CameraViewport : public QWidget {
public:
    explicit CameraViewport(QWidget* parent = nullptr);

    void setLabel(const QString& label);
    void setFrame(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void setNoSignal();
    void setError(const QString& msg);
    bool hasFrame() const { return static_cast<bool>(msg_); }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    void drawStatic(QPainter& p);
    void drawFrame(QPainter& p);
    void drawOverlay(QPainter& p);

    sensor_msgs::msg::Image::ConstSharedPtr msg_;  // keeps the buffer alive for painting
    QString label_;
    QString error_;
    QTimer* static_timer_;   // repaints the noise at 10 Hz while signal-less
    QElapsedTimer frame_clock_;
    double fps_ = 0.0;
};
