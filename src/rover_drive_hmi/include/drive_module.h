#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QKeyEvent>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// VirtualJoystick
// A click-and-drag joystick widget.  The user can click inside the circle and
// drag to set the output direction, or use WASD/arrow keys.  The knob snaps
// back to center when released.
// ─────────────────────────────────────────────────────────────────────────────
class VirtualJoystick : public QWidget {
    Q_OBJECT
public:
    explicit VirtualJoystick(QWidget* parent = nullptr);

    // Normalised axis values in [-1, 1]
    float axisX() const { return axis_x_; }
    float axisY() const { return axis_y_; }  // positive = forward

    void setAxisFromKey(int dx, int dy);  // incremental nudge via keyboard
    void resetAxes();

signals:
    void axisChanged(float x, float y);

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;

private:
    void updateFromPos(const QPointF& pos);

    float axis_x_ = 0.0f;
    float axis_y_ = 0.0f;
    bool  pressed_ = false;
};


// ─────────────────────────────────────────────────────────────────────────────
// DriveModule — GuiModule plugin for driving the rover
//
// Publishes geometry_msgs/Twist to /cmd_vel at 20 Hz while enabled.
// Linear.x  = forward/backward (joystick Y axis)
// Angular.z = left/right turn  (joystick X axis)
//
// Controls:
//   Virtual joystick — click & drag
//   W/A/S/D or Arrow keys — keyboard driving
//   Speed slider        — adjusts max linear speed (0.1–2.0 m/s)
//   Space               — emergency stop (zeroes velocity)
// ─────────────────────────────────────────────────────────────────────────────
class DriveModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Drive"; }
    std::string layoutHint()  const override { return "main"; }
    std::string sectionName() const override { return "Drive"; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override;
    void     stop()  override;

    std::vector<std::pair<std::string,std::string>> keybindings() const override {
        return {
            { "W / ↑",     "Drive forward"        },
            { "S / ↓",     "Drive backward"       },
            { "A / ←",     "Turn left"            },
            { "D / →",     "Turn right"           },
            { "Space",     "Emergency stop"       },
            { "Click+drag","Virtual joystick"     },
        };
    }

private:
    void publishTwist(float linear_x, float angular_z);
    void onPublishTimer();
    void onKeyStop();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    QTimer* publish_timer_ = nullptr;

    VirtualJoystick* joystick_   = nullptr;
    QLabel*          speed_lbl_  = nullptr;
    QLabel*          status_lbl_ = nullptr;

    float max_linear_  = 0.5f;   // m/s
    float max_angular_ = 1.0f;   // rad/s
    bool  enabled_     = false;
};
