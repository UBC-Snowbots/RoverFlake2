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
// VirtualJoystick defined in drive_module.cpp (Q_OBJECT in .cpp avoids
// AUTOMOC header-scanning issues with private include directories).
class VirtualJoystick;


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
