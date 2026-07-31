// lighting_module.h — "Lighting"
//
// 5 LED boards: front-left(0), front-right(1), left(2), right(3), back(4).
// Publishes the full desired-brightness array (5 floats, 0-100) as
// std_msgs/Float64MultiArray to /lights/cmd; subscribes /lights/feedback
// for hardware-confirmed brightness. The front pair is ganged behind one
// control cluster. Section: Electricals. defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QWidget>

#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class QTimer;

// Top-down rover outline with five light glyphs whose glow intensity tracks
// hardware-confirmed brightness. Renders dim outlines until feedback arrives.
class RoverLightingView : public QWidget {
public:
    explicit RoverLightingView(QWidget* parent = nullptr);
    void setPercents(const std::array<double, 5>& percents, bool have_feedback);

protected:
    void paintEvent(QPaintEvent*) override;

private:
    std::array<double, 5> percents_{};
    bool have_feedback_ = false;
};

class LightingModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Lighting"; }
    std::string sectionName() const override { return "Electricals"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override {}
    void     stop() override {}

private:
    // Wire indices — must match firmware led_panel_index assignment.
    static constexpr int BOARD_FRONT_LEFT  = 0;
    static constexpr int BOARD_FRONT_RIGHT = 1;
    static constexpr int BOARD_LEFT        = 2;
    static constexpr int BOARD_RIGHT       = 3;
    static constexpr int BOARD_BACK        = 4;
    static constexpr int NUM_BOARDS        = 5;

    // UI clusters: 0=Front (gangs boards 0+1), 1=Left, 2=Right, 3=Back.
    static constexpr int NUM_CLUSTERS = 4;
    static const int CLUSTER_BOARDS[NUM_CLUSTERS][2];  // second entry -1 if none

    QWidget* makeCluster(int cluster, const char* title, const char* accent);
    void     applyToggleStyle(int cluster, bool on);
    void     setClusterValue(int cluster, double pct);  // writes desired_ + publishes
    void     publishCmd();
    void     onFeedback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    double desired_[NUM_BOARDS] = {};                     // last commanded, 0-100
    double remembered_[NUM_CLUSTERS] = {50, 50, 50, 50};  // restored on toggle-ON
    bool   cluster_on_[NUM_CLUSTERS] = {};

    QPushButton* toggle_btns_[NUM_CLUSTERS] = {};
    QSlider*     sliders_[NUM_CLUSTERS]     = {};
    QLabel*      value_lbls_[NUM_CLUSTERS]  = {};
    QSlider*     master_slider_             = nullptr;
    QLabel*      master_lbl_                = nullptr;
    QLabel*      status_                    = nullptr;
    RoverLightingView* view_                = nullptr;

    // Watchdog: armed on every publish, cancelled by feedback. If it fires,
    // commands are going unanswered and the status line shows a warning.
    static constexpr int FEEDBACK_TIMEOUT_MS = 1500;
    QTimer* confirm_timer_ = nullptr;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};
