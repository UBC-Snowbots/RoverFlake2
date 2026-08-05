// morse_module.h — "Morse"
//
// Heist console helper. Thin client of the rover_vision morse pipeline:
// cam_0/morse_led_brightness (Int32, 30 fps) → live LED/bar, /morse_decoded
// (String) → decoded text. Plus manual .- decode and text→morse encode for
// keying the vault code. Section: Tasks. defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>
#include <rover_hmi_core/stale_monitor.h>

#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QProgressBar>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class MorseModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Morse"; }
    std::string sectionName() const override { return "Tasks"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    std::function<void(bool)> toggleCallback() override {
        return [this](bool on) { onVisibility(on); };
    }

private:
    static constexpr int kBrightnessThreshold = 220;  // matches morse_decoder_node.py

    void onVisibility(bool on);
    void subscribe();
    void onBrightness(int v);
    void onDecoded(const QString& s);

    QLabel*         led_dot_     = nullptr;
    QLabel*         signal_lbl_  = nullptr;
    QProgressBar*   bar_         = nullptr;
    QLabel*         current_lbl_ = nullptr;
    QPlainTextEdit* history_     = nullptr;
    QLineEdit*      manual_in_   = nullptr;
    QLabel*         manual_out_  = nullptr;
    QLineEdit*      enc_in_      = nullptr;
    QLabel*         enc_out_     = nullptr;

    QString last_word_;
    rover_hmi_core::StaleMonitor stale_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  brightness_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr decoded_sub_;
};
