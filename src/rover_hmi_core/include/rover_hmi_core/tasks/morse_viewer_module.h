// morse_viewer_module.h — "Morse Decode"
//
// Thin viewer for the rover_vision morse pipeline: shows /morse_decoded
// (std_msgs/String) as it arrives. All decode logic lives in
// morse_decoder_node.py — this panel only displays its output.
// Section: Tasks. defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QLabel>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MorseViewerModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Morse Decode"; }
    std::string sectionName() const override { return "Tasks"; }
    std::string layoutHint()  const override { return "right"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;
    void setNode(rclcpp::Node::SharedPtr node) override;

private:
    void onDecoded(std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    QLabel* word_ = nullptr;      // word currently being decoded (grows per letter)
    QLabel* message_ = nullptr;   // completed words, oldest first
    QString last_;                // previous /morse_decoded payload
    QStringList words_;           // committed words
};
