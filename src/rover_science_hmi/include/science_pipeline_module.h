#pragma once

#include <rover_hmi_core/gui_module.h>
#include <QLabel>
#include <QPushButton>
#include <QFrame>
#include <QStackedWidget>
#include <array>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/science_sensor_data.hpp"

class SciencePipelineModule : public rover_hmi_core::GuiModule
{
public:
    std::string name()           const override { return "Science Pipeline"; }
    std::string sectionName()    const override { return "Science"; }
    std::string layoutHint()     const override { return "large"; }
    bool        defaultVisible() const override { return true; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;

    std::vector<std::pair<std::string,std::string>> keybindings() const override {
        return {
            { "Mark Done (btn)", "Advance to next pipeline step" },
            { "Back (btn)",      "Return to previous step"       },
            { "Reset (btn)",     "Reset pipeline to step 0"      },
            { "Vial circles",    "Toggle vial filled state"      },
        };
    }

private:
    void updateStepDisplay();
    void advanceStep();
    void retreatStep();
    void resetPipeline();
    void onSensorData(const rover_msgs::msg::ScienceSensorData::SharedPtr msg);

    int  current_step_    = 0;
    bool steps_done_[4]   = {};
    bool vials_filled_[6] = {};

    // Step bar frames (top)
    QFrame*  step_frames_[4]    = {};
    QLabel*  step_dot_lbls_[4]  = {};
    QLabel*  step_name_lbls_[4] = {};

    // Detail panel
    QStackedWidget* detail_stack_ = nullptr;

    // Sensor display labels
    QLabel* ultrasonic_lbl_  = nullptr;
    QLabel* flow1_lbl_       = nullptr;
    QLabel* flow2_lbl_       = nullptr;
    QLabel* spectro_lbls_[6] = {};

    // Vial buttons (step 2)
    QPushButton* vial_btns_[6]  = {};
    QLabel*      vials_count_lbl_ = nullptr;

    // Navigation
    QPushButton* back_btn_  = nullptr;
    QPushButton* next_btn_  = nullptr;
    QPushButton* reset_btn_ = nullptr;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rover_msgs::msg::ScienceSensorData>::SharedPtr sensor_sub_;
};
