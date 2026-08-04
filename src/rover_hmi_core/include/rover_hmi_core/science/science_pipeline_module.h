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
#include "rover_msgs/msg/science_module.hpp"
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
            { "Checklist items", "Click to toggle; auto-check from telemetry" },
            { "Mark Done (btn)", "Manual override: advance a step"  },
            { "Back (btn)",      "Return to previous step"          },
            { "Reset (btn)",     "Reset pipeline to step 0"         },
            { "Vial circles",    "Toggle vial filled state"         },
        };
    }

private:
    void updateStepDisplay();
    void advanceStep();
    void retreatStep();
    void resetPipeline();
    void onSensorData(const rover_msgs::msg::ScienceSensorData::SharedPtr msg);
    void onCommand(const rover_msgs::msg::ScienceModule::SharedPtr msg);

    // Live checklist: 3 items per step, auto-checked from telemetry and
    // clickable as a manual override. All checked → step auto-advances.
    QPushButton* makeCheck(int step, int idx, const QString& text);
    void setCheck(int step, int idx, bool on);
    void setVial(int idx, bool filled);
    bool stepSatisfied(int s) const;
    void evalAdvance();

    int  current_step_    = 0;
    bool steps_done_[4]   = {};
    bool vials_filled_[6] = {};
    bool flow_seen_       = false;   // flow observed during COLLECT
    bool flow_now_        = false;

    QPushButton* checks_[4][3] = {};

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
    rclcpp::Subscription<rover_msgs::msg::ScienceModule>::SharedPtr cmd_sub_;
};
