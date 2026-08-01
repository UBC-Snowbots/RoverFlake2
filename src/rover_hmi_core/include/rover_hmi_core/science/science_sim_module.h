#pragma once

#include <rover_hmi_core/gui_module.h>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/science_sensor_data.hpp"

// Publishes mock /science/sensor_data so the pipeline/analysis panels can be
// exercised without the science embedded node. Desk-testing tool only.
class ScienceSimModule : public rover_hmi_core::GuiModule
{
public:
    std::string name()        const override { return "Science Sim"; }
    std::string layoutHint()  const override { return "right"; }
    std::string sectionName() const override { return "Science"; }
    QWidget*    createWidget(QWidget* parent) override;
    void        setNode(rclcpp::Node::SharedPtr node) override;

private:
    void onTick();
    void updateBtns();

    bool sim_on_    = false;
    int  drill_dir_ = 0;      // -1 raise, 0 hold, +1 lower
    bool flow1_     = false;
    bool flow2_     = false;
    bool sample_    = false;  // sample loaded → NPK/fluoro/gas read plausible values
    bool spectro_   = false;
    int  tick_      = 0;
    float depth_in_ = 0.0f;   // ultrasonic drill depth [0, 12] in

    QPushButton* master_btn_  = nullptr;
    QPushButton* lower_btn_   = nullptr;
    QPushButton* hold_btn_    = nullptr;
    QPushButton* raise_btn_   = nullptr;
    QPushButton* flow1_btn_   = nullptr;
    QPushButton* flow2_btn_   = nullptr;
    QPushButton* sample_btn_  = nullptr;
    QPushButton* spectro_btn_ = nullptr;
    QLabel*      depth_lbl_   = nullptr;
    QLabel*      status_lbl_  = nullptr;
    QTimer*      timer_       = nullptr;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<rover_msgs::msg::ScienceSensorData>::SharedPtr pub_;
};
