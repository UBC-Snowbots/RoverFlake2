#pragma once

#include <rover_hmi_core/gui_module.h>
#include <rover_hmi_core/stale_monitor.h>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QFile>
#include <QTextStream>
#include <array>
#include <deque>
#include <limits>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/science_sensor_data.hpp"

// Forward declaration — SpectroPaint is defined in the cpp (needs Q_OBJECT + moc)
class SpectroPaint;

class ScienceAnalysisModule : public rover_hmi_core::GuiModule
{
public:
    std::string name()           const override { return "Science Analysis"; }
    std::string sectionName()    const override { return "Science"; }
    std::string layoutHint()     const override { return "large"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override;
    void     stop() override;

private:
    void onSensorData(const rover_msgs::msg::ScienceSensorData::SharedPtr msg);
    void toggleRecording();
    void appendNpkRecord(float n, float p, float k);

    // Spectro
    static constexpr int kNumVials = 3;
    QLabel*       spectro_lbls_[kNumVials] = {};
    SpectroPaint* spectro_chart_   = nullptr;
    std::array<float,kNumVials> spectro_vals_{};

    // NPK
    QLabel*       npk_n_lbl_   = nullptr;
    QLabel*       npk_p_lbl_   = nullptr;
    QLabel*       npk_k_lbl_   = nullptr;
    QPushButton*  npk_rec_btn_ = nullptr;
    QLabel*       npk_status_  = nullptr;
    bool          npk_recording_ = false;
    QFile*        npk_file_    = nullptr;
    QTextStream*  npk_stream_  = nullptr;

    // Fluorometer
    QLabel*       fluoro_val_lbl_ = nullptr;
    QProgressBar* fluoro_bar_     = nullptr;

    // Gas sensor
    QLabel*       gas_val_lbl_  = nullptr;
    QLabel*       gas_avg_lbl_  = nullptr;
    std::deque<float> gas_history_;

    // Flow sensors

    QLabel* banner_ = nullptr;
    rover_hmi_core::StaleMonitor stale_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rover_msgs::msg::ScienceSensorData>::SharedPtr sub_;
};
