// camera_module.h — FNAF-style camera viewer: one viewport, rover-map
// selector, active-camera-only subscription to /<name>/image_raw_decoded.
#pragma once

#include <rover_hmi_core/gui_module.h>
#include <rover_hmi_core/cameras/camera_config.h>

#include <QElapsedTimer>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraViewport;
class CameraMap;

class CameraModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Cameras"; }
    std::string layoutHint()  const override { return "main"; }
    std::string sectionName() const override { return "Cameras"; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override;
    void     stop()  override;

    // Drop the subscription while the tile is hidden; resubscribe on show.
    std::function<void(bool)> toggleCallback() override {
        return [this](bool on) { onVisibility(on); };
    }

    std::vector<std::pair<std::string,std::string>> keybindings() const override {
        return {
            { "1..9",  "Switch camera (module focused)" },
            { "Click", "Switch camera via rover map"    },
        };
    }

private:
    void switchTo(int idx);
    void onLivenessTick();
    void onVisibility(bool on);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    std::vector<rover_hmi_core::camera_config::Camera> cams_;
    CameraViewport* viewport_ = nullptr;
    CameraMap*      map_      = nullptr;
    QTimer*         liveness_timer_ = nullptr;
    QElapsedTimer   last_frame_;
    int active_ = -1;
    bool visible_ = true;
};
