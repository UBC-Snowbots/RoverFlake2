// camera_module.h — FNAF-style camera viewer: single viewport with a
// bottom-right selector list (arrow keys), or a 2-column grid of all feeds.
// Only the visible view's cameras are subscribed, on /<name>/image_raw_decoded.
#pragma once

#include <rover_hmi_core/gui_module.h>
#include <rover_hmi_core/cameras/camera_config.h>

#include <QElapsedTimer>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraViewport;
class CameraListOverlay;
class QStackedWidget;

class CameraModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Cameras"; }
    std::string layoutHint()  const override { return "main"; }
    std::string sectionName() const override { return "Cameras"; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override;
    void     stop()  override;

    // Drop all subscriptions while the tile is hidden; resubscribe on show.
    std::function<void(bool)> toggleCallback() override {
        return [this](bool on) { onVisibility(on); };
    }

    std::vector<std::pair<std::string,std::string>> keybindings() const override {
        return {
            { "1..9",       "Switch camera (module focused)" },
            { "Up/Down",    "Prev/next camera (single view)" },
            { "G",          "Toggle grid view"               },
            { "Click cell", "Zoom into camera (grid view)"   },
        };
    }

private:
    void switchTo(int idx);
    void setGrid(bool on);
    void resubscribe();     // rebuild subscriptions for the current mode
    void unsubscribeAll();
    void onLivenessTick();
    void onVisibility(bool on);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
    makeSub(int idx, CameraViewport* target);

    rclcpp::Node::SharedPtr node_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;

    std::vector<rover_hmi_core::camera_config::Camera> cams_;
    CameraViewport* viewport_ = nullptr;        // single view (stack page 0)
    std::vector<CameraViewport*> cells_;        // grid view  (stack page 1)
    CameraListOverlay* list_ = nullptr;
    QStackedWidget* stack_ = nullptr;
    QTimer*         liveness_timer_ = nullptr;
    std::vector<QElapsedTimer> last_frames_;
    int  active_  = -1;
    bool grid_    = false;
    bool visible_ = true;
};
