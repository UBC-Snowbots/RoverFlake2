// camera_module.h — dwindle-tiled grid of camera feeds with a bottom button
// bar selecting which cameras are in the grid. Only in-grid cameras are
// subscribed, on /<name>/image_raw_decoded.
#pragma once

#include <rover_hmi_core/gui_module.h>
#include <rover_hmi_core/cameras/camera_config.h>

#include <QElapsedTimer>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraViewport;
class CameraGrid;
class QPushButton;

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
            { "1..9",       "Toggle camera in/out of grid"   },
            { "Alt+Arrow",  "Focus cell"                     },
            { "Alt+Shift+Arrow",      "Resize cell" },
            { "Alt+Ctrl+Shift+Arrow", "Swap cells"  },
            { "Alt+J",      "Toggle cell split"              },
        };
    }

    // Alt chords act on camera cells while the module is visible; the panel
    // tiling keeps them otherwise.
    std::function<bool(TilingOp, int, int)> tilingOpsCallback() override {
        return [this](TilingOp op, int dx, int dy) {
            return visible_ && gridOp(op, dx, dy);
        };
    }

    // Persisted in saved layouts: grid membership and cell arrangement.
    QJsonObject saveState() const override;
    void restoreState(const QJsonObject& state) override;

private:
    void toggleInGrid(int idx);
    void rebuildGrid();     // sync the dwindle grid + buttons to in_grid_
    bool gridOp(TilingOp op, int dx, int dy);
    void resubscribe();     // rebuild subscriptions for the current membership
    void unsubscribeAll();
    void onLivenessTick();
    void onVisibility(bool on);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
    makeSub(int idx, CameraViewport* target);

    rclcpp::Node::SharedPtr node_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;

    std::vector<rover_hmi_core::camera_config::Camera> cams_;
    std::vector<CameraViewport*> cells_;
    std::vector<QPushButton*> buttons_;   // bottom selector, one per camera
    CameraGrid* grid_widget_ = nullptr;   // dwindle-tiled cell container
    QTimer*     liveness_timer_ = nullptr;
    std::vector<QElapsedTimer> last_frames_;
    std::vector<bool> in_grid_;   // grid membership per camera
    bool visible_ = true;
};
