#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QImage>

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/image.hpp"

// Defined in ptz_camera_module.cpp (Q_OBJECT-free helpers kept out of the header).
class PtzPreview;
class PtzKeyFilter;

// PtzCameraModule — live feed + pan/tilt/zoom panel for the PTZ IP camera
// (src/ptz_cam, feature/ptz_camera).
//
// Control contract (same as the old rover_hmi dashboard): everything rides one
// geometry_msgs/Vector3 on /ptz/control — x=pan, y=tilt, z=zoom; sign is the
// direction, 0 stops that axis. Press = start, release = stop. Rover-side,
// pitch_tilt_node consumes x/y (serial gimbal) and the zoom node consumes z.
//
// ROS interface (all served by ptz_cam):
//   sub /ip_camera/camera/image_raw   sensor_msgs/Image      gscam RTSP feed
//   pub /ptz/control           geometry_msgs/Vector3  x=pan, y=tilt, z=zoom; 0 = stop
class PtzCameraModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "PTZ Camera"; }
    std::string layoutHint()  const override { return "main"; }
    std::string sectionName() const override { return "Cameras"; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override;
    void     stop()  override;

    std::vector<std::pair<std::string,std::string>> keybindings() const override {
        return {
            { "I / K",       "Tilt up / down (hold)"   },
            { "J / L",       "Pan left / right (hold)" },
            { "Z / X",       "Zoom in / out (hold)"    },
        };
    }

private:
    static QImage imageMsgToQImage(const sensor_msgs::msg::Image& msg);
    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onHealthTick();
    bool onKey(int key, bool pressed);
    void panTiltEvent(int dx, int dy, bool pressed);
    void zoomEvent(int dz, bool pressed);
    void publishControl();

    void stopAll();
    void setStatus(const QString& text, const char* color);

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pt_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr  img_sub_;

    PtzPreview* preview_    = nullptr;
    QLabel*     status_lbl_ = nullptr;
    QLabel*     feed_lbl_   = nullptr;
    QTimer*     health_timer_ = nullptr;
    PtzKeyFilter* key_filter_ = nullptr;

    int   pan_dir_  = 0;      // held direction per axis: -1 / 0 / +1
    int   tilt_dir_ = 0;
    int   zoom_dir_ = 0;
    float speed_    = 1.0f;   // slider multiplier; driver applies its own serial scaling
    int   frames_   = 0;      // frames since last health tick (→ fps)
    int   frame_w_  = 0, frame_h_ = 0;
    std::chrono::steady_clock::time_point last_frame_{};
    bool  enabled_  = false;
};
