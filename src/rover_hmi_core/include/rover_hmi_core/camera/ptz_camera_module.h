#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QImage>
#include <QPointer>

#include <chrono>

class QPushButton;
class QDialog;

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

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
// Panorama is a backend routine (ptz_cam panorama_node drives both gimbal
// servos through a serpentine sweep and stitches on the Jetson); this panel
// only triggers it, mirrors its status topic, and shows the resulting image.
//
// ROS interface (all served by ptz_cam):
//   sub /ip_camera/image_raw   sensor_msgs/Image      gscam RTSP feed
//   pub /ptz/control           geometry_msgs/Vector3  x=pan, y=tilt, z=zoom; 0 = stop
//   srv /ptz/panorama/{start,cancel}  std_srvs/Trigger   backend sweep control
//   sub /ptz/panorama/status   std_msgs/String        sweep progress (mirrored)
//   sub /ptz/panorama/image    sensor_msgs/Image      stitched result
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
            { "P",           "Panorama sweep (start / cancel)" },
        };
    }

private:
    using TriggerClient = rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr;

    static QImage imageMsgToQImage(const sensor_msgs::msg::Image& msg);
    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onHealthTick();
    bool onKey(int key, bool pressed);
    void panTiltEvent(int dx, int dy, bool pressed);
    void zoomEvent(int dz, bool pressed);
    void publishControl();

    void callTrigger(const TriggerClient& client, const QString& label);
    void stopAll();
    void setStatus(const QString& text, const char* color);

    // Panorama: trigger/monitor the backend sweep, display its result.
    void panoToggle();
    void panoSetActive(bool active);
    void onPanoStatus(const std_msgs::msg::String::ConstSharedPtr& msg);
    void showPanoResult(const QImage& img);

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pt_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr  img_sub_;

    PtzPreview* preview_    = nullptr;
    QLabel*     status_lbl_ = nullptr;
    QLabel*     feed_lbl_   = nullptr;
    QTimer*     health_timer_ = nullptr;
    PtzKeyFilter* key_filter_ = nullptr;

    TriggerClient pano_start_, pano_cancel_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  pano_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pano_img_sub_;
    QPushButton* pano_btn_ = nullptr;
    bool pano_active_ = false;
    // One reusable result window: repeated results update it, never stack.
    QPointer<QDialog> pano_dlg_;
    QLabel* pano_dlg_lbl_ = nullptr;   // only touched while pano_dlg_ is alive
    QImage  pano_result_;              // backs the dialog's Save PNG button

    int   pan_dir_  = 0;      // held direction per axis: -1 / 0 / +1
    int   tilt_dir_ = 0;
    int   zoom_dir_ = 0;
    float speed_    = 1.0f;   // slider multiplier; driver applies its own serial scaling
    int   frames_   = 0;      // frames since last health tick (→ fps)
    int   frame_w_  = 0, frame_h_ = 0;
    std::chrono::steady_clock::time_point last_frame_{};
    bool  enabled_  = false;
};
