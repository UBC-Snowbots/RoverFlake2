#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QImage>

#include <chrono>

class QPushButton;

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

// Defined in ptz_camera_module.cpp (Q_OBJECT-free helpers kept out of the header).
class PtzPreview;
class PtzKeyFilter;

// PtzCameraModule — live feed + pan/tilt/zoom/focus panel for the Hi3510-based
// PTZ IP camera (src/ptz_cam, feature/ptz_camera).
//
// Protocol reference: Hi3510 ptzctrl.cgi as documented in the OEM "IP Camera CGI
// User Guide" (Dericam/FDT editions, e.g. johnlose.de/wp-content/uploads/2018/04/
// Dericam-Camera-CGI-RTSP-User-Guide-v1.0.2.pdf). Zoom/focus (and the serial
// pan/tilt gimbal) are continuous motor moves that run until an explicit stop,
// so every control here is press = start / release = stop. Zoom/focus taps
// shorter than kMinPulseMs defer the stop to the 200 ms mark, giving a fixed
// incremental step per tap; longer holds stay continuous.
//
// Panorama is a backend routine (ptz_cam panorama_node drives both gimbal
// servos through a serpentine sweep and stitches on the Jetson); this panel
// only triggers it, mirrors its status topic, and shows the resulting image.
//
// ROS interface (all served by ptz_cam):
//   sub /ip_camera/image_raw   sensor_msgs/Image      gscam RTSP feed
//   pub /ptz/control           geometry_msgs/Vector3  x=pan, y=tilt, 0 = stop
//   srv /ip_camera/{zoom,focus}_{in,out}_start, zoom_stop, focus_stop  std_srvs/Trigger
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
            { "Z / X",       "Zoom in / out"           },
            { "N / M",       "Focus in / out"          },
            { "Tap / hold",  "Tap = 200 ms step, hold = continuous" },
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
    void publishPanTilt();

    // Tap→timed-pulse / hold→continuous state, one per zoom and focus rocker.
    struct Rocker {
        QTimer* pulse = nullptr;   // single-shot; fires the deferred stop
        std::chrono::steady_clock::time_point pressed_at{};
    };
    void rockerPress(const TriggerClient& start, const QString& label, Rocker& r);
    void rockerRelease(const TriggerClient& halt, const QString& label, Rocker& r);

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
    TriggerClient zoom_in_, zoom_out_, zoom_stop_, focus_in_, focus_out_, focus_stop_;

    PtzPreview* preview_    = nullptr;
    QLabel*     status_lbl_ = nullptr;
    QLabel*     feed_lbl_   = nullptr;
    QTimer*     health_timer_ = nullptr;
    PtzKeyFilter* key_filter_ = nullptr;

    static constexpr int kMinPulseMs = 200;  // width of one tap-step pulse
    Rocker zoom_rocker_, focus_rocker_;

    TriggerClient pano_start_, pano_cancel_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  pano_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pano_img_sub_;
    QPushButton* pano_btn_ = nullptr;
    bool pano_active_ = false;

    int   pan_dir_  = 0;      // held direction per axis: -1 / 0 / +1
    int   tilt_dir_ = 0;
    float speed_    = 1.0f;   // slider multiplier; driver applies its own serial scaling
    int   frames_   = 0;      // frames since last health tick (→ fps)
    int   frame_w_  = 0, frame_h_ = 0;
    std::chrono::steady_clock::time_point last_frame_{};
    bool  enabled_  = false;
};
