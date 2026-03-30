#include "rover_vision_cpp/cameras.h"

#include <opencv2/imgproc.hpp>

static constexpr int    kWidth  = 640;
static constexpr int    kHeight = 480;
static constexpr int    kFps    = 30;
static constexpr int    kMaxStdCameras = 12;
static constexpr double kTimerPeriodS  = 0.033; // ~30 Hz

CameraPublisher::CameraPublisher(const rclcpp::NodeOptions & options)
: Node("camera_pub_node", options)
{
  has_realsense_ = setupRealsense();

  for (const auto & dev_path : detectStandardCameras()) {
    cv::VideoCapture cap(dev_path);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Could not open camera: %s", dev_path.c_str());
      continue;
    }
    std_dev_paths_.push_back(dev_path);
    std_caps_.push_back(std::move(cap));

    auto pub = create_publisher<sensor_msgs::msg::Image>(
      "cam_" + std::to_string(std_publishers_.size()), 10);
    std_publishers_.push_back(pub);
  }

  if (!has_realsense_ && std_caps_.empty()) {
    RCLCPP_FATAL(get_logger(), "No cameras found — shutting down.");
    throw std::runtime_error("No cameras available");
  }

  timer_ = create_wall_timer(
    std::chrono::duration<double>(kTimerPeriodS),
    std::bind(&CameraPublisher::timerCallback, this));

  RCLCPP_INFO(get_logger(), "CameraPublisher initialized (realsense=%s, std_cams=%zu)",
    has_realsense_ ? "true" : "false", std_caps_.size());
}

CameraPublisher::~CameraPublisher()
{
  if (has_realsense_) {
    try { rs_pipeline_.stop(); } catch (...) {}
  }
  for (auto & cap : std_caps_) {
    cap.release();
  }
}

// -----------------------------------------------------------------------------

bool CameraPublisher::setupRealsense()
{
  try {
    rs2::context ctx;
    if (ctx.query_devices().size() == 0) {
      RCLCPP_INFO(get_logger(), "No RealSense devices found.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "RealSense device detected.");

    rs_config_.enable_stream(RS2_STREAM_COLOR,    kWidth, kHeight, RS2_FORMAT_BGR8, kFps);
    rs_config_.enable_stream(RS2_STREAM_INFRARED, 1, kWidth, kHeight, RS2_FORMAT_Y8,   kFps);
    rs_config_.enable_stream(RS2_STREAM_INFRARED, 2, kWidth, kHeight, RS2_FORMAT_Y8,   kFps);
    rs_config_.enable_stream(RS2_STREAM_DEPTH,    kWidth, kHeight, RS2_FORMAT_Z16,  kFps);

    rs_pipeline_.start(rs_config_);

    rs_publishers_["color"]  = create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw",         10);
    rs_publishers_["infra1"] = create_publisher<sensor_msgs::msg::Image>("/camera/infra1/image_rect_raw",   10);
    rs_publishers_["infra2"] = create_publisher<sensor_msgs::msg::Image>("/camera/infra2/image_rect_raw",   10);
    rs_publishers_["depth"]  = create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_rect_raw",    10);

    RCLCPP_INFO(get_logger(), "RealSense pipeline started.");
    return true;

  } catch (const rs2::error & e) {
    RCLCPP_WARN(get_logger(), "RealSense error: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "RealSense init failed: %s", e.what());
  }
  return false;
}

void CameraPublisher::publishRealsenseFrame(const cv::Mat & frame, const std::string & type)
{
  auto it = rs_publishers_.find(type);
  if (it == rs_publishers_.end()) return;

  std_msgs::msg::Header header;
  header.stamp = now();

  sensor_msgs::msg::Image::SharedPtr msg;

  if (type == "color") {
    cv::Mat rgb;
    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
    msg = cv_bridge::CvImage(header, "rgb8", rgb).toImageMsg();
  } else if (type == "infra1" || type == "infra2") {
    msg = cv_bridge::CvImage(header, "mono8", frame).toImageMsg();
  } else if (type == "depth") {
    msg = cv_bridge::CvImage(header, "16UC1", frame).toImageMsg();
  } else {
    return;
  }

  it->second->publish(*msg);
}

std::vector<std::string> CameraPublisher::detectStandardCameras()
{
  std::vector<std::string> result;
  for (int i = 0; i < kMaxStdCameras; ++i) {
    std::string path = "/dev/video" + std::to_string(i);
    cv::VideoCapture cap(path);
    if (!cap.isOpened()) continue;

    // Skip RealSense devices (they expose multiple /dev/video nodes)
    std::string backend = cap.getBackendName();
    cap.release();
    if (backend.find("REALSENSE") != std::string::npos ||
        backend.find("realsense") != std::string::npos) {
      RCLCPP_INFO(get_logger(), "Skipping RealSense node: %s", path.c_str());
      continue;
    }
    result.push_back(path);
  }
  RCLCPP_INFO(get_logger(), "Standard cameras detected: %zu", result.size());
  return result;
}

void CameraPublisher::timerCallback()
{
  // --- RealSense ---
  if (has_realsense_) {
    try {
      rs2::frameset frames;
      if (!rs_pipeline_.poll_for_frames(&frames)) return; // non-blocking

      if (auto f = frames.get_color_frame()) {
        cv::Mat img(cv::Size(kWidth, kHeight), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
        publishRealsenseFrame(img, "color");
      }
      if (auto f = frames.get_depth_frame()) {
        cv::Mat img(cv::Size(kWidth, kHeight), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
        publishRealsenseFrame(img, "depth");
      }
      for (int idx = 1; idx <= 2; ++idx) {
        if (auto f = frames.get_infrared_frame(idx)) {
          cv::Mat img(cv::Size(kWidth, kHeight), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
          publishRealsenseFrame(img, "infra" + std::to_string(idx));
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "RealSense frame error: %s", e.what());
    }
  }

  // --- Standard cameras ---
  for (size_t i = 0; i < std_caps_.size(); ++i) {
    cv::Mat frame;
    if (!std_caps_[i].read(frame)) continue;

    cv::Mat rgb;
    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);

    std_msgs::msg::Header header;
    header.stamp = now();
    auto msg = cv_bridge::CvImage(header, "rgb8", rgb).toImageMsg();
    std_publishers_[i]->publish(*msg);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CameraPublisher>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}