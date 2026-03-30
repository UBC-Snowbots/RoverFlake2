#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#include <string>
#include <vector>
#include <memory>

class CameraPublisher : public rclcpp::Node{
public:
  explicit CameraPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~CameraPublisher();

private:
  // RealSense
  bool setupRealsense();
  void publishRealsenseFrame(const cv::Mat & frame, const std::string & stream_type);

  // Standard cameras
  std::vector<std::string> detectStandardCameras();

  // Timer
  void timerCallback();

  // RealSense members
  rs2::pipeline rs_pipeline_;
  rs2::config rs_config_;
  bool has_realsense_{false};
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> rs_publishers_;

  // Standard camera members
  std::vector<cv::VideoCapture> std_caps_;
  std::vector<std::string> std_dev_paths_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> std_publishers_;

  rclcpp::TimerBase::SharedPtr timer_;
};