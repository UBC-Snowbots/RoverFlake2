#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

struct Detection {
  cv::Rect  box;
  float     confidence;
  int       class_id;
};

class YOLODetectorNode : public rclcpp::Node
{
public:
  explicit YOLODetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Inference
  std::vector<Detection> runInference(const cv::Mat & frame);
  cv::Mat                preprocess(const cv::Mat & frame);
  std::vector<Detection> postprocess(const cv::Mat & output, int orig_w, int orig_h);
  cv::Scalar             classColour(int class_id);

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_;

  // Model
  cv::dnn::Net        net_;
  std::vector<std::string> class_names_;

  // Config
  float conf_threshold_{0.4f};
  float nms_threshold_{0.45f};
  int   input_size_{640};
};