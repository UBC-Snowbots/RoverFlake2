

#ifndef ENCODER_H
#define ENCODER_H

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>  // ROS 2 sensor_msgs
#include <sensor_msgs/image_encodings.hpp>

// STD Includes
#include <iostream>
#include <vector>
#include <memory>

// ROS 2 Includes
#include "rclcpp/rclcpp.hpp"            // ROS 2 Node base class
#include "std_msgs/msg/string.hpp"      // ROS 2 message types
#include "sensor_msgs/msg/image.hpp"    // ROS 2 Image message type

// Important!
class EncodeFeed : public rclcpp::Node {
  public:
      explicit EncodeFeed(const rclcpp::NodeOptions& options);
      void initialize();  // New initialization method
  
  private:
      // Image transport
      std::shared_ptr<image_transport::ImageTransport> image_transport_;
      image_transport::Publisher bounder;
      
      // Publishers and subscribers
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr my_publisher;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr my_subscriber;
      
      // Parameters
      bool draw_markers;
      int camera;
      std::string topic_to_subscribe_to;
      
      // Member functions
      void subscriberCallBack(const sensor_msgs::msg::Image::SharedPtr msg);
      std::vector<int> fetchMarkerIds(const cv::Mat& image);
      cv::Mat rosToMat(const sensor_msgs::msg::Image::SharedPtr image);
      
      bool is_initialized_ = false;
  };
    

#endif // ENCODE_H