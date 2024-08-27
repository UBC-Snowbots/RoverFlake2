#ifndef CAMERA_SYS_PUBLISHER_NODE_H
#define CAMERA_SYS_PUBLISHER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/msg/detail/compressed_image__struct.hpp>
#include <vector>
#include <bitset>

class CamerasSysNode : public rclcpp::Node {
public:
    CamerasSysNode();
    ~CamerasSysNode();

private:
    void timer_callback();
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);

    // sensor_msgs::msg::CompressedImage::SharedPtr cv_mat_to_compressed_image_msg(const cv::Mat & frame, const std::string & frame_id);

    image_transport::Publisher pub_fcam_;
    // image_transport::Publisher pub_bcam_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr msg_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture capf_;
    // cv::VideoCapture capb_;
    // cv::VideoWriter writer_fcam_;
    // cv::VideoWriter writer_bcam_;
    std_msgs::msg::String::SharedPtr last_msg_;


};

#endif // CAMERA_SYS_PUBLISHER_NODE_H
