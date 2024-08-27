#ifndef CAMERA_SYS_SUBSCRIBER_NODE_HPP_
#define CAMERA_SYS_SUBSCRIBER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// #include <compressed_image_transport/compressed_subscriber.h>

class VideoSubscriberNode : public rclcpp::Node {
public:
    VideoSubscriberNode();

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg);
    
    image_transport::Publisher pub_fcam_;
    image_transport::Subscriber sub_fcam_;
    // image_transport::Subscriber sub_bcam_;
};

#endif  // CAMERA_SYS_SUBSCRIBER_NODE_HPP_
