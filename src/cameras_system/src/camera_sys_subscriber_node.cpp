#include "cameras_system/camera_sys_subscriber_node.hpp"

VideoSubscriberNode::VideoSubscriberNode() : Node("video_subscriber_node") {
    // Initialize the image transport subscriber
    image_transport::ImageTransport it(this);
    pub_fcam_ = it.advertise("fcam/test_feed", 1);
    sub_fcam_ = it.subscribe("fcam/compressed_feed", 1, &VideoSubscriberNode::image_callback, this, "compressed");
    // sub_bcam_ = it.subscribe("bcam/compressed_feed", 1, &VideoSubscriberNode::image_callback, this, "compressed");
}

void VideoSubscriberNode::image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg) {
    // test message
    try {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        cv::imshow("Received Video - " + msg->header.frame_id, frame);
        cv::waitKey(1);
    } catch (const cv::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Could not decode image: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
