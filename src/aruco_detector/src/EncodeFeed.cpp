#include "aruco_detector/EncodeFeed.h"

EncodeFeed::EncodeFeed(const rclcpp::NodeOptions& options)
    : rclcpp::Node("encode_feed", options) 
{
\

    // Get parameters from the parameter server
    this->declare_parameter("camera", 1);
    camera = this->get_parameter("camera").as_int();
    topic_to_subscribe_to = "vehicle_1/main_feed/image_decoded";

    // Setup Publisher
    my_publisher = this->create_publisher<std_msgs::msg::String>("identified", 10);

    RCLCPP_INFO(this->get_logger(), "Node initialization successful.");
    if (draw_markers) {
        RCLCPP_INFO(this->get_logger(), "I WILL ATTEMPT TO ENCODE");
    } else {
        RCLCPP_INFO(this->get_logger(), "I WILL NOT ATTEMPT TO ENCODE");
    }
}

void EncodeFeed::initialize() {
    if (!is_initialized_) {
        // Initialize image transport
        image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        
        // Create the image publisher
        bounder = image_transport_->advertise("colour_detect_feed", 1);
        
        // Create the subscriber
        my_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            topic_to_subscribe_to,
            5,
            std::bind(&EncodeFeed::subscriberCallBack, this, std::placeholders::_1)
        );
        
        is_initialized_ = true;
    }
}

void EncodeFeed::subscriberCallBack(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received msg");
    std::vector<int> markerIds = fetchMarkerIds(rosToMat(msg));
    
    std::stringstream ss;
    ss << "Detected markers: ";
    for (int markerId : markerIds) {
        ss << markerId << " ";
    }
    std::cout << std::endl;
}

std::vector<int> EncodeFeed::fetchMarkerIds(const cv::Mat& image) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters);
    
    if (draw_markers) {
        cv::Mat outputImage;
        image.copyTo(outputImage);
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
        bounder.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", outputImage).toImageMsg());
        RCLCPP_INFO(this->get_logger(), "Attempted to draw markers.");
    }
    
    return markerIds;
}

cv::Mat EncodeFeed::rosToMat(const sensor_msgs::msg::Image::SharedPtr image) {
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(image, image->encoding);
    return image_ptr->image;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EncodeFeed>(rclcpp::NodeOptions());
    node->initialize();  // Initialize after construction
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}