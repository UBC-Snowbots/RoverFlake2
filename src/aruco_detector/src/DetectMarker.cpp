#include "aruco_detector/DetectMarker.h"

DetectMarker::DetectMarker(const rclcpp::NodeOptions& options)
    : rclcpp::Node("marker_qr_detection", options) 
{
    // Initialize ArUco parameters
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    parameters = cv::aruco::DetectorParameters::create();

    // Get parameters from the parameter server
    this->declare_parameter("draw_markers", true);
    this->declare_parameter("camera", 1);
    draw_markers = this->get_parameter("draw_markers").as_bool();
    camera = this->get_parameter("camera").as_int();
    topic_to_subscribe_to = "vehicle_1/main_feed/image_decoded";

    // Setup Publisher
    my_publisher = this->create_publisher<std_msgs::msg::String>("identified", 10);

    RCLCPP_INFO(this->get_logger(), "Node initialization successful.");
    if (draw_markers) {
        RCLCPP_INFO(this->get_logger(), "I WILL ATTEMPT TO DRAW DETECTED MARKERS");
    } else {
        RCLCPP_INFO(this->get_logger(), "I WILL NOT ATTEMPT TO DRAW DETECTED MARKERS");
    }
}

void DetectMarker::initialize() {
    if (!is_initialized_) {
        // Initialize image transport
        image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        
        // Create the image publisher
        bounder = image_transport_->advertise("bounding_boxes", 1);
        
        // Create the subscriber
        my_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            topic_to_subscribe_to,
            5,
            std::bind(&DetectMarker::subscriberCallBack, this, std::placeholders::_1)
        );
        
        is_initialized_ = true;
    }
}

void DetectMarker::subscriberCallBack(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received msg");
    std::vector<int> markerIds = fetchMarkerIds(rosToMat(msg));
    
    std::stringstream ss;
    ss << "Detected markers: ";
    for (int markerId : markerIds) {
        ss << markerId << " ";
    }
    std::cout << std::endl;
}

std::vector<int> DetectMarker::fetchMarkerIds(const cv::Mat& image) {
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

cv::Mat DetectMarker::rosToMat(const sensor_msgs::msg::Image::SharedPtr image) {
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(image, image->encoding);
    return image_ptr->image;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectMarker>(rclcpp::NodeOptions());
    node->initialize();  // Initialize after construction
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    
}
