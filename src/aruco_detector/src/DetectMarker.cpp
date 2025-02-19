#include <aruco_detector/DetectMarker.h>
#include <rclcpp/rclcpp.hpp>

DetectMarker::DetectMarker(const rclcpp::NodeOptions &options)
  : rclcpp::Node("marker_qr_detection", options) {

    // Initialize ArUco parameters
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    parameters = cv::aruco::DetectorParameters::create();

    std::string topic_to_subscribe_to;

    // Get parameters from the parameter server (or launch file)
    this->declare_parameter("draw_markers", true);
    this->declare_parameter("camera", 1);

    draw_markers = this->get_parameter("draw_markers").as_bool();
    camera = this->get_parameter("camera").as_int();

    topic_to_subscribe_to = "camera/color/image_raw"; // Just do this for now, we can change later

    // Create ImageTransport object (without shared_from_this)
    image_transport::ImageTransport it(shared_from_this());  // Pass Node pointer to ImageTransport

    // Create the subscriber using a member function as the callback
    my_subscriber = it.subscribe(topic_to_subscribe_to, 5, &DetectMarker::subscriberCallBack, this);

    // Setup Publisher(s)
    my_publisher = this->create_publisher<std_msgs::msg::String>("identified", 10);

    // Create an image publisher
    bounder = it.advertise("bounding_boxes", 1);

    RCLCPP_INFO(this->get_logger(), "Node initialization successful.");
    if (draw_markers) {
        RCLCPP_INFO(this->get_logger(), "I WILL ATTEMPT TO DRAW DETECTED MARKERS");
    } else {
        RCLCPP_INFO(this->get_logger(), "I WILL NOT ATTEMPT TO DRAW DETECTED MARKERS");
    }
}

// Callback function
void DetectMarker::subscriberCallBack(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::vector<int> markerIds = fetchMarkerIds(rosToMat(msg));

    // Prepare the detected marker IDs as a string message
    std::stringstream ss;
    ss << "Detected markers: ";
    for (int markerId : markerIds) {
        ss << markerId << " ";
    }

    // Create the message to publish
    auto message = std_msgs::msg::String();
    message.data = ss.str();

    // Publish the message with the marker IDs
    my_publisher->publish(message);

    RCLCPP_INFO(this->get_logger(), "Markers detected and published: %s", message.data.c_str());
}

// Fetch marker IDs from the image
std::vector<int> DetectMarker::fetchMarkerIds(const cv::Mat &image) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    
    // Detect markers using ArUco
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters);
    
    if (draw_markers) {
        cv::Mat outputImage;
        image.copyTo(outputImage);
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

        // Publish the detected image with bounding boxes
        bounder.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", outputImage).toImageMsg());
        RCLCPP_INFO(this->get_logger(), "Attempted to draw markers.");
    }

    return markerIds;
}

// Convert ROS image message to OpenCV format
cv::Mat DetectMarker::rosToMat(const sensor_msgs::msg::Image::SharedPtr image) {
    // Convert ROS image message to OpenCV format
    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image, image->encoding);
    return image_ptr->image;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create the node with ROS 2 options
    rclcpp::spin(std::make_shared<DetectMarker>(rclcpp::NodeOptions()));

    rclcpp::shutdown();
    return 0;
}

// DetectMarker::DetectMarker(const rclcpp::NodeOptions &options)
//   : rclcpp::Node("marker_qr_detection", options) {

//     // Initialize ArUco parameters
//     dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
//     parameters = cv::aruco::DetectorParameters::create();

//     std::string topic_to_subscribe_to;

//     // Get parameters from the parameter server (or launch file)
//     this->declare_parameter("draw_markers", true);
//     this->declare_parameter("camera", 1);

//     draw_markers = this->get_parameter("draw_markers").as_bool();
//     camera = this->get_parameter("camera").as_int();

//     topic_to_subscribe_to = "camera/color/image_raw"; // Just do this for now, we can change later

//     // Create ImageTransport object
//     image_transport::ImageTransport it(shared_from_this());  // Pass Node pointer to ImageTransport

//     // Create the subscriber using a member function as the callback
//     my_subscriber = it.subscribe(topic_to_subscribe_to, 5, 
//         std::bind(&DetectMarker::subscriberCallBack, this, std::placeholders::_1));

//     // Setup Publisher(s)
//     my_publisher = this->create_publisher<std_msgs::msg::String>("identified", 10);

//     // Create an image publisher
//     bounder = it.advertise("bounding_boxes", 1);

//     RCLCPP_INFO(this->get_logger(), "Node initialization successful.");
//     if (draw_markers) {
//         RCLCPP_INFO(this->get_logger(), "I WILL ATTEMPT TO DRAW DETECTED MARKERS");
//     } else {
//         RCLCPP_INFO(this->get_logger(), "I WILL NOT ATTEMPT TO DRAW DETECTED MARKERS");
//     }
// }

// void DetectMarker::subscriberCallBack(const sensor_msgs::msg::Image::SharedPtr msg) {
//     std::vector<int> markerIds = fetchMarkerIds(rosToMat(msg));

//     // Prepare the detected marker IDs as a string message
//     std::stringstream ss;
//     ss << "Detected markers: ";
//     for (int markerId : markerIds) {
//         ss << markerId << " ";
//     }

//     // Create the message to publish
//     auto message = std_msgs::msg::String();
//     message.data = ss.str();

//     // Publish the message with the marker IDs
//     my_publisher->publish(message);

//     RCLCPP_INFO(this->get_logger(), "Markers detected and published: %s", message.data.c_str());
// }

// std::vector<int> DetectMarker::fetchMarkerIds(const cv::Mat &image) {
//     std::vector<int> markerIds;
//     std::vector<std::vector<cv::Point2f>> markerCorners;
    
//     // Detect markers using ArUco
//     cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters);
    
//     if (draw_markers) {
//         cv::Mat outputImage;
//         image.copyTo(outputImage);
//         cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

//         // Publish the detected image with bounding boxes
//         bounder.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", outputImage).toImageMsg());
//         RCLCPP_INFO(this->get_logger(), "Attempted to draw markers.");
//     }

//     return markerIds;
// }

// cv::Mat DetectMarker::rosToMat(const sensor_msgs::msg::Image::SharedPtr image) {
//     // Convert ROS image message to OpenCV format
//     cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image, image->encoding);
//     return image_ptr->image;
// }

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);

//     // Create the node with ROS 2 options
//     rclcpp::spin(std::make_shared<DetectMarker>(rclcpp::NodeOptions()));

//     rclcpp::shutdown();
//     return 0;
// }



// // DetectMarker::DetectMarker(const rclcpp::NodeOptions &options) 
// //   : rclcpp::Node("marker_qr_detection", options) {
// //     // Initialize ArUco parameters
// //     dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
// //     parameters = cv::aruco::DetectorParameters::create();

// //     std::string topic_to_subscribe_to;

// //     // Get parameters from the parameter server (or launch file)
// //     this->declare_parameter("draw_markers", true);
// //     this->declare_parameter("camera", 1);

// //     draw_markers = this->get_parameter("draw_markers").as_bool();
// //     camera = this->get_parameter("camera").as_int();

// //     topic_to_subscribe_to = "camera/color/image_raw"; // Just do this for now, we can change later

// //     // Create ImageTransport object
// //     image_transport::ImageTransport it(shared_from_this());  // Pass Node pointer to ImageTransport

// //     // Create the subscriber using a lambda as the callback
// //     my_subscriber = it.subscribe(topic_to_subscribe_to, 5, 
// //         [this](const sensor_msgs::msg::Image::SharedPtr msg) {
// //             // Directly call the subscriber callback from the lambda
// //             this->subscriberCallBack(msg);
// //         });

// //     // Setup Publisher(s)
// //     my_publisher = this->create_publisher<std_msgs::msg::String>("identified", 10);

// //     // Create an image publisher
// //     bounder = it.advertise("bounding_boxes", 1);

// //     RCLCPP_INFO(this->get_logger(), "Node initialization successful.");
// //     if (draw_markers) {
// //         RCLCPP_INFO(this->get_logger(), "I WILL ATTEMPT TO DRAW DETECTED MARKERS");
// //     } else {
// //         RCLCPP_INFO(this->get_logger(), "I WILL NOT ATTEMPT TO DRAW DETECTED MARKERS");
// //     }
// // }

// // void DetectMarker::subscriberCallBack(const sensor_msgs::msg::Image::SharedPtr msg) {
// //     // ROS_INFO("Received message");
// //     std::vector<int> markerIds = fetchMarkerIds(rosToMat(msg));
// //     // std::cout << "Markers detected:";
// //     // for (int markerId : markerIds) { 
// //     //     std::cout << " " << markerId; 
// //     // }
// //     // std::cout << std::endl;
// //     // RCLCPP_INFO(this->get_logger(), "Received message");
// //     // std::vector<int> markerIds = fetchMarkerIds(rosToMat(msg));

// //     // Prepare the detected marker IDs as a string message
// //     std::stringstream ss;
// //     ss << "Detected markers: ";
// //     for (int markerId : markerIds) {
// //         ss << markerId << " ";
// //     }

// //     // Create the message to publish
// //     auto message = std_msgs::msg::String();
// //     message.data = ss.str();

// //     // Publish the message with the marker IDs
// //     my_publisher->publish(message);

// //     RCLCPP_INFO(this->get_logger(), "Markers detected and published: %s", message.data.c_str());
// // }

// // std::vector<int> DetectMarker::fetchMarkerIds(const cv::Mat& image) {
// //     // std::vector<int> markerIds;
// //     // std::vector<std::vector<cv::Point2f>> markerCorners;
// //     // cv::aruco::detectMarkers(
// //     //     image, dictionary, markerCorners, markerIds, parameters
// //     // );

// //     // if (draw_markers) {
// //     //     cv::Mat outputImage;
// //     //     image.copyTo(outputImage);
// //     //     cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

// //     //     // Convert to ROS message and publish the image with bounding boxes
// //     //     bounder.publish(
// //     //         cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", outputImage)
// //     //         .toImageMsg()
// //     //     );
// //     //     RCLCPP_INFO(this->get_logger(), "Attempted drawing markers");
// //     // }
// //     // return markerIds;
    
// //     std::vector<int> markerIds;
// //     std::vector<std::vector<cv::Point2f>> markerCorners;
// //     cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters);
    
// //     if (draw_markers) {
// //         cv::Mat outputImage;
// //         image.copyTo(outputImage);
// //         cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

// //         // Publish the detected image with bounding boxes
// //         bounder.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", outputImage).toImageMsg());
// //         RCLCPP_INFO(this->get_logger(), "Attempted to draw markers.");
// //     }

// //     return markerIds;
// // }

// // cv::Mat DetectMarker::rosToMat(const sensor_msgs::msg::Image::SharedPtr image) {
// //     cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image, image->encoding);
// //     return image_ptr->image;
// // }

// // int main(int argc, char **argv) {
// //     rclcpp::init(argc, argv);

// //     // Create the node with ROS 2 options
// //     rclcpp::spin(std::make_shared<DetectMarker>(rclcpp::NodeOptions()));

// //     rclcpp::shutdown();
// //     return 0;
// // }
