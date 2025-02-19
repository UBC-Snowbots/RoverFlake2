/*
 * Created By: Ihsan Olawale, Rowan Zawadski
 * Repurposed By: King Cammy
 * Created On: July 17th, 2022
 */


#ifndef MARKER_QR_DETECTION_DETECT_MARKER_H
#define MARKER_QR_DETECTION_DETECT_MARKER_H

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

// ROS 2 Includes
#include "rclcpp/rclcpp.hpp"            // ROS 2 Node base class
#include "std_msgs/msg/string.hpp"      // ROS 2 message types
#include "sensor_msgs/msg/image.hpp"    // ROS 2 Image message type

// Important!
#define YOUR_MOM "MILF"

class DetectMarker : public rclcpp::Node, public std::enable_shared_from_this<DetectMarker> {  // Added enable_shared_from_this
  public:
    DetectMarker(const rclcpp::NodeOptions &options);

  private:
    /**
     * Callback function for when a new image is received
     *
     * @param msg the image received in the callback
     */

    // Callback function that will be called when a message is received
    void subscriberCallBack(const sensor_msgs::msg::Image::SharedPtr msg);

    std::vector<int> fetchMarkerIds(const cv::Mat &image);
    cv::Mat rosToMat(const sensor_msgs::msg::Image::SharedPtr image);

    image_transport::Subscriber my_subscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr my_publisher;
    image_transport::Publisher bounder;

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    bool draw_markers = true;
    int camera = 2;

    // ROS 2 Subscribers and Publishers
    //  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr my_subscriber;
    //  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr my_publisher;
    //  image_transport::Publisher bounder;  // Keep this if using image_transport
};

#endif // MARKER_QR_DETECTION_DETECT_MARKER_H


// /*
//  * Created By: Ihsan Olawale, Rowan Zawadski
//  * Repurposed By: King Cammy
//  * Created On: July 17th, 2022
//  * Description: An example node that subscribes to a topic publishing strings,
//  *              and re-publishes everything it receives to another topic with
//  *              a "!" at the end
//  */

//  #ifndef MARKER_QR_DETECTION_DETECT_MARKER_H
//  #define MARKER_QR_DETECTION_DETECT_MARKER_H
 
//  // OpenCV
//  #include <opencv2/core/core.hpp>
//  #include <opencv2/aruco.hpp>
//  #include <opencv2/highgui.hpp>
//  #include <opencv2/opencv.hpp>
 
//  // Image Conversion
//  #include <cv_bridge/cv_bridge.h>
//  #include <image_transport/image_transport.hpp>
//  #include <sensor_msgs/msg/image.hpp>  // ROS 2 sensor_msgs
//  #include <sensor_msgs/image_encodings.hpp>
 
//  // STD Includes
//  #include <iostream>
//  #include <vector>
 
//  // ROS 2 Includes
//  #include "rclcpp/rclcpp.hpp"            // ROS 2 Node base class
//  #include "std_msgs/msg/string.hpp"      // ROS 2 message types
//  #include "sensor_msgs/msg/image.hpp"    // ROS 2 Image message type
 
//  // Important!
//  #define YOUR_MOM "MILF"
 
 
//  class DetectMarker : public rclcpp::Node {
//    public:
//      DetectMarker(const rclcpp::NodeOptions &options);
 
//    private:
//      /**
//       * Callback function for when a new image is received
//       *
//       * @param msg the image received in the callback
//       */

//      // Callback function that will be called when a message is received
//     void subscriberCallBack(const sensor_msgs::msg::Image::SharedPtr msg);

//     std::vector<int> fetchMarkerIds(const cv::Mat &image);
//     cv::Mat rosToMat(const sensor_msgs::msg::Image::SharedPtr image);

//     image_transport::Subscriber my_subscriber;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr my_publisher;
//     image_transport::Publisher bounder;

//     cv::Ptr<cv::aruco::Dictionary> dictionary;
//     cv::Ptr<cv::aruco::DetectorParameters> parameters;
//     bool draw_markers = true;
//     int camera = 2;

//     // ROS 2 Subscribers and Publishers
//     //  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr my_subscriber;
//     //  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr my_publisher;
//     //  image_transport::Publisher bounder;  // Keep this if using image_transport
//  };
 
//  #endif // MARKER_QR_DETECTION_DETECT_MARKER_H
 