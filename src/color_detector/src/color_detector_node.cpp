/**
 * Created by: Cameron Basara
 * Date: May 25, 2024
 * Purpose: Classify different colours in a video feed / still
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ColorDetectorNode : public rclcpp::Node {
public:
    ColorDetectorNode() : Node("color_detector_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/usb_cam/image_raw", 10, std::bind(&ColorDetectorNode::image_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/cd_cam/image_processed", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Convert to HSV color space
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        // Define color range for blue in HSV
        cv::Scalar lower_blue(110, 50, 50);
        cv::Scalar upper_blue(130, 255, 255);

        // Threshold the HSV image to get only blue colors
        cv::Mat mask;
        cv::inRange(hsv_image, lower_blue, upper_blue, mask);

        // Bitwise-AND mask and original image
        cv::Mat result;
        cv::bitwise_and(cv_ptr->image, cv_ptr->image, result, mask = mask);

        // Convert result to ROS Image message and publish
        auto processed_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result).toImageMsg();
        publisher_->publish(*processed_msg);

        // Display the images
        cv::imshow("Original Image", cv_ptr->image);
        cv::imshow("Detected Blue Color", result);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
