#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d.hpp"

using std::placeholders::_1;

// --- requires tuning
constexpr int IMG_WIDTH = 640;
constexpr int IMG_HEIGHT = 480;
static const cv::Mat K = (cv::Mat_<double>(3, 3) << 
  600.0, 0.0,   320.0,
  0.0,   600.0, 240.0,
  0.0,   0.0,   1.0);
static const cv::Mat D = (cv::Mat_<double>(4, 1) << -0.50, 0.0, 0.0, 0.0);
constexpr double BALANCE = 1.0;  // 0 = crop black borders, 1 = keep full FOV
// ---

class FisheyeUndistortNode : public rclcpp::Node
{
public:
  FisheyeUndistortNode()
  : Node("fisheye_undistort_node")
  {
    cv::Mat new_K;
    cv::Size size(IMG_WIDTH, IMG_HEIGHT);
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
      K, D, size, cv::Mat::eye(3, 3, CV_64F), new_K, BALANCE);
    cv::fisheye::initUndistortRectifyMap(
      K, D, cv::Mat::eye(3, 3, CV_64F), new_K, size, CV_16SC2, map1_, map2_);

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "realsense435i/image_raw_decoded", rclcpp::QoS(5),
      std::bind(&FisheyeUndistortNode::onImage, this, _1));
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_rect", 10);
  }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }

    if (cv_ptr->image.cols != IMG_WIDTH || cv_ptr->image.rows != IMG_HEIGHT) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Frame size doesn't match hardcoded IMG_WIDTH/IMG_HEIGHT");
      return;
    }

    cv::Mat undistorted;
    cv::remap(cv_ptr->image, undistorted, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    auto out_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, undistorted)
      .toImageMsg();
    pub_->publish(*out_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  cv::Mat map1_, map2_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FisheyeUndistortNode>());
  rclcpp::shutdown();
  return 0;
}
