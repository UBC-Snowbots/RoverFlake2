// encode_feed_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

class EncodeFeed : public rclcpp::Node {
public:
  explicit EncodeFeed(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("encode_feed", options) 
  {
    // Parameters
    this->declare_parameter<int>("camera", 1); // kept for compatibility
    camera_ = this->get_parameter("camera").as_int();

    // Kept your original topic name
    input_topic_ = "color_detection_annotated";

    // image_transport publisher (same name you used before)
    image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    pub_ = image_transport_->advertise("color_detected_auto", 1);

    // Raw image subscriber (unchanged API)
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_topic_, 5,
      std::bind(&EncodeFeed::onImage, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Node initialization successful. Republishing images without overlays.");
  }

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      // This preserves encoding and data; zero copy if possible for some encodings
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      pub_.publish(cv_ptr->toImageMsg()); // same header/encoding
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge error: %s", e.what());
    }
  }

  int camera_;
  std::string input_topic_;
  image_transport::Publisher pub_;
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EncodeFeed>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
