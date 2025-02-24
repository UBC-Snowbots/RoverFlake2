#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "sweep/sweep.hpp" 

using namespace std::chrono_literals;

class SweepScanNode : public rclcpp::Node
{
public:
  SweepScanNode()
  : Node("sweep_scan_node")
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("frame_id", "laser");
    this->declare_parameter<int>("rotation_speed", 5);
    this->declare_parameter<int>("sample_rate", 1000);

    port_ = this->get_parameter("port").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    rotation_speed_ = this->get_parameter("rotation_speed").as_int();
    sample_rate_ = this->get_parameter("sample_rate").as_int();

    // Create publisher
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    try {
      // Initialize Sweep device
      sweep_context_ = std::make_shared<sweep::Sweep>(port_);
      sweep_context_->set_motor_speed(rotation_speed_);
      sweep_context_->set_sample_rate(sample_rate_);
      RCLCPP_INFO(this->get_logger(), "Starting scan...");
      sweep_context_->start_scanning();

      // Create a timer to call publish_scan() at 1/rotation_speed seconds
      auto period = std::chrono::duration<double>(1.0 / rotation_speed_);
      timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(period),
          std::bind(&SweepScanNode::publish_scan, this));

      RCLCPP_INFO(this->get_logger(), "Sweep scan node initialized successfully");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize Sweep: %s", e.what());
      if (sweep_context_) {
        sweep_context_->stop_scanning();
      }
      std::exit(EXIT_FAILURE);
    }
  }

  ~SweepScanNode()
  {
    if (sweep_context_) {
      try {
        sweep_context_->stop_scanning();
        RCLCPP_INFO(this->get_logger(), "Stopped scanning and closed Sweep device");
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error closing Sweep device: %s", e.what());
      }
    }
  }

private:
  void publish_scan()
  {
    // Get a scan (assumed to return a std::vector<Sample>)
    std::vector<Sample> scan = sweep_context_->get_scan();

    if (scan.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty scan");
      return;
    }

    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.angle_min = 0.0;
    msg.angle_max = 2.0 * M_PI;

    size_t num_samples = scan.size();
    msg.angle_increment = (msg.angle_max - msg.angle_min) / static_cast<double>(num_samples);
    msg.scan_time = 1.0 / static_cast<double>(rotation_speed_);
    msg.time_increment = msg.scan_time / static_cast<double>(num_samples);
    msg.range_min = 0.1;   // in meters
    msg.range_max = 40.0;  // in meters

    // Preallocate ranges and intensities
    msg.ranges.resize(num_samples, std::numeric_limits<float>::infinity());
    msg.intensities.resize(num_samples, 0.0);

    // Process each sample.
    // (Assuming the scan is already ordered in angle)
    for (size_t i = 0; i < num_samples; ++i) {
      const Sample & sample = scan[i];
      // Convert distance from centimeters to meters
      float distance_m = static_cast<float>(sample.distance) / 100.0f;
      if (distance_m >= msg.range_min && distance_m <= msg.range_max) {
        msg.ranges[i] = distance_m;
      } else {
        msg.ranges[i] = std::numeric_limits<float>::infinity();
      }
      msg.intensities[i] = static_cast<float>(sample.signal_strength);
    }

    scan_pub_->publish(msg);
  }

  // Member variables
  std::string port_;
  std::string frame_id_;
  int rotation_speed_;
  int sample_rate_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<sweep::Sweep> sweep_context_;  // Adjust namespace/type as per your Sweep library
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SweepScanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
