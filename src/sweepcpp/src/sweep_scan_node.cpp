#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sweep/sweep.hpp"

using namespace std::chrono_literals;

class SweepScannerNode : public rclcpp::Node
{
public:
  SweepScannerNode() try
    : Node("sweep_scan_node"),
      _scanner_thread_active{false}
  {
    // Declare parameters
    this->declare_parameter("topic", "scan");
    this->declare_parameter("serial_port", "/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DM00L4EB-if00-port0"); // usbport /dev/ttyUSB0 
    this->declare_parameter("rotation_speed", 10); // 1-10
    this->declare_parameter("sample_rate", 1000); //500-1750
    this->declare_parameter("frame_id", "lidar");

    // Get parameters
    const std::string topic       = this->get_parameter("topic").as_string();
    const std::string serial_port = this->get_parameter("serial_port").as_string();
    _frame_id       = this->get_parameter("frame_id").as_string();
    _rotation_speed = this->get_parameter("rotation_speed").as_int();
    _sample_rate    = this->get_parameter("sample_rate").as_int();

    RCLCPP_INFO(get_logger(), 
      "node config:\n  topic: %s\n  port: %s\n  speed: %d Hz\n  rate: %d Hz\n  frame: %s",
      topic.c_str(), serial_port.c_str(), _rotation_speed, _sample_rate, _frame_id.c_str());

    // Initialize Sweep device
    _scanner = std::make_shared<sweep::sweep>(serial_port.c_str());

    // Create publisher
    _lidar_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(topic, 10);

    // Start scanner thread
    _scanner_thread_active = true;
    _scanner_thread = std::thread(&SweepScannerNode::scannerThreadFunc, this);
  }
  catch (sweep::device_error const & e)
  {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
  }

  ~SweepScannerNode() override
  {
    _scanner_thread_active = false;
    if (_scanner_thread.joinable())
      _scanner_thread.join();
  }

private:
  void scannerThreadFunc() try
  {
    RCLCPP_INFO(get_logger(), "Configuring Sweep scanner.");
    _scanner->set_motor_speed(_rotation_speed);
    while (!_scanner->get_motor_ready())
      std::this_thread::sleep_for(100ms);

    _scanner->set_sample_rate(_sample_rate);
    _scanner->start_scanning();

    RCLCPP_INFO(get_logger(), "Starting data acquisition.");
    while (_scanner_thread_active)
    {
      // Obtain a full scan
      sweep::scan scan = _scanner->get_scan();

      sensor_msgs::msg::LaserScan laser_scan_msg;
      laser_scan_msg.header.frame_id = _frame_id;
      laser_scan_msg.header.stamp = this->now();
      
      // Calculate samples per rotation
      float samples_per_rotation = static_cast<float>(_sample_rate) / static_cast<float>(_rotation_speed);

      laser_scan_msg.angle_min       = 0.0;
      laser_scan_msg.angle_max       = 2.0 * M_PI;
      laser_scan_msg.angle_increment = laser_scan_msg.angle_max / samples_per_rotation;
      laser_scan_msg.time_increment  = 1.0f / static_cast<float>(_sample_rate);
      laser_scan_msg.range_min       = 0.0f;
      laser_scan_msg.range_max       = 25.0f;

      // Pre-fill ranges with infinity
      laser_scan_msg.ranges.assign(scan.samples.size(), std::numeric_limits<float>::infinity());

      size_t idx = 0;
      for (auto [angle_milli_deg, distance_cm, signal_strength] : scan.samples)
      {
        laser_scan_msg.ranges[idx] = static_cast<float>(distance_cm) / 100.0f;
        ++idx;
      }

      _lidar_pub->publish(laser_scan_msg);
    }
    _scanner->stop_scanning();
  }
  catch (sweep::device_error const & e)
  {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    _scanner->stop_scanning();
  }

  // Member variables
  std::string _frame_id;
  int _rotation_speed;
  int _sample_rate;
  std::shared_ptr<sweep::sweep> _scanner;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_pub;
  std::thread _scanner_thread;
  bool _scanner_thread_active;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SweepScannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
