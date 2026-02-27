#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sweep/sweep.hpp"

using namespace std::chrono_literals;

#define LOG_BOTH(msg, ...) \
  do { \
    printf(msg "\n", ##__VA_ARGS__); \
    fflush(stdout); \
    RCLCPP_INFO(get_logger(), msg, ##__VA_ARGS__); \
  } while(0)

#define LOG_ERROR(msg, ...) \
  do { \
    fprintf(stderr, "ERROR: " msg "\n", ##__VA_ARGS__); \
    fflush(stderr); \
    RCLCPP_ERROR(get_logger(), msg, ##__VA_ARGS__); \
  } while(0)

#define LOG_WARN(msg, ...) \
  do { \
    fprintf(stderr, "WARN: " msg "\n", ##__VA_ARGS__); \
    fflush(stderr); \
    RCLCPP_WARN(get_logger(), msg, ##__VA_ARGS__); \
  } while(0)

class SweepScannerNode : public rclcpp::Node
{
public:
  SweepScannerNode() try
    : Node("sweep_scan_node"),
      _scanner_thread_active{false}
  {
    printf("=== PURE SCAN NODE STARTING ===\n");
    fflush(stdout);

    // Declare parameters
    this->declare_parameter("topic", "scan");
    this->declare_parameter("serial_port", "/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DM00L4EB-if00-port0");  // usbport /dev/ttyUSB0
    this->declare_parameter("rotation_speed", 10);  // 1-10
    this->declare_parameter("sample_rate", 1000);  //500-1750
    this->declare_parameter("frame_id", "lidar");

    printf("Parameters declared\n");
    fflush(stdout);

    // Get parameters
    const std::string topic       = this->get_parameter("topic").as_string();
    const std::string serial_port = this->get_parameter("serial_port").as_string();
    _frame_id       = this->get_parameter("frame_id").as_string();
    _rotation_speed = this->get_parameter("rotation_speed").as_int();
    _sample_rate    = this->get_parameter("sample_rate").as_int();

    printf("node config:\n  topic: %s\n  port: %s\n  speed: %d Hz\n  rate: %d Hz\n  frame: %s\n",
      topic.c_str(), serial_port.c_str(), _rotation_speed, _sample_rate, _frame_id.c_str());
    fflush(stdout);

    RCLCPP_INFO(get_logger(),
      "node config:\n  topic: %s\n  port: %s\n  speed: %d Hz\n  rate: %d Hz\n  frame: %s",
      topic.c_str(), serial_port.c_str(), _rotation_speed, _sample_rate, _frame_id.c_str());

    printf("Initializing Sweep device...\n");
    fflush(stdout);

    // Initialize Sweep device
    _scanner = std::make_shared<sweep::sweep>(serial_port.c_str());

    printf("Sweep device initialized!\n");
    fflush(stdout);

    // Create publisher
    _lidar_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(topic, 10);

    printf("Publisher created, starting scanner thread...\n");
    fflush(stdout);

    // Start scanner thread
    _scanner_thread_active = true;
    _scanner_thread = std::thread(&SweepScannerNode::scannerThreadFunc, this);

    printf("Scanner thread started!\n");
    fflush(stdout);
  }
  catch (sweep::device_error const & e)
  {
    fprintf(stderr, "CONSTRUCTOR ERROR (sweep): %s\n", e.what());
    fflush(stderr);
    RCLCPP_ERROR(get_logger(), "%s", e.what());
  }
  catch (std::exception const & e)
  {
    fprintf(stderr, "CONSTRUCTOR ERROR (std): %s\n", e.what());
    fflush(stderr);
  }
  catch (...)
  {
    fprintf(stderr, "CONSTRUCTOR ERROR (unknown)\n");
    fflush(stderr);
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
    printf("Step 1: Configuring Sweep scanner.\n"); fflush(stdout);

    printf("Step 2: Setting motor speed to %d\n", _rotation_speed); fflush(stdout);
    _scanner->set_motor_speed(_rotation_speed);

    printf("Step 3: Waiting for motor ready...\n"); fflush(stdout);
    while (!_scanner->get_motor_ready())
      std::this_thread::sleep_for(100ms);
    printf("Motor ready!\n"); fflush(stdout);

    printf("Step 4: Setting sample rate to %d\n", _sample_rate); fflush(stdout);
    _scanner->set_sample_rate(_sample_rate);

    printf("Step 5: Starting scanning...\n"); fflush(stdout);
    _scanner->start_scanning();

    printf("Step 6: Starting data acquisition loop.\n"); fflush(stdout);
    int scan_count = 0;

    while (_scanner_thread_active)
    {
      printf("Step 7.%d: Requesting scan #%d\n", scan_count, scan_count); fflush(stdout);

      try {
        sweep::scan scan = _scanner->get_scan();
        printf("Step 8.%d: Scan received with %zu samples\n", scan_count, scan.samples.size()); fflush(stdout);

        if (scan.samples.empty())
        {
          printf("WARN: Empty scan received, skipping\n"); fflush(stdout);
          continue;
        }

        printf("Step 9.%d: Creating LaserScan message\n", scan_count); fflush(stdout);
        sensor_msgs::msg::LaserScan laser_scan_msg;
        laser_scan_msg.header.frame_id = _frame_id;
        laser_scan_msg.header.stamp = this->now();

        laser_scan_msg.angle_min       = 0.0;
        laser_scan_msg.angle_max       = 2.0 * M_PI;
        laser_scan_msg.angle_increment = (2.0 * M_PI) / static_cast<float>(scan.samples.size());
        laser_scan_msg.time_increment  = 1.0f / static_cast<float>(_sample_rate);
        laser_scan_msg.range_min       = 0.0f;
        laser_scan_msg.range_max       = 25.0f;

        printf("Step 10.%d: Reserving space for %zu samples\n", scan_count, scan.samples.size()); fflush(stdout);
        laser_scan_msg.ranges.reserve(scan.samples.size());
        laser_scan_msg.intensities.reserve(scan.samples.size());

        printf("Step 11.%d: Processing samples...\n", scan_count); fflush(stdout);

        for (size_t idx = 0; idx < scan.samples.size(); ++idx)
        {
          try {
            const auto& sample = scan.samples[idx];
            int32_t angle_milli_deg = sample.angle;
            int32_t distance_cm = sample.distance;
            int32_t signal_strength = sample.signal_strength;

            float range_m = static_cast<float>(distance_cm) / 100.0f;
            float angle_rad = static_cast<float>(angle_milli_deg) / 1000.0f * M_PI / 180.0f;

            laser_scan_msg.ranges.push_back(range_m);
            laser_scan_msg.intensities.push_back(static_cast<float>(signal_strength));

            if (idx % 50 == 0)
            {
              printf("  [%zu/%zu] angle=%d milli-deg (%.2f rad), dist=%d cm (%.2f m), signal=%d\n",
                idx, scan.samples.size(), angle_milli_deg, angle_rad, distance_cm, range_m, signal_strength);
              fflush(stdout);
            }
          }
          catch (std::exception const & e)
          {
            fprintf(stderr, "ERROR: Error processing sample %zu: %s\n", idx, e.what()); fflush(stderr);
            break;
          }
        }

        printf("Step 12.%d: Publishing scan with %zu points\n", scan_count, laser_scan_msg.ranges.size()); fflush(stdout);
        _lidar_pub->publish(laser_scan_msg);
        scan_count++;
        printf("Step 13.%d: Scan #%d published successfully\n", scan_count, scan_count); fflush(stdout);
      }
      catch (std::exception const & e)
      {
        fprintf(stderr, "ERROR: Error in scan loop iteration %d: %s\n", scan_count, e.what()); fflush(stderr);
        std::this_thread::sleep_for(500ms);
      }
    }

    printf("Step 14: Stopping scanning...\n"); fflush(stdout);
    _scanner->stop_scanning();
    printf("Step 15: Scanner stopped.\n"); fflush(stdout);
  }
  catch (sweep::device_error const & e)
  {
    fprintf(stderr, "FATAL: Sweep device error: %s\n", e.what()); fflush(stderr);
    try {
      _scanner->stop_scanning();
    } catch (...) {}
  }
  catch (std::exception const & e)
  {
    fprintf(stderr, "FATAL: Standard exception: %s\n", e.what()); fflush(stderr);
    try {
      _scanner->stop_scanning();
    } catch (...) {}
  }
  catch (...)
  {
    fprintf(stderr, "FATAL: Unknown exception caught in scanner thread\n"); fflush(stderr);
    try {
      _scanner->stop_scanning();
    } catch (...) {}
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
