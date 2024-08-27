#include "rclcpp/rclcpp.hpp"
#include <pcl/point_types.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sweep/sweep.hpp>  

class SweepNode : public rclcpp::Node
{
public:
    SweepNode() : Node("sweep_node")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("serial_baudrate", 115200);
        this->declare_parameter<int>("rotation_speed", 5);
        this->declare_parameter<int>("sample_rate", 500);
        this->declare_parameter<std::string>("frame_id", "laser_frame");

        auto serial_port = this->get_parameter("serial_port").as_string();
        auto serial_baudrate = this->get_parameter("serial_baudrate").as_int();
        auto rotation_speed = this->get_parameter("rotation_speed").as_int();
        auto sample_rate = this->get_parameter("sample_rate").as_int();
        auto frame_id = this->get_parameter("frame_id").as_string();

        // Setup publisher
        scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc2", 10);

        // Create Sweep Driver Object
        sweep_device_ = std::make_shared<sweep::sweep>(serial_port.c_str());

        // Set Rotation Speed and Sample Rate
        sweep_device_->set_motor_speed(rotation_speed);
        sweep_device_->set_sample_rate(sample_rate);

        RCLCPP_INFO(this->get_logger(), "Expected rotation frequency: %ld (Hz)", rotation_speed);

        // Start scanning
        sweep_device_->start_scanning();

        // Create a timer to call the publish_scan method periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200), std::bind(&SweepNode::publish_scan, this));
    }

    ~SweepNode()
    {
        sweep_device_->stop_scanning();
    }

private:
    void publish_scan()
    {
        const sweep::scan scan = sweep_device_->get_scan();

        pcl::PointCloud<pcl::PointXYZ> cloud;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        rclcpp::Time ros_now = this->get_clock()->now();

        cloud.height = 1;
        cloud.width = scan.samples.size();
        cloud.points.resize(cloud.width * cloud.height);

        int i = 0;
        for (const sweep::sample &sample : scan.samples)
        {
            float angle = static_cast<float>(sample.angle) / 1000.0f; // millidegrees to degrees
            int32_t range = sample.distance;

            // Polar to Cartesian Conversion
            float x = (range * std::cos(DEG2RAD(angle))) / 100.0f;
            float y = (range * std::sin(DEG2RAD(angle))) / 100.0f;

            cloud.points[i].x = x;
            cloud.points[i].y = y;
            i++;
        }

        // Convert pcl PointCloud to ROS PointCloud2 message
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.frame_id = this->get_parameter("frame_id").as_string();
        cloud_msg.header.stamp = ros_now;

        RCLCPP_DEBUG(this->get_logger(), "Publishing a full scan");
        scan_pub_->publish(cloud_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
    std::shared_ptr<sweep::sweep> sweep_device_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SweepNode>();
    rclcpp::spin(node);
    // try
    // {
    //     rclcpp::spin(node);
    // }
    // catch (const sweep::device_error &e)
    // {
        // RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
    // }

    rclcpp::shutdown();
    return 0;
}
