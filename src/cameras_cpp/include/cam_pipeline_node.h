

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"



class CamPipe : public rclcpp::Node {
public:
    CamPipe() : Node("camera_pipeline") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        //command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        //gear_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", qos);
        output_pub = this->create_publisher<sensor_msgs::msg::Image>("/cam_to_send_topic", qos);
        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        image_sub_0 = this->create_subscription<sensor_msgs::msg::Image>(
            "/cam0/raw", qos, [this](sensor_msgs::msg::Image::SharedPtr msg) {
                this->imageCallback(msg, 0);
              }
            );
            image_sub_0 = this->create_subscription<sensor_msgs::msg::Image>(
                "/cam1/raw", qos, [this](sensor_msgs::msg::Image::SharedPtr msg) {
                    this->imageCallback(msg, 1);
                  }
                );
            // std::system("notify-send 'CAMERA_PIPELINE is online'");
        // g29_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        //     "/joy", 10, std::bind(&SampleNode::joy_callback, this, std::placeholders::_1, 0));
    }

        ~CamPipe(){
            // std::system("notify-send 'CAMERA_PIPELINE is offline'");

        }


private:
 
    //Not going to worry about arrays here
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_0;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_1;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_pub;
    int selected_index = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg, int index);

};