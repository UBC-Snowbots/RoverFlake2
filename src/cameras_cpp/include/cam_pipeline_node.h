// This is to be run ON BOARD the rover

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int16.hpp"



class CamPipe : public rclcpp::Node {
public:
    CamPipe() : Node("camera_pipeline", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        //* Parameters
        rclcpp::Parameter output_topic_parameter;
        rclcpp::Parameter input_topics_parameter;
        rclcpp::Parameter cam_selection_topic_parameter;
        this->get_parameter("cam_selection_topic", cam_selection_topic_parameter);
        this->get_parameter("cam_input_topics", input_topics_parameter);
        this->get_parameter("cam_output_topic", output_topic_parameter);
        std::string cam_output_topic = output_topic_parameter.as_string();
        std::string cam_selection_topic = cam_selection_topic_parameter.as_string();
        std::vector<std::string> input_topics = input_topics_parameter.as_string_array();
        int num_input_feeds = input_topics.size();
        image_subs.resize(num_input_feeds);
        RCLCPP_INFO(this->get_logger(), "I found %d input topics. Will create a subscriber for each one:", num_input_feeds);
        {
            int i = 0;
            //* Dynamically create subscribers based on how many image feeds
            for(const auto& topic : input_topics){
                RCLCPP_INFO(this->get_logger(), "%s", topic.c_str());
                image_subs[i] = this->create_subscription<sensor_msgs::msg::Image>(
                    topic, qos, [this, i](sensor_msgs::msg::Image::SharedPtr msg) {
                        this->imageCallback(msg, i);
                      }
                    );
                i++;
            }
        }


        //command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        //gear_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", qos);
        output_pub = this->create_publisher<sensor_msgs::msg::Image>(cam_output_topic, qos);
        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));

        image_sub_0 = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", qos, [this](sensor_msgs::msg::Image::SharedPtr msg) {
                this->imageCallback(msg, 1);
              }
            );
            image_sub_1 = this->create_subscription<sensor_msgs::msg::Image>(
                "/vehicle_1/rear_feed/image_raw", qos, [this](sensor_msgs::msg::Image::SharedPtr msg) {
                    this->imageCallback(msg, 2);
                  }
                );
        // image_sub_0 = this->create_subscription<sensor_msgs::msg::Image>(
        //     "/cam0/raw", qos, [this](sensor_msgs::msg::Image::SharedPtr msg) {
        //         this->imageCallback(msg, 0);
        //       }
        //     );
        //     image_sub_0 = this->create_subscription<sensor_msgs::msg::Image>(
        //         "/cam1/raw", qos, [this](sensor_msgs::msg::Image::SharedPtr msg) {
        //             this->imageCallback(msg, 1);
        //           }
        //         );

        selection_sub = this->create_subscription<std_msgs::msg::Int16>(
            cam_selection_topic, qos, std::bind(&CamPipe::selectIndexCallback, this, std::placeholders::_1));
            // std::system("notify-send 'CAMERA_PIPELINE is online'");
        // g29_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        //     "/joy", 10, std::bind(&SampleNode::joy_callback, this, std::placeholders::_1, 0));
    }

        ~CamPipe(){
            // std::system("notify-send 'CAMERA_PIPELINE is offline'"); //? commenting this out as no point in sending a notification on board, aint no monitor on board the rover!

        }


private:

    //Not going to worry about arrays here
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_0;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_1;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_pub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr selection_sub;
    int selected_index = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg, int index);
    void selectIndexCallback(const std_msgs::msg::Int16::SharedPtr msg);
};
