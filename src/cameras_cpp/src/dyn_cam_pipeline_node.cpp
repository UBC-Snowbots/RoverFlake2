#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

class CameraBroadcaster : public rclcpp::Node {
public:
  CameraBroadcaster() : Node("camera_broadcaster") {
    auto qos = rclcpp::QoS(1).best_effort();
    
    // Find all available cameras
    find_cameras();
    
    // Listen for which cam feed to display
    broadcast_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/broadcast_cameras", qos,
      [this](std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        std::vector<int32_t> indices(msg->data.begin(), msg->data.end());
        update_broadcasts(indices);
      });
    
    // Cam list for refeerence
    list_pub_ = this->create_publisher<std_msgs::msg::String>("/camera_list", qos);
    publish_camera_list();
    
    RCLCPP_INFO(this->get_logger(), "Camera broadcaster ready with %zu cameras", camera_topics_.size());
  }

private:
  std::vector<std::string> camera_topics_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> camera_subs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> broadcast_pubs_;
  std::vector<int> active_broadcasts_;
  
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr broadcast_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr list_pub_;

  void find_cameras() {
    auto topics = this->get_topic_names_and_types();
    
    for (auto& [name, types] : topics) {
      for (auto& type : types) {
        if (type == "sensor_msgs/msg/Image" && name.find("cam") != std::string::npos) {
          camera_topics_.push_back(name);
          RCLCPP_INFO(this->get_logger(), "Camera %zu: %s", camera_topics_.size()-1, name.c_str());
          break;
        }
      }
    }
    
    // Create subscribers for all cameras
    auto qos = rclcpp::QoS(1).best_effort();
    for (size_t i = 0; i < camera_topics_.size(); ++i) {
      auto sub = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topics_[i], qos,
        [this, i](sensor_msgs::msg::Image::SharedPtr msg) {
          // Only forward if this camera is being compressed
          for (size_t j = 0; j < active_broadcasts_.size(); ++j) {
            if (active_broadcasts_[j] == (int)i) {
              broadcast_pubs_[j]->publish(*msg);
              break;
            }
          }
        });
      camera_subs_.push_back(sub);
    }
  }
  
  void update_broadcasts(const std::vector<int32_t>& camera_indices) {
    // Clear existing broadcast publishers
    broadcast_pubs_.clear();
    active_broadcasts_.clear();
    
    auto qos = rclcpp::QoS(1).best_effort();
    
    // Create new publishers for requested cameras
    for (size_t i = 0; i < camera_indices.size(); ++i) {
      int cam_idx = camera_indices[i];
      if (cam_idx >= 0 && cam_idx < (int)camera_topics_.size()) {
        std::string topic = "/broadcast/cam_" + std::to_string(i) + "/image_raw";
        auto pub = this->create_publisher<sensor_msgs::msg::Image>(topic, qos);
        broadcast_pubs_.push_back(pub);
        active_broadcasts_.push_back(cam_idx);
        
        RCLCPP_INFO(this->get_logger(), "Broadcasting camera %d (%s) on %s", 
                   cam_idx, camera_topics_[cam_idx].c_str(), topic.c_str());
      }
    }
  }
  
  void publish_camera_list() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Available cameras:\n";
    for (size_t i = 0; i < camera_topics_.size(); ++i) {
      msg->data += std::to_string(i) + ": " + camera_topics_[i] + "\n";
    }
    list_pub_->publish(std::move(msg));
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraBroadcaster>());
  rclcpp::shutdown();
  return 0;
}