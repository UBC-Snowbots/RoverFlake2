#include "death_ray_magnetometer.h"

DeathRayMagnetometerNode::DeathRayMagnetometerNode() : Node("death_ray_magnetometer_node") {
    heading_pub_ = this->create_publisher<std_msgs::msg::Float32>("death_ray_heading", 10);

    heading_feedback_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(HEADING_FEEDBACK_PUBLISH_FREQUENCY_MS),
        std::bind(&DeathRayMagnetometerNode::publishHeadingFeedback, this)
    );
}

DeathRayMagnetometerNode::~DeathRayMagnetometerNode() {
    // Placeholder for node cleanup
    // Explicit destructor will be needed to clean up GPIO/serial connections
}

void DeathRayMagnetometerNode::publishHeadingFeedback() {
    // TODO: update this to actually read heading from the GPIO pins
    // This is for sanity checking the node on my laptop
    std_msgs::msg::Float32 message;
    message.data = 180.0;

    heading_pub_->publish(message);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeathRayMagnetometerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
