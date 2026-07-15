#include "death_ray_magnetometer.h"

DeathRayMagnetometerNode::DeathRayMagnetometerNode() : Node("death_ray_magnetometer_node") {
    // Placeholder for real constructor while I bootstrap the node
    RCLCPP_INFO(this->get_logger(), "Starting magnetometer node");
}

DeathRayMagnetometerNode::~DeathRayMagnetometerNode() {

}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeathRayMagnetometerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
