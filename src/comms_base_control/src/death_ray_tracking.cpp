#include "death_ray_tracking.h"

DeathRayTracking::DeathRayTracking() : Node("death_ray_tracking_node") {
    RCLCPP_INFO(this->get_logger(), "DeathRayTracking node initialized.");

    death_ray_pub_ = this->create_publisher<std_msgs::msg::Float32>("death_ray_commands", rclcpp::QoS(10));

    rover_gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gnss_fix", rclcpp::QoS(10), std::bind(&DeathRayTracking::roverGnssCallback, this, std::placeholders::_1));
    
    comms_gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/base_fix", rclcpp::QoS(10), std::bind(&DeathRayTracking::commsGnssCallback, this, std::placeholders::_1));

    magnometer_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/death_ray_feedback", rclcpp::QoS(10), std::bind(&DeathRayTracking::magnetometerCallback, this, std::placeholders::_1));

    tracking_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&DeathRayTracking::trackingLoop, this));

    this->rover_latitude_ = 0.0;
    this->rover_longitude_ = 0.0;
    this->rover_has_fix_ = false;
    this->comms_latitude_ = 0.0;
    this->comms_longitude_ = 0.0;
    this->comms_has_fix_ = false;
    this->magnetometer_feedback_ = 0.0;
    this->tracking_active_ = true;
}

DeathRayTracking::~DeathRayTracking() {
    RCLCPP_INFO(this->get_logger(), "DeathRayTracking node destroyed.");
}

/**
 * Main tracking loop that runs at a fixed interval.
 * This function checks if tracking is active and if valid GNSS coordinates are available for both the rover and the comms base.
 * If both conditions are met, it performs tracking calculations.
 */
void DeathRayTracking::trackingLoop() {
    if (!tracking_active_) {
        RCLCPP_DEBUG(this->get_logger(), "Tracking is inactive. Skipping tracking loop.");
        return;
    }

    // Check if we have valid GNSS coordinates for both the rover and the comms base
    if (!rover_has_fix_ || !comms_has_fix_) {
        RCLCPP_DEBUG(this->get_logger(), "Invalid GNSS coordinates. Skipping tracking loop.");
        return;
    }

    double delta_lon = rover_longitude_ - comms_longitude_;
    double y = sin(delta_lon) * cos(rover_latitude_);
    double x = cos(comms_latitude_) * sin(rover_latitude_) - sin(comms_latitude_) * cos(rover_latitude_) * cos(delta_lon);
    double bearing = atan2(y, x);
    bearing = radToDeg(bearing);
    double normalized_bearing = fmod((bearing + 360.0), 360.0);
    double error = normalized_bearing - magnetometer_feedback_;
    if (error > 180.0) {
        error -= 360.0;
    } else if (error < -180.0) {
        error += 360.0;
    }

    RCLCPP_DEBUG(this->get_logger(), "Tracking error: %f", error);

    // Clamp the error to a maximum of 5 degrees since the loop runs every 100ms
    error = std::clamp(error, -5, 5);
    death_ray_pub_->publish(std_msgs::msg::Float32().set__data(static_cast<float>(error)));
}

/**
 * Callback for the /rover_fix subscriber.
 * Tracks the rover based on the received NavSatFix messages.
 * 
 */
void DeathRayTracking::roverGnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received rover GNSS fix: Latitude: %f, Longitude: %f", msg->latitude, msg->longitude);
    
    if (msg->latitude != 0.0 && msg->longitude != 0.0) {
        this->rover_latitude_ = degToRad(msg->latitude);
        this->rover_longitude_ = degToRad(msg->longitude);
        this->rover_has_fix_ = true;
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Did not update rover coordinates since one or both were zero.");
    }
}

/**
 * Callback for the /base_fix subscriber.
 * Receives and decodes NavSatFix messages.
 * 
 * This function updates the comms GNSS coordinates used for tracking.
 * If the received coordinates are valid (non-zero), they are stored for use in tracking calculations.
 */
void DeathRayTracking::commsGnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received comms GNSS fix: Latitude: %f, Longitude: %f", msg->latitude, msg->longitude);
    if (msg->latitude != 0.0 && msg->longitude != 0.0) {
        this->comms_latitude_ = degToRad(msg->latitude);
        this->comms_longitude_ = degToRad(msg->longitude);
        this->comms_has_fix_ = true;
    }
}

/**
 * Callback for the /death_ray_feedback subscriber.
 * Receives and decodes Float32 messages from the magnetometer.
 * 
 * This function updates the magnetometer feedback value used for tracking.
 */
void DeathRayTracking::magnetometerCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received magnetometer feedback: %f", msg->data);
    this->magnetometer_feedback_ = msg->data;
}

double DeathRayTracking::degToRad(double degrees) {
    return degrees * (M_PI / 180.0);
}

double DeathRayTracking::radToDeg(double radians) {
    return radians * (180.0 / M_PI);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeathRayTracking>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}