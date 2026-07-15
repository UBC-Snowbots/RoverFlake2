#ifndef DEATH_RAY_MAGNETOMETER_H
#define DEATH_RAY_MAGNETOMETER_H

#include "rclcpp/rclcpp.hpp"

/**
 * @brief DeathRayMagnetometerNode reads the direction the death ray is facing
 * and publishes it
 */
class DeathRayMagnetometerNode : public rclcpp::Node {
public:
    DeathRayMagnetometerNode();
    ~DeathRayMagnetometerNode();

private:
    float heading = 0.0;
};

#endif
