#pragma once

#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "moteus.h"

namespace moteus = mjbots::moteus;

#define NUM_JOINTS 6

class MoteusPoll : public rclcpp::Node {
public:
    MoteusPoll();

private:
    void poll();

    std::vector<std::shared_ptr<moteus::Controller>> controllers_;
    std::shared_ptr<moteus::Transport> transport_;
    rclcpp::TimerBase::SharedPtr timer_;
};
