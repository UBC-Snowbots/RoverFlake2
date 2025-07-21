// RoverUtils.hpp
// Simple header‑only utility that mirrors the C# helpers in rclcpp.
//
//   • getRosTimestamp()  → builtin_interfaces::msg::Time stamped with **system** clock
//   • createHeader()     → std_msgs::msg::Header with frame_id + current stamp
//
// Compile with:  target_link_libraries(your_target rclcpp)  and include this file.

#pragma once

#include <cstdint>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/header.hpp"

namespace rover_utils
{
/* -------------------------------------------------------------------------- */
/*  Low‑level helper – current time as builtin_interfaces::msg::Time          */
/* -------------------------------------------------------------------------- */
inline builtin_interfaces::msg::Time getRosTimestamp()
{
  // We ask the **system clock** so that it matches DateTime.UtcNow in the C# version.
  // If you need simulated / ROS time, swap RCL_SYSTEM_TIME for RCL_ROS_TIME or
  // call node->now().
  const rclcpp::Time now = rclcpp::Clock(RCL_SYSTEM_TIME).now();

  // rclcpp::Time gives nanoseconds as a 64‑bit integer – use integer math to
  // avoid the FP truncation the C# version does.
  const int64_t total_nsec = now.nanoseconds();

  builtin_interfaces::msg::Time stamp;
  stamp.sec     = static_cast<int32_t>(total_nsec / 1'000'000'000);
  stamp.nanosec = static_cast<uint32_t>(total_nsec % 1'000'000'000);
  return stamp;
}

/* -------------------------------------------------------------------------- */
/*  Convenience – make a Header with frame_id and current stamp               */
/* -------------------------------------------------------------------------- */
inline std_msgs::msg::Header createHeader(const std::string & frame_id = "rover_unset")
{
  std_msgs::msg::Header hdr;
  hdr.frame_id = frame_id;
  hdr.stamp    = getRosTimestamp();
  return hdr;
}

} // namespace rover_utils