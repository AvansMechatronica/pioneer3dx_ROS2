#ifndef AR_TIME_TO_ROS2_TIMESTAMP_HPP
#define AR_TIME_TO_ROS2_TIMESTAMP_HPP

#include <rclcpp/rclcpp.hpp>

#ifdef ADEPT_PKG
  #include "ariaUtil.h"
#else
  #include "Aria/ariaUtil.h"
#endif

// Convert ARIA ArTime to ROS 2 rclcpp::Time
inline rclcpp::Time convertArTimeToROS2(const ArTime& t)
{
  // ARIA times are relative to an internal clock, so find how long ago 't' was
  ArTime arianow;
  const double dt_sec = static_cast<double>(t.mSecSince(arianow)) / 1000.0;

  // Compute ROS 2 timestamp by subtracting that delta from now
  auto now = rclcpp::Clock().now();
  auto adjusted_time = now - rclcpp::Duration::from_seconds(dt_sec);

  return adjusted_time;
}

#endif  // AR_TIME_TO_ROS2_TIMESTAMP_HPP
