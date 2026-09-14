#ifndef MULLET_CORE_PX4_TOPIC_UTILS_HPP_
#define MULLET_CORE_PX4_TOPIC_UTILS_HPP_

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace mullet_core
{

// Builds a PX4 uXRCE-DDS topic name from a vehicle FMU namespace prefix and a topic suffix.
//
// Args:
//   fmu_prefix: Vehicle FMU namespace, e.g. "/px4_1", or empty for the default (unprefixed)
//     vehicle.
//   suffix: Topic suffix starting with '/', e.g. "/fmu/in/vehicle_command".
//
// Returns:
//   The fully-qualified topic name.
inline std::string px4Topic(const std::string & fmu_prefix, const std::string & suffix)
{
  if (fmu_prefix.empty()) {
    return suffix;
  }
  if (fmu_prefix.front() == '/') {
    return fmu_prefix + suffix;
  }
  return "/" + fmu_prefix + suffix;
}

// Returns the current time in microseconds, as required by the `timestamp` field on PX4
// messages.
//
// Args:
//   clock: ROS clock used as the timestamp source.
//
// Returns:
//   Current clock time converted to microseconds.
inline std::uint64_t px4NowUs(rclcpp::Clock & clock)
{
  return static_cast<std::uint64_t>(clock.now().nanoseconds() / 1000);
}

// QoS profile shared by all PX4 uXRCE-DDS publishers and subscriptions: best-effort,
// transient-local, keep-last(1), matching what PX4's micro-ROS agent expects.
//
// Returns:
//   The PX4-compatible ROS QoS profile.
inline rclcpp::QoS px4Qos()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.transient_local();
  return qos;
}

// Converts a publish rate in Hz to a wall-timer period, clamping the rate to `min_hz` first so a
// misconfigured (zero or negative) rate parameter can't produce a zero/negative period.
//
// Args:
//   rate_hz: Requested publish rate in hertz.
//   min_hz: Minimum allowed publish rate in hertz.
//
// Returns:
//   The clamped rate expressed as a nanosecond timer period.
inline std::chrono::nanoseconds ratePeriod(double rate_hz, double min_hz)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / std::max(rate_hz, min_hz)));
}

}  // namespace mullet_core

#endif  // MULLET_CORE_PX4_TOPIC_UTILS_HPP_
