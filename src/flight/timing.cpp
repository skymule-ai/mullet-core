#include "mullet_core/flight/drone_state_machine.hpp"

#include <algorithm>
#include <cmath>

namespace mullet_core
{

bool DroneStateMachine::phaseTimedOut(const rclcpp::Time & current_time, double timeout_s) const
{
  return (current_time - phase_enter_time_).seconds() >= timeout_s;
}

void DroneStateMachine::updateTimingStats(const rclcpp::Time & current_time)
{
  if (!timing_initialized_) {
    timing_initialized_ = true;
    last_tick_time_ = current_time;
    return;
  }

  const auto expected_period_ms = 1000.0 / std::max(config_.control_rate_hz, 1.0);
  const auto dt_ms = (current_time - last_tick_time_).seconds() * 1000.0;
  const auto jitter_ms = std::fabs(dt_ms - expected_period_ms);
  max_tick_jitter_ms_ = std::max(max_tick_jitter_ms_, jitter_ms);
  if (dt_ms > expected_period_ms * 1.5) {
    ++missed_deadline_count_;
  }
  last_tick_time_ = current_time;
}

void DroneStateMachine::maybeLogTiming(const rclcpp::Time & current_time)
{
  if ((current_time - last_timing_log_time_).seconds() < config_.timing_log_period_s) {
    return;
  }

  RCLCPP_INFO(
    get_logger(), "timing: state=%s max_jitter_ms=%.3f missed_deadlines=%lu",
    stateName(state_), max_tick_jitter_ms_, missed_deadline_count_);
  max_tick_jitter_ms_ = 0.0;
  last_timing_log_time_ = current_time;
}

}  // namespace mullet_core
