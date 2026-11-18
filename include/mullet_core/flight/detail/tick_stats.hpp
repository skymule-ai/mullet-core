#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace mullet_core::state_machine_detail
{

/// Control-loop timing health: worst-case tick jitter and late ticks, logged periodically.
///
/// Measured on `std::chrono::steady_clock`, the clock the node's wall timer fires on, so the
/// numbers stay meaningful under `use_sim_time`. Callers pass the time in; nothing here reads a
/// clock, which keeps it deterministic to test.
class TickStats
{
public:
  using Clock = std::chrono::steady_clock;

  /// Records one control tick. The first call only sets the baseline.
  ///
  /// Args:
  ///   now: Steady time at the start of this tick.
  ///   control_rate_hz: Expected tick rate; clamped to at least 1 Hz.
  void update(Clock::time_point now, double control_rate_hz)
  {
    if (!last_tick_time_) {
      last_tick_time_ = now;
      return;
    }

    const double expected_period_ms = 1000.0 / std::max(control_rate_hz, 1.0);
    const double dt_ms = Millis(now - *last_tick_time_).count();
    max_jitter_ms_ = std::max(max_jitter_ms_, std::fabs(dt_ms - expected_period_ms));
    if (dt_ms > expected_period_ms * kLateTickFactor) {
      ++missed_deadline_count_;
    }
    last_tick_time_ = now;
  }

  /// Logs the stats at most once every `log_period_s`, then resets the max jitter.
  ///
  /// The missed-deadline count is cumulative and never reset. The first call only starts the
  /// log period, so nothing is logged before a full period has elapsed.
  ///
  /// Args:
  ///   logger: Logger of the owning node.
  ///   now: Steady time, the same clock as `update()`.
  ///   log_period_s: Minimum interval between log lines.
  ///   state_label: Current FSM state wire label, included in the log line.
  void maybeLog(
    const rclcpp::Logger & logger, Clock::time_point now, double log_period_s,
    const char * state_label)
  {
    if (!last_log_time_) {
      last_log_time_ = now;
      return;
    }
    if (std::chrono::duration<double>(now - *last_log_time_).count() < log_period_s) {
      return;
    }

    RCLCPP_INFO(
      logger, "timing: state=%s max_jitter_ms=%.3f missed_deadlines=%zu",
      state_label, max_jitter_ms_, missed_deadline_count_);
    max_jitter_ms_ = 0.0;
    last_log_time_ = now;
  }

private:
  using Millis = std::chrono::duration<double, std::milli>;

  // A tick this many periods after the previous one counts as a missed deadline.
  static constexpr double kLateTickFactor = 1.5;

  std::optional<Clock::time_point> last_tick_time_;  // Empty until the first update().
  std::optional<Clock::time_point> last_log_time_;   // Empty until the first maybeLog().
  double max_jitter_ms_{0.0};
  std::size_t missed_deadline_count_{0};
};

}  // namespace mullet_core::state_machine_detail
