#include "drone_state_machine_helpers.hpp"

#include <cmath>

namespace mullet_core::state_machine_detail
{

bool retryDue(const rclcpp::Time & now, const rclcpp::Time & last, double interval_s)
{
  return last.nanoseconds() == 0 || (now - last).seconds() >= interval_s;
}

bool finiteInRange(double value, double min_value, double max_value)
{
  return std::isfinite(value) && value >= min_value && value <= max_value;
}

float wrapPi(float angle_rad)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  constexpr double kBoundaryTolerance = 1e-6;
  const auto wrapped = std::remainder(static_cast<double>(angle_rad), kTwoPi);
  if (wrapped <= -kPi || std::fabs(wrapped + kPi) <= kBoundaryTolerance) {
    return static_cast<float>(kPi);
  }
  return static_cast<float>(wrapped);
}

bool isActiveTakeoffState(DroneState state)
{
  switch (state) {
    case DroneState::kTakeoffRequested:
    case DroneState::kWarmupSetpoints:
    case DroneState::kRequestOffboard:
    case DroneState::kRequestArm:
    case DroneState::kTakeoff:
      return true;
    case DroneState::kInit:
    case DroneState::kReady:
    case DroneState::kEnableOffboardRequested:
    case DroneState::kWarmupOffboardHoldSetpoints:
    case DroneState::kRequestOffboardOnly:
    case DroneState::kOffboardHold:
    case DroneState::kArmRequested:
    case DroneState::kArmCommand:
    case DroneState::kArmedOffboardHold:
    case DroneState::kDisarmCommand:
    case DroneState::kReleaseOffboardRequest:
    case DroneState::kDisarmedOffboardReleaseFailed:
    case DroneState::kDisableOffboardRequested:
    case DroneState::kHover:
    case DroneState::kCommand:
    case DroneState::kCancelingToLand:
    case DroneState::kLanding:
    case DroneState::kTakeoffFailed:
    case DroneState::kTakeoffCanceled:
    case DroneState::kFailsafeHold:
    case DroneState::kDisarmed:
      return false;
  }
  return false;
}

bool isLandActionAllowedFromState(DroneState state)
{
  return state == DroneState::kHover || state == DroneState::kCommand ||
         isActiveTakeoffState(state);
}

}  // namespace mullet_core::state_machine_detail
