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
  constexpr float kPi = 3.14159265358979323846F;
  constexpr float kTwoPi = 2.0F * kPi;
  auto wrapped = static_cast<float>(std::remainder(angle_rad, kTwoPi));
  if (wrapped <= -kPi) {
    wrapped += kTwoPi;
  }
  return wrapped;
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
