#include "mullet_core/drone_state_machine.hpp"

#include "action_lifecycle_coordinator.hpp"
#include "drone_state_machine_helpers.hpp"

namespace mullet_core
{

bool DroneStateMachine::haveActiveTakeoffGoal() const
{
  return action_lifecycle_->hasActiveTakeoffGoal();
}

bool DroneStateMachine::haveActiveLandGoal() const
{
  return action_lifecycle_->hasActiveLandGoal();
}

bool DroneStateMachine::haveActiveEnableOffboardGoal() const
{
  return action_lifecycle_->hasActiveEnableOffboardGoal();
}

bool DroneStateMachine::haveActiveDisableOffboardGoal() const
{
  return action_lifecycle_->hasActiveDisableOffboardGoal();
}

bool DroneStateMachine::haveActiveArmGoal() const
{
  return action_lifecycle_->hasActiveArmGoal();
}

bool DroneStateMachine::haveActiveDisarmGoal() const
{
  return action_lifecycle_->hasActiveDisarmGoal();
}

bool DroneStateMachine::activeTakeoffState() const
{
  return state_machine_detail::isActiveTakeoffState(state_);
}

bool DroneStateMachine::landActionAllowedFromState() const
{
  return state_machine_detail::isLandActionAllowedFromState(state_);
}

bool DroneStateMachine::goalCancelRequested() const
{
  return action_lifecycle_->takeoffCancelRequested() && haveActiveTakeoffGoal();
}

bool DroneStateMachine::enableOffboardCancelRequested() const
{
  return action_lifecycle_->enableOffboardCancelRequested() && haveActiveEnableOffboardGoal();
}

bool DroneStateMachine::disableOffboardCancelRequested() const
{
  return action_lifecycle_->disableOffboardCancelRequested() && haveActiveDisableOffboardGoal();
}

bool DroneStateMachine::shouldRecoverThroughLanding() const
{
  return px4_->isArmed() || takeoff_arm_requested_;
}

bool DroneStateMachine::landedConfirmed(const rclcpp::Time & current_time) const
{
  const double max_age_s = std::min(5.0, config_.landed_confirmation_s + 0.5);
  return px4_->landedContinuouslyFor(
    current_time, config_.landed_confirmation_s, max_age_s);
}

bool DroneStateMachine::directDisarmAllowed(const rclcpp::Time & current_time) const
{
  return px4_->hasVehicleStatus() && px4_->isArmed() && landedConfirmed(current_time);
}

const char * DroneStateMachine::stateName(DroneState state)
{
  switch (state) {
    case DroneState::kInit:
      return "INIT";
    case DroneState::kReady:
      return "READY";
    case DroneState::kEnableOffboardRequested:
      return "ENABLE_OFFBOARD_REQUESTED";
    case DroneState::kWarmupOffboardHoldSetpoints:
      return "WARMUP_OFFBOARD_HOLD_SETPOINTS";
    case DroneState::kRequestOffboardOnly:
      return "REQUEST_OFFBOARD_ONLY";
    case DroneState::kOffboardHold:
      return "OFFBOARD_HOLD";
    case DroneState::kArmRequested:
      return "ARM_REQUESTED";
    case DroneState::kArmCommand:
      return "ARM_COMMAND";
    case DroneState::kArmedOffboardHold:
      return "ARMED_OFFBOARD_HOLD";
    case DroneState::kDisarmCommand:
      return "DISARM_COMMAND";
    case DroneState::kReleaseOffboardRequest:
      return "RELEASE_OFFBOARD_REQUEST";
    case DroneState::kDisarmedOffboardReleaseFailed:
      return "DISARMED_OFFBOARD_RELEASE_FAILED";
    case DroneState::kDisableOffboardRequested:
      return "DISABLE_OFFBOARD_REQUESTED";
    case DroneState::kTakeoffRequested:
      return "TAKEOFF_REQUESTED";
    case DroneState::kWarmupSetpoints:
      return "WARMUP_SETPOINTS";
    case DroneState::kRequestOffboard:
      return "REQUEST_OFFBOARD";
    case DroneState::kRequestArm:
      return "REQUEST_ARM";
    case DroneState::kTakeoff:
      return "TAKEOFF";
    case DroneState::kHover:
      return "HOVER";
    case DroneState::kCommand:
      return "COMMAND";
    case DroneState::kCancelingToLand:
      return "CANCELING_TO_LAND";
    case DroneState::kLanding:
      return "LANDING";
    case DroneState::kTakeoffFailed:
      return "TAKEOFF_FAILED";
    case DroneState::kTakeoffCanceled:
      return "TAKEOFF_CANCELED";
    case DroneState::kFailsafeHold:
      return "FAILSAFE_HOLD";
    case DroneState::kDisarmed:
      return "DISARMED";
  }
  return "UNKNOWN";
}

}  // namespace mullet_core
