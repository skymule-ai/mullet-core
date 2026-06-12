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
  const double max_age_s = std::max(5.0, config_.landed_confirmation_s + 0.5);
  return px4_->landedContinuouslyFor(
    current_time, config_.landed_confirmation_s, max_age_s);
}

bool DroneStateMachine::directDisarmAllowed(const rclcpp::Time & current_time) const
{
  return px4_->hasVehicleStatus() && px4_->isArmed() && landedConfirmed(current_time);
}

bool DroneStateMachine::anyActionActive() const
{
  return haveActiveTakeoffGoal() || haveActiveLandGoal() || haveActiveEnableOffboardGoal() ||
         haveActiveDisableOffboardGoal() || haveActiveArmGoal() || haveActiveDisarmGoal();
}

bool DroneStateMachine::canTakeoff() const
{
  if (anyActionActive() || state_ == DroneState::kCancelingToLand || state_ == DroneState::kLanding) {
    return false;
  }
  if (state_ != DroneState::kReady && state_ != DroneState::kArmedHold)
  {
    return false;
  }
  if (!px4_->hasVehicleStatus() || !px4_->localPositionValid()) {
    return false;
  }
  if (px4_->isArmed() && state_ != DroneState::kArmedHold) {
    return false;
  }
  return true;
}

bool DroneStateMachine::canLand(const rclcpp::Time & current_time) const
{
  if (haveActiveLandGoal() || haveActiveDisableOffboardGoal() ||
    haveActiveArmGoal() || haveActiveDisarmGoal() || haveActiveEnableOffboardGoal())
  {
    return false;
  }
  const bool interrupting_takeoff = activeTakeoffState();
  if (!landActionAllowedFromState()) {
    return false;
  }
  if (interrupting_takeoff && !haveActiveTakeoffGoal()) {
    return false;
  }
  if (!interrupting_takeoff && haveActiveTakeoffGoal()) {
    return false;
  }
  if (!px4_->hasVehicleStatus()) {
    return false;
  }
  return interrupting_takeoff || (px4_->isArmed() && !landedConfirmed(current_time));
}

bool DroneStateMachine::canEnableOffboard() const
{
  return state_ == DroneState::kReady && !anyActionActive() &&
         px4_->hasVehicleStatus() && px4_->localPositionValid() &&
         !px4_->isArmed() && !px4_->isOffboard();
}

bool DroneStateMachine::canDisableOffboard() const
{
  return state_ == DroneState::kReady && !anyActionActive() &&
         px4_->hasVehicleStatus() && px4_->localPositionValid() &&
         !px4_->isArmed() && px4_->isOffboard();
}

bool DroneStateMachine::canArm(const rclcpp::Time & current_time) const
{
  if (anyActionActive()) {
    return false;
  }
  if (state_ == DroneState::kArmedHold) {
    return px4_->isArmed() && landedConfirmed(current_time);
  }
  if (state_ != DroneState::kReady) {
    return false;
  }
  if (!px4_->hasVehicleStatus() || !px4_->localPositionValid() || !px4_->hasLandDetected()) {
    return false;
  }
  if (px4_->isArmed() || !landedConfirmed(current_time)) {
    return false;
  }
  return true;
}

bool DroneStateMachine::canDisarm(const rclcpp::Time & current_time) const
{
  if (haveActiveDisarmGoal() || haveActiveDisableOffboardGoal() ||
    state_ == DroneState::kReleaseOffboardRequest ||
    state_ == DroneState::kDisarmedOffboardReleaseFailed)
  {
    return false;
  }
  return state_ == DroneState::kArmedHold && directDisarmAllowed(current_time);
}

bool DroneStateMachine::canEmergencyLand(const rclcpp::Time & current_time) const
{
  if (state_ == DroneState::kDisarmed || state_ == DroneState::kLanding) {
    return false;
  }
  return px4_->hasVehicleStatus() && px4_->isArmed() && !landedConfirmed(current_time);
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
    case DroneState::kArmRequested:
      return "ARM_REQUESTED";
    case DroneState::kArmCommand:
      return "ARM_COMMAND";
    case DroneState::kArmedHold:
      return "ARMED_HOLD";
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

const char * DroneStateMachine::navStateName(std::uint8_t nav_state)
{
  using VehicleStatus = px4_msgs::msg::VehicleStatus;

  switch (nav_state) {
    case VehicleStatus::NAVIGATION_STATE_MANUAL:
      return "MANUAL";
    case VehicleStatus::NAVIGATION_STATE_ALTCTL:
      return "ALTCTL";
    case VehicleStatus::NAVIGATION_STATE_POSCTL:
      return "POSCTL";
    case VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
      return "AUTO_MISSION";
    case VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
      return "AUTO_LOITER";
    case VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
      return "AUTO_RTL";
    case VehicleStatus::NAVIGATION_STATE_ACRO:
      return "ACRO";
    case VehicleStatus::NAVIGATION_STATE_DESCEND:
      return "DESCEND";
    case VehicleStatus::NAVIGATION_STATE_TERMINATION:
      return "TERMINATION";
    case VehicleStatus::NAVIGATION_STATE_OFFBOARD:
      return "OFFBOARD";
    case VehicleStatus::NAVIGATION_STATE_STAB:
      return "STAB";
    case VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
      return "AUTO_TAKEOFF";
    case VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
      return "AUTO_LAND";
    case VehicleStatus::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
      return "AUTO_FOLLOW_TARGET";
    case VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND:
      return "AUTO_PRECLAND";
    case VehicleStatus::NAVIGATION_STATE_ORBIT:
      return "ORBIT";
    case VehicleStatus::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
      return "AUTO_VTOL_TAKEOFF";
  }
  return "UNKNOWN";
}

const char * DroneStateMachine::armingStateName(std::uint8_t arming_state)
{
  using VehicleStatus = px4_msgs::msg::VehicleStatus;

  switch (arming_state) {
    case VehicleStatus::ARMING_STATE_DISARMED:
      return "DISARMED";
    case VehicleStatus::ARMING_STATE_ARMED:
      return "ARMED";
  }
  return "UNKNOWN";
}

}  // namespace mullet_core
