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
  if (state_ != DroneState::kReady && state_ != DroneState::kOffboardHold &&
    state_ != DroneState::kArmedOffboardHold)
  {
    return false;
  }
  if (!px4_->hasVehicleStatus() || !px4_->localPositionValid()) {
    return false;
  }
  if (px4_->isArmed() && state_ != DroneState::kArmedOffboardHold) {
    return false;
  }
  return state_ != DroneState::kOffboardHold || px4_->isOffboard();
}

bool DroneStateMachine::canLand() const
{
  if (haveActiveLandGoal() || haveActiveDisableOffboardGoal() ||
    haveActiveArmGoal() || haveActiveDisarmGoal() || haveActiveEnableOffboardGoal())
  {
    return false;
  }
  if (state_ == DroneState::kOffboardHold && !px4_->isArmed()) {
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
  return interrupting_takeoff || px4_->isArmed();
}

bool DroneStateMachine::canEnableOffboard() const
{
  return state_ == DroneState::kReady && !anyActionActive() &&
         px4_->hasVehicleStatus() && px4_->localPositionValid() &&
         !px4_->isArmed() && !px4_->isOffboard();
}

bool DroneStateMachine::canDisableOffboard() const
{
  return state_ == DroneState::kOffboardHold && !anyActionActive() &&
         px4_->hasVehicleStatus() && px4_->localPositionValid() &&
         !px4_->isArmed() && px4_->isOffboard();
}

bool DroneStateMachine::canArm(const rclcpp::Time & current_time) const
{
  if (anyActionActive()) {
    return false;
  }
  if (state_ == DroneState::kArmedOffboardHold) {
    return px4_->isArmed() && landedConfirmed(current_time);
  }
  if (state_ != DroneState::kReady && state_ != DroneState::kOffboardHold) {
    return false;
  }
  if (!px4_->hasVehicleStatus() || !px4_->localPositionValid() || !px4_->hasLandDetected()) {
    return false;
  }
  if (px4_->isArmed() || !landedConfirmed(current_time)) {
    return false;
  }
  return state_ != DroneState::kOffboardHold || px4_->isOffboard();
}

bool DroneStateMachine::canDisarm(const rclcpp::Time & current_time) const
{
  if (haveActiveDisarmGoal() || haveActiveDisableOffboardGoal() ||
    state_ == DroneState::kReleaseOffboardRequest ||
    state_ == DroneState::kDisarmedOffboardReleaseFailed)
  {
    return false;
  }
  if ((state_ == DroneState::kReady || state_ == DroneState::kOffboardHold) && !px4_->isArmed()) {
    return true;
  }
  return state_ == DroneState::kArmedOffboardHold && directDisarmAllowed(current_time);
}

bool DroneStateMachine::canEmergencyLand() const
{
  if (state_ == DroneState::kDisarmed || state_ == DroneState::kLanding) {
    return false;
  }
  if (state_ == DroneState::kOffboardHold && !px4_->isArmed()) {
    return false;
  }
  return px4_->hasVehicleStatus() && px4_->isArmed();
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

const char * DroneStateMachine::navStateName(std::uint8_t nav_state)
{
  switch (nav_state) {
    case 0:
      return "MANUAL";
    case 1:
      return "ALTCTL";
    case 2:
      return "POSCTL";
    case 3:
      return "AUTO_MISSION";
    case 4:
      return "AUTO_LOITER";
    case 5:
      return "AUTO_RTL";
    case 10:
      return "ACRO";
    case 12:
      return "DESCEND";
    case 13:
      return "TERMINATION";
    case 14:
      return "OFFBOARD";
    case 15:
      return "STAB";
    case 17:
      return "AUTO_TAKEOFF";
    case 18:
      return "AUTO_LAND";
    case 19:
      return "AUTO_FOLLOW_TARGET";
    case 20:
      return "AUTO_PRECLAND";
    case 21:
      return "ORBIT";
    case 22:
      return "AUTO_VTOL_TAKEOFF";
  }
  return "UNKNOWN";
}

const char * DroneStateMachine::armingStateName(std::uint8_t arming_state)
{
  switch (arming_state) {
    case 0:
      return "INIT";
    case 1:
      return "STANDBY";
    case 2:
      return "ARMED";
    case 3:
      return "STANDBY_ERROR";
    case 4:
      return "SHUTDOWN";
    case 5:
      return "IN_AIR_RESTORE";
  }
  return "UNKNOWN";
}

}  // namespace mullet_core
