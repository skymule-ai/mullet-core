#include "mullet_core/flight/drone_state_machine.hpp"

namespace mullet_core
{

bool DroneStateMachine::takingOff() const
{
  return state_ == DroneState::kTransition && phase_ != Phase::kLanding &&
         phase_ != Phase::kReturnHome;
}

bool DroneStateMachine::shouldRecoverThroughLanding() const
{
  return px4_->isArmed() || takeoff_arm_requested_;
}

// Reasons a Takeoff goal would be rejected right now, or nullptr if eligible. Single source of
// truth shared by canTakeoff() (the heartbeat's can_takeoff flag) and handleTakeoffGoal() - the
// goal-parameter checks (altitude range, yaw) stay in handleTakeoffGoal since they need the goal.
const char * DroneStateMachine::takeoffRejectReason() const
{
  if (state_ != DroneState::kIdle) {
    return "state is not IDLE";
  }
  if (!px4_->hasVehicleStatus() || !px4_->localPositionValid()) {
    return "PX4 telemetry is not ready";
  }
  if (px4_->isArmed()) {
    return "vehicle is already armed";
  }
  return nullptr;
}

bool DroneStateMachine::canTakeoff() const
{
  return takeoffRejectReason() == nullptr;
}

// The single land command: accepted whenever the vehicle is armed or mid-takeoff, from any
// state, unless it is already landing.
const char * DroneStateMachine::landRejectReason() const
{
  if (!px4_->hasVehicleStatus()) {
    return "PX4 telemetry is not ready";
  }
  if (state_ == DroneState::kTransition && phase_ == Phase::kLanding) {
    return "already landing";
  }
  if (!px4_->isArmed() && !takingOff()) {
    return "vehicle is not armed";
  }
  return nullptr;
}

bool DroneStateMachine::canLand() const
{
  return landRejectReason() == nullptr;
}

const char * DroneStateMachine::returnHomeRejectReason() const
{
  if (state_ != DroneState::kHold && state_ != DroneState::kMission) {
    return "state is not HOLD or MISSION";
  }
  if (!home_valid_) {
    return "no home latched: this node has not taken off";
  }
  if (!px4_->isOffboard()) {
    return "PX4 is not in offboard: the pilot has control";
  }
  if (!px4_->localPositionValid()) {
    return "local position is not valid";
  }
  return nullptr;
}

bool DroneStateMachine::canReturnHome() const
{
  return returnHomeRejectReason() == nullptr;
}

const char * DroneStateMachine::startMissionRejectReason() const
{
  if (state_ != DroneState::kHold) {
    return "state is not HOLD";
  }
  if (!px4_->isOffboard()) {
    return "PX4 is not in offboard: the pilot has control";
  }
  return nullptr;
}

bool DroneStateMachine::canStartMission() const
{
  return startMissionRejectReason() == nullptr;
}

const char * DroneStateMachine::holdRejectReason() const
{
  if (state_ == DroneState::kMission ||
    (state_ == DroneState::kTransition && phase_ == Phase::kReturnHome))
  {
    return nullptr;
  }
  return "nothing to stop: not in MISSION or return home";
}

bool DroneStateMachine::canHold() const
{
  return holdRejectReason() == nullptr;
}

}  // namespace mullet_core
