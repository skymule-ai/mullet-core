#include "mullet_core/flight/drone_state_machine.hpp"

namespace mullet_core
{

const char * DroneStateMachine::stateName(DroneState state)
{
  switch (state) {
    case DroneState::kIdle: return "IDLE";
    case DroneState::kTransition: return "TRANSITION";
    case DroneState::kHold: return "HOLD";
    case DroneState::kMission: return "MISSION";
  }
  return "UNKNOWN";
}

const char * DroneStateMachine::phaseName(Phase phase)
{
  switch (phase) {
    case Phase::kWarmup: return "warmup";
    case Phase::kRequestOffboard: return "request_offboard";
    case Phase::kRequestArm: return "request_arm";
    case Phase::kAscent: return "ascent";
    case Phase::kLanding: return "landing";
    case Phase::kReturnHome: return "return_home";
  }
  return "unknown";
}

const char * DroneStateMachine::activeActionName() const
{
  if (state_ != DroneState::kTransition) {
    return "";
  }
  switch (phase_) {
    case Phase::kLanding: return "land";
    case Phase::kReturnHome: return "return_home";
    case Phase::kWarmup:
    case Phase::kRequestOffboard:
    case Phase::kRequestArm:
    case Phase::kAscent: return "takeoff";
  }
  return "";
}

const char * DroneStateMachine::navStateName(std::uint8_t nav_state)
{
  using VehicleStatus = px4_msgs::msg::VehicleStatus;

  switch (nav_state) {
    case VehicleStatus::NAVIGATION_STATE_MANUAL: return "MANUAL";
    case VehicleStatus::NAVIGATION_STATE_ALTCTL: return "ALTCTL";
    case VehicleStatus::NAVIGATION_STATE_POSCTL: return "POSCTL";
    case VehicleStatus::NAVIGATION_STATE_AUTO_MISSION: return "AUTO_MISSION";
    case VehicleStatus::NAVIGATION_STATE_AUTO_LOITER: return "AUTO_LOITER";
    case VehicleStatus::NAVIGATION_STATE_AUTO_RTL: return "AUTO_RTL";
    case VehicleStatus::NAVIGATION_STATE_ACRO: return "ACRO";
    case VehicleStatus::NAVIGATION_STATE_DESCEND: return "DESCEND";
    case VehicleStatus::NAVIGATION_STATE_TERMINATION: return "TERMINATION";
    case VehicleStatus::NAVIGATION_STATE_OFFBOARD: return "OFFBOARD";
    case VehicleStatus::NAVIGATION_STATE_STAB: return "STAB";
    case VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF: return "AUTO_TAKEOFF";
    case VehicleStatus::NAVIGATION_STATE_AUTO_LAND: return "AUTO_LAND";
    case VehicleStatus::NAVIGATION_STATE_AUTO_FOLLOW_TARGET: return "AUTO_FOLLOW_TARGET";
    case VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND: return "AUTO_PRECLAND";
    case VehicleStatus::NAVIGATION_STATE_ORBIT: return "ORBIT";
    case VehicleStatus::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF: return "AUTO_VTOL_TAKEOFF";
  }
  return "UNKNOWN";
}

const char * DroneStateMachine::armingStateName(std::uint8_t arming_state)
{
  using VehicleStatus = px4_msgs::msg::VehicleStatus;

  switch (arming_state) {
    case VehicleStatus::ARMING_STATE_DISARMED: return "DISARMED";
    case VehicleStatus::ARMING_STATE_ARMED: return "ARMED";
  }
  return "UNKNOWN";
}

}  // namespace mullet_core
