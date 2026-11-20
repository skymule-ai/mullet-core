#include "mullet_core/flight/flight_states.hpp"

namespace mullet_core
{

DroneState wireState(const FlightState & state)
{
  return std::visit(overloaded {
    [](const Idle &) { return DroneState::kIdle; },
    [](const TakingOff &) { return DroneState::kTransition; },
    [](const Landing &) { return DroneState::kTransition; },
    [](const ReturningHome &) { return DroneState::kTransition; },
    [](const Hold &) { return DroneState::kHold; },
    [](const Mission &) { return DroneState::kMission; }
  }, state);
}

const char * activeActionName(const FlightState & state)
{
  return std::visit(overloaded {
    [](const Idle &) { return ""; },
    [](const TakingOff &) { return "takeoff"; },
    [](const Landing &) { return "land"; },
    [](const ReturningHome &) { return "return_home"; },
    [](const Hold &) { return ""; },
    [](const Mission &) { return ""; }
  }, state);
}

// The position target the current state streams to PX4; fallback for currentPose().
Pose flownPose(const FlightState & state)
{
  return std::visit(overloaded {
    [](const Idle &) { return Pose{}; }, // No caller can run in this state
    [](const TakingOff & s) { return s.target; },
    [](const Landing &) { return Pose{}; }, // No caller can run in this state
    [](const ReturningHome & s) { return s.carrot; },
    [](const Hold & s) { return s.target; },
    [](const Mission & s) { return s.hold; }
  }, state);
}

}  // namespace mullet_core