#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/time.hpp>

namespace mullet_core
{

// States managed by DroneStateMachine. Multi-step work (takeoff, land, return home) all runs
// inside kTransition; which one is decided by the private DroneStateMachine::Phase.
enum class DroneState : std::uint8_t
{
  kIdle,        // On the ground, or otherwise not under supervisor control.
  kTransition,  // Takeoff, land, or return home in progress.
  kHold,        // Airborne, holding a latched position.
  kMission,     // Airborne, following the external mission command stream.
};

}  // namespace mullet_core
