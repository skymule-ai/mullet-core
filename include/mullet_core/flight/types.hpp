#ifndef MULLET_CORE_FLIGHT_TYPES_HPP_
#define MULLET_CORE_FLIGHT_TYPES_HPP_

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

// Latest valid mullet_interfaces/msg/MissionCommand, in NED. `mode` is one of its mode
// constants; z is NaN in the planar modes, the position is read only in the TRAJECTORY modes,
// the acceleration is NaN when a command has no feedforward, and the yaw is NaN when the
// command leaves heading to mullet_core.
struct LatchedCommand
{
  std::uint8_t mode{0};
  float x_ned_m{0.0F};
  float y_ned_m{0.0F};
  float z_ned_m{0.0F};
  float vx_ned_mps{0.0F};
  float vy_ned_mps{0.0F};
  float vz_ned_mps{0.0F};
  float ax_ned_mps2{0.0F};
  float ay_ned_mps2{0.0F};
  float az_ned_mps2{0.0F};
  float yaw_rad{0.0F};
  rclcpp::Time stamp{};
  bool valid{false};
};

}  // namespace mullet_core

#endif  // MULLET_CORE_FLIGHT_TYPES_HPP_
