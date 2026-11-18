#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include "mullet_interfaces/msg/mission_command.hpp"

namespace mullet_core
{

using MissionCommand = mullet_interfaces::msg::MissionCommand;

// Latest valid mullet_interfaces/msg/MissionCommand, in NED: each array is [north, east, down]
// in m, m/s and m/s². `mode` is one of the message's mode constants. z is NaN in the planar
// modes, the position is read only in the TRAJECTORY modes, the acceleration is NaN when a
// command has no feedforward, and the yaw is NaN when the command leaves heading to mullet_core.
struct LatchedCommand
{
  std::uint8_t mode{0};
  std::array<float, 3> position{};
  std::array<float, 3> velocity{};
  std::array<float, 3> acceleration{};
  float yaw_rad{};
  rclcpp::Time stamp{};
};

/// Latches an already validated command.
///
/// Args:
///   msg: Command that passed validation in `DroneStateMachine::onCommand()`.
///   stamp: When mullet_core received it. Freshness is measured on the node's clock, never on
///     the sender's.
LatchedCommand fromMsg(const MissionCommand & msg, rclcpp::Time stamp);

/// Returns whether `mode` commands all three axes, so mullet_core does not hold the altitude.
bool is3d(std::uint8_t mode);

/// Returns whether `mode` is TRAJECTORY or TRAJECTORY_3D.
bool isTrajectory(std::uint8_t mode);

/// Returns whether `mode` streams a velocity setpoint to PX4: the VELOCITY modes and also the
/// TRAJECTORY modes, which carry velocity as feedforward.
bool carriesVelocity(std::uint8_t mode);

/// Returns whether a command was ever latched and is younger than `command_timeout_s`.
bool commandFresh(
  const std::optional<LatchedCommand> & latest_command, const rclcpp::Time & current_time,
  double command_timeout_s);

}  // namespace mullet_core
