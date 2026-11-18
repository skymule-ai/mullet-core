#include "mullet_core/flight/mission_command.hpp"

#include "mullet_core/flight/drone_state_machine.hpp"

#include <cmath>

namespace mullet_core
{

namespace
{
// Why `msg` must not be latched, or nullptr when it is valid. Every mode needs the fields it
// tracks finite on its axes. Planar modes keep z NaN because mullet_core holds the altitude;
// 3D modes command z too. The position is only read, and so only checked, in TRAJECTORY modes.
const char * commandRejectReason(const MissionCommand & msg)
{
  const auto & position = msg.position_ned_m;
  const auto & velocity = msg.velocity_ned_mps;
  const auto & acceleration = msg.acceleration_ned_mps2;
  if (std::isinf(msg.yaw_ned_rad)) {
    return "yaw_ned_rad must be finite or NaN";
  }
  const bool position_set = std::isfinite(position[0]) && std::isfinite(position[1]);
  const bool velocity_set = std::isfinite(velocity[0]) && std::isfinite(velocity[1]);
  const bool acceleration_set = std::isfinite(acceleration[0]) && std::isfinite(acceleration[1]);
  const bool acceleration_unset = std::isnan(acceleration[0]) && std::isnan(acceleration[1]);
  if (!is3d(msg.mode) && (!std::isnan(velocity[2]) || !std::isnan(acceleration[2]) ||
    (isTrajectory(msg.mode) && !std::isnan(position[2]))))
  {
    return "z must be NaN in planar modes: mullet_core holds the altitude";
  }
  switch (msg.mode) {
    case MissionCommand::TRAJECTORY:
      if (!position_set || !velocity_set) {
        return "TRAJECTORY needs a finite horizontal position and velocity";
      }
      if (!acceleration_set && !acceleration_unset) {
        return "acceleration feedforward must set both horizontal axes or neither";
      }
      return nullptr;
    case MissionCommand::TRAJECTORY_3D:
      if (!position_set || !std::isfinite(position[2]) || !velocity_set ||
        !std::isfinite(velocity[2]))
      {
        return "TRAJECTORY_3D needs a finite position and velocity on all three axes";
      }
      if (!(acceleration_set && std::isfinite(acceleration[2])) &&
        !(acceleration_unset && std::isnan(acceleration[2])))
      {
        return "acceleration feedforward must set all three axes or none";
      }
      return nullptr;
    case MissionCommand::VELOCITY:
      if (!velocity_set) {
        return "VELOCITY needs a finite horizontal velocity";
      }
      if (!acceleration_set && !acceleration_unset) {
        return "acceleration feedforward must set both horizontal axes or neither";
      }
      return nullptr;
    case MissionCommand::ACCELERATION:
      return acceleration_set ? nullptr : "ACCELERATION needs a finite horizontal acceleration";
    case MissionCommand::VELOCITY_3D:
      if (!velocity_set || !std::isfinite(velocity[2])) {
        return "VELOCITY_3D needs a finite velocity on all three axes";
      }
      if (!(acceleration_set && std::isfinite(acceleration[2])) &&
        !(acceleration_unset && std::isnan(acceleration[2])))
      {
        return "acceleration feedforward must set all three axes or none";
      }
      return nullptr;
    case MissionCommand::ACCELERATION_3D:
      return acceleration_set && std::isfinite(acceleration[2]) ? nullptr :
             "ACCELERATION_3D needs a finite acceleration on all three axes";
    default:
      return "mode must be TRAJECTORY, VELOCITY, ACCELERATION or one of their _3D variants";
  }
}
}  // namespace

LatchedCommand fromMsg(const MissionCommand & msg, rclcpp::Time stamp)
{
  return {
    .mode = msg.mode,
    .position = msg.position_ned_m,
    .velocity = msg.velocity_ned_mps,
    .acceleration = msg.acceleration_ned_mps2,
    .yaw_rad = msg.yaw_ned_rad,
    .stamp = stamp,
  };
}

bool is3d(std::uint8_t mode)
{
  return mode == MissionCommand::VELOCITY_3D ||
         mode == MissionCommand::ACCELERATION_3D ||
         mode == MissionCommand::TRAJECTORY_3D;
}

bool isTrajectory(std::uint8_t mode)
{
  return mode == MissionCommand::TRAJECTORY || mode == MissionCommand::TRAJECTORY_3D;
}

bool carriesVelocity(std::uint8_t mode)
{
  return isTrajectory(mode) ||
         mode == MissionCommand::VELOCITY || mode == MissionCommand::VELOCITY_3D;
}

// Latches every valid command regardless of state; only MISSION ever streams it to PX4.
void DroneStateMachine::onCommand(const MissionCommand::SharedPtr msg)
{
  if (const auto * reason = commandRejectReason(*msg)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid mission command: %s", reason);
    return;
  }

  latest_command_ = fromMsg(*msg, now());
}

bool commandFresh(
  const std::optional<LatchedCommand> & latest_command, const rclcpp::Time & current_time,
  double command_timeout_s)
{
  return latest_command &&
         (current_time - latest_command->stamp).seconds() < command_timeout_s;
}

}  // namespace mullet_core
