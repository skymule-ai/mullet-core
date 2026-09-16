#include "mullet_core/flight/drone_state_machine.hpp"

#include <cmath>

namespace mullet_core
{

namespace
{
using MissionCommand = mullet_interfaces::msg::MissionCommand;

// Why `msg` must not be latched, or nullptr when it is valid. Every mode needs the fields it
// tracks finite on its axes. Planar modes keep z NaN because mullet_core holds the altitude;
// 3D modes command z too.
const char * commandRejectReason(const MissionCommand & msg)
{
  const auto & velocity = msg.velocity_ned_mps;
  const auto & acceleration = msg.acceleration_ned_mps2;
  if (std::isinf(msg.yaw_ned_rad)) {
    return "yaw_ned_rad must be finite or NaN";
  }
  const bool three_d =
    msg.mode == MissionCommand::VELOCITY_3D || msg.mode == MissionCommand::ACCELERATION_3D;
  const bool velocity_set = std::isfinite(velocity[0]) && std::isfinite(velocity[1]);
  const bool acceleration_set = std::isfinite(acceleration[0]) && std::isfinite(acceleration[1]);
  const bool acceleration_unset = std::isnan(acceleration[0]) && std::isnan(acceleration[1]);
  if (!three_d && (!std::isnan(velocity[2]) || !std::isnan(acceleration[2]))) {
    return "z must be NaN in planar modes: mullet_core holds the altitude";
  }
  switch (msg.mode) {
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
      return "mode must be VELOCITY, ACCELERATION, VELOCITY_3D or ACCELERATION_3D";
  }
}
}  // namespace

// Latches every valid command regardless of state; only MISSION ever streams it to PX4.
void DroneStateMachine::onCommand(const MissionCommand::SharedPtr msg)
{
  if (const auto * reason = commandRejectReason(*msg)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid mission command: %s", reason);
    return;
  }

  latest_command_.mode = msg->mode;
  latest_command_.vx_ned_mps = msg->velocity_ned_mps[0];
  latest_command_.vy_ned_mps = msg->velocity_ned_mps[1];
  latest_command_.vz_ned_mps = msg->velocity_ned_mps[2];
  latest_command_.ax_ned_mps2 = msg->acceleration_ned_mps2[0];
  latest_command_.ay_ned_mps2 = msg->acceleration_ned_mps2[1];
  latest_command_.az_ned_mps2 = msg->acceleration_ned_mps2[2];
  latest_command_.yaw_rad = msg->yaw_ned_rad;
  latest_command_.stamp = now();
  latest_command_.valid = true;
}

bool DroneStateMachine::commandFresh(const rclcpp::Time & current_time) const
{
  return latest_command_.valid &&
         (current_time - latest_command_.stamp).seconds() < config_.command_timeout_s;
}

}  // namespace mullet_core
