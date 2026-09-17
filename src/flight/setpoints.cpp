#include "mullet_core/flight/drone_state_machine.hpp"

#include "flight/detail/helpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mullet_core
{

namespace
{
using Setpoint = OffboardHeartbeatPublisher::Setpoint;
constexpr auto kUnset = OffboardHeartbeatPublisher::kUnset;

Setpoint positionSetpoint(float x, float y, float z, float yaw_rad)
{
  Setpoint setpoint{};
  setpoint.active = true;
  setpoint.position = {x, y, z};
  setpoint.yaw_rad = yaw_rad;
  return setpoint;
}

float clampAbs(float value, double limit)
{
  return static_cast<float>(std::clamp(static_cast<double>(value), -limit, limit));
}
}  // namespace

bool DroneStateMachine::takeoffSettled(const rclcpp::Time & current_time)
{
  if (!px4_->hasLocalPosition()) {
    hover_settle_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    return false;
  }
  const auto & position = px4_->localPosition();
  const auto z_error = position.z_valid ? std::fabs(position.z - takeoff_z_ned_m_) :
    std::numeric_limits<float>::infinity();
  const auto vz = position.v_z_valid ? std::fabs(position.vz) :
    std::numeric_limits<float>::infinity();
  float yaw_error = std::numeric_limits<float>::infinity();
  const bool yaw_settled = takeoffYawSettled(position.heading, &yaw_error);
  if (!yaw_settled && !active_takeoff_uses_current_yaw_) {
    RCLCPP_DEBUG(
      get_logger(),
      "Waiting for takeoff yaw alignment: error=%.3f rad tolerance=%.3f rad",
      yaw_error, config_.takeoff_yaw_tolerance_rad);
  }
  const auto altitude_tolerance_m = std::min(
    config_.hover_altitude_tolerance_m, config_.takeoff_acceptance_m);
  const bool settled = z_error <= altitude_tolerance_m &&
    vz <= config_.hover_vertical_velocity_tolerance_mps && yaw_settled;
  if (!settled) {
    hover_settle_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    return false;
  }
  if (hover_settle_start_time_.nanoseconds() == 0) {
    hover_settle_start_time_ = current_time;
  }
  return (current_time - hover_settle_start_time_).seconds() >= config_.hover_settle_time_s;
}

bool DroneStateMachine::takeoffYawSettled(float current_heading, float * yaw_error_rad) const
{
  if (active_takeoff_uses_current_yaw_) {
    if (yaw_error_rad != nullptr) {
      *yaw_error_rad = 0.0F;
    }
    return true;
  }
  if (!std::isfinite(current_heading)) {
    if (yaw_error_rad != nullptr) {
      *yaw_error_rad = std::numeric_limits<float>::infinity();
    }
    return false;
  }

  const auto yaw_error = state_machine_detail::wrapPi(
    active_takeoff_target_yaw_rad_ - current_heading);
  if (yaw_error_rad != nullptr) {
    *yaw_error_rad = yaw_error;
  }
  return std::fabs(yaw_error) <= config_.takeoff_yaw_tolerance_rad;
}

bool DroneStateMachine::latchTakeoffTarget()
{
  if (!px4_->hasLocalPosition()) {
    RCLCPP_ERROR(get_logger(), "Cannot latch takeoff target: local position is unavailable");
    return false;
  }

  const auto & position = px4_->localPosition();
  if (!position.xy_valid || !position.z_valid) {
    RCLCPP_ERROR(
      get_logger(), "Cannot latch takeoff target: invalid local position flags xy_valid=%s z_valid=%s",
      position.xy_valid ? "true" : "false", position.z_valid ? "true" : "false");
    return false;
  }

  takeoff_x_ned_m_ = position.x;
  takeoff_y_ned_m_ = position.y;
  takeoff_origin_z_ned_m_ = position.z;
  takeoff_z_ned_m_ = position.z;
  hover_x_ned_m_ = takeoff_x_ned_m_;
  hover_y_ned_m_ = takeoff_y_ned_m_;
  hover_z_ned_m_ = takeoff_z_ned_m_;
  hover_yaw_rad_ = std::isfinite(position.heading) ? position.heading :
    static_cast<float>(config_.hover_yaw_rad);
  return true;
}

bool DroneStateMachine::configureTakeoffTarget(const Takeoff::Goal & goal)
{
  if (!latchTakeoffTarget()) {
    return false;
  }
  takeoff_z_ned_m_ = takeoff_origin_z_ned_m_ - goal.altitude_m;
  hover_z_ned_m_ = takeoff_z_ned_m_;
  active_takeoff_uses_current_yaw_ = goal.use_current_yaw;
  if (!goal.use_current_yaw) {
    active_takeoff_target_yaw_rad_ = state_machine_detail::wrapPi(goal.yaw_rad);
    hover_yaw_rad_ = active_takeoff_target_yaw_rad_;
  } else if (!std::isfinite(px4_->localPosition().heading)) {
    RCLCPP_ERROR(get_logger(), "Cannot latch takeoff target: current heading is not finite");
    return false;
  } else {
    active_takeoff_target_yaw_rad_ = state_machine_detail::wrapPi(px4_->localPosition().heading);
  }
  RCLCPP_INFO(
    get_logger(), "Takeoff target latched: x=%.2f y=%.2f z=%.2f yaw=%.2f",
    takeoff_x_ned_m_, takeoff_y_ned_m_, takeoff_z_ned_m_, hover_yaw_rad_);
  return true;
}

void DroneStateMachine::latchHoverTarget()
{
  const auto & position = px4_->localPosition();
  if (position.xy_valid) {
    hover_x_ned_m_ = position.x;
    hover_y_ned_m_ = position.y;
  }
  if (position.z_valid) {
    hover_z_ned_m_ = position.z;
  }
  if (std::isfinite(position.heading)) {
    hover_yaw_rad_ = position.heading;
  }
}

void DroneStateMachine::latchHoverTargetPreservingTakeoffYaw()
{
  latchHoverTarget();
  hover_yaw_rad_ = active_takeoff_target_yaw_rad_;
}

OffboardHeartbeatPublisher::Setpoint DroneStateMachine::buildOffboardSetpoint() const
{
  const auto hold = positionSetpoint(hover_x_ned_m_, hover_y_ned_m_, hover_z_ned_m_, hover_yaw_rad_);
  switch (state_) {
    case DroneState::kIdle:
      return {};
    case DroneState::kHold:
      return hold;
    case DroneState::kMission:
      return commandFresh(now()) ? missionSetpoint() : hold;
    case DroneState::kTransition:
      if (phase_ == Phase::kLanding) {
        return {};  // PX4 lands in AUTO_LAND; offboard setpoints would be ignored anyway.
      }
      if (phase_ == Phase::kReturnHome) {
        return hold;
      }
      return positionSetpoint(takeoff_x_ned_m_, takeoff_y_ned_m_, takeoff_z_ned_m_, hover_yaw_rad_);
  }
  return {};
}

bool DroneStateMachine::missionCommandIs3d() const
{
  return latest_command_.mode == MissionCommand::VELOCITY_3D ||
         latest_command_.mode == MissionCommand::ACCELERATION_3D ||
         latest_command_.mode == MissionCommand::TRAJECTORY_3D;
}

OffboardHeartbeatPublisher::Setpoint DroneStateMachine::missionSetpoint() const
{
  const auto & command = latest_command_;
  const bool three_d = missionCommandIs3d();
  const bool trajectory_mode =
    command.mode == MissionCommand::TRAJECTORY || command.mode == MissionCommand::TRAJECTORY_3D;
  const bool velocity_mode = trajectory_mode || command.mode == MissionCommand::VELOCITY ||
    command.mode == MissionCommand::VELOCITY_3D;
  Setpoint setpoint{};
  setpoint.active = true;
  // Planar modes hold the altitude latched on entering MISSION; 3D modes leave z to the command.
  setpoint.position = {kUnset, kUnset, three_d ? kUnset : hover_z_ned_m_};
  if (trajectory_mode) {
    // PX4 flies at its own speed limits toward a far position setpoint, so the command may only
    // lead the drone by a bounded distance, measured from where the drone is.
    const auto & current = px4_->localPosition();
    if (current.xy_valid) {
      float dx = command.x_ned_m - current.x;
      float dy = command.y_ned_m - current.y;
      const auto lead = std::hypot(dx, dy);
      const auto max_lead = static_cast<float>(config_.max_xy_position_lead_m);
      if (lead > max_lead) {
        dx *= max_lead / lead;
        dy *= max_lead / lead;
      }
      setpoint.position[0] = current.x + dx;
      setpoint.position[1] = current.y + dy;
    }
    if (three_d && current.z_valid) {
      setpoint.position[2] =
        current.z + clampAbs(command.z_ned_m - current.z, config_.max_z_position_lead_m);
    }
  }
  if (velocity_mode) {
    setpoint.velocity = {
      clampAbs(command.vx_ned_mps, config_.max_vx_ned_mps),
      clampAbs(command.vy_ned_mps, config_.max_vy_ned_mps),
      three_d ? clampAbs(command.vz_ned_mps, config_.max_vz_ned_mps) : kUnset};
  }
  // ACCELERATION modes always carry it; TRAJECTORY and VELOCITY modes only as feedforward.
  if (std::isfinite(command.ax_ned_mps2)) {
    setpoint.acceleration = {
      clampAbs(command.ax_ned_mps2, config_.max_accel_ned_mps2),
      clampAbs(command.ay_ned_mps2, config_.max_accel_ned_mps2),
      three_d ? clampAbs(command.az_ned_mps2, config_.max_accel_ned_mps2) : kUnset};
  }
  const auto vx = setpoint.velocity[0];
  const auto vy = setpoint.velocity[1];
  if (std::isfinite(command.yaw_rad)) {
    setpoint.yaw_rad = command.yaw_rad;
  } else if (velocity_mode && std::hypot(vx, vy) > 1e-3F) {
    setpoint.yaw_rad = std::atan2(vy, vx);
  } else {
    setpoint.yaw_rad = hover_yaw_rad_;
  }
  return setpoint;
}

}  // namespace mullet_core
