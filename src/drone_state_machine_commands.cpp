#include "mullet_core/drone_state_machine.hpp"

#include "drone_state_machine_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mullet_core
{

void DroneStateMachine::onCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (state_ == DroneState::kCancelingToLand || state_ == DroneState::kLanding ||
    haveActiveLandGoal())
  {
    RCLCPP_DEBUG(get_logger(), "Ignoring external command while landing");
    return;
  }

  if (msg->data.size() < 3) {
    RCLCPP_WARN(get_logger(), "Invalid command: expected [vx_ned_mps, vy_ned_mps, mode]");
    return;
  }

  const auto vx_ned_mps = msg->data[0];
  const auto vy_ned_mps = msg->data[1];
  const auto mode = msg->data[2];
  if (!std::isfinite(vx_ned_mps) || !std::isfinite(vy_ned_mps) || !std::isfinite(mode)) {
    RCLCPP_WARN(get_logger(), "Invalid command: non-finite value rejected");
    return;
  }
  if (!std::isfinite(mode) || std::fabs(mode - 1.0F) > 1e-3F) {
    RCLCPP_WARN(get_logger(), "Invalid command mode %.3f: expected mode=1", mode);
    return;
  }

  command_buffer_.latest_velocity.vx_ned_mps = vx_ned_mps;
  command_buffer_.latest_velocity.vy_ned_mps = vy_ned_mps;
  command_buffer_.latest_velocity.vz_ned_mps = 0.0F;
  command_buffer_.latest_velocity.stamp = now();
  command_buffer_.latest_velocity.valid = true;
}

void DroneStateMachine::onLandRequest(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  command_buffer_.land_requested = true;
}

bool DroneStateMachine::commandFresh(const rclcpp::Time & current_time) const
{
  return command_buffer_.latest_velocity.valid &&
         (current_time - command_buffer_.latest_velocity.stamp).seconds() < config_.command_timeout_s;
}

bool DroneStateMachine::takeoffSettled(const rclcpp::Time & current_time)
{
  if (!px4_->hasLocalPosition()) {
    hover_settle_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    return false;
  }
  const auto & position = px4_->localPosition();
  const auto z_error = position.z_valid ? std::fabs(position.z - takeoff_z_ned_m_) :
    std::numeric_limits<float>::infinity();
  const auto vz = position.v_z_valid ? std::fabs(position.vz) : std::numeric_limits<float>::infinity();
  float yaw_error = std::numeric_limits<float>::infinity();
  const bool yaw_settled = takeoffYawSettled(position.heading, &yaw_error);
  if (!yaw_settled && !active_takeoff_uses_current_yaw_) {
    RCLCPP_DEBUG(
      get_logger(),
      "Waiting for takeoff yaw alignment: error=%.3f rad tolerance=%.3f rad",
      yaw_error, config_.takeoff_yaw_tolerance_rad);
  }
  const auto altitude_tolerance_m = std::min(
    config_.hover_altitude_tolerance_m,
    config_.takeoff_acceptance_m);

  const bool settled =
    z_error <= altitude_tolerance_m &&
    vz <= config_.hover_vertical_velocity_tolerance_mps &&
    yaw_settled;
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
  const auto current_z_ned_m = position.z;
  takeoff_origin_z_ned_m_ = current_z_ned_m;
  takeoff_z_ned_m_ = current_z_ned_m - static_cast<float>(std::fabs(config_.takeoff_altitude_m));
  hover_x_ned_m_ = takeoff_x_ned_m_;
  hover_y_ned_m_ = takeoff_y_ned_m_;
  hover_z_ned_m_ = takeoff_z_ned_m_;
  hover_yaw_rad_ = std::isfinite(position.heading) ? position.heading : static_cast<float>(config_.hover_yaw_rad);

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

void DroneStateMachine::publishTakeoffSetpoint()
{
  px4_->publishOffboardControlMode(true, false);
  px4_->publishPositionSetpoint(takeoff_x_ned_m_, takeoff_y_ned_m_, takeoff_z_ned_m_, hover_yaw_rad_);
}

void DroneStateMachine::publishHoverSetpoint()
{
  px4_->publishOffboardControlMode(true, false);
  px4_->publishPositionSetpoint(hover_x_ned_m_, hover_y_ned_m_, hover_z_ned_m_, hover_yaw_rad_);
}

void DroneStateMachine::clearExternalCommandBuffer()
{
  command_buffer_ = ExternalCommandBuffer{};
}

void DroneStateMachine::resetTakeoffAttemptState()
{
  takeoff_arm_requested_ = false;
  active_takeoff_uses_current_yaw_ = true;
  active_takeoff_target_yaw_rad_ = 0.0F;
}

}  // namespace mullet_core
