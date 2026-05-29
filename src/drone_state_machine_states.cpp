#include "mullet_core/drone_state_machine.hpp"

#include "drone_state_machine_helpers.hpp"

#include <algorithm>
#include <cmath>

namespace mullet_core
{

void DroneStateMachine::transitionTo(DroneState new_state, const char * reason)
{
  if (state_ == new_state) {
    return;
  }

  RCLCPP_INFO(
    get_logger(), "State: %s -> %s (%s)", stateName(state_), stateName(new_state), reason);
  state_ = new_state;
  state_enter_time_ = now();
  if (new_state == DroneState::kRequestOffboard || new_state == DroneState::kRequestOffboardOnly ||
    new_state == DroneState::kDisableOffboardRequested ||
    new_state == DroneState::kRequestArm || new_state == DroneState::kCancelingToLand ||
    new_state == DroneState::kLanding)
  {
    resetVehicleCommandRetryTimer();
  }
  if (new_state == DroneState::kLanding) {
    landing_timeout_reported_ = false;
  }
  if (new_state == DroneState::kTakeoff) {
    hover_settle_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }
}

void DroneStateMachine::handleInit(const rclcpp::Time & /*now*/)
{
  if (!px4_->hasVehicleStatus() || !px4_->localPositionValid()) {
    return;
  }

  transitionTo(DroneState::kReady, "PX4 state and local position available");
}

void DroneStateMachine::handleReady(const rclcpp::Time & /*now*/)
{
}

void DroneStateMachine::handleEnableOffboardRequested(const rclcpp::Time & current_time)
{
  publishEnableOffboardFeedback(current_time);
  transitionTo(
    DroneState::kWarmupOffboardHoldSetpoints,
    "enable_offboard goal committed");
}

void DroneStateMachine::handleWarmupOffboardHoldSetpoints(const rclcpp::Time & current_time)
{
  publishHoverSetpoint();
  publishEnableOffboardFeedback(current_time);
  if ((current_time - state_enter_time_).seconds() >= config_.offboard_warmup_s) {
    transitionTo(DroneState::kRequestOffboardOnly, "offboard hold warmup complete");
    return;
  }
  if (phaseTimedOut(current_time, config_.warmup_timeout_s)) {
    abortEnableOffboardGoal("enable_offboard warmup timed out");
    finishEnableOffboardAttempt(
      px4_->localPositionValid() ? DroneState::kReady : DroneState::kInit,
      px4_->localPositionValid() ? "ready after enable_offboard warmup timeout" :
      "waiting for telemetry after enable_offboard warmup timeout");
  }
}

void DroneStateMachine::handleRequestOffboardOnly(const rclcpp::Time & current_time)
{
  publishHoverSetpoint();
  publishEnableOffboardFeedback(current_time);
  if (phaseTimedOut(current_time, config_.request_offboard_timeout_s)) {
    abortEnableOffboardGoal("enable_offboard request timed out");
    finishEnableOffboardAttempt(
      px4_->localPositionValid() ? DroneState::kReady : DroneState::kInit,
      px4_->localPositionValid() ? "ready after enable_offboard request timeout" :
      "waiting for telemetry after enable_offboard request timeout");
    return;
  }
  if (state_machine_detail::retryDue(
      current_time, last_vehicle_command_time_, config_.command_retry_interval_s))
  {
    px4_->setOffboardMode();
    last_vehicle_command_time_ = current_time;
  }
  if (px4_->isOffboard()) {
    publishEnableOffboardFeedback(current_time);
    succeedEnableOffboardGoal("Offboard enabled");
    transitionTo(DroneState::kOffboardHold, "offboard accepted for disarmed hold");
  }
}

void DroneStateMachine::handleOffboardHold(const rclcpp::Time & /*current_time*/)
{
  if (!px4_->hasVehicleStatus() || !px4_->localPositionValid()) {
    RCLCPP_WARN(get_logger(), "Leaving OFFBOARD_HOLD: PX4 telemetry/local position is invalid");
    transitionTo(DroneState::kInit, "offboard hold telemetry invalid");
    return;
  }
  if (px4_->isArmed() && !haveActiveTakeoffGoal()) {
    RCLCPP_ERROR(get_logger(), "Vehicle armed unexpectedly in OFFBOARD_HOLD; starting landing recovery");
    beginLanding("unexpected armed state in offboard hold", true);
    return;
  }
  if (!px4_->isOffboard()) {
    RCLCPP_WARN(get_logger(), "Leaving OFFBOARD_HOLD: PX4 left offboard mode");
    transitionTo(DroneState::kReady, "PX4 left offboard hold");
    return;
  }

  publishHoverSetpoint();
}

void DroneStateMachine::handleDisableOffboardRequested(const rclcpp::Time & current_time)
{
  publishDisableOffboardFeedback(current_time);

  if (!px4_->hasVehicleStatus() || !px4_->localPositionValid()) {
    abortDisableOffboardGoal("disable_offboard telemetry invalid");
    transitionTo(DroneState::kInit, "disable_offboard telemetry invalid");
    return;
  }
  if (px4_->isArmed()) {
    abortDisableOffboardGoal("disable_offboard aborted: vehicle armed unexpectedly");
    beginLanding("unexpected armed state during disable_offboard", true);
    return;
  }
  if (!px4_->isOffboard()) {
    publishDisableOffboardFeedback(current_time);
    succeedDisableOffboardGoal("Offboard disabled");
    transitionTo(DroneState::kReady, "offboard disabled");
    return;
  }
  if (phaseTimedOut(current_time, config_.disable_offboard_timeout_s)) {
    abortDisableOffboardGoal("disable_offboard request timed out");
    finishDisableOffboardAttempt("disable_offboard request timed out");
    return;
  }

  publishHoverSetpoint();
  if (state_machine_detail::retryDue(
      current_time, last_vehicle_command_time_, config_.command_retry_interval_s))
  {
    px4_->setPositionMode();
    last_vehicle_command_time_ = current_time;
  }
}

void DroneStateMachine::handleTakeoffRequested(const rclcpp::Time & /*now*/)
{
  publishTakeoffFeedback(0.0F);
  transitionTo(DroneState::kWarmupSetpoints, "takeoff goal committed");
}

void DroneStateMachine::handleWarmup(const rclcpp::Time & current_time)
{
  publishTakeoffSetpoint();
  publishTakeoffFeedback(0.1F);
  if ((current_time - state_enter_time_).seconds() >= config_.offboard_warmup_s) {
    transitionTo(DroneState::kRequestOffboard, "offboard proof-of-life warmup complete");
    return;
  }
  if (phaseTimedOut(current_time, config_.warmup_timeout_s)) {
    failTakeoffOrLand("takeoff warmup timed out");
  }
}

void DroneStateMachine::handleRequestOffboard(const rclcpp::Time & current_time)
{
  publishTakeoffSetpoint();
  publishTakeoffFeedback(0.25F);
  if (phaseTimedOut(current_time, config_.request_offboard_timeout_s)) {
    failTakeoffOrLand("offboard request timed out");
    return;
  }
  if (state_machine_detail::retryDue(
      current_time, last_vehicle_command_time_, config_.command_retry_interval_s))
  {
    px4_->setOffboardMode();
    last_vehicle_command_time_ = current_time;
  }
  if (px4_->isOffboard()) {
    transitionTo(DroneState::kRequestArm, "offboard accepted");
  }
}

void DroneStateMachine::handleRequestArm(const rclcpp::Time & current_time)
{
  publishTakeoffSetpoint();
  publishTakeoffFeedback(0.4F);
  if (phaseTimedOut(current_time, config_.request_arm_timeout_s)) {
    failTakeoffOrLand("arm request timed out");
    return;
  }
  if (!px4_->isOffboard()) {
    failTakeoffOrLand("offboard mode lost before arming");
    return;
  }
  if (state_machine_detail::retryDue(
      current_time, last_vehicle_command_time_, config_.command_retry_interval_s))
  {
    px4_->arm();
    takeoff_arm_requested_ = true;
    last_vehicle_command_time_ = current_time;
  }
  if (px4_->isArmed()) {
    transitionTo(DroneState::kTakeoff, "vehicle armed");
  }
}

void DroneStateMachine::handleTakeoff(const rclcpp::Time & current_time)
{
  publishTakeoffSetpoint();
  publishTakeoffFeedback(0.6F);
  if (phaseTimedOut(current_time, config_.takeoff_timeout_s)) {
    failTakeoffOrLand("takeoff timed out");
    return;
  }
  if (!px4_->isOffboard()) {
    failTakeoffOrLand("offboard mode lost during takeoff");
    return;
  }
  if (takeoffSettled(current_time)) {
    if (active_takeoff_uses_current_yaw_) {
      latchHoverTarget();
    } else {
      latchHoverTargetPreservingTakeoffYaw();
    }
    publishTakeoffFeedback(1.0F);
    succeedTakeoffGoal("Takeoff complete");
    transitionTo(DroneState::kHover, "takeoff target reached");
  }
}

void DroneStateMachine::handleHover(const rclcpp::Time & current_time)
{
  publishHoverSetpoint();
  if (commandFresh(current_time)) {
    transitionTo(DroneState::kCommand, "fresh external velocity command");
  }
}

void DroneStateMachine::handleCommand(const rclcpp::Time & current_time)
{
  if (!commandFresh(current_time)) {
    latchHoverTarget();
    transitionTo(DroneState::kFailsafeHold, "external command stale");
    publishHoverSetpoint();
    return;
  }

  const auto & command = command_buffer_.latest_velocity;
  const auto vx_ned_mps = static_cast<float>(std::clamp(
    static_cast<double>(command.vx_ned_mps), -config_.max_vx_ned_mps, config_.max_vx_ned_mps));
  const auto vy_ned_mps = static_cast<float>(std::clamp(
    static_cast<double>(command.vy_ned_mps), -config_.max_vy_ned_mps, config_.max_vy_ned_mps));
  const auto vz_ned_mps = static_cast<float>(std::clamp(
    static_cast<double>(command.vz_ned_mps), -config_.max_vz_ned_mps, config_.max_vz_ned_mps));
  const auto horizontal_speed = std::hypot(vx_ned_mps, vy_ned_mps);
  const auto yaw_rad = horizontal_speed > 1e-3F ? std::atan2(vy_ned_mps, vx_ned_mps) : hover_yaw_rad_;

  px4_->publishOffboardControlMode(false, true);
  px4_->publishVelocitySetpoint(vx_ned_mps, vy_ned_mps, vz_ned_mps, yaw_rad);
}

void DroneStateMachine::handleCancelingToLand(const rclcpp::Time & current_time)
{
  if (px4_->localPositionValid()) {
    latchHoverTarget();
    publishHoverSetpoint();
  }
  if (!position_mode_requested_for_landing_) {
    px4_->setPositionMode();
    position_mode_requested_for_landing_ = true;
  }
  if (state_machine_detail::retryDue(
      current_time, last_vehicle_command_time_, config_.command_retry_interval_s))
  {
    px4_->land();
    last_vehicle_command_time_ = current_time;
  }
  transitionTo(DroneState::kLanding, "landing command sent after cancel");
}

void DroneStateMachine::handleLanding(const rclcpp::Time & current_time)
{
  publishLandFeedback(current_time);

  if (state_machine_detail::retryDue(
      current_time, last_vehicle_command_time_, config_.command_retry_interval_s))
  {
    px4_->land();
    last_vehicle_command_time_ = current_time;
  }
  if (px4_->isDisarmed()) {
    succeedLandGoal("Vehicle disarmed after landing");
    cancelTakeoffGoal("Takeoff canceled; vehicle disarmed after landing");
    resetTakeoffAttemptState();
    clearExternalCommandBuffer();
    transitionTo(DroneState::kDisarmed, "vehicle disarmed");
    return;
  }
  const auto landing_elapsed_s = landing_start_time_.nanoseconds() == 0 ?
    (current_time - state_enter_time_).seconds() : (current_time - landing_start_time_).seconds();
  if (!landing_timeout_reported_ && landing_elapsed_s >= config_.landing_timeout_s) {
    constexpr const char * timeout_message = "Landing timed out while vehicle is still armed";
    RCLCPP_ERROR(get_logger(), "%s", timeout_message);
    abortLandGoal(timeout_message);
    if (goalCancelRequested()) {
      cancelTakeoffGoal(timeout_message);
    } else {
      abortTakeoffGoal(timeout_message);
    }
    resetTakeoffAttemptState();
    landing_timeout_reported_ = true;
  }
}

void DroneStateMachine::handleFailsafeHold(const rclcpp::Time & current_time)
{
  if (px4_->localPositionValid()) {
    publishHoverSetpoint();
  } else {
    startLanding("local position invalid during failsafe hold");
    px4_->land();
    last_vehicle_command_time_ = current_time;
    return;
  }

  if (commandFresh(current_time)) {
    transitionTo(DroneState::kCommand, "fresh external velocity command restored");
  }
}

void DroneStateMachine::handleDisarmed(const rclcpp::Time & /*current_time*/)
{
  transitionTo(
    px4_->localPositionValid() ? DroneState::kReady : DroneState::kInit,
    px4_->localPositionValid() ? "ready after disarm" : "waiting for telemetry after disarm");
}

}  // namespace mullet_core
