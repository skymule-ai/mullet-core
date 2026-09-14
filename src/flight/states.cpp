#include "mullet_core/flight/drone_state_machine.hpp"

#include <cmath>

#include "flight/detail/action_lifecycle_coordinator.hpp"
#include "flight/detail/helpers.hpp"

namespace mullet_core
{

namespace
{
// Takeoff::Feedback.progress values reported for each takeoff phase; the vehicle is airborne
// only once past kRequestArm, so the bulk of progress (0.6-1.0) is covered by ascent.
constexpr float kTakeoffProgressWarmup = 0.1F;
constexpr float kTakeoffProgressRequestOffboard = 0.25F;
constexpr float kTakeoffProgressRequestArm = 0.4F;
constexpr float kTakeoffProgressAscent = 0.6F;
}  // namespace

void DroneStateMachine::transitionTo(DroneState next, const char * reason)
{
  if (state_ == next) {
    return;
  }
  RCLCPP_INFO(get_logger(), "State: %s -> %s (%s)", stateName(state_), stateName(next), reason);
  previous_state_ = state_;
  state_ = next;
  state_enter_time_ = now();
  phase_enter_time_ = state_enter_time_;
}

void DroneStateMachine::enterPhase(Phase phase, const char * reason)
{
  transitionTo(DroneState::kTransition, reason);
  phase_ = phase;
  phase_enter_time_ = now();
  resetVehicleCommandRetryTimer();
  RCLCPP_INFO(get_logger(), "TRANSITION phase -> %s", phaseName(phase_));
}

void DroneStateMachine::handleTransition(const rclcpp::Time & t)
{
  switch (phase_) {
    case Phase::kLanding:
      handleLanding(t);
      return;
    case Phase::kReturnHome:
      handleReturnHome();
      return;
    case Phase::kWarmup:
    case Phase::kRequestOffboard:
    case Phase::kRequestArm:
    case Phase::kAscent:
      handleTakingOff(t);
      return;
  }
}

void DroneStateMachine::handleTakingOff(const rclcpp::Time & t)
{
  const float progress = phase_ == Phase::kWarmup ? kTakeoffProgressWarmup :
    phase_ == Phase::kRequestOffboard ? kTakeoffProgressRequestOffboard :
    phase_ == Phase::kRequestArm ? kTakeoffProgressRequestArm : kTakeoffProgressAscent;
  publishTakeoffFeedback(progress);
  if (phase_ == Phase::kWarmup) {
    if ((t - phase_enter_time_).seconds() < config_.offboard_warmup_s) {
      return;
    }
    enterPhase(Phase::kRequestOffboard, "offboard warmup complete");
  }
  if (phase_ == Phase::kRequestOffboard) {
    if (phaseTimedOut(t, config_.request_offboard_timeout_s)) {
      failTakeoff("offboard request timed out");
      return;
    }
    if (state_machine_detail::retryDue(t, last_vehicle_command_time_, config_.command_retry_interval_s)) {
      px4_->setOffboardMode();
      last_vehicle_command_time_ = t;
    }
    if (!px4_->isOffboard()) {
      return;
    }
    enterPhase(Phase::kRequestArm, "PX4 entered offboard");
  }
  if (phase_ == Phase::kRequestArm) {
    if (phaseTimedOut(t, config_.request_arm_timeout_s)) {
      failTakeoff("arm request timed out");
      return;
    }
    if (!px4_->isOffboard()) {
      failTakeoff("offboard mode lost before arming");
      return;
    }
    if (state_machine_detail::retryDue(t, last_vehicle_command_time_, config_.command_retry_interval_s)) {
      px4_->arm();
      takeoff_arm_requested_ = true;
      last_vehicle_command_time_ = t;
    }
    if (!px4_->isArmed()) {
      return;
    }
    enterPhase(Phase::kAscent, "PX4 armed");
  }
  if (phaseTimedOut(t, config_.takeoff_timeout_s)) {
    failTakeoff("takeoff timed out");
    return;
  }
  if (takeoffSettled(t)) {
    if (active_takeoff_uses_current_yaw_) {
      latchHoverTarget();
    } else {
      latchHoverTargetPreservingTakeoffYaw();
    }
    publishTakeoffFeedback(1.0F);
    action_lifecycle_->takeoff().succeed("Takeoff complete");
    transitionTo(DroneState::kHold, "takeoff target reached");
  }
}

void DroneStateMachine::handleLanding(const rclcpp::Time & t)
{
  publishLandFeedback(t);
  if (position_mode_requested_for_landing_) {
    px4_->setPositionMode();
    position_mode_requested_for_landing_ = false;
    resetVehicleCommandRetryTimer();
  }
  if (state_machine_detail::retryDue(t, last_vehicle_command_time_, config_.command_retry_interval_s)) {
    px4_->land();
    last_vehicle_command_time_ = t;
  }
  if (px4_->isDisarmed()) {
    action_lifecycle_->land().succeed("Vehicle disarmed after landing");
    transitionTo(DroneState::kIdle, "vehicle disarmed");
    return;
  }
  if (!landing_timeout_reported_ && (t - landing_start_time_).seconds() >= config_.landing_timeout_s) {
    constexpr const char * kMessage = "Landing timed out while vehicle is still armed";
    RCLCPP_ERROR(get_logger(), "%s", kMessage);
    action_lifecycle_->land().abort(kMessage);
    landing_timeout_reported_ = true;
  }
}

void DroneStateMachine::handleReturnHome()
{
  const auto & position = px4_->localPosition();
  const float distance_m = std::hypot(takeoff_x_ned_m_ - position.x, takeoff_y_ned_m_ - position.y);
  publishReturnHomeFeedback(distance_m);
  if (distance_m <= config_.return_home_acceptance_m) {
    latchHoverTarget();
    action_lifecycle_->returnHome().succeed("Arrived home");
    transitionTo(DroneState::kHold, "arrived home");
    return;
  }
  // Slide the hold target toward home by one tick of travel, so PX4 never sees a setpoint step
  // it would chase at its own maximum speed.
  const auto step_m = static_cast<float>(config_.return_home_speed_mps / config_.control_rate_hz);
  const float dx = takeoff_x_ned_m_ - hover_x_ned_m_;
  const float dy = takeoff_y_ned_m_ - hover_y_ned_m_;
  const float carrot_distance_m = std::hypot(dx, dy);
  if (carrot_distance_m <= step_m) {
    hover_x_ned_m_ = takeoff_x_ned_m_;
    hover_y_ned_m_ = takeoff_y_ned_m_;
  } else {
    hover_x_ned_m_ += dx / carrot_distance_m * step_m;
    hover_y_ned_m_ += dy / carrot_distance_m * step_m;
  }
}

void DroneStateMachine::handleMission(const rclcpp::Time & t)
{
  // While the stream is fresh the drone moves, so keep the hold target on it: a stale stream
  // then holds wherever the drone was, not where the mission started.
  if (commandFresh(t)) {
    latchHoverTarget();
  }
}

}  // namespace mullet_core
