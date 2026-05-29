#include "mullet_core/drone_state_machine.hpp"

#include "action_lifecycle_coordinator.hpp"
#include "drone_state_machine_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mullet_core
{

rclcpp_action::GoalResponse DroneStateMachine::handleTakeoffGoal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Takeoff::Goal> goal)
{
  (void)uuid;
  if (haveActiveLandGoal() || haveActiveDisableOffboardGoal() ||
    state_ == DroneState::kCancelingToLand ||
    state_ == DroneState::kLanding)
  {
    RCLCPP_WARN(get_logger(), "Rejecting takeoff goal: landing or disable_offboard is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (haveActiveEnableOffboardGoal()) {
    RCLCPP_WARN(get_logger(), "Rejecting takeoff goal: enable_offboard action is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (state_ != DroneState::kReady && state_ != DroneState::kOffboardHold) {
    RCLCPP_WARN(get_logger(), "Rejecting takeoff goal: state is %s", stateName(state_));
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (haveActiveTakeoffGoal()) {
    RCLCPP_WARN(get_logger(), "Rejecting takeoff goal: another takeoff is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!px4_->hasVehicleStatus() || !px4_->localPositionValid()) {
    RCLCPP_WARN(get_logger(), "Rejecting takeoff goal: PX4 telemetry is not ready");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (px4_->isArmed()) {
    RCLCPP_WARN(get_logger(), "Rejecting takeoff goal: vehicle is already armed");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (state_ == DroneState::kOffboardHold && !px4_->isOffboard()) {
    RCLCPP_WARN(get_logger(), "Rejecting takeoff goal: OFFBOARD_HOLD is no longer offboard");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!state_machine_detail::finiteInRange(
      goal->altitude_m, config_.min_takeoff_altitude_m, config_.max_takeoff_altitude_m))
  {
    RCLCPP_WARN(
      get_logger(), "Rejecting takeoff goal: altitude %.2f outside [%.2f, %.2f] m",
      goal->altitude_m, config_.min_takeoff_altitude_m, config_.max_takeoff_altitude_m);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->use_current_yaw && !std::isfinite(px4_->localPosition().heading)) {
    RCLCPP_WARN(
      get_logger(), "Rejecting takeoff goal: use_current_yaw requires a finite current heading");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goal->use_current_yaw && !std::isfinite(goal->yaw_rad)) {
    RCLCPP_WARN(get_logger(), "Rejecting takeoff goal: yaw is not finite");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DroneStateMachine::handleTakeoffCancel(
  const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
  if (!action_lifecycle_->isActiveTakeoffGoal(goal_handle)) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  action_lifecycle_->requestTakeoffCancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DroneStateMachine::handleTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
  const bool from_offboard_hold = state_ == DroneState::kOffboardHold;
  if (!action_lifecycle_->acceptTakeoffGoal(goal_handle)) {
    action_lifecycle_->abortRejectedTakeoff(
      goal_handle, "Takeoff rejected: another takeoff is active");
    return;
  }
  takeoff_arm_requested_ = false;
  active_takeoff_uses_current_yaw_ = true;
  active_takeoff_target_yaw_rad_ = 0.0F;
  if (!configureTakeoffTarget(*goal_handle->get_goal())) {
    abortTakeoffGoal("Takeoff rejected: local position invalid while latching target");
    resetTakeoffAttemptState();
    const auto telemetry_ready = px4_->localPositionValid();
    const auto next_state = telemetry_ready ? DroneState::kReady : DroneState::kInit;
    const auto * reason = telemetry_ready ? "ready after failed target latch" :
      "waiting for telemetry after failed target latch";
    transitionTo(next_state, reason);
    return;
  }
  if (from_offboard_hold) {
    publishTakeoffSetpoint();
    transitionTo(DroneState::kRequestArm, "takeoff action accepted from offboard hold");
    return;
  }
  transitionTo(DroneState::kTakeoffRequested, "takeoff action accepted");
}

rclcpp_action::GoalResponse DroneStateMachine::handleLandGoal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Land::Goal> goal)
{
  (void)uuid;
  (void)goal;

  if (haveActiveLandGoal()) {
    RCLCPP_WARN(get_logger(), "Rejecting land goal: another land action is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (haveActiveDisableOffboardGoal()) {
    RCLCPP_WARN(get_logger(), "Rejecting land goal: disable_offboard action is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (state_ == DroneState::kOffboardHold && !px4_->isArmed()) {
    RCLCPP_WARN(
      get_logger(),
      "Land rejected: vehicle is disarmed in OFFBOARD hold; no landing action is required.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (haveActiveEnableOffboardGoal()) {
    RCLCPP_WARN(get_logger(), "Rejecting land goal: enable_offboard action is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const bool interrupting_takeoff = activeTakeoffState();
  if (!landActionAllowedFromState()) {
    RCLCPP_WARN(get_logger(), "Rejecting land goal: state is %s", stateName(state_));
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (interrupting_takeoff && !haveActiveTakeoffGoal()) {
    RCLCPP_WARN(get_logger(), "Rejecting land goal: no active takeoff action to interrupt");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!interrupting_takeoff && haveActiveTakeoffGoal()) {
    RCLCPP_WARN(get_logger(), "Rejecting land goal: takeoff action is active outside takeoff");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!px4_->hasVehicleStatus()) {
    RCLCPP_WARN(get_logger(), "Rejecting land goal: PX4 telemetry is not ready");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!interrupting_takeoff && !px4_->isArmed()) {
    RCLCPP_WARN(
      get_logger(), "Rejecting land goal: vehicle is not armed; state/telemetry may be stale");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DroneStateMachine::handleLandCancel(
  const std::shared_ptr<GoalHandleLand> goal_handle)
{
  if (action_lifecycle_->isActiveLandGoal(goal_handle)) {
    RCLCPP_WARN(get_logger(), "Rejecting land cancel request: landing is already committed");
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void DroneStateMachine::handleLandAccepted(const std::shared_ptr<GoalHandleLand> goal_handle)
{
  if (!action_lifecycle_->acceptLandGoal(goal_handle)) {
    action_lifecycle_->abortRejectedLand(
      goal_handle, "Land rejected: another land action is active");
    return;
  }

  const bool interrupting_takeoff = activeTakeoffState() && haveActiveTakeoffGoal();
  const bool recover_through_landing = shouldRecoverThroughLanding();
  if (interrupting_takeoff) {
    preemptTakeoffForLandAction();
    clearExternalCommandBuffer();
    if (!recover_through_landing) {
      succeedLandGoal("Takeoff canceled before arming; vehicle already disarmed");
      resetTakeoffAttemptState();
      transitionTo(
        px4_->localPositionValid() ? DroneState::kReady : DroneState::kInit,
        px4_->localPositionValid() ? "ready after pre-arm land action" :
        "waiting for telemetry after pre-arm land action");
      return;
    }
  }

  startLanding("land action accepted");
}

rclcpp_action::GoalResponse DroneStateMachine::handleEnableOffboardGoal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const EnableOffboard::Goal> goal)
{
  (void)uuid;
  (void)goal;

  if (state_ != DroneState::kReady) {
    RCLCPP_WARN(get_logger(), "Rejecting enable_offboard goal: state is %s", stateName(state_));
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (haveActiveTakeoffGoal() || haveActiveLandGoal() || haveActiveEnableOffboardGoal() ||
    haveActiveDisableOffboardGoal())
  {
    RCLCPP_WARN(get_logger(), "Rejecting enable_offboard goal: another action is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!px4_->hasVehicleStatus() || !px4_->localPositionValid()) {
    RCLCPP_WARN(get_logger(), "Rejecting enable_offboard goal: PX4 telemetry is not ready");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (px4_->isArmed()) {
    RCLCPP_WARN(get_logger(), "Rejecting enable_offboard goal: vehicle is already armed");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (px4_->isOffboard()) {
    RCLCPP_WARN(get_logger(), "Rejecting enable_offboard goal: vehicle is already offboard");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DroneStateMachine::handleEnableOffboardCancel(
  const std::shared_ptr<GoalHandleEnableOffboard> goal_handle)
{
  if (!action_lifecycle_->isActiveEnableOffboardGoal(goal_handle)) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  action_lifecycle_->requestEnableOffboardCancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DroneStateMachine::handleEnableOffboardAccepted(
  const std::shared_ptr<GoalHandleEnableOffboard> goal_handle)
{
  if (!action_lifecycle_->acceptEnableOffboardGoal(goal_handle)) {
    action_lifecycle_->abortRejectedEnableOffboard(
      goal_handle, "EnableOffboard rejected: another enable_offboard action is active");
    return;
  }
  if (!latchOffboardHoldTarget()) {
    abortEnableOffboardGoal("EnableOffboard rejected: local position invalid while latching hold target");
    finishEnableOffboardAttempt(
      px4_->localPositionValid() ? DroneState::kReady : DroneState::kInit,
      px4_->localPositionValid() ? "ready after failed offboard hold target latch" :
      "waiting for telemetry after failed offboard hold target latch");
    return;
  }
  transitionTo(DroneState::kEnableOffboardRequested, "enable_offboard action accepted");
}

rclcpp_action::GoalResponse DroneStateMachine::handleDisableOffboardGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const DisableOffboard::Goal> goal)
{
  (void)uuid;
  (void)goal;

  if (state_ != DroneState::kOffboardHold) {
    RCLCPP_WARN(get_logger(), "Rejecting disable_offboard goal: state is %s", stateName(state_));
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (haveActiveTakeoffGoal() || haveActiveLandGoal() || haveActiveEnableOffboardGoal() ||
    haveActiveDisableOffboardGoal())
  {
    RCLCPP_WARN(get_logger(), "Rejecting disable_offboard goal: another action is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!px4_->hasVehicleStatus() || !px4_->localPositionValid()) {
    RCLCPP_WARN(get_logger(), "Rejecting disable_offboard goal: PX4 telemetry is not ready");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (px4_->isArmed()) {
    RCLCPP_WARN(get_logger(), "Rejecting disable_offboard goal: vehicle is armed");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!px4_->isOffboard()) {
    RCLCPP_WARN(get_logger(), "Rejecting disable_offboard goal: vehicle is not offboard");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DroneStateMachine::handleDisableOffboardCancel(
  const std::shared_ptr<GoalHandleDisableOffboard> goal_handle)
{
  if (!action_lifecycle_->isActiveDisableOffboardGoal(goal_handle)) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  action_lifecycle_->requestDisableOffboardCancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DroneStateMachine::handleDisableOffboardAccepted(
  const std::shared_ptr<GoalHandleDisableOffboard> goal_handle)
{
  if (!action_lifecycle_->acceptDisableOffboardGoal(goal_handle)) {
    action_lifecycle_->abortRejectedDisableOffboard(
      goal_handle, "DisableOffboard rejected: another disable_offboard action is active");
    return;
  }
  transitionTo(DroneState::kDisableOffboardRequested, "disable_offboard action accepted");
}

void DroneStateMachine::publishTakeoffFeedback(float progress)
{
  if (!haveActiveTakeoffGoal()) {
    return;
  }
  auto feedback = std::make_shared<Takeoff::Feedback>();
  feedback->state = stateName(state_);
  feedback->target_altitude_m = takeoff_origin_z_ned_m_ - takeoff_z_ned_m_;
  if (px4_->hasLocalPosition() && px4_->localPosition().z_valid) {
    feedback->current_altitude_m = takeoff_origin_z_ned_m_ - px4_->localPosition().z;
    feedback->altitude_error_m = std::fabs(
      feedback->target_altitude_m - feedback->current_altitude_m);
  } else {
    feedback->current_altitude_m = 0.0F;
    feedback->altitude_error_m = std::numeric_limits<float>::infinity();
  }
  feedback->progress = progress;
  action_lifecycle_->publishTakeoffFeedback(feedback);
}

void DroneStateMachine::publishLandFeedback(const rclcpp::Time & current_time)
{
  if (!haveActiveLandGoal()) {
    return;
  }

  auto feedback = std::make_shared<Land::Feedback>();
  feedback->state = px4_->isArmed() ? "WAITING_FOR_DISARM" : stateName(state_);
  feedback->armed = px4_->isArmed();
  feedback->elapsed_s = static_cast<float>(
    landing_start_time_.nanoseconds() == 0 ? 0.0 :
    std::max(0.0, (current_time - landing_start_time_).seconds()));
  action_lifecycle_->publishLandFeedback(feedback);
}

void DroneStateMachine::publishEnableOffboardFeedback(const rclcpp::Time & current_time)
{
  if (!haveActiveEnableOffboardGoal()) {
    return;
  }

  auto feedback = std::make_shared<EnableOffboard::Feedback>();
  feedback->state = stateName(state_);
  feedback->offboard = px4_->isOffboard();
  feedback->elapsed_s = static_cast<float>(
    std::max(0.0, (current_time - state_enter_time_).seconds()));
  action_lifecycle_->publishEnableOffboardFeedback(feedback);
}

void DroneStateMachine::publishDisableOffboardFeedback(const rclcpp::Time & current_time)
{
  if (!haveActiveDisableOffboardGoal()) {
    return;
  }

  auto feedback = std::make_shared<DisableOffboard::Feedback>();
  feedback->state = stateName(state_);
  feedback->offboard = px4_->isOffboard();
  feedback->elapsed_s = static_cast<float>(
    std::max(0.0, (current_time - state_enter_time_).seconds()));
  action_lifecycle_->publishDisableOffboardFeedback(feedback);
}

void DroneStateMachine::succeedTakeoffGoal(const char * message)
{
  action_lifecycle_->succeedTakeoff(message);
}

void DroneStateMachine::abortTakeoffGoal(const char * message)
{
  action_lifecycle_->abortTakeoff(message);
}

void DroneStateMachine::cancelTakeoffGoal(const char * message)
{
  action_lifecycle_->cancelTakeoff(message);
}

void DroneStateMachine::preemptTakeoffForLandAction()
{
  abortTakeoffGoal("Takeoff preempted by land action");
}

void DroneStateMachine::succeedLandGoal(const char * message)
{
  action_lifecycle_->succeedLand(message);
}

void DroneStateMachine::abortLandGoal(const char * message)
{
  action_lifecycle_->abortLand(message);
}

void DroneStateMachine::succeedEnableOffboardGoal(const char * message)
{
  action_lifecycle_->succeedEnableOffboard(message);
}

void DroneStateMachine::abortEnableOffboardGoal(const char * message)
{
  action_lifecycle_->abortEnableOffboard(message);
}

void DroneStateMachine::cancelEnableOffboardGoal(const char * message)
{
  action_lifecycle_->cancelEnableOffboard(message);
}

void DroneStateMachine::succeedDisableOffboardGoal(const char * message)
{
  action_lifecycle_->succeedDisableOffboard(message);
}

void DroneStateMachine::abortDisableOffboardGoal(const char * message)
{
  action_lifecycle_->abortDisableOffboard(message);
}

void DroneStateMachine::cancelDisableOffboardGoal(const char * message)
{
  action_lifecycle_->cancelDisableOffboard(message);
}

void DroneStateMachine::startLanding(const char * reason)
{
  clearExternalCommandBuffer();
  position_mode_requested_for_landing_ = true;
  resetVehicleCommandRetryTimer();
  landing_start_time_ = now();
  landing_timeout_reported_ = false;
  transitionTo(DroneState::kLanding, reason);
}

void DroneStateMachine::beginLanding(const char * reason, bool request_position_mode)
{
  if (!request_position_mode) {
    startLanding(reason);
    return;
  }

  clearExternalCommandBuffer();
  position_mode_requested_for_landing_ = !request_position_mode;
  resetVehicleCommandRetryTimer();
  landing_start_time_ = now();
  landing_timeout_reported_ = false;
  transitionTo(DroneState::kCancelingToLand, reason);
}

void DroneStateMachine::failTakeoffOrLand(const char * reason)
{
  abortTakeoffGoal(reason);
  if (shouldRecoverThroughLanding()) {
    beginLanding(reason, true);
  } else {
    resetTakeoffAttemptState();
    transitionTo(DroneState::kTakeoffFailed, reason);
  }
}

void DroneStateMachine::finishEnableOffboardAttempt(DroneState fallback_state, const char * reason)
{
  clearExternalCommandBuffer();
  resetVehicleCommandRetryTimer();
  transitionTo(fallback_state, reason);
}

void DroneStateMachine::finishDisableOffboardAttempt(const char * reason)
{
  clearExternalCommandBuffer();
  resetVehicleCommandRetryTimer();
  if (!px4_->localPositionValid()) {
    transitionTo(DroneState::kInit, reason);
    return;
  }
  if (!px4_->isArmed() && px4_->isOffboard()) {
    transitionTo(DroneState::kOffboardHold, reason);
    return;
  }
  transitionTo(DroneState::kReady, reason);
}

}  // namespace mullet_core
