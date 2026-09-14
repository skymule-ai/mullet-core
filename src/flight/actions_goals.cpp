#include "mullet_core/flight/drone_state_machine.hpp"

#include "flight/detail/action_lifecycle_coordinator.hpp"
#include "flight/detail/helpers.hpp"

#include <cmath>

namespace mullet_core
{

rclcpp_action::GoalResponse DroneStateMachine::handleTakeoffGoal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Takeoff::Goal> goal)
{
  (void)uuid;
  if (const char * reason = takeoffRejectReason()) {
    RCLCPP_WARN(
      get_logger(), "Rejecting takeoff goal: %s (state=%s)", reason, stateName(state_));
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
  if (!action_lifecycle_->takeoff().isActive(goal_handle)) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  action_lifecycle_->takeoff().requestCancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DroneStateMachine::handleTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
  if (!action_lifecycle_->takeoff().accept(goal_handle)) {
    action_lifecycle_->takeoff().abortRejected(
      goal_handle, "Takeoff rejected: another takeoff is active");
    return;
  }
  takeoff_arm_requested_ = false;
  active_takeoff_uses_current_yaw_ = true;
  active_takeoff_target_yaw_rad_ = 0.0F;
  hover_settle_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  if (!configureTakeoffTarget(*goal_handle->get_goal())) {
    action_lifecycle_->takeoff().abort("Takeoff rejected: local position invalid while latching target");
    return;
  }
  home_valid_ = true;
  enterPhase(Phase::kWarmup, "takeoff action accepted");
}

rclcpp_action::GoalResponse DroneStateMachine::handleLandGoal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Land::Goal> goal)
{
  (void)uuid;
  (void)goal;

  if (const char * reason = landRejectReason()) {
    RCLCPP_WARN(get_logger(), "Rejecting land goal: %s (state=%s)", reason, stateName(state_));
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DroneStateMachine::handleLandCancel(
  const std::shared_ptr<GoalHandleLand> goal_handle)
{
  if (action_lifecycle_->land().isActive(goal_handle)) {
    RCLCPP_WARN(get_logger(), "Rejecting land cancel request: landing is already committed");
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void DroneStateMachine::handleLandAccepted(const std::shared_ptr<GoalHandleLand> goal_handle)
{
  if (!action_lifecycle_->land().accept(goal_handle)) {
    action_lifecycle_->land().abortRejected(
      goal_handle, "Land rejected: another land action is active");
    return;
  }

  action_lifecycle_->returnHome().abort("Return home preempted by land action");
  if (takingOff()) {
    action_lifecycle_->takeoff().abort("Takeoff preempted by land action");
    if (!shouldRecoverThroughLanding()) {
      action_lifecycle_->land().succeed("Takeoff canceled before arming; vehicle already disarmed");
      transitionTo(DroneState::kIdle, "land action before arming");
      return;
    }
  }
  startLanding("land action accepted");
}

rclcpp_action::GoalResponse DroneStateMachine::handleReturnHomeGoal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ReturnHome::Goal> goal)
{
  (void)uuid;
  (void)goal;

  if (const char * reason = returnHomeRejectReason()) {
    RCLCPP_WARN(
      get_logger(), "Rejecting return_home goal: %s (state=%s)", reason, stateName(state_));
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DroneStateMachine::handleReturnHomeCancel(
  const std::shared_ptr<GoalHandleReturnHome> goal_handle)
{
  if (!action_lifecycle_->returnHome().isActive(goal_handle)) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  action_lifecycle_->returnHome().requestCancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DroneStateMachine::handleReturnHomeAccepted(
  const std::shared_ptr<GoalHandleReturnHome> goal_handle)
{
  if (!action_lifecycle_->returnHome().accept(goal_handle)) {
    action_lifecycle_->returnHome().abortRejected(
      goal_handle, "ReturnHome rejected: another return_home action is active");
    return;
  }
  latchHoverTarget();  // The carrot starts at the vehicle, not at an older hold point.
  enterPhase(Phase::kReturnHome, "return_home action accepted");
}

void DroneStateMachine::onStartMission(Trigger::Response & response)
{
  if (const char * reason = startMissionRejectReason()) {
    RCLCPP_WARN(get_logger(), "Rejecting start_mission: %s (state=%s)", reason, stateName(state_));
    response.success = false;
    response.message = reason;
    return;
  }
  transitionTo(DroneState::kMission, "start_mission requested");
  response.success = true;
  response.message = "Mission started";
}

void DroneStateMachine::onHold(Trigger::Response & response)
{
  if (const char * reason = holdRejectReason()) {
    RCLCPP_WARN(get_logger(), "Rejecting hold: %s (state=%s)", reason, stateName(state_));
    response.success = false;
    response.message = reason;
    return;
  }
  action_lifecycle_->returnHome().abort("Return home stopped by hold");
  latchHoverTarget();
  transitionTo(DroneState::kHold, "hold requested");
  response.success = true;
  response.message = "Holding position";
}

}  // namespace mullet_core
