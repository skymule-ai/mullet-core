#include "mullet_core/flight/drone_state_machine.hpp"

#include "flight/detail/helpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mullet_core
{

const char * DroneStateMachine::takeoffGoalRejectReason(const Takeoff::Goal & goal) const
{
  if (const char * reason = takeoffRejectReason()) {
    return reason;
  }
  if (!state_machine_detail::finiteInRange(
      goal.altitude_m, config_.min_takeoff_altitude_m, config_.max_takeoff_altitude_m))
  {
    return "altitude out of range";
  }
  if (goal.use_current_yaw && !std::isfinite(px4_->localPosition().heading)) {
    return "use_current_yaw requires a finite current heading";
  }
  if (!goal.use_current_yaw && !std::isfinite(goal.yaw_rad)) {
    return "yaw is not finite";
  }
  return nullptr;
}

void DroneStateMachine::onTakeoffAccepted(const Takeoff::Goal & goal)
{
  takeoff_arm_requested_ = false;
  active_takeoff_uses_current_yaw_ = true;
  active_takeoff_target_yaw_rad_ = 0.0F;
  hover_settle_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  if (!configureTakeoffTarget(goal)) {
    takeoff_action_->abort("Failed to configure takeoff target");
    return;
  }
  home_valid_ = true;
  enterPhase(Phase::kWarmup, "takeoff action accepted");
}

void DroneStateMachine::onLandAccepted(const Land::Goal &)
{
  return_home_action_->abort("Return home preempted by land action");
  if (takingOff()) {
    takeoff_action_->abort("Takeoff preempted by land action");
    if (!shouldRecoverThroughLanding()) {
      land_action_->succeed("Takeoff canceled before arming; vehicle already disarmed");
      transitionTo(DroneState::kIdle, "land action before arming");
      return;
    }
  }
  startLanding("land action accepted");
}

void DroneStateMachine::onReturnHomeAccepted(const ReturnHome::Goal &)
{
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
  return_home_action_->abort("Return home stopped by hold");
  latchHoverTarget();
  transitionTo(DroneState::kHold, "hold requested");
  response.success = true;
  response.message = "Holding position";
}

void DroneStateMachine::publishTakeoffFeedback(float progress)
{
  if (!takeoff_action_->active()) {
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
  takeoff_action_->publish(feedback);
}

void DroneStateMachine::publishLandFeedback(const rclcpp::Time & current_time)
{
  if (!land_action_->active()) {
    return;
  }

  auto feedback = std::make_shared<Land::Feedback>();
  feedback->state = px4_->isArmed() ? "WAITING_FOR_DISARM" : stateName(state_);
  feedback->armed = px4_->isArmed();
  feedback->elapsed_s = static_cast<float>(
    std::max(0.0, (current_time - landing_start_time_).seconds()));
  land_action_->publish(feedback);
}

void DroneStateMachine::publishReturnHomeFeedback(float distance_m)
{
  if (!return_home_action_->active()) {
    return;
  }

  auto feedback = std::make_shared<ReturnHome::Feedback>();
  feedback->distance_m = distance_m;
  return_home_action_->publish(feedback);
}

void DroneStateMachine::startLanding(const char * reason)
{
  position_mode_requested_for_landing_ = true;
  landing_start_time_ = now();
  landing_timeout_reported_ = false;
  enterPhase(Phase::kLanding, reason);
}

void DroneStateMachine::failTakeoff(const char * reason)
{
  takeoff_action_->abort(reason);
  endTakeoff(reason);
}

void DroneStateMachine::endTakeoff(const char * reason)
{
  if (shouldRecoverThroughLanding()) {
    startLanding(reason);
  } else {
    transitionTo(DroneState::kIdle, reason);
  }
}

void DroneStateMachine::resetVehicleCommandRetryTimer()
{
  last_vehicle_command_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

}  // namespace mullet_core
