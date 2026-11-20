#include "mullet_core/flight/drone_state_machine.hpp"

#include "flight/detail/helpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mullet_core
{

namespace
{
// A takeoff must end through landing rather than a plain reset to IDLE once PX4 may be armed.
bool shouldRecoverThroughLanding(const TakingOff & takeoff, bool px4_armed)
{
  return px4_armed || takeoff.arm_requested;
}
}  // namespace

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
  auto takeoff = makeTakeoff(goal);
  if (!takeoff) {
    takeoff_action_->abort("Failed to configure takeoff target");
    return;
  }
  home_ = std::array<float, 2>{takeoff->target.x, takeoff->target.y};
  transitionTo(std::move(*takeoff), "takeoff action accepted");
}

void DroneStateMachine::onLandAccepted(const Land::Goal &)
{
  return_home_action_->abort("Return home preempted by land action");
  if (const auto * takeoff = std::get_if<TakingOff>(&state_)) {
    takeoff_action_->abort("Takeoff preempted by land action");
    if (!shouldRecoverThroughLanding(*takeoff, px4_->isArmed())) {
      land_action_->succeed("Takeoff canceled before arming; vehicle already disarmed");
      transitionTo(Idle{}, "land action before arming");
      return;
    }
  }
  transitionTo(Landing{.start = now()}, "land action accepted");
}

void DroneStateMachine::onReturnHomeAccepted(const ReturnHome::Goal &)
{
  // The carrot starts at the vehicle, not at an older hold point.
  transitionTo(
    ReturningHome{.carrot = currentPose(flownPose(state_))}, "return_home action accepted");
}

void DroneStateMachine::onStartMission(Trigger::Response & response)
{
  if (const char * reason = startMissionRejectReason()) {
    RCLCPP_WARN(
      get_logger(), "Rejecting start_mission: %s (state=%s)", reason,
      stateName(wireState(state_)));
    response.success = false;
    response.message = reason;
    return;
  }
  // startMissionRejectReason() guarantees Hold; the mission starts from its hold pose, so a
  // planar mission keeps Hold's altitude.
  transitionTo(
    Mission{.hold = std::get<Hold>(state_).target, .enter = now()}, "start_mission requested");
  response.success = true;
  response.message = "Mission started";
}

void DroneStateMachine::onHold(Trigger::Response & response)
{
  if (const char * reason = holdRejectReason()) {
    RCLCPP_WARN(
      get_logger(), "Rejecting hold: %s (state=%s)", reason, stateName(wireState(state_)));
    response.success = false;
    response.message = reason;
    return;
  }
  return_home_action_->abort("Return home stopped by hold");
  transitionTo(Hold{.target = currentPose(flownPose(state_))}, "hold requested");
  response.success = true;
  response.message = "Holding position";
}

void DroneStateMachine::publishTakeoffFeedback(const TakingOff & s, float progress)
{
  if (!takeoff_action_->active()) {
    return;
  }
  auto feedback = std::make_shared<Takeoff::Feedback>();
  feedback->state = stateName(wireState(state_));
  feedback->target_altitude_m = s.origin_z - s.target.z;
  if (px4_->hasLocalPosition() && px4_->localPosition().z_valid) {
    feedback->current_altitude_m = s.origin_z - px4_->localPosition().z;
    feedback->altitude_error_m = std::fabs(
      feedback->target_altitude_m - feedback->current_altitude_m);
  } else {
    feedback->current_altitude_m = 0.0F;
    feedback->altitude_error_m = std::numeric_limits<float>::infinity();
  }
  feedback->progress = progress;
  takeoff_action_->publish(feedback);
}

void DroneStateMachine::publishLandFeedback(const Landing & s, const rclcpp::Time & current_time)
{
  if (!land_action_->active()) {
    return;
  }

  auto feedback = std::make_shared<Land::Feedback>();
  feedback->state = px4_->isArmed() ? "WAITING_FOR_DISARM" : stateName(wireState(state_));
  feedback->armed = px4_->isArmed();
  feedback->elapsed_s = static_cast<float>(
    std::max(0.0, (current_time - s.start).seconds()));
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

DroneStateMachine::Transition DroneStateMachine::failTakeoff(
  const TakingOff & takeoff, const char * reason)
{
  takeoff_action_->abort(reason);
  return endTakeoff(takeoff, reason);
}

DroneStateMachine::Transition DroneStateMachine::endTakeoff(
  const TakingOff & takeoff, const char * reason)
{
  if (shouldRecoverThroughLanding(takeoff, px4_->isArmed())) {
    return Transition{Landing{.start = now()}, reason};
  }
  return Transition{Idle{}, reason};
}

}  // namespace mullet_core
