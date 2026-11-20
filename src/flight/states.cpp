#include "mullet_core/flight/drone_state_machine.hpp"

#include <cmath>

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

void DroneStateMachine::transitionTo(FlightState next, const char * reason)
{
  if (wireState(state_) != wireState(next)) {
    RCLCPP_INFO(get_logger(), "State: %s -> %s (%s)", stateName(wireState(state_)), stateName(wireState(next)), reason);
    previous_state_ = wireState(state_);
  }
  state_ = std::move(next);
}

std::optional<DroneStateMachine::Transition> DroneStateMachine::tick(TakingOff & s, const rclcpp::Time & t)
{
  using Step = TakingOff::Step;
  // A step change is not a state transition: the alternative is mutated in place, restarting
  // the step timer and the PX4 command retry timer (so the next step's command fires now).
  auto advanceStep = [&](Step next, const char * reason) {
    s.step = next;
    s.step_enter = t;
    s.last_command.reset();
    RCLCPP_INFO(get_logger(), "TRANSITION step -> %s (%s)", stepName(next), reason);
  };

  const float progress = s.step == Step::kWarmup ? kTakeoffProgressWarmup :
    s.step == Step::kRequestOffboard ? kTakeoffProgressRequestOffboard :
    s.step == Step::kRequestArm ? kTakeoffProgressRequestArm : kTakeoffProgressAscent;
  publishTakeoffFeedback(s, progress);
  if (s.step == Step::kWarmup) {
    if ((t - s.step_enter).seconds() < config_.offboard_warmup_s) {
      return std::nullopt;
    }
    advanceStep(Step::kRequestOffboard, "offboard warmup complete");
  }
  if (s.step == Step::kRequestOffboard) {
    if ((t - s.step_enter).seconds() >= config_.request_offboard_timeout_s) {
      return failTakeoff(s, "offboard request timed out");
    }
    if (state_machine_detail::retryDue(t, s.last_command, config_.command_retry_interval_s)) {
      px4_->setOffboardMode();
      s.last_command = t;
    }
    if (!px4_->isOffboard()) {
      return std::nullopt;
    }
    advanceStep(Step::kRequestArm, "PX4 entered offboard mode");
  }
  if (s.step == Step::kRequestArm) {
    if ((t - s.step_enter).seconds() >= config_.request_arm_timeout_s) {
      return failTakeoff(s, "arm request timed out");
    }
    if (!px4_->isOffboard()) {
      return failTakeoff(s, "offboard mode lost before arming");
    }
    if (state_machine_detail::retryDue(t, s.last_command, config_.command_retry_interval_s)) {
      px4_->arm();
      s.arm_requested = true;
      s.last_command = t;
    }
    if (!px4_->isArmed()) {
      return std::nullopt;
    }
    advanceStep(Step::kAscent, "PX4 armed");
  }
  if (s.step == Step::kAscent && (t - s.step_enter).seconds() >= config_.takeoff_timeout_s) {
    return failTakeoff(s, "takeoff timed out");
  }
  if (takeoffSettled(s, t)) {
    auto pose = currentPose(s.target);
    if (!s.use_current_yaw) {
      pose.yaw = s.target.yaw;
    }
    publishTakeoffFeedback(s, 1.0F);
    takeoff_action_->succeed("Takeoff complete");
    return Transition{Hold{.target = pose}, "takeoff target reached"};
  }
  return std::nullopt;
}

std::optional<DroneStateMachine::Transition> DroneStateMachine::tick(
  Landing & s, const rclcpp::Time & t)
{
  publishLandFeedback(s, t);
  if (!s.position_mode_sent) {
    // last_command stays empty, so NAV_LAND below still goes out in this same first tick.
    px4_->setPositionMode();
    s.position_mode_sent = true;
  }
  if (state_machine_detail::retryDue(t, s.last_command, config_.command_retry_interval_s)) {
    px4_->land();
    s.last_command = t;
  }
  if (px4_->isDisarmed()) {
    land_action_->succeed("Vehicle disarmed after landing");
    return Transition{Idle{}, "vehicle disarmed"};
  }
  if (!s.timeout_reported && (t - s.start).seconds() >= config_.landing_timeout_s) {
    constexpr const char * kMessage = "Landing timed out while vehicle is still armed";
    RCLCPP_ERROR(get_logger(), "%s", kMessage);
    land_action_->abort(kMessage);
    s.timeout_reported = true;  // keep landing: only the goal gives up, not the vehicle
  }
  return std::nullopt;
}

std::optional<DroneStateMachine::Transition> DroneStateMachine::tick(
  ReturningHome & s, const rclcpp::Time &)
{
  // returnHomeRejectReason() only accepts with a home latched, and home_ is never cleared.
  const auto [home_x, home_y] = *home_;
  const auto & position = px4_->localPosition();
  const float distance_m = std::hypot(home_x - position.x, home_y - position.y);
  publishReturnHomeFeedback(distance_m);
  if (distance_m <= config_.return_home_acceptance_m) {
    return_home_action_->succeed("Arrived home");
    return Transition{Hold{.target = currentPose(s.carrot)}, "arrived home"};
  }
  // Slide the carrot toward home by one tick of travel, so PX4 never sees a setpoint step it
  // would chase at its own maximum speed.
  const auto step_m = static_cast<float>(config_.return_home_speed_mps / config_.control_rate_hz);
  const float dx = home_x - s.carrot.x;
  const float dy = home_y - s.carrot.y;
  const float carrot_distance_m = std::hypot(dx, dy);
  if (carrot_distance_m <= step_m) {
    s.carrot.x = home_x;
    s.carrot.y = home_y;
  } else {
    s.carrot.x += dx / carrot_distance_m * step_m;
    s.carrot.y += dy / carrot_distance_m * step_m;
  }
  return std::nullopt;
}

std::optional<DroneStateMachine::Transition> DroneStateMachine::tick(
  Mission & s, const rclcpp::Time & t)
{
  // While the stream is fresh the drone moves, so keep the hold pose on it: a stale stream
  // then holds wherever the drone was, not where the mission started. In planar modes the
  // altitude stays latched from entering MISSION, since the command stream never controls it;
  // in 3D modes the stream moves z too, so the hold follows it.
  if (commandFresh(latest_command_, t, config_.command_timeout_s)) {
    const auto mission_z_ned_m = s.hold.z;
    s.hold = currentPose(s.hold);
    if (!is3d(latest_command_->mode)) {
      s.hold.z = mission_z_ned_m;
    }
    return std::nullopt;
  }
  // A mission that stops streaming (its node crashed or was restarted) must not keep MISSION
  // forever: after command_loss_hold_s, counted from the later of the last command and entering
  // MISSION, drop to HOLD on the pose the stream left behind (no fresh latch).
  const auto last_sign_of_life = latest_command_ && latest_command_->stamp > s.enter ?
    latest_command_->stamp : s.enter;
  if ((t - last_sign_of_life).seconds() >= config_.command_loss_hold_s) {
    RCLCPP_WARN(get_logger(), "Mission command stream lost for %.1f s", config_.command_loss_hold_s);
    return Transition{Hold{.target = s.hold}, "mission command stream lost"};
  }
  return std::nullopt;
}

}  // namespace mullet_core
