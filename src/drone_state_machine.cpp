#include "mullet_core/drone_state_machine.hpp"
#include "mullet_core/vehicle_instance.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

namespace mullet_core
{

namespace
{
rclcpp::QoS commandQoS()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

bool retryDue(const rclcpp::Time & now, const rclcpp::Time & last, double interval_s)
{
  return last.nanoseconds() == 0 || (now - last).seconds() >= interval_s;
}

bool finiteInRange(double value, double min_value, double max_value)
{
  return std::isfinite(value) && value >= min_value && value <= max_value;
}

float wrapPi(float angle_rad)
{
  constexpr float kPi = 3.14159265358979323846F;
  constexpr float kTwoPi = 2.0F * kPi;
  auto wrapped = static_cast<float>(std::remainder(angle_rad, kTwoPi));
  if (wrapped <= -kPi) {
    wrapped += kTwoPi;
  }
  return wrapped;
}
}  // namespace

DroneStateMachine::DroneStateMachine()
: Node("mullet_core")
{
  loadParameters();

  const auto fmu_prefix =
    config_.vehicle_instance == 0 ? std::string{} : "/px4_" + std::to_string(config_.vehicle_instance);
  const auto target_system = targetSystemFromVehicleInstance(config_.vehicle_instance);
  px4_ = std::make_unique<Px4Interface>(*this, fmu_prefix, target_system);

  command_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    command_topic_, commandQoS(),
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {onCommand(msg);});
  land_sub_ = create_subscription<std_msgs::msg::Empty>(
    land_topic_, rclcpp::QoS(1),
    [this](const std_msgs::msg::Empty::SharedPtr msg) {onLandRequest(msg);});
  takeoff_action_server_ = rclcpp_action::create_server<Takeoff>(
    this, "takeoff",
    std::bind(&DroneStateMachine::handleTakeoffGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DroneStateMachine::handleTakeoffCancel, this, std::placeholders::_1),
    std::bind(&DroneStateMachine::handleTakeoffAccepted, this, std::placeholders::_1));

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / std::max(config_.control_rate_hz, 1.0)));
  state_enter_time_ = now();
  last_timing_log_time_ = state_enter_time_;
  control_timer_ = create_wall_timer(period, [this]() {controlTick();});

  RCLCPP_INFO(
    get_logger(),
    "mullet_core started: instance=%d fmu_prefix='%s' target_system=%u control_rate=%.1f Hz",
    config_.vehicle_instance, fmu_prefix.c_str(), static_cast<unsigned int>(target_system),
    config_.control_rate_hz);
}

void DroneStateMachine::loadParameters()
{
  config_.vehicle_instance = declare_parameter<int>("vehicle_instance", config_.vehicle_instance);
  config_.control_rate_hz = declare_parameter<double>("control_rate_hz", config_.control_rate_hz);
  config_.offboard_warmup_s = declare_parameter<double>("offboard_warmup_s", config_.offboard_warmup_s);
  config_.command_retry_interval_s = declare_parameter<double>(
    "command_retry_interval_s", config_.command_retry_interval_s);
  config_.command_timeout_s = declare_parameter<double>("command_timeout_s", config_.command_timeout_s);
  config_.takeoff_altitude_m = declare_parameter<double>("takeoff_altitude_m", config_.takeoff_altitude_m);
  config_.takeoff_acceptance_m = declare_parameter<double>(
    "takeoff_acceptance_m", config_.takeoff_acceptance_m);
  config_.takeoff_yaw_tolerance_rad = declare_parameter<double>(
    "takeoff_yaw_tolerance_rad", config_.takeoff_yaw_tolerance_rad);
  config_.hover_acceptance_radius_m = declare_parameter<double>(
    "hover_acceptance_radius_m", config_.hover_acceptance_radius_m);
  config_.hover_yaw_rad = declare_parameter<double>("hover_yaw_rad", config_.hover_yaw_rad);
  config_.max_vx_ned_mps = declare_parameter<double>("max_vx_ned_mps", config_.max_vx_ned_mps);
  config_.max_vy_ned_mps = declare_parameter<double>("max_vy_ned_mps", config_.max_vy_ned_mps);
  config_.max_vz_ned_mps = declare_parameter<double>("max_vz_ned_mps", config_.max_vz_ned_mps);
  config_.timing_log_period_s = declare_parameter<double>(
    "timing_log_period_s", config_.timing_log_period_s);
  config_.min_takeoff_altitude_m = declare_parameter<double>(
    "min_takeoff_altitude_m", config_.min_takeoff_altitude_m);
  config_.max_takeoff_altitude_m = declare_parameter<double>(
    "max_takeoff_altitude_m", config_.max_takeoff_altitude_m);
  config_.warmup_timeout_s = declare_parameter<double>("warmup_timeout_s", config_.warmup_timeout_s);
  config_.request_offboard_timeout_s = declare_parameter<double>(
    "request_offboard_timeout_s", config_.request_offboard_timeout_s);
  config_.request_arm_timeout_s = declare_parameter<double>(
    "request_arm_timeout_s", config_.request_arm_timeout_s);
  config_.takeoff_timeout_s = declare_parameter<double>("takeoff_timeout_s", config_.takeoff_timeout_s);
  config_.landing_timeout_s = declare_parameter<double>("landing_timeout_s", config_.landing_timeout_s);
  config_.hover_altitude_tolerance_m = declare_parameter<double>(
    "hover_altitude_tolerance_m", config_.hover_altitude_tolerance_m);
  config_.hover_vertical_velocity_tolerance_mps = declare_parameter<double>(
    "hover_vertical_velocity_tolerance_mps", config_.hover_vertical_velocity_tolerance_mps);
  config_.hover_settle_time_s = declare_parameter<double>(
    "hover_settle_time_s", config_.hover_settle_time_s);
  command_topic_ = declare_parameter<std::string>("command_topic", command_topic_);
  land_topic_ = declare_parameter<std::string>("land_topic", land_topic_);

  config_.control_rate_hz = std::max(config_.control_rate_hz, 2.0);
  config_.offboard_warmup_s = std::max(config_.offboard_warmup_s, 1.0);
  config_.command_retry_interval_s = std::max(config_.command_retry_interval_s, 0.1);
  config_.command_timeout_s = std::max(config_.command_timeout_s, 0.0);
  config_.takeoff_altitude_m = std::max(config_.takeoff_altitude_m, 0.1);
  config_.takeoff_acceptance_m = std::max(config_.takeoff_acceptance_m, 0.05);
  config_.takeoff_yaw_tolerance_rad = std::max(config_.takeoff_yaw_tolerance_rad, 0.0);
  config_.timing_log_period_s = std::max(config_.timing_log_period_s, 0.25);
  config_.min_takeoff_altitude_m = std::max(config_.min_takeoff_altitude_m, 0.1);
  config_.max_takeoff_altitude_m = std::max(config_.max_takeoff_altitude_m, config_.min_takeoff_altitude_m);
  config_.warmup_timeout_s = std::max(config_.warmup_timeout_s, config_.offboard_warmup_s);
  config_.request_offboard_timeout_s = std::max(config_.request_offboard_timeout_s, 0.1);
  config_.request_arm_timeout_s = std::max(config_.request_arm_timeout_s, 0.1);
  config_.takeoff_timeout_s = std::max(config_.takeoff_timeout_s, 0.1);
  config_.landing_timeout_s = std::max(config_.landing_timeout_s, 0.1);
  config_.hover_altitude_tolerance_m = std::max(config_.hover_altitude_tolerance_m, 0.05);
  config_.hover_vertical_velocity_tolerance_mps = std::max(
    config_.hover_vertical_velocity_tolerance_mps, 0.01);
  config_.hover_settle_time_s = std::max(config_.hover_settle_time_s, 0.0);
}

void DroneStateMachine::controlTick()
{
  const auto current_time = now();
  updateTimingStats(current_time);

  if (command_buffer_.land_requested && state_ != DroneState::kLanding &&
    state_ != DroneState::kDisarmed)
  {
    command_buffer_.land_requested = false;
    abortTakeoffGoal("Takeoff preempted by land request");
    beginLanding("land request", false);
  }

  if (goalCancelRequested() && state_ != DroneState::kCancelingToLand &&
    state_ != DroneState::kLanding && state_ != DroneState::kDisarmed)
  {
    if (shouldRecoverThroughLanding()) {
      beginLanding("takeoff action canceled after arm request", true);
    } else {
      cancelTakeoffGoal("Takeoff canceled before arming");
      resetTakeoffAttemptState();
      transitionTo(DroneState::kTakeoffCanceled, "takeoff canceled before arming");
    }
  }

  switch (state_) {
    case DroneState::kInit:
      handleInit(current_time);
      break;
    case DroneState::kReady:
      handleReady(current_time);
      break;
    case DroneState::kTakeoffRequested:
      handleTakeoffRequested(current_time);
      break;
    case DroneState::kWarmupSetpoints:
      handleWarmup(current_time);
      break;
    case DroneState::kRequestOffboard:
      handleRequestOffboard(current_time);
      break;
    case DroneState::kRequestArm:
      handleRequestArm(current_time);
      break;
    case DroneState::kTakeoff:
      handleTakeoff(current_time);
      break;
    case DroneState::kHover:
      handleHover(current_time);
      break;
    case DroneState::kCommand:
      handleCommand(current_time);
      break;
    case DroneState::kCancelingToLand:
      handleCancelingToLand(current_time);
      break;
    case DroneState::kLanding:
      handleLanding(current_time);
      break;
    case DroneState::kTakeoffFailed:
    case DroneState::kTakeoffCanceled:
      if (shouldRecoverThroughLanding()) {
        transitionTo(DroneState::kLanding, "takeoff terminal state requires landing");
      } else {
        resetTakeoffAttemptState();
        transitionTo(DroneState::kReady, "takeoff terminal state");
      }
      break;
    case DroneState::kFailsafeHold:
      handleFailsafeHold(current_time);
      break;
    case DroneState::kDisarmed:
      break;
  }

  maybeLogTiming(current_time);
}

void DroneStateMachine::onCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
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

rclcpp_action::GoalResponse DroneStateMachine::handleTakeoffGoal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Takeoff::Goal> goal)
{
  (void)uuid;
  if (state_ != DroneState::kReady) {
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
  if (!finiteInRange(goal->altitude_m, config_.min_takeoff_altitude_m, config_.max_takeoff_altitude_m)) {
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
  if (goal_handle != active_takeoff_goal_) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  takeoff_cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DroneStateMachine::handleTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
  active_takeoff_goal_ = goal_handle;
  takeoff_cancel_requested_ = false;
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
  transitionTo(DroneState::kTakeoffRequested, "takeoff action accepted");
}

void DroneStateMachine::transitionTo(DroneState new_state, const char * reason)
{
  if (state_ == new_state) {
    return;
  }

  RCLCPP_INFO(
    get_logger(), "State: %s -> %s (%s)", stateName(state_), stateName(new_state), reason);
  state_ = new_state;
  state_enter_time_ = now();
  if (new_state == DroneState::kRequestOffboard || new_state == DroneState::kRequestArm ||
    new_state == DroneState::kCancelingToLand || new_state == DroneState::kLanding)
  {
    last_vehicle_command_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
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
  if (retryDue(current_time, last_vehicle_command_time_, config_.command_retry_interval_s)) {
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
  if (retryDue(current_time, last_vehicle_command_time_, config_.command_retry_interval_s)) {
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
  if (retryDue(current_time, last_vehicle_command_time_, config_.command_retry_interval_s)) {
    px4_->land();
    last_vehicle_command_time_ = current_time;
  }
  transitionTo(DroneState::kLanding, "landing command sent after cancel");
}

void DroneStateMachine::handleLanding(const rclcpp::Time & current_time)
{
  if (retryDue(current_time, last_vehicle_command_time_, config_.command_retry_interval_s)) {
    px4_->land();
    last_vehicle_command_time_ = current_time;
  }
  if (px4_->isDisarmed()) {
    cancelTakeoffGoal("Takeoff canceled; vehicle disarmed after landing");
    resetTakeoffAttemptState();
    command_buffer_.land_requested = false;
    transitionTo(DroneState::kDisarmed, "vehicle disarmed");
    transitionTo(
      px4_->localPositionValid() ? DroneState::kReady : DroneState::kInit,
      px4_->localPositionValid() ? "ready after disarm" : "waiting for telemetry after disarm");
    return;
  }
  if (!landing_timeout_reported_ && phaseTimedOut(current_time, config_.landing_timeout_s)) {
    constexpr const char * timeout_message = "Landing timeout expired while vehicle is still armed";
    RCLCPP_ERROR(get_logger(), "Landing timeout expired while vehicle is still armed");
    if (takeoff_cancel_requested_) {
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
    transitionTo(DroneState::kLanding, "local position invalid during failsafe hold");
    px4_->land();
    last_vehicle_command_time_ = current_time;
    return;
  }

  if (commandFresh(current_time)) {
    transitionTo(DroneState::kCommand, "fresh external velocity command restored");
  }
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

  const auto yaw_error = wrapPi(active_takeoff_target_yaw_rad_ - current_heading);
  if (yaw_error_rad != nullptr) {
    *yaw_error_rad = yaw_error;
  }
  return std::fabs(yaw_error) <= config_.takeoff_yaw_tolerance_rad;
}

bool DroneStateMachine::phaseTimedOut(const rclcpp::Time & current_time, double timeout_s) const
{
  return (current_time - state_enter_time_).seconds() >= timeout_s;
}

bool DroneStateMachine::haveActiveTakeoffGoal() const
{
  return active_takeoff_goal_ != nullptr;
}

bool DroneStateMachine::goalCancelRequested() const
{
  return takeoff_cancel_requested_ && haveActiveTakeoffGoal();
}

bool DroneStateMachine::shouldRecoverThroughLanding() const
{
  return px4_->isArmed() || takeoff_arm_requested_;
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
    active_takeoff_target_yaw_rad_ = wrapPi(goal.yaw_rad);
    hover_yaw_rad_ = active_takeoff_target_yaw_rad_;
  } else if (!std::isfinite(px4_->localPosition().heading)) {
    RCLCPP_ERROR(get_logger(), "Cannot latch takeoff target: current heading is not finite");
    return false;
  } else {
    active_takeoff_target_yaw_rad_ = wrapPi(px4_->localPosition().heading);
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
  active_takeoff_goal_->publish_feedback(feedback);
}

void DroneStateMachine::succeedTakeoffGoal(const char * message)
{
  if (!haveActiveTakeoffGoal()) {
    return;
  }
  auto result = std::make_shared<Takeoff::Result>();
  result->success = true;
  result->message = message;
  active_takeoff_goal_->succeed(result);
  active_takeoff_goal_.reset();
  takeoff_cancel_requested_ = false;
}

void DroneStateMachine::abortTakeoffGoal(const char * message)
{
  if (!haveActiveTakeoffGoal()) {
    return;
  }
  auto result = std::make_shared<Takeoff::Result>();
  result->success = false;
  result->message = message;
  active_takeoff_goal_->abort(result);
  active_takeoff_goal_.reset();
  takeoff_cancel_requested_ = false;
}

void DroneStateMachine::cancelTakeoffGoal(const char * message)
{
  if (!haveActiveTakeoffGoal()) {
    return;
  }
  auto result = std::make_shared<Takeoff::Result>();
  result->success = false;
  result->message = message;
  active_takeoff_goal_->canceled(result);
  active_takeoff_goal_.reset();
  takeoff_cancel_requested_ = false;
}

void DroneStateMachine::beginLanding(const char * reason, bool request_position_mode)
{
  position_mode_requested_for_landing_ = !request_position_mode;
  transitionTo(request_position_mode ? DroneState::kCancelingToLand : DroneState::kLanding, reason);
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

void DroneStateMachine::resetTakeoffAttemptState()
{
  takeoff_arm_requested_ = false;
  active_takeoff_uses_current_yaw_ = true;
  active_takeoff_target_yaw_rad_ = 0.0F;
}

void DroneStateMachine::updateTimingStats(const rclcpp::Time & current_time)
{
  if (!timing_initialized_) {
    timing_initialized_ = true;
    last_tick_time_ = current_time;
    return;
  }

  const auto expected_period_ms = 1000.0 / std::max(config_.control_rate_hz, 1.0);
  const auto dt_ms = (current_time - last_tick_time_).seconds() * 1000.0;
  const auto jitter_ms = std::fabs(dt_ms - expected_period_ms);
  max_tick_jitter_ms_ = std::max(max_tick_jitter_ms_, jitter_ms);
  if (dt_ms > expected_period_ms * 1.5) {
    ++missed_deadline_count_;
  }
  last_tick_time_ = current_time;
}

void DroneStateMachine::maybeLogTiming(const rclcpp::Time & current_time)
{
  if ((current_time - last_timing_log_time_).seconds() < config_.timing_log_period_s) {
    return;
  }

  RCLCPP_INFO(
    get_logger(), "timing: state=%s max_jitter_ms=%.3f missed_deadlines=%lu",
    stateName(state_), max_tick_jitter_ms_, missed_deadline_count_);
  max_tick_jitter_ms_ = 0.0;
  last_timing_log_time_ = current_time;
}

const char * DroneStateMachine::stateName(DroneState state)
{
  switch (state) {
    case DroneState::kInit:
      return "INIT";
    case DroneState::kReady:
      return "READY";
    case DroneState::kTakeoffRequested:
      return "TAKEOFF_REQUESTED";
    case DroneState::kWarmupSetpoints:
      return "WARMUP_SETPOINTS";
    case DroneState::kRequestOffboard:
      return "REQUEST_OFFBOARD";
    case DroneState::kRequestArm:
      return "REQUEST_ARM";
    case DroneState::kTakeoff:
      return "TAKEOFF";
    case DroneState::kHover:
      return "HOVER";
    case DroneState::kCommand:
      return "COMMAND";
    case DroneState::kCancelingToLand:
      return "CANCELING_TO_LAND";
    case DroneState::kLanding:
      return "LANDING";
    case DroneState::kTakeoffFailed:
      return "TAKEOFF_FAILED";
    case DroneState::kTakeoffCanceled:
      return "TAKEOFF_CANCELED";
    case DroneState::kFailsafeHold:
      return "FAILSAFE_HOLD";
    case DroneState::kDisarmed:
      return "DISARMED";
  }
  return "UNKNOWN";
}

}  // namespace mullet_core
