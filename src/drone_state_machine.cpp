#include "mullet_core/drone_state_machine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
}  // namespace

DroneStateMachine::DroneStateMachine()
: Node("mullet_core")
{
  loadParameters();

  const auto fmu_prefix =
    config_.vehicle_instance == 0 ? std::string{} : "/px4_" + std::to_string(config_.vehicle_instance);
  const auto target_system = static_cast<std::uint8_t>(config_.vehicle_instance + 1);
  px4_ = std::make_unique<Px4Interface>(*this, fmu_prefix, target_system);

  command_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    command_topic_, commandQoS(),
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {onCommand(msg);});
  land_sub_ = create_subscription<std_msgs::msg::Empty>(
    land_topic_, rclcpp::QoS(1),
    [this](const std_msgs::msg::Empty::SharedPtr msg) {onLandRequest(msg);});

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / std::max(config_.control_rate_hz, 1.0)));
  state_enter_time_ = now();
  last_timing_log_time_ = state_enter_time_;
  control_timer_ = create_wall_timer(period, [this]() {controlTick();});

  RCLCPP_INFO(
    get_logger(),
    "mullet_core started: instance=%d fmu_prefix='%s' target_system=%u control_rate=%.1f Hz",
    config_.vehicle_instance, fmu_prefix.c_str(), target_system, config_.control_rate_hz);
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
  config_.hover_acceptance_radius_m = declare_parameter<double>(
    "hover_acceptance_radius_m", config_.hover_acceptance_radius_m);
  config_.hover_yaw_rad = declare_parameter<double>("hover_yaw_rad", config_.hover_yaw_rad);
  config_.max_vx_ned_mps = declare_parameter<double>("max_vx_ned_mps", config_.max_vx_ned_mps);
  config_.max_vy_ned_mps = declare_parameter<double>("max_vy_ned_mps", config_.max_vy_ned_mps);
  config_.max_vz_ned_mps = declare_parameter<double>("max_vz_ned_mps", config_.max_vz_ned_mps);
  config_.timing_log_period_s = declare_parameter<double>(
    "timing_log_period_s", config_.timing_log_period_s);
  command_topic_ = declare_parameter<std::string>("command_topic", command_topic_);
  land_topic_ = declare_parameter<std::string>("land_topic", land_topic_);

  config_.control_rate_hz = std::max(config_.control_rate_hz, 2.0);
  config_.offboard_warmup_s = std::max(config_.offboard_warmup_s, 1.0);
  config_.command_retry_interval_s = std::max(config_.command_retry_interval_s, 0.1);
  config_.command_timeout_s = std::max(config_.command_timeout_s, 0.0);
  config_.takeoff_altitude_m = std::max(config_.takeoff_altitude_m, 0.1);
  config_.takeoff_acceptance_m = std::max(config_.takeoff_acceptance_m, 0.05);
  config_.timing_log_period_s = std::max(config_.timing_log_period_s, 0.25);
}

void DroneStateMachine::controlTick()
{
  const auto current_time = now();
  updateTimingStats(current_time);

  if (command_buffer_.land_requested && state_ != DroneState::kLanding &&
    state_ != DroneState::kDisarmed)
  {
    command_buffer_.land_requested = false;
    transitionTo(DroneState::kLanding, "land request");
  }

  switch (state_) {
    case DroneState::kInit:
      handleInit(current_time);
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
    case DroneState::kLanding:
      handleLanding(current_time);
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
    new_state == DroneState::kLanding)
  {
    last_vehicle_command_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }
}

void DroneStateMachine::handleInit(const rclcpp::Time & /*now*/)
{
  if (!px4_->hasVehicleStatus() || !px4_->localPositionValid()) {
    return;
  }

  latchTakeoffTarget();
  transitionTo(DroneState::kWarmupSetpoints, "PX4 state and local position available");
}

void DroneStateMachine::handleWarmup(const rclcpp::Time & current_time)
{
  publishTakeoffSetpoint();
  if ((current_time - state_enter_time_).seconds() >= config_.offboard_warmup_s) {
    transitionTo(DroneState::kRequestOffboard, "offboard proof-of-life warmup complete");
  }
}

void DroneStateMachine::handleRequestOffboard(const rclcpp::Time & current_time)
{
  publishTakeoffSetpoint();
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
  if (!px4_->isOffboard()) {
    transitionTo(DroneState::kRequestOffboard, "offboard mode lost before arming");
    return;
  }
  if (retryDue(current_time, last_vehicle_command_time_, config_.command_retry_interval_s)) {
    px4_->arm();
    last_vehicle_command_time_ = current_time;
  }
  if (px4_->isArmed()) {
    transitionTo(DroneState::kTakeoff, "vehicle armed");
  }
}

void DroneStateMachine::handleTakeoff(const rclcpp::Time & /*now*/)
{
  publishTakeoffSetpoint();
  if (!px4_->isOffboard()) {
    transitionTo(DroneState::kRequestOffboard, "offboard mode lost during takeoff");
    return;
  }
  if (takeoffReached()) {
    latchHoverTarget();
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

void DroneStateMachine::handleLanding(const rclcpp::Time & current_time)
{
  if (retryDue(current_time, last_vehicle_command_time_, config_.command_retry_interval_s)) {
    px4_->land();
    last_vehicle_command_time_ = current_time;
  }
  if (px4_->isDisarmed()) {
    transitionTo(DroneState::kDisarmed, "vehicle disarmed");
  }
}

void DroneStateMachine::handleFailsafeHold(const rclcpp::Time & current_time)
{
  if (px4_->localPositionValid()) {
    publishHoverSetpoint();
  } else {
    transitionTo(DroneState::kLanding, "local position invalid during failsafe hold");
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

bool DroneStateMachine::takeoffReached() const
{
  if (!px4_->hasLocalPosition() || !px4_->localPosition().z_valid) {
    return false;
  }
  return std::fabs(px4_->localPosition().z - takeoff_z_ned_m_) <= config_.takeoff_acceptance_m;
}

void DroneStateMachine::latchTakeoffTarget()
{
  const auto & position = px4_->localPosition();
  takeoff_x_ned_m_ = position.xy_valid ? position.x : 0.0F;
  takeoff_y_ned_m_ = position.xy_valid ? position.y : 0.0F;
  const auto current_z_ned_m = position.z_valid ? position.z : 0.0F;
  takeoff_z_ned_m_ = current_z_ned_m - static_cast<float>(std::fabs(config_.takeoff_altitude_m));
  hover_x_ned_m_ = takeoff_x_ned_m_;
  hover_y_ned_m_ = takeoff_y_ned_m_;
  hover_z_ned_m_ = takeoff_z_ned_m_;
  hover_yaw_rad_ = std::isfinite(position.heading) ? position.heading : static_cast<float>(config_.hover_yaw_rad);

  RCLCPP_INFO(
    get_logger(), "Takeoff target latched: x=%.2f y=%.2f z=%.2f yaw=%.2f",
    takeoff_x_ned_m_, takeoff_y_ned_m_, takeoff_z_ned_m_, hover_yaw_rad_);
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
    case DroneState::kLanding:
      return "LANDING";
    case DroneState::kFailsafeHold:
      return "FAILSAFE_HOLD";
    case DroneState::kDisarmed:
      return "DISARMED";
  }
  return "UNKNOWN";
}

}  // namespace mullet_core
