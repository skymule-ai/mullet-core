#include "mullet_core/drone_state_machine.hpp"
#include "mullet_core/vehicle_instance.hpp"

#include "action_lifecycle_coordinator.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

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

void declareDeprecatedCompatibilityParameters(rclcpp::Node & node)
{
  node.declare_parameter<double>("takeoff_altitude_m", 2.0);
  node.declare_parameter<double>("hover_acceptance_radius_m", 0.25);
}

}  // namespace

DroneStateMachine::DroneStateMachine(const rclcpp::NodeOptions & options)
: Node("mullet_core", options)
{
  loadParameters();

  const auto fmu_prefix =
    config_.vehicle_instance == 0 || config_.px4_namespace_prefix.empty() ?
    std::string{} :
    "/" + config_.px4_namespace_prefix + "_" + std::to_string(config_.vehicle_instance);
  const auto target_system = targetSystemFromVehicleInstance(config_.vehicle_instance);
  px4_ = std::make_unique<Px4Interface>(*this, fmu_prefix, target_system);
  action_lifecycle_ = std::make_unique<state_machine_detail::ActionLifecycleCoordinator>();

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
  land_action_server_ = rclcpp_action::create_server<Land>(
    this, "land",
    std::bind(&DroneStateMachine::handleLandGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DroneStateMachine::handleLandCancel, this, std::placeholders::_1),
    std::bind(&DroneStateMachine::handleLandAccepted, this, std::placeholders::_1));
  arm_action_server_ = rclcpp_action::create_server<Arm>(
    this, "arm",
    std::bind(&DroneStateMachine::handleArmGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DroneStateMachine::handleArmCancel, this, std::placeholders::_1),
    std::bind(&DroneStateMachine::handleArmAccepted, this, std::placeholders::_1));
  disarm_action_server_ = rclcpp_action::create_server<Disarm>(
    this, "disarm",
    std::bind(
      &DroneStateMachine::handleDisarmGoal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&DroneStateMachine::handleDisarmCancel, this, std::placeholders::_1),
    std::bind(&DroneStateMachine::handleDisarmAccepted, this, std::placeholders::_1));
  enable_offboard_action_server_ = rclcpp_action::create_server<EnableOffboard>(
    this, "enable_offboard",
    std::bind(
      &DroneStateMachine::handleEnableOffboardGoal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&DroneStateMachine::handleEnableOffboardCancel, this, std::placeholders::_1),
    std::bind(&DroneStateMachine::handleEnableOffboardAccepted, this, std::placeholders::_1));
  disable_offboard_action_server_ = rclcpp_action::create_server<DisableOffboard>(
    this, "disable_offboard",
    std::bind(
      &DroneStateMachine::handleDisableOffboardGoal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&DroneStateMachine::handleDisableOffboardCancel, this, std::placeholders::_1),
    std::bind(&DroneStateMachine::handleDisableOffboardAccepted, this, std::placeholders::_1));

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

DroneStateMachine::~DroneStateMachine() = default;

void DroneStateMachine::loadParameters()
{
  config_.vehicle_instance = declare_parameter<int>("vehicle_instance", config_.vehicle_instance);
  config_.px4_namespace_prefix = declare_parameter<std::string>(
    "px4_namespace_prefix", config_.px4_namespace_prefix);
  config_.control_rate_hz = declare_parameter<double>("control_rate_hz", config_.control_rate_hz);
  config_.offboard_warmup_s = declare_parameter<double>("offboard_warmup_s", config_.offboard_warmup_s);
  config_.command_retry_interval_s = declare_parameter<double>(
    "command_retry_interval_s", config_.command_retry_interval_s);
  config_.command_timeout_s = declare_parameter<double>("command_timeout_s", config_.command_timeout_s);
  declareDeprecatedCompatibilityParameters(*this);
  config_.takeoff_acceptance_m = declare_parameter<double>(
    "takeoff_acceptance_m", config_.takeoff_acceptance_m);
  config_.takeoff_yaw_tolerance_rad = declare_parameter<double>(
    "takeoff_yaw_tolerance_rad", config_.takeoff_yaw_tolerance_rad);
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
  config_.disable_offboard_timeout_s = declare_parameter<double>(
    "disable_offboard_timeout_s", config_.disable_offboard_timeout_s);
  config_.request_arm_timeout_s = declare_parameter<double>(
    "request_arm_timeout_s", config_.request_arm_timeout_s);
  config_.armed_hold_duration_s = declare_parameter<double>(
    "armed_hold_duration_s", config_.armed_hold_duration_s);
  config_.request_disarm_timeout_s = declare_parameter<double>(
    "request_disarm_timeout_s", config_.request_disarm_timeout_s);
  config_.request_position_mode_timeout_s = declare_parameter<double>(
    "request_position_mode_timeout_s", config_.request_position_mode_timeout_s);
  config_.landed_confirmation_s = declare_parameter<double>(
    "landed_confirmation_s", config_.landed_confirmation_s);
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
  config_.takeoff_acceptance_m = std::max(config_.takeoff_acceptance_m, 0.05);
  config_.takeoff_yaw_tolerance_rad = std::max(config_.takeoff_yaw_tolerance_rad, 0.0);
  config_.timing_log_period_s = std::max(config_.timing_log_period_s, 0.25);
  config_.min_takeoff_altitude_m = std::max(config_.min_takeoff_altitude_m, 0.1);
  config_.max_takeoff_altitude_m = std::max(config_.max_takeoff_altitude_m, config_.min_takeoff_altitude_m);
  config_.warmup_timeout_s = std::max(config_.warmup_timeout_s, config_.offboard_warmup_s);
  config_.request_offboard_timeout_s = std::max(config_.request_offboard_timeout_s, 0.1);
  config_.disable_offboard_timeout_s = std::max(config_.disable_offboard_timeout_s, 0.1);
  config_.request_arm_timeout_s = std::max(config_.request_arm_timeout_s, 0.1);
  config_.armed_hold_duration_s = std::max(config_.armed_hold_duration_s, 0.1);
  config_.request_disarm_timeout_s = std::max(config_.request_disarm_timeout_s, 0.1);
  config_.request_position_mode_timeout_s = std::max(
    config_.request_position_mode_timeout_s, 0.1);
  config_.landed_confirmation_s = std::max(config_.landed_confirmation_s, 0.0);
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

  if (command_buffer_.land_requested) {
    command_buffer_.land_requested = false;
    if (state_ == DroneState::kDisarmed) {
      RCLCPP_WARN(get_logger(), "Ignoring emergency land request: vehicle is already disarmed");
    } else if (state_ == DroneState::kOffboardHold && !px4_->isArmed()) {
      RCLCPP_WARN(
        get_logger(), "Ignoring emergency land request: vehicle is disarmed in OFFBOARD_HOLD");
    } else if (haveActiveEnableOffboardGoal()) {
      abortEnableOffboardGoal("EnableOffboard preempted by emergency land request");
      if (px4_->isOffboard() && px4_->localPositionValid()) {
        finishEnableOffboardAttempt(
          DroneState::kOffboardHold,
          "offboard hold after emergency land preempted enable_offboard");
      } else {
        finishEnableOffboardAttempt(
          px4_->localPositionValid() ? DroneState::kReady : DroneState::kInit,
          px4_->localPositionValid() ? "ready after emergency land preempted enable_offboard" :
          "waiting for telemetry after emergency land preempted enable_offboard");
      }
    } else if (haveActiveDisableOffboardGoal()) {
      abortDisableOffboardGoal("DisableOffboard preempted by emergency land request");
      finishDisableOffboardAttempt("emergency land preempted disable_offboard");
    } else if (haveActiveArmGoal()) {
      abortArmGoal("Arm preempted by emergency land request");
      beginLanding("emergency land preempted arm", true);
    } else if (state_ == DroneState::kLanding) {
      RCLCPP_WARN(get_logger(), "Emergency land request received while already landing");
      resetVehicleCommandRetryTimer();
    } else {
      abortTakeoffGoal("Takeoff preempted by emergency land request");
      startLanding("emergency land request");
    }
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

  if (enableOffboardCancelRequested()) {
    cancelEnableOffboardGoal("EnableOffboard canceled before completion");
    finishEnableOffboardAttempt(
      px4_->localPositionValid() ? DroneState::kReady : DroneState::kInit,
      px4_->localPositionValid() ? "ready after enable_offboard cancel" :
      "waiting for telemetry after enable_offboard cancel");
  }

  if (disableOffboardCancelRequested()) {
    cancelDisableOffboardGoal("DisableOffboard canceled before completion");
    finishDisableOffboardAttempt("disable_offboard canceled");
  }

  if (action_lifecycle_->armCancelRequested() && haveActiveArmGoal()) {
    cancelArmGoal("Arm canceled before completion");
    if (!px4_->isArmed() && px4_->isOffboard()) {
      release_offboard_after_disarm_ = false;
      transitionTo(DroneState::kReleaseOffboardRequest, "arm canceled; release offboard");
    } else {
      transitionTo(
        px4_->localPositionValid() ? DroneState::kReady : DroneState::kInit,
        px4_->localPositionValid() ? "ready after arm cancel" : "waiting for telemetry after arm cancel");
    }
  }

  if (action_lifecycle_->disarmCancelRequested() && haveActiveDisarmGoal()) {
    cancelDisarmGoal("Disarm canceled before completion");
    if (state_ == DroneState::kDisarmCommand) {
      armed_hold_deadline_ = current_time +
        rclcpp::Duration::from_seconds(config_.armed_hold_duration_s);
      transitionTo(DroneState::kArmedOffboardHold, "disarm canceled");
    }
  }

  switch (state_) {
    case DroneState::kInit:
      handleInit(current_time);
      break;
    case DroneState::kReady:
      handleReady(current_time);
      break;
    case DroneState::kEnableOffboardRequested:
      handleEnableOffboardRequested(current_time);
      break;
    case DroneState::kWarmupOffboardHoldSetpoints:
      handleWarmupOffboardHoldSetpoints(current_time);
      break;
    case DroneState::kRequestOffboardOnly:
      handleRequestOffboardOnly(current_time);
      break;
    case DroneState::kOffboardHold:
      handleOffboardHold(current_time);
      break;
    case DroneState::kArmRequested:
      handleArmRequested(current_time);
      break;
    case DroneState::kArmCommand:
      handleArmCommand(current_time);
      break;
    case DroneState::kArmedOffboardHold:
      handleArmedOffboardHold(current_time);
      break;
    case DroneState::kDisarmCommand:
      handleDisarmCommand(current_time);
      break;
    case DroneState::kReleaseOffboardRequest:
      handleReleaseOffboardRequest(current_time);
      break;
    case DroneState::kDisarmedOffboardReleaseFailed:
      handleDisarmedOffboardReleaseFailed(current_time);
      break;
    case DroneState::kDisableOffboardRequested:
      handleDisableOffboardRequested(current_time);
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
        startLanding("takeoff terminal state requires landing");
      } else {
        resetTakeoffAttemptState();
        transitionTo(DroneState::kReady, "takeoff terminal state");
      }
      break;
    case DroneState::kFailsafeHold:
      handleFailsafeHold(current_time);
      break;
    case DroneState::kDisarmed:
      handleDisarmed(current_time);
      break;
  }

  maybeLogTiming(current_time);
}

}  // namespace mullet_core
