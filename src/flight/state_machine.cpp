#include "mullet_core/flight/drone_state_machine.hpp"
#include "mullet_core/px4/vehicle_instance.hpp"

#include "flight/detail/action_lifecycle_coordinator.hpp"
#include "flight/detail/helpers.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>

namespace mullet_core
{

namespace
{
// QoS for the external command subscription: best-effort, volatile, keep-last(1).
rclcpp::QoS commandQoS()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

// Joins a ROS namespace and a leaf name into an absolute path, e.g. ("/uav_0", "takeoff") ->
// "/uav_0/takeoff".
std::string joinRosPath(const std::string & ns, const std::string & name)
{
  if (ns.empty() || ns == "/") {
    return "/" + name;
  }
  if (ns.back() == '/') {
    return ns + name;
  }
  return ns + "/" + name;
}

std::string fmuPrefix(const ControlConfig & config)
{
  return config.vehicle_instance == 0 || config.px4_namespace_prefix.empty() ?
    std::string{} :
    "/" + config.px4_namespace_prefix + "_" + std::to_string(config.vehicle_instance);
}

}  // namespace

DroneStateMachine::DroneStateMachine(const rclcpp::NodeOptions & options)
: Node("mullet_core", options)
{
  loadParameters();

  const auto fmu_prefix = fmuPrefix(config_);
  const auto target_system = targetSystemFromVehicleInstance(config_.vehicle_instance);
  drone_id_ = config_.drone_id.empty() ?
    "uav_" + std::to_string(config_.vehicle_instance) : config_.drone_id;
  px4_ = std::make_unique<Px4Interface>(*this, fmu_prefix, target_system);
  offboard_heartbeat_ = std::make_unique<OffboardHeartbeatPublisher>(
    *this, fmu_prefix, config_.control_rate_hz, [this]() {return buildOffboardSetpoint();});
  action_lifecycle_ = std::make_unique<state_machine_detail::ActionLifecycleCoordinator>();

  initializeSubscriptions();
  initializeActionServers();
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / std::max(config_.control_rate_hz, 1.0)));
  state_enter_time_ = now();
  last_timing_log_time_ = state_enter_time_;
  control_timer_ = create_wall_timer(period, [this]() {controlTick();});
  paddock_heartbeat_ = std::make_unique<PaddockHeartbeatPublisher>(
    *this, config_.heartbeat_topic, config_.heartbeat_rate_hz,
    [this]() {return buildHeartbeatMessage();});

  RCLCPP_INFO(
    get_logger(),
    "mullet_core started: drone_id='%s' instance=%d fmu_prefix='%s' target_system=%u control_rate=%.1f Hz heartbeat=%.1f Hz",
    drone_id_.c_str(), config_.vehicle_instance, fmu_prefix.c_str(),
    static_cast<unsigned int>(target_system), config_.control_rate_hz,
    config_.heartbeat_rate_hz);
}

DroneStateMachine::~DroneStateMachine() = default;

void DroneStateMachine::initializeSubscriptions()
{
  command_sub_ = create_subscription<MissionCommand>(
    command_topic_, commandQoS(),
    [this](const MissionCommand::SharedPtr msg) {onCommand(msg);});
}

void DroneStateMachine::initializeActionServers()
{
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
  return_home_action_server_ = rclcpp_action::create_server<ReturnHome>(
    this, "return_home",
    std::bind(
      &DroneStateMachine::handleReturnHomeGoal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&DroneStateMachine::handleReturnHomeCancel, this, std::placeholders::_1),
    std::bind(&DroneStateMachine::handleReturnHomeAccepted, this, std::placeholders::_1));
  start_mission_service_ = create_service<Trigger>(
    "start_mission",
    [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
      onStartMission(*response);
    });
  hold_service_ = create_service<Trigger>(
    "hold",
    [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
      onHold(*response);
    });
}

// Declares every ROS parameter backing `config_`/topic names and clamps each to a safe minimum,
// so a misconfigured launch file can't produce e.g. a zero control rate or an inverted altitude
// range.
void DroneStateMachine::loadParameters()
{
  const auto declare_and_clamp = [this](const char * name, double & value, double minimum) {
      value = std::max(declare_parameter<double>(name, value), minimum);
    };

  config_.vehicle_instance = declare_parameter<int>("vehicle_instance", config_.vehicle_instance);
  config_.px4_namespace_prefix = declare_parameter<std::string>(
    "px4_namespace_prefix", config_.px4_namespace_prefix);
  config_.drone_id = declare_parameter<std::string>("drone_id", config_.drone_id);
  declare_and_clamp("control_rate_hz", config_.control_rate_hz, 2.0);
  declare_and_clamp("heartbeat_rate_hz", config_.heartbeat_rate_hz, 1.0);
  config_.heartbeat_topic = declare_parameter<std::string>("heartbeat_topic", config_.heartbeat_topic);
  declare_and_clamp("offboard_warmup_s", config_.offboard_warmup_s, 1.0);
  declare_and_clamp("command_retry_interval_s", config_.command_retry_interval_s, 0.1);
  declare_and_clamp("command_timeout_s", config_.command_timeout_s, 0.0);
  declare_and_clamp("takeoff_acceptance_m", config_.takeoff_acceptance_m, 0.05);
  declare_and_clamp("takeoff_yaw_tolerance_rad", config_.takeoff_yaw_tolerance_rad, 0.0);
  config_.hover_yaw_rad = declare_parameter<double>("hover_yaw_rad", config_.hover_yaw_rad);
  declare_and_clamp("max_vx_ned_mps", config_.max_vx_ned_mps, 0.0);
  declare_and_clamp("max_vy_ned_mps", config_.max_vy_ned_mps, 0.0);
  declare_and_clamp("max_vz_ned_mps", config_.max_vz_ned_mps, 0.0);
  declare_and_clamp("max_accel_ned_mps2", config_.max_accel_ned_mps2, 0.0);
  declare_and_clamp("max_xy_position_lead_m", config_.max_xy_position_lead_m, 0.0);
  declare_and_clamp("max_z_position_lead_m", config_.max_z_position_lead_m, 0.0);
  declare_and_clamp("timing_log_period_s", config_.timing_log_period_s, 0.25);
  declare_and_clamp("min_takeoff_altitude_m", config_.min_takeoff_altitude_m, 0.1);
  declare_and_clamp(
    "max_takeoff_altitude_m", config_.max_takeoff_altitude_m,
    config_.min_takeoff_altitude_m);
  declare_and_clamp("request_offboard_timeout_s", config_.request_offboard_timeout_s, 0.1);
  declare_and_clamp("request_arm_timeout_s", config_.request_arm_timeout_s, 0.1);
  declare_and_clamp("takeoff_timeout_s", config_.takeoff_timeout_s, 0.1);
  declare_and_clamp("landing_timeout_s", config_.landing_timeout_s, 0.1);
  declare_and_clamp("hover_altitude_tolerance_m", config_.hover_altitude_tolerance_m, 0.05);
  declare_and_clamp(
    "hover_vertical_velocity_tolerance_mps",
    config_.hover_vertical_velocity_tolerance_mps, 0.01);
  declare_and_clamp("hover_settle_time_s", config_.hover_settle_time_s, 0.0);
  declare_and_clamp("return_home_speed_mps", config_.return_home_speed_mps, 0.1);
  declare_and_clamp("return_home_acceptance_m", config_.return_home_acceptance_m, 0.05);
  command_topic_ = declare_parameter<std::string>("command_topic", command_topic_);
}

// Assembles the full `/mullet/heartbeat` message from current PX4 telemetry, FSM state, and
// action activity. Called once per heartbeat tick by PaddockHeartbeatPublisher; this is the sole
// place the message content is populated, since it needs deep FSM-internal state.
mullet_interfaces::msg::DroneHeartbeat DroneStateMachine::buildHeartbeatMessage()
{
  mullet_interfaces::msg::DroneHeartbeat msg{};
  const auto current_time = now();
  const bool local_position_valid = px4_->localPositionValid();
  const bool has_vehicle_status = px4_->hasVehicleStatus();
  const auto ros_namespace = std::string(get_namespace());

  msg.schema_version = 3;
  msg.stamp = current_time;
  msg.sequence = heartbeat_sequence_++;
  msg.drone_id = drone_id_;
  msg.vehicle_instance = static_cast<std::uint8_t>(config_.vehicle_instance);
  msg.target_system = targetSystemFromVehicleInstance(config_.vehicle_instance);
  msg.ros_namespace = ros_namespace;
  msg.px4_fmu_prefix = fmuPrefix(config_);

  msg.takeoff_action_path = joinRosPath(ros_namespace, "takeoff");
  msg.land_action_path = joinRosPath(ros_namespace, "land");
  msg.return_home_action_path = joinRosPath(ros_namespace, "return_home");
  msg.start_mission_service_path = joinRosPath(ros_namespace, "start_mission");
  msg.hold_service_path = joinRosPath(ros_namespace, "hold");

  msg.state = static_cast<std::uint16_t>(state_);
  msg.state_label = stateName(state_);
  msg.previous_state = static_cast<std::uint16_t>(previous_state_);
  msg.previous_state_label = stateName(previous_state_);
  msg.active_action = activeActionName();

  msg.has_vehicle_status = has_vehicle_status;
  msg.has_local_position = px4_->hasLocalPosition();
  msg.has_land_detected = px4_->hasLandDetected();
  msg.has_command_stream = latest_command_.valid;

  msg.armed = px4_->isArmed();
  msg.offboard = px4_->isOffboard();
  msg.px4_failsafe = px4_->isFailsafe();
  msg.landed = px4_->isLanded();
  msg.local_position_valid = local_position_valid;
  msg.command_stream_valid = commandFresh(current_time);

  if (has_vehicle_status) {
    msg.px4_nav_state = px4_->vehicleStatus().nav_state;
    msg.px4_nav_state_label = navStateName(msg.px4_nav_state);
    msg.px4_arming_state = px4_->vehicleStatus().arming_state;
    msg.px4_arming_state_label = armingStateName(msg.px4_arming_state);
  } else {
    msg.px4_nav_state_label = "UNKNOWN";
    msg.px4_arming_state_label = "UNKNOWN";
  }

  if (local_position_valid) {
    const auto & position = px4_->localPosition();
    msg.x = position.x;
    msg.y = position.y;
    msg.z = position.z;
    msg.yaw = position.heading;
    if (position.xy_global && position.z_global) {
      const auto global = state_machine_detail::localToGlobal(
        position.ref_lat, position.ref_lon, position.ref_alt, position.x, position.y, position.z);
      msg.global_position_valid = true;
      msg.lat = global.lat_deg;
      msg.lon = global.lon_deg;
      msg.alt = global.alt_m;
    }
  }

  msg.can_takeoff = canTakeoff();
  msg.can_land = canLand();
  msg.can_return_home = canReturnHome();
  msg.can_start_mission = canStartMission();
  msg.can_hold = canHold();

  return msg;
}

// Runs one control-loop iteration at `control_rate_hz`: PX4 overrides first (they outrank any
// command), then pending action cancellations, then the current state's tick handler.
void DroneStateMachine::controlTick()
{
  const auto current_time = now();
  updateTimingStats(current_time);
  handleExternalOverride();
  handleActionCancellations();
  dispatchState(current_time);
  maybeLogTiming(current_time);
}

void DroneStateMachine::handleExternalOverride()
{
  const bool airborne_under_offboard = state_ == DroneState::kHold ||
    state_ == DroneState::kMission ||
    (state_ == DroneState::kTransition &&
    (phase_ == Phase::kAscent || phase_ == Phase::kReturnHome));
  if (!airborne_under_offboard) {
    return;
  }
  if (px4_->isDisarmed()) {
    action_lifecycle_->takeoff().abort("Takeoff aborted: vehicle disarmed externally");
    action_lifecycle_->returnHome().abort("Return home aborted: vehicle disarmed externally");
    transitionTo(DroneState::kIdle, "PX4 disarmed externally");
    return;
  }
  if (!px4_->isOffboard()) {
    // Keep the hold target on the vehicle while the pilot flies, so re-entering offboard holds
    // in place instead of flying back to a stale setpoint.
    latchHoverTarget();
    action_lifecycle_->takeoff().abort("Takeoff aborted: PX4 left offboard");
    action_lifecycle_->returnHome().abort("Return home aborted: PX4 left offboard");
    transitionTo(DroneState::kHold, "PX4 left offboard; pilot has control");
  }
}

void DroneStateMachine::handleActionCancellations()
{
  if (action_lifecycle_->takeoff().cancelRequested()) {
    action_lifecycle_->takeoff().cancel("Takeoff canceled");
    endTakeoff("takeoff canceled");
  }
  if (action_lifecycle_->returnHome().cancelRequested()) {
    action_lifecycle_->returnHome().cancel("Return home canceled; holding");
    latchHoverTarget();
    transitionTo(DroneState::kHold, "return home canceled");
  }
}

void DroneStateMachine::dispatchState(const rclcpp::Time & current_time)
{
  switch (state_) {
    case DroneState::kIdle:
    case DroneState::kHold:
      break;
    case DroneState::kTransition:
      handleTransition(current_time);
      break;
    case DroneState::kMission:
      handleMission(current_time);
      break;
  }
}

}  // namespace mullet_core
