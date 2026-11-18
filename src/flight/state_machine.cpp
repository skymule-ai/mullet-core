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
  config_ = loadConfig(*this);

  const auto fmu_prefix = fmuPrefix(config_);
  const auto target_system = targetSystemFromVehicleInstance(config_.vehicle_instance);
  px4_ = std::make_unique<Px4Interface>(*this, fmu_prefix, target_system);
  offboard_heartbeat_ = std::make_unique<OffboardHeartbeatPublisher>(
    *this, fmu_prefix, config_.control_rate_hz, [this]() {return buildOffboardSetpoint();});
  action_lifecycle_ = std::make_unique<state_machine_detail::ActionLifecycleCoordinator>();

  initializeSubscriptions();
  initializeActionServers();
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / std::max(config_.control_rate_hz, 1.0)));
  state_enter_time_ = now();
  control_timer_ = create_wall_timer(period, [this]() {controlTick();});
  paddock_heartbeat_ = std::make_unique<PaddockHeartbeatPublisher>(
    *this, config_.heartbeat_topic, config_.heartbeat_rate_hz,
    [this]() {return buildHeartbeatMessage();});
  localization_status_ = std::make_unique<LocalizationStatusPublisher>(*this, fmu_prefix, config_.drone_id);

  RCLCPP_INFO(
    get_logger(),
    "mullet_core started: drone_id='%s' instance=%d fmu_prefix='%s' target_system=%u control_rate=%.1f Hz heartbeat=%.1f Hz",
    config_.drone_id.c_str(), config_.vehicle_instance, fmu_prefix.c_str(),
    static_cast<unsigned int>(target_system), config_.control_rate_hz,
    config_.heartbeat_rate_hz);
}

DroneStateMachine::~DroneStateMachine() = default;

void DroneStateMachine::initializeSubscriptions()
{
  command_sub_ = create_subscription<MissionCommand>(
    config_.command_topic, commandQoS(),
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
  msg.drone_id = config_.drone_id;
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
  msg.has_command_stream = latest_command_.has_value();

  msg.armed = px4_->isArmed();
  msg.offboard = px4_->isOffboard();
  msg.px4_failsafe = px4_->isFailsafe();
  msg.landed = px4_->isLanded();
  msg.local_position_valid = local_position_valid;
  msg.command_stream_valid = commandFresh(latest_command_, current_time, config_.command_timeout_s);

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
  const auto tick_time = std::chrono::steady_clock::now();  // steady: jitter only
  tick_stats_.update(tick_time, config_.control_rate_hz);
  handleExternalOverride();
  handleActionCancellations();
  dispatchState(current_time);
  tick_stats_.maybeLog(get_logger(), tick_time, config_.timing_log_period_s, stateName(state_));
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
