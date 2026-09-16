#include "mullet_core/flight/drone_state_machine.hpp"
#include "mullet_core/px4/topic_utils.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <mullet_interfaces/msg/drone_heartbeat.hpp>
#include <mullet_interfaces/msg/mission_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mullet_core
{
namespace
{

using namespace std::chrono_literals;

using Land = mullet_interfaces::action::Land;
using ReturnHome = mullet_interfaces::action::ReturnHome;
using Takeoff = mullet_interfaces::action::Takeoff;
using Trigger = std_srvs::srv::Trigger;
using DroneHeartbeat = mullet_interfaces::msg::DroneHeartbeat;
using MissionCommand = mullet_interfaces::msg::MissionCommand;
using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
using VehicleCommand = px4_msgs::msg::VehicleCommand;
using VehicleLandDetected = px4_msgs::msg::VehicleLandDetected;
using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
using VehicleStatus = px4_msgs::msg::VehicleStatus;

// EKF origin the fake PX4 reports; heartbeat lat/lon/alt are derived from it.
constexpr double kRefLatDeg = 47.0;
constexpr double kRefLonDeg = 8.0;
constexpr float kRefAltM = 400.0F;
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

rclcpp::QoS px4QoS()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.transient_local();
  return qos;
}

rclcpp::QoS commandQoS()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

class FakePx4Peer
{
public:
  explicit FakePx4Peer(const rclcpp::Node::SharedPtr & node, const std::string & fmu_prefix = "")
  : node_(node)
  {
    // PX4's own names: versioned messages carry a _v<MESSAGE_VERSION> suffix.
    const auto local_position_topic = px4Topic(
      fmu_prefix, "/fmu/out/vehicle_local_position", VehicleLocalPosition::MESSAGE_VERSION);
    const auto vehicle_status_topic = px4Topic(
      fmu_prefix, "/fmu/out/vehicle_status", VehicleStatus::MESSAGE_VERSION);
    const auto vehicle_land_detected_topic = px4Topic(
      fmu_prefix, "/fmu/out/vehicle_land_detected", VehicleLandDetected::MESSAGE_VERSION);
    const auto vehicle_command_topic = px4Topic(
      fmu_prefix, "/fmu/in/vehicle_command", VehicleCommand::MESSAGE_VERSION);
    const auto trajectory_topic = px4Topic(
      fmu_prefix, "/fmu/in/trajectory_setpoint", TrajectorySetpoint::MESSAGE_VERSION);
    local_position_pub_ = node_->create_publisher<VehicleLocalPosition>(
      local_position_topic, px4QoS());
    vehicle_status_pub_ = node_->create_publisher<VehicleStatus>(
      vehicle_status_topic, px4QoS());
    vehicle_land_detected_pub_ = node_->create_publisher<VehicleLandDetected>(
      vehicle_land_detected_topic, px4QoS());
    vehicle_command_sub_ = node_->create_subscription<VehicleCommand>(
      vehicle_command_topic, px4QoS(),
      [this](const VehicleCommand::SharedPtr msg) {onVehicleCommand(msg);});
    trajectory_sub_ = node_->create_subscription<TrajectorySetpoint>(
      trajectory_topic, px4QoS(),
      [this](const TrajectorySetpoint::SharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        trajectory_setpoints_.push_back(*msg);
        if (std::isfinite(msg->position[0]) && std::isfinite(msg->position[1]) &&
          std::isfinite(msg->position[2]))
        {
          events_.push_back(Event{EventType::kPositionSetpoint, msg->position[2]});
        }
      });
    control_mode_sub_ = node_->create_subscription<OffboardControlMode>(
      fmu_prefix + "/fmu/in/offboard_control_mode", px4QoS(),
      [this](const OffboardControlMode::SharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        control_modes_.push_back(*msg);
      });
  }

  void publishReadyTelemetry(float z_ned_m = 0.0F, float heading_rad = 0.0F)
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      armed_ = false;
      offboard_ = false;
    }
    publishStatusFromSnapshot(false, false);
    publishLocalPosition(z_ned_m, heading_rad, 0.0F);
    VehicleLandDetected msg{};
    msg.landed = true;
    vehicle_land_detected_pub_->publish(msg);
  }

  void setArmed(bool armed)
  {
    bool offboard_snapshot = false;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      armed_ = armed;
      offboard_snapshot = offboard_;
    }
    publishStatusFromSnapshot(armed, offboard_snapshot);
  }

  void setOffboard(bool offboard)
  {
    bool armed_snapshot = false;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      offboard_ = offboard;
      armed_snapshot = armed_;
    }
    publishStatusFromSnapshot(armed_snapshot, offboard);
  }

  void publishLocalPosition(
    float z_ned_m, float heading_rad, float vz_ned_mps, float x_ned_m = 0.0F,
    float y_ned_m = 0.0F)
  {
    VehicleLocalPosition msg{};
    msg.xy_valid = true;
    msg.z_valid = true;
    msg.v_z_valid = true;
    msg.xy_global = true;
    msg.z_global = true;
    msg.ref_lat = kRefLatDeg;
    msg.ref_lon = kRefLonDeg;
    msg.ref_alt = kRefAltM;
    msg.x = x_ned_m;
    msg.y = y_ned_m;
    msg.z = z_ned_m;
    msg.vz = vz_ned_mps;
    msg.heading = heading_rad;
    local_position_pub_->publish(msg);
  }

  void publishInvalidLocalPosition()
  {
    VehicleLocalPosition msg{};
    msg.xy_valid = false;
    msg.z_valid = false;
    msg.v_z_valid = false;
    msg.heading = 0.0F;
    local_position_pub_->publish(msg);
  }

  void setAutoArm(bool enabled) {auto_arm_.store(enabled);}
  void setAutoOffboard(bool enabled) {auto_offboard_.store(enabled);}
  void setAutoDisarmOnLand(bool enabled) {auto_disarm_on_land_.store(enabled);}

  std::size_t commandCount(std::uint16_t command) const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<std::size_t>(
      std::count_if(vehicle_commands_.begin(), vehicle_commands_.end(),
      [command](const VehicleCommand & msg) {return msg.command == command;}));
  }

  std::size_t setModeCount(float main_mode, float sub_mode) const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<std::size_t>(
      std::count_if(vehicle_commands_.begin(), vehicle_commands_.end(),
      [main_mode, sub_mode](const VehicleCommand & msg) {
        return msg.command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
               std::fabs(msg.param1 - main_mode) < 1e-3F &&
               std::fabs(msg.param2 - sub_mode) < 1e-3F;
      }));
  }

  std::size_t velocitySetpointCount() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<std::size_t>(
      std::count_if(trajectory_setpoints_.begin(), trajectory_setpoints_.end(),
      [](const TrajectorySetpoint & msg) {
        return std::isfinite(msg.velocity[0]) && std::isfinite(msg.velocity[1]);
      }));
  }

  TrajectorySetpoint latestSetpoint() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return trajectory_setpoints_.empty() ? TrajectorySetpoint{} : trajectory_setpoints_.back();
  }

  // Whether any setpoint / control mode streamed so far satisfies `predicate`.
  bool sawSetpoint(const std::function<bool(const TrajectorySetpoint &)> & predicate) const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return std::any_of(trajectory_setpoints_.begin(), trajectory_setpoints_.end(), predicate);
  }

  bool sawControlMode(const std::function<bool(const OffboardControlMode &)> & predicate) const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return std::any_of(control_modes_.begin(), control_modes_.end(), predicate);
  }

  bool positionSetpointBeforeFirstArm(float z_ned_m) const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    bool saw_setpoint = false;
    for (const auto & event : events_) {
      if (event.type == EventType::kPositionSetpoint &&
        std::fabs(event.z_ned_m - z_ned_m) < 1e-3F)
      {
        saw_setpoint = true;
      }
      if (event.type == EventType::kArmCommand) {
        return saw_setpoint;
      }
    }
    return false;
  }

private:
  enum class EventType
  {
    kPositionSetpoint,
    kArmCommand,
  };

  struct Event
  {
    EventType type;
    float z_ned_m;
  };

  void onVehicleCommand(const VehicleCommand::SharedPtr msg)
  {
    bool should_publish_status = false;
    bool armed_snapshot = false;
    bool offboard_snapshot = false;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      vehicle_commands_.push_back(*msg);
      if (msg->command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM &&
        msg->param1 > 0.5F)
      {
        events_.push_back(Event{EventType::kArmCommand, 0.0F});
      }

      if (msg->command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
        std::fabs(msg->param2 - 6.0F) < 1e-3F && auto_offboard_.load())
      {
        offboard_ = true;
        should_publish_status = true;
      }
      if (msg->command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
        std::fabs(msg->param1 - 1.0F) < 1e-3F && std::fabs(msg->param2 - 3.0F) < 1e-3F)
      {
        offboard_ = false;
        should_publish_status = true;
      }
      if (msg->command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM &&
        msg->param1 > 0.5F && auto_arm_.load())
      {
        armed_ = true;
        should_publish_status = true;
      }
      if (msg->command == VehicleCommand::VEHICLE_CMD_NAV_LAND && auto_disarm_on_land_.load()) {
        armed_ = false;
        offboard_ = false;
        should_publish_status = true;
      }

      if (should_publish_status) {
        armed_snapshot = armed_;
        offboard_snapshot = offboard_;
      }
    }
    if (should_publish_status) {
      publishStatusFromSnapshot(armed_snapshot, offboard_snapshot);
    }
  }

  void publishStatusFromSnapshot(bool armed, bool offboard)
  {
    VehicleStatus msg{};
    msg.arming_state = armed ? VehicleStatus::ARMING_STATE_ARMED :
      VehicleStatus::ARMING_STATE_DISARMED;
    msg.nav_state = offboard ? VehicleStatus::NAVIGATION_STATE_OFFBOARD :
      VehicleStatus::NAVIGATION_STATE_POSCTL;
    vehicle_status_pub_->publish(msg);
  }

  rclcpp::Node::SharedPtr node_;
  bool armed_{false};
  bool offboard_{false};
  std::atomic_bool auto_arm_{true};
  std::atomic_bool auto_offboard_{true};
  std::atomic_bool auto_disarm_on_land_{false};
  mutable std::mutex mutex_{};
  std::vector<VehicleCommand> vehicle_commands_{};
  std::vector<TrajectorySetpoint> trajectory_setpoints_{};
  std::vector<OffboardControlMode> control_modes_{};
  std::vector<Event> events_{};
  rclcpp::Publisher<VehicleLocalPosition>::SharedPtr local_position_pub_;
  rclcpp::Publisher<VehicleStatus>::SharedPtr vehicle_status_pub_;
  rclcpp::Publisher<VehicleLandDetected>::SharedPtr vehicle_land_detected_pub_;
  rclcpp::Subscription<VehicleCommand>::SharedPtr vehicle_command_sub_;
  rclcpp::Subscription<TrajectorySetpoint>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<OffboardControlMode>::SharedPtr control_mode_sub_;
};

class DroneStateMachineBehavior : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    start("", {});
  }

  // Builds the node under test, a fake PX4 peer on `fmu_prefix`, and the clients.
  void start(const std::string & fmu_prefix, const std::vector<rclcpp::Parameter> & extra_parameters)
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    fake_node_ = std::make_shared<rclcpp::Node>("fake_px4_peer");
    client_node_ = std::make_shared<rclcpp::Node>("mullet_core_client");
    fake_px4_ = std::make_unique<FakePx4Peer>(fake_node_, fmu_prefix);

    std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("control_rate_hz", 50.0),
      rclcpp::Parameter("offboard_warmup_s", 1.0),
      rclcpp::Parameter("command_retry_interval_s", 0.1),
      rclcpp::Parameter("command_timeout_s", 0.25),
      rclcpp::Parameter("takeoff_acceptance_m", 0.05),
      rclcpp::Parameter("takeoff_yaw_tolerance_rad", 0.05),
      rclcpp::Parameter("request_offboard_timeout_s", 0.3),
      rclcpp::Parameter("request_arm_timeout_s", 0.3),
      rclcpp::Parameter("takeoff_timeout_s", 0.3),
      rclcpp::Parameter("landing_timeout_s", 0.3),
      rclcpp::Parameter("hover_altitude_tolerance_m", 0.05),
      rclcpp::Parameter("hover_vertical_velocity_tolerance_mps", 0.05),
      rclcpp::Parameter("hover_settle_time_s", 0.0),
      rclcpp::Parameter("return_home_speed_mps", 5.0),
      rclcpp::Parameter("return_home_acceptance_m", 0.5),
    };
    parameters.insert(parameters.end(), extra_parameters.begin(), extra_parameters.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    state_machine_ = std::make_shared<DroneStateMachine>(options);
    command_pub_ = client_node_->create_publisher<MissionCommand>("command", commandQoS());
    takeoff_client_ = rclcpp_action::create_client<Takeoff>(client_node_, "takeoff");
    land_client_ = rclcpp_action::create_client<Land>(client_node_, "land");
    return_home_client_ = rclcpp_action::create_client<ReturnHome>(client_node_, "return_home");
    start_mission_client_ = client_node_->create_client<Trigger>("start_mission");
    hold_client_ = client_node_->create_client<Trigger>("hold");
    heartbeat_sub_ = client_node_->create_subscription<DroneHeartbeat>(
      "/mullet/heartbeat", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      [this](const DroneHeartbeat::SharedPtr msg) {
        const std::lock_guard<std::mutex> lock(heartbeat_mutex_);
        heartbeats_.push_back(*msg);
      });

    executor_->add_node(fake_node_);
    executor_->add_node(client_node_);
    executor_->add_node(state_machine_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    ASSERT_TRUE(takeoff_client_->wait_for_action_server(2s));
    ASSERT_TRUE(land_client_->wait_for_action_server(2s));
    ASSERT_TRUE(return_home_client_->wait_for_action_server(2s));
    ASSERT_TRUE(start_mission_client_->wait_for_service(2s));
    ASSERT_TRUE(hold_client_->wait_for_service(2s));
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    takeoff_client_.reset();
    land_client_.reset();
    return_home_client_.reset();
    start_mission_client_.reset();
    hold_client_.reset();
    command_pub_.reset();
    state_machine_.reset();
    fake_px4_.reset();
    fake_node_.reset();
    client_node_.reset();
    executor_.reset();
  }

  template<typename PredicateT>
  bool waitFor(PredicateT predicate, std::chrono::milliseconds timeout = 3s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return predicate();
  }

  bool waitForState(const std::string & label, std::chrono::milliseconds timeout = 3s)
  {
    return waitFor([this, &label]() {return latestHeartbeat().state_label == label;}, timeout);
  }

  void publishReadyAndWait()
  {
    for (int i = 0; i < 5; ++i) {
      fake_px4_->publishReadyTelemetry();
      std::this_thread::sleep_for(50ms);
    }
  }

  std::vector<DroneHeartbeat> heartbeats() const
  {
    const std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    return heartbeats_;
  }

  DroneHeartbeat latestHeartbeat() const
  {
    const std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    return heartbeats_.empty() ? DroneHeartbeat{} : heartbeats_.back();
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<Takeoff>> sendTakeoffGoal(
    float altitude_m = 2.0F, bool use_current_yaw = true, float yaw_rad = 0.0F)
  {
    Takeoff::Goal goal{};
    goal.altitude_m = altitude_m;
    goal.use_current_yaw = use_current_yaw;
    goal.yaw_rad = yaw_rad;
    auto future = takeoff_client_->async_send_goal(goal);
    if (future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<Land>> sendLandGoal()
  {
    auto future = land_client_->async_send_goal(Land::Goal{});
    if (future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<ReturnHome>> sendReturnHomeGoal()
  {
    auto future = return_home_client_->async_send_goal(ReturnHome::Goal{});
    if (future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<Trigger::Response> callTrigger(const rclcpp::Client<Trigger>::SharedPtr & client)
  {
    auto request = client->async_send_request(std::make_shared<Trigger::Request>());
    if (request.future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return request.future.get();
  }

  template<typename ActionT>
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult waitResult(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>> & handle)
  {
    auto future = client->async_get_result(handle);
    EXPECT_EQ(future.wait_for(4s), std::future_status::ready);
    return future.get();
  }

  rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult waitTakeoffResult(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<Takeoff>> & handle)
  {
    return waitResult<Takeoff>(takeoff_client_, handle);
  }

  rclcpp_action::ClientGoalHandle<Land>::WrappedResult waitLandResult(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<Land>> & handle)
  {
    return waitResult<Land>(land_client_, handle);
  }

  rclcpp_action::ClientGoalHandle<ReturnHome>::WrappedResult waitReturnHomeResult(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<ReturnHome>> & handle)
  {
    return waitResult<ReturnHome>(return_home_client_, handle);
  }

  void completeTakeoff(const std::shared_ptr<rclcpp_action::ClientGoalHandle<Takeoff>> & handle)
  {
    ASSERT_TRUE(waitFor([this]() {
      return fake_px4_->setModeCount(1.0F, 6.0F) > 0 &&
             fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) > 0;
    }, 3s));
    fake_px4_->publishLocalPosition(-2.0F, 0.0F, 0.0F);
    auto result = waitTakeoffResult(handle);
    ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
    ASSERT_TRUE(result.result->success);
    ASSERT_EQ(result.result->message, "Takeoff complete");
  }

  void enterHold()
  {
    publishReadyAndWait();
    auto takeoff = sendTakeoffGoal();
    ASSERT_NE(takeoff, nullptr);
    completeTakeoff(takeoff);
    ASSERT_TRUE(waitForState("HOLD"));
  }

  void enterMission()
  {
    enterHold();
    auto response = callTrigger(start_mission_client_);
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success) << response->message;
    ASSERT_TRUE(waitForState("MISSION"));
  }

  void publishCommand(
    std::uint8_t mode, float vx_ned_mps, float vy_ned_mps, float ax_ned_mps2, float ay_ned_mps2,
    float vz_ned_mps = kNan, float az_ned_mps2 = kNan)
  {
    MissionCommand command{};
    command.mode = mode;
    command.velocity_ned_mps = {vx_ned_mps, vy_ned_mps, vz_ned_mps};
    command.acceleration_ned_mps2 = {ax_ned_mps2, ay_ned_mps2, az_ned_mps2};
    command.yaw_ned_rad = kNan;
    command_pub_->publish(command);
  }

  void publishVelocityCommand(float vx_ned_mps, float vy_ned_mps)
  {
    publishCommand(MissionCommand::VELOCITY, vx_ned_mps, vy_ned_mps, kNan, kNan);
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  rclcpp::Node::SharedPtr fake_node_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<DroneStateMachine> state_machine_;
  std::unique_ptr<FakePx4Peer> fake_px4_;
  rclcpp_action::Client<Takeoff>::SharedPtr takeoff_client_;
  rclcpp_action::Client<Land>::SharedPtr land_client_;
  rclcpp_action::Client<ReturnHome>::SharedPtr return_home_client_;
  rclcpp::Client<Trigger>::SharedPtr start_mission_client_;
  rclcpp::Client<Trigger>::SharedPtr hold_client_;
  rclcpp::Publisher<MissionCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<DroneHeartbeat>::SharedPtr heartbeat_sub_;
  mutable std::mutex heartbeat_mutex_;
  std::vector<DroneHeartbeat> heartbeats_;
  std::thread spin_thread_;
};

// --- Heartbeat ---------------------------------------------------------------------------

TEST_F(DroneStateMachineBehavior, HeartbeatPublishesBeforePx4TelemetryReady)
{
  ASSERT_TRUE(waitFor([this]() {return !heartbeats().empty();}, 2s));

  const auto heartbeat = latestHeartbeat();
  EXPECT_EQ(heartbeat.schema_version, 3U);
  EXPECT_EQ(heartbeat.drone_id, "uav_0");
  EXPECT_EQ(heartbeat.vehicle_instance, 0U);
  EXPECT_EQ(heartbeat.target_system, 1U);
  EXPECT_EQ(heartbeat.ros_namespace, "/");
  EXPECT_EQ(heartbeat.takeoff_action_path, "/takeoff");
  EXPECT_EQ(heartbeat.land_action_path, "/land");
  EXPECT_EQ(heartbeat.return_home_action_path, "/return_home");
  EXPECT_EQ(heartbeat.start_mission_service_path, "/start_mission");
  EXPECT_EQ(heartbeat.hold_service_path, "/hold");
  EXPECT_FALSE(heartbeat.has_vehicle_status);
  EXPECT_FALSE(heartbeat.has_local_position);
  EXPECT_FALSE(heartbeat.px4_failsafe);
  EXPECT_FALSE(heartbeat.local_position_valid);
  EXPECT_FALSE(heartbeat.global_position_valid);
  EXPECT_EQ(heartbeat.state_label, "IDLE");
  EXPECT_EQ(heartbeat.active_action, "");
}

TEST_F(DroneStateMachineBehavior, HeartbeatSequenceIncrementsMonotonically)
{
  ASSERT_TRUE(waitFor([this]() {return heartbeats().size() >= 3U;}, 2s));

  const auto samples = heartbeats();
  for (std::size_t i = 1; i < samples.size(); ++i) {
    EXPECT_GT(samples[i].sequence, samples[i - 1].sequence);
  }
}

TEST_F(DroneStateMachineBehavior, HeartbeatInvalidLocalPositionDoesNotImplyPose)
{
  publishReadyAndWait();
  fake_px4_->publishInvalidLocalPosition();

  ASSERT_TRUE(waitFor([this]() {
    const auto heartbeat = latestHeartbeat();
    return heartbeat.has_local_position && !heartbeat.local_position_valid;
  }, 2s));

  const auto heartbeat = latestHeartbeat();
  EXPECT_DOUBLE_EQ(heartbeat.x, 0.0);
  EXPECT_DOUBLE_EQ(heartbeat.y, 0.0);
  EXPECT_DOUBLE_EQ(heartbeat.z, 0.0);
  EXPECT_DOUBLE_EQ(heartbeat.yaw, 0.0);
  EXPECT_FALSE(heartbeat.global_position_valid);
  EXPECT_DOUBLE_EQ(heartbeat.lat, 0.0);
}

TEST_F(DroneStateMachineBehavior, HeartbeatReportsGlobalPositionFromEkfOrigin)
{
  publishReadyAndWait();
  // 111.19492664 m north of the origin is 0.001 degrees of latitude; z is down.
  fake_px4_->publishLocalPosition(-10.0F, 0.0F, 0.0F, 111.19492664F, 0.0F);

  ASSERT_TRUE(waitFor([this]() {
    const auto heartbeat = latestHeartbeat();
    return heartbeat.global_position_valid && heartbeat.x > 100.0;
  }, 2s));

  const auto heartbeat = latestHeartbeat();
  EXPECT_NEAR(heartbeat.lat, kRefLatDeg + 0.001, 1e-6);
  EXPECT_NEAR(heartbeat.lon, kRefLonDeg, 1e-9);
  EXPECT_NEAR(heartbeat.alt, kRefAltM + 10.0, 1e-3);
}

TEST_F(DroneStateMachineBehavior, HeartbeatCapabilitiesFollowTelemetry)
{
  ASSERT_TRUE(waitFor([this]() {return !heartbeats().empty();}, 2s));
  EXPECT_FALSE(latestHeartbeat().can_takeoff);
  EXPECT_FALSE(latestHeartbeat().can_land);

  publishReadyAndWait();

  ASSERT_TRUE(waitFor([this]() {return latestHeartbeat().can_takeoff;}, 2s));
  const auto heartbeat = latestHeartbeat();
  EXPECT_EQ(heartbeat.state_label, "IDLE");
  EXPECT_FALSE(heartbeat.can_land);
  EXPECT_FALSE(heartbeat.can_return_home);
  EXPECT_FALSE(heartbeat.can_start_mission);
  EXPECT_FALSE(heartbeat.can_hold);
}

// --- Takeoff -----------------------------------------------------------------------------

TEST_F(DroneStateMachineBehavior, TakeoffRejectedBeforePx4TelemetryReady)
{
  EXPECT_EQ(sendTakeoffGoal(), nullptr);
}

TEST_F(DroneStateMachineBehavior, TakeoffHappyPathStreamsSetpointThenArmsAndHolds)
{
  publishReadyAndWait();

  auto handle = sendTakeoffGoal();
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([this]() {
    const auto heartbeat = latestHeartbeat();
    return heartbeat.state_label == "TRANSITION" && heartbeat.active_action == "takeoff";
  }, 2s));
  completeTakeoff(handle);

  EXPECT_TRUE(fake_px4_->positionSetpointBeforeFirstArm(-2.0F));
  EXPECT_GT(fake_px4_->setModeCount(1.0F, 6.0F), 0U);
  ASSERT_TRUE(waitForState("HOLD"));
  const auto heartbeat = latestHeartbeat();
  EXPECT_EQ(heartbeat.previous_state_label, "TRANSITION");
  EXPECT_EQ(heartbeat.active_action, "");
  EXPECT_TRUE(heartbeat.can_land);
  EXPECT_TRUE(heartbeat.can_return_home);
  EXPECT_TRUE(heartbeat.can_start_mission);
  EXPECT_FALSE(heartbeat.can_takeoff);
}

TEST_F(DroneStateMachineBehavior, DuplicateTakeoffGoalWhileActiveIsRejected)
{
  publishReadyAndWait();

  auto first = sendTakeoffGoal();
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(sendTakeoffGoal(), nullptr);

  auto cancel_future = takeoff_client_->async_cancel_goal(first);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto result = waitTakeoffResult(first);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(result.result->message, "Takeoff canceled");
}

TEST_F(DroneStateMachineBehavior, CancelBeforeArmingReturnsToIdleWithoutLanding)
{
  publishReadyAndWait();
  fake_px4_->setAutoOffboard(false);
  fake_px4_->setAutoArm(false);

  auto handle = sendTakeoffGoal();
  ASSERT_NE(handle, nullptr);
  auto cancel_future = takeoff_client_->async_cancel_goal(handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto result = waitTakeoffResult(handle);

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_TRUE(waitForState("IDLE"));
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
}

TEST_F(DroneStateMachineBehavior, CancelAfterArmRequestResolvesGoalAndLands)
{
  publishReadyAndWait();
  fake_px4_->setAutoArm(false);

  auto handle = sendTakeoffGoal();
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) > 0;
  }, 3s));

  auto cancel_future = takeoff_client_->async_cancel_goal(handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto result = waitTakeoffResult(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > 0;
  }, 2s));
  // PX4 never armed, so landing recovery ends as soon as it sees the vehicle disarmed.
  EXPECT_TRUE(waitForState("IDLE"));
}

TEST_F(DroneStateMachineBehavior, LandActionPreemptsActivePreArmTakeoff)
{
  publishReadyAndWait();
  fake_px4_->setAutoOffboard(false);
  fake_px4_->setAutoArm(false);

  auto takeoff = sendTakeoffGoal();
  ASSERT_NE(takeoff, nullptr);
  auto land = sendLandGoal();
  ASSERT_NE(land, nullptr);

  auto takeoff_result = waitTakeoffResult(takeoff);
  auto land_result = waitLandResult(land);

  EXPECT_EQ(takeoff_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(takeoff_result.result->message, "Takeoff preempted by land action");
  EXPECT_EQ(land_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(
    land_result.result->message,
    "Takeoff canceled before arming; vehicle already disarmed");
  EXPECT_TRUE(waitForState("IDLE"));
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
}

TEST_F(DroneStateMachineBehavior, TakeoffTimeoutAbortsActionAndLands)
{
  publishReadyAndWait();
  fake_px4_->setAutoArm(false);

  auto takeoff = sendTakeoffGoal();
  ASSERT_NE(takeoff, nullptr);
  ASSERT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) > 0;
  }, 3s));

  auto result = waitTakeoffResult(takeoff);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->message, "arm request timed out");
  EXPECT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > 0;
  }, 2s));
}

// --- Mission, hold and return home -------------------------------------------------------

TEST_F(DroneStateMachineBehavior, StartMissionAndHoldRejectedOutsideTheirStates)
{
  publishReadyAndWait();

  auto start = callTrigger(start_mission_client_);
  ASSERT_NE(start, nullptr);
  EXPECT_FALSE(start->success);
  EXPECT_EQ(start->message, "state is not HOLD");

  auto hold = callTrigger(hold_client_);
  ASSERT_NE(hold, nullptr);
  EXPECT_FALSE(hold->success);
  EXPECT_EQ(sendReturnHomeGoal(), nullptr);
  EXPECT_EQ(latestHeartbeat().state_label, "IDLE");
}

TEST_F(DroneStateMachineBehavior, MissionFollowsCommandsAndHoldStopsIt)
{
  enterMission();

  publishVelocityCommand(0.2F, 0.0F);
  ASSERT_TRUE(waitFor([this]() {return fake_px4_->velocitySetpointCount() > 0;}));
  EXPECT_TRUE(latestHeartbeat().can_hold);

  auto hold = callTrigger(hold_client_);
  ASSERT_NE(hold, nullptr);
  EXPECT_TRUE(hold->success);
  ASSERT_TRUE(waitForState("HOLD"));
  EXPECT_TRUE(waitFor([this]() {
    const auto setpoint = fake_px4_->latestSetpoint();
    return std::isfinite(setpoint.position[0]) && std::isfinite(setpoint.position[2]);
  }));
}

TEST_F(DroneStateMachineBehavior, StaleStreamInMissionHoldsPositionWithoutLeavingMission)
{
  enterMission();

  publishVelocityCommand(0.2F, 0.0F);
  ASSERT_TRUE(waitFor([this]() {return latestHeartbeat().command_stream_valid;}));
  ASSERT_TRUE(waitFor([this]() {return !latestHeartbeat().command_stream_valid;}));

  EXPECT_EQ(latestHeartbeat().state_label, "MISSION");
  EXPECT_TRUE(waitFor([this]() {
    const auto setpoint = fake_px4_->latestSetpoint();
    return std::isfinite(setpoint.position[0]) && std::isfinite(setpoint.position[2]);
  }));
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
}

TEST_F(DroneStateMachineBehavior, VelocityModeHoldsAltitudeAndForwardsFeedforward)
{
  enterMission();

  publishCommand(MissionCommand::VELOCITY, 0.2F, -0.1F, 0.5F, 0.0F);
  // Horizontal velocity with acceleration feedforward, altitude held at the takeoff target.
  EXPECT_TRUE(waitFor([this]() {
    return fake_px4_->sawSetpoint([](const TrajectorySetpoint & setpoint) {
      return std::isnan(setpoint.position[0]) && std::isnan(setpoint.position[1]) &&
             std::fabs(setpoint.position[2] + 2.0F) < 1e-3F &&
             std::fabs(setpoint.velocity[0] - 0.2F) < 1e-6F &&
             std::fabs(setpoint.velocity[1] + 0.1F) < 1e-6F && std::isnan(setpoint.velocity[2]) &&
             std::fabs(setpoint.acceleration[0] - 0.5F) < 1e-6F;
    });
  }));
  EXPECT_TRUE(waitFor([this]() {
    return fake_px4_->sawControlMode([](const OffboardControlMode & mode) {
      return mode.position && mode.velocity && mode.acceleration;
    });
  }));
}

TEST_F(DroneStateMachineBehavior, AccelerationModeStreamsClampedAccelerationOnly)
{
  enterMission();

  publishCommand(MissionCommand::ACCELERATION, kNan, kNan, 10.0F, -0.5F);
  // max_accel_ned_mps2 defaults to 3.0; no velocity loop, altitude still held.
  EXPECT_TRUE(waitFor([this]() {
    return fake_px4_->sawSetpoint([](const TrajectorySetpoint & setpoint) {
      return std::isnan(setpoint.velocity[0]) && std::isnan(setpoint.velocity[1]) &&
             std::fabs(setpoint.position[2] + 2.0F) < 1e-3F &&
             std::fabs(setpoint.acceleration[0] - 3.0F) < 1e-6F &&
             std::fabs(setpoint.acceleration[1] + 0.5F) < 1e-6F;
    });
  }));
  EXPECT_TRUE(waitFor([this]() {
    return fake_px4_->sawControlMode([](const OffboardControlMode & mode) {
      return mode.position && !mode.velocity && mode.acceleration;
    });
  }));
}

TEST_F(DroneStateMachineBehavior, Velocity3dModeStreamsClampedZWithoutHoldingAltitude)
{
  enterMission();

  publishCommand(MissionCommand::VELOCITY_3D, 0.2F, -0.1F, kNan, kNan, -2.0F);
  // max_vz_ned_mps defaults to 0.5; no position loop on any axis.
  EXPECT_TRUE(waitFor([this]() {
    return fake_px4_->sawSetpoint([](const TrajectorySetpoint & setpoint) {
      return std::isnan(setpoint.position[0]) && std::isnan(setpoint.position[2]) &&
             std::fabs(setpoint.velocity[0] - 0.2F) < 1e-6F &&
             std::fabs(setpoint.velocity[2] + 0.5F) < 1e-6F && std::isnan(setpoint.acceleration[2]);
    });
  }));
  EXPECT_TRUE(waitFor([this]() {
    return fake_px4_->sawControlMode([](const OffboardControlMode & mode) {
      return !mode.position && mode.velocity && !mode.acceleration;
    });
  }));
}

TEST_F(DroneStateMachineBehavior, Acceleration3dModeStreamsClampedAccelerationOnAllAxes)
{
  enterMission();

  publishCommand(MissionCommand::ACCELERATION_3D, kNan, kNan, 0.5F, 0.0F, kNan, -10.0F);
  EXPECT_TRUE(waitFor([this]() {
    return fake_px4_->sawSetpoint([](const TrajectorySetpoint & setpoint) {
      return std::isnan(setpoint.position[2]) && std::isnan(setpoint.velocity[2]) &&
             std::fabs(setpoint.acceleration[0] - 0.5F) < 1e-6F &&
             std::fabs(setpoint.acceleration[2] + 3.0F) < 1e-6F;
    });
  }));
}

TEST_F(DroneStateMachineBehavior, Stale3dStreamHoldsTheCurrentAltitude)
{
  enterMission();

  // The drone climbs from the 2 m takeoff to 5 m while the 3D stream is fresh.
  publishCommand(MissionCommand::VELOCITY_3D, 0.0F, 0.0F, kNan, kNan, -0.3F);
  ASSERT_TRUE(waitFor([this]() {return latestHeartbeat().command_stream_valid;}));
  fake_px4_->publishLocalPosition(-5.0F, 0.0F, 0.0F);
  std::this_thread::sleep_for(100ms);
  publishCommand(MissionCommand::VELOCITY_3D, 0.0F, 0.0F, kNan, kNan, -0.3F);
  ASSERT_TRUE(waitFor([this]() {return !latestHeartbeat().command_stream_valid;}));

  EXPECT_TRUE(waitFor([this]() {
    return std::fabs(fake_px4_->latestSetpoint().position[2] + 5.0F) < 1e-3F;
  }));
  EXPECT_EQ(latestHeartbeat().state_label, "MISSION");
}

TEST_F(DroneStateMachineBehavior, MalformedMissionCommandsAreNeverLatched)
{
  enterMission();

  const auto publish_and_settle = [this](std::uint8_t mode, float vx, float ax, float ay, float vz) {
      publishCommand(mode, vx, 0.0F, ax, ay, vz);
      std::this_thread::sleep_for(60ms);
    };
  publish_and_settle(0U, 0.2F, kNan, kNan, kNan);                        // mode unset
  publish_and_settle(MissionCommand::VELOCITY, 0.2F, kNan, kNan, -0.5F);  // z commanded
  publish_and_settle(MissionCommand::VELOCITY, kNan, kNan, kNan, kNan);   // no velocity
  publish_and_settle(MissionCommand::VELOCITY, 0.2F, 0.5F, kNan, kNan);   // half feedforward
  publish_and_settle(MissionCommand::ACCELERATION, 0.2F, kNan, kNan, kNan);  // no acceleration
  publish_and_settle(MissionCommand::VELOCITY_3D, 0.2F, kNan, kNan, kNan);   // no vz
  publish_and_settle(MissionCommand::ACCELERATION_3D, kNan, 0.5F, 0.1F, kNan);  // no az

  EXPECT_FALSE(latestHeartbeat().has_command_stream);
  EXPECT_EQ(fake_px4_->velocitySetpointCount(), 0U);
  EXPECT_EQ(latestHeartbeat().state_label, "MISSION");
}

TEST_F(DroneStateMachineBehavior, ReturnHomeFliesBackThenHolds)
{
  enterHold();
  fake_px4_->publishLocalPosition(-2.0F, 0.0F, 0.0F, 10.0F, 0.0F);
  ASSERT_TRUE(waitFor([this]() {return latestHeartbeat().x > 9.0;}));

  auto handle = sendReturnHomeGoal();
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([this]() {
    const auto heartbeat = latestHeartbeat();
    return heartbeat.state_label == "TRANSITION" && heartbeat.active_action == "return_home";
  }, 2s));
  // The setpoint slides from the vehicle toward home instead of jumping there.
  ASSERT_TRUE(waitFor([this]() {
    const auto x = fake_px4_->latestSetpoint().position[0];
    return x < 9.5F && x > 0.5F;
  }, 2s));

  fake_px4_->publishLocalPosition(-2.0F, 0.0F, 0.0F, 0.1F, 0.0F);
  auto result = waitReturnHomeResult(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result.result->message, "Arrived home");
  EXPECT_TRUE(waitForState("HOLD"));
}

TEST_F(DroneStateMachineBehavior, HoldDuringReturnHomeStopsInPlace)
{
  enterHold();
  fake_px4_->publishLocalPosition(-2.0F, 0.0F, 0.0F, 10.0F, 0.0F);
  ASSERT_TRUE(waitFor([this]() {return latestHeartbeat().x > 9.0;}));

  auto handle = sendReturnHomeGoal();
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([this]() {return latestHeartbeat().can_hold;}, 2s));

  auto hold = callTrigger(hold_client_);
  ASSERT_NE(hold, nullptr);
  EXPECT_TRUE(hold->success);
  auto result = waitReturnHomeResult(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->message, "Return home stopped by hold");
  EXPECT_TRUE(waitForState("HOLD"));
}

// --- PX4 overrides -----------------------------------------------------------------------

TEST_F(DroneStateMachineBehavior, ExternalDisarmInHoldReturnsToIdle)
{
  enterHold();

  fake_px4_->setArmed(false);

  EXPECT_TRUE(waitForState("IDLE"));
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
}

TEST_F(DroneStateMachineBehavior, OffboardLossInMissionDropsToHoldWithoutLanding)
{
  enterMission();

  fake_px4_->setOffboard(false);

  ASSERT_TRUE(waitForState("HOLD"));
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
  EXPECT_TRUE(waitFor([this]() {
    const auto heartbeat = latestHeartbeat();
    return !heartbeat.offboard && !heartbeat.can_start_mission && heartbeat.can_land;
  }));
}

// --- Land --------------------------------------------------------------------------------

TEST_F(DroneStateMachineBehavior, LandFromMissionSendsNavLandAndSucceedsAfterDisarm)
{
  enterMission();
  publishVelocityCommand(0.2F, 0.0F);
  ASSERT_TRUE(waitFor([this]() {return fake_px4_->velocitySetpointCount() > 0;}));

  fake_px4_->setAutoDisarmOnLand(true);
  auto land = sendLandGoal();
  ASSERT_NE(land, nullptr);
  auto result = waitLandResult(land);

  EXPECT_GT(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result.result->message, "Vehicle disarmed after landing");
  EXPECT_TRUE(waitForState("IDLE"));
}

TEST_F(DroneStateMachineBehavior, LandGoalWhileAlreadyLandingIsRejected)
{
  enterHold();

  auto first_land = sendLandGoal();
  ASSERT_NE(first_land, nullptr);
  ASSERT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > 0;
  }));

  EXPECT_EQ(sendLandGoal(), nullptr);
  EXPECT_TRUE(waitFor([this]() {return !latestHeartbeat().can_land;}));

  fake_px4_->setAutoDisarmOnLand(true);
  auto result = waitLandResult(first_land);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(DroneStateMachineBehavior, LandCancellationRemainsRejectedAfterCommitment)
{
  enterHold();

  auto land = sendLandGoal();
  ASSERT_NE(land, nullptr);
  ASSERT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > 0;
  }));
  auto cancel_future = land_client_->async_cancel_goal(land);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(cancel_future.get()->goals_canceling.empty());

  fake_px4_->setAutoDisarmOnLand(true);
  EXPECT_EQ(waitLandResult(land).code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(DroneStateMachineBehavior, LandingTimeoutAbortsActionButKeepsRetryingUntilDisarm)
{
  enterHold();

  auto land = sendLandGoal();
  ASSERT_NE(land, nullptr);
  ASSERT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > 0;
  }, 2s));

  auto land_result = waitLandResult(land);
  EXPECT_EQ(land_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(land_result.result->message, "Landing timed out while vehicle is still armed");

  const auto before_retry_count = fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND);
  EXPECT_TRUE(waitFor([this, before_retry_count]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > before_retry_count;
  }, 2s));

  fake_px4_->setAutoDisarmOnLand(true);
  EXPECT_TRUE(waitForState("IDLE"));
}

// --- Namespacing -------------------------------------------------------------------------

class DroneStateMachineBehaviorPrefixedTopics : public DroneStateMachineBehavior
{
protected:
  void SetUp() override
  {
    start(
      "/uav_2",
      {rclcpp::Parameter("vehicle_instance", 2), rclcpp::Parameter("px4_namespace_prefix", "uav")});
  }
};

TEST_F(
  DroneStateMachineBehaviorPrefixedTopics,
  NonZeroVehicleInstanceUsesConfiguredNamespacePrefixForPx4Topics)
{
  publishReadyAndWait();

  auto handle = sendTakeoffGoal();
  ASSERT_NE(handle, nullptr);
  completeTakeoff(handle);

  const auto heartbeat = latestHeartbeat();
  EXPECT_EQ(heartbeat.px4_fmu_prefix, "/uav_2");
  EXPECT_EQ(heartbeat.target_system, 3U);
}

}  // namespace
}  // namespace mullet_core
