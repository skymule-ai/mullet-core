#include "mullet_core/drone_state_machine.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <mullet_interfaces/msg/drone_heartbeat.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace mullet_core
{
namespace
{

using namespace std::chrono_literals;

using DisableOffboard = mullet_interfaces::action::DisableOffboard;
using Arm = mullet_interfaces::action::Arm;
using Disarm = mullet_interfaces::action::Disarm;
using EnableOffboard = mullet_interfaces::action::EnableOffboard;
using Land = mullet_interfaces::action::Land;
using Takeoff = mullet_interfaces::action::Takeoff;
using DroneHeartbeat = mullet_interfaces::msg::DroneHeartbeat;
using VehicleCommand = px4_msgs::msg::VehicleCommand;
using VehicleLandDetected = px4_msgs::msg::VehicleLandDetected;
using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
using VehicleStatus = px4_msgs::msg::VehicleStatus;

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
    const auto local_position_topic = fmu_prefix + "/fmu/out/vehicle_local_position";
    const auto vehicle_status_topic = fmu_prefix + "/fmu/out/vehicle_status_v1";
    const auto vehicle_land_detected_topic = fmu_prefix + "/fmu/out/vehicle_land_detected";
    const auto vehicle_command_topic = fmu_prefix + "/fmu/in/vehicle_command";
    const auto trajectory_topic = fmu_prefix + "/fmu/in/trajectory_setpoint";
    const auto offboard_mode_topic = fmu_prefix + "/fmu/in/offboard_control_mode";
    local_position_pub_ = node_->create_publisher<VehicleLocalPosition>(
      local_position_topic, px4QoS());
    vehicle_status_pub_ = node_->create_publisher<VehicleStatus>(
      vehicle_status_topic, px4QoS());
    vehicle_land_detected_pub_ = node_->create_publisher<VehicleLandDetected>(
      vehicle_land_detected_topic, px4QoS());
    vehicle_command_sub_ = node_->create_subscription<VehicleCommand>(
      vehicle_command_topic, px4QoS(),
      [this](const VehicleCommand::SharedPtr msg) {onVehicleCommand(msg);});
    trajectory_sub_ = node_->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      trajectory_topic, px4QoS(),
      [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        trajectory_setpoints_.push_back(*msg);
        if (std::isfinite(msg->position[0]) && std::isfinite(msg->position[1]) &&
          std::isfinite(msg->position[2]))
        {
          events_.push_back(Event{EventType::kPositionSetpoint, msg->position[2]});
        }
      });
    offboard_mode_sub_ = node_->create_subscription<px4_msgs::msg::OffboardControlMode>(
      offboard_mode_topic, px4QoS(),
      [this](const px4_msgs::msg::OffboardControlMode::SharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        offboard_modes_.push_back(*msg);
      });
  }

  void publishReadyTelemetry(float z_ned_m = 0.0F, float heading_rad = 0.0F)
  {
    bool armed_snapshot = false;
    bool offboard_snapshot = false;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      armed_ = false;
      offboard_ = false;
      armed_snapshot = armed_;
      offboard_snapshot = offboard_;
    }
    publishStatusFromSnapshot(armed_snapshot, offboard_snapshot);
    publishLocalPosition(z_ned_m, heading_rad, 0.0F);
    publishLandDetected(true);
  }

  void setArmed(bool armed)
  {
    bool armed_snapshot = false;
    bool offboard_snapshot = false;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      armed_ = armed;
      armed_snapshot = armed_;
      offboard_snapshot = offboard_;
    }
    publishStatusFromSnapshot(armed_snapshot, offboard_snapshot);
  }

  void setOffboard(bool offboard)
  {
    bool armed_snapshot = false;
    bool offboard_snapshot = false;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      offboard_ = offboard;
      armed_snapshot = armed_;
      offboard_snapshot = offboard_;
    }
    publishStatusFromSnapshot(armed_snapshot, offboard_snapshot);
  }

  void publishLocalPosition(float z_ned_m, float heading_rad, float vz_ned_mps)
  {
    VehicleLocalPosition msg{};
    msg.xy_valid = true;
    msg.z_valid = true;
    msg.v_z_valid = true;
    msg.x = 0.0F;
    msg.y = 0.0F;
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

  void publishLandDetected(bool landed)
  {
    VehicleLandDetected msg{};
    msg.landed = landed;
    vehicle_land_detected_pub_->publish(msg);
  }

  void publishStatus()
  {
    bool armed_snapshot = false;
    bool offboard_snapshot = false;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      armed_snapshot = armed_;
      offboard_snapshot = offboard_;
    }
    publishStatusFromSnapshot(armed_snapshot, offboard_snapshot);
  }

  void setAutoArm(bool enabled) {auto_arm_.store(enabled);}
  void setAutoOffboard(bool enabled) {auto_offboard_.store(enabled);}
  void setAutoPositionMode(bool enabled) {auto_position_mode_.store(enabled);}
  void setAutoDisarmOnLand(bool enabled) {auto_disarm_on_land_.store(enabled);}
  void setAutoDisarm(bool enabled) {auto_disarm_.store(enabled);}

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
      [](const px4_msgs::msg::TrajectorySetpoint & msg) {
        return std::isfinite(msg.velocity[0]) && std::isfinite(msg.velocity[1]);
      }));
  }

  std::size_t positionSetpointCountAtZ(float z_ned_m) const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<std::size_t>(
      std::count_if(trajectory_setpoints_.begin(), trajectory_setpoints_.end(),
      [z_ned_m](const px4_msgs::msg::TrajectorySetpoint & msg) {
        return std::isfinite(msg.position[0]) &&
               std::isfinite(msg.position[1]) &&
               std::isfinite(msg.position[2]) &&
               std::fabs(msg.position[2] - z_ned_m) < 1e-3F;
      }));
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
        std::fabs(msg->param1 - 1.0F) < 1e-3F &&
        std::fabs(msg->param2 - 3.0F) < 1e-3F && auto_position_mode_.load())
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
      if (msg->command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM &&
        msg->param1 < 0.5F && auto_disarm_.load())
      {
        armed_ = false;
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
  std::atomic_bool auto_disarm_{true};
  std::atomic_bool auto_offboard_{true};
  std::atomic_bool auto_position_mode_{true};
  std::atomic_bool auto_disarm_on_land_{false};
  mutable std::mutex mutex_{};
  std::vector<VehicleCommand> vehicle_commands_{};
  std::vector<px4_msgs::msg::TrajectorySetpoint> trajectory_setpoints_{};
  std::vector<px4_msgs::msg::OffboardControlMode> offboard_modes_{};
  std::vector<Event> events_{};
  rclcpp::Publisher<VehicleLocalPosition>::SharedPtr local_position_pub_;
  rclcpp::Publisher<VehicleStatus>::SharedPtr vehicle_status_pub_;
  rclcpp::Publisher<VehicleLandDetected>::SharedPtr vehicle_land_detected_pub_;
  rclcpp::Subscription<VehicleCommand>::SharedPtr vehicle_command_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_sub_;
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
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    fake_node_ = std::make_shared<rclcpp::Node>("fake_px4_peer");
    client_node_ = std::make_shared<rclcpp::Node>("mullet_core_action_client");
    fake_px4_ = std::make_unique<FakePx4Peer>(fake_node_);

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("control_rate_hz", 50.0),
      rclcpp::Parameter("offboard_warmup_s", 1.0),
      rclcpp::Parameter("command_retry_interval_s", 0.1),
      rclcpp::Parameter("command_timeout_s", 0.25),
      rclcpp::Parameter("takeoff_altitude_m", 2.0),
      rclcpp::Parameter("takeoff_acceptance_m", 0.05),
      rclcpp::Parameter("takeoff_yaw_tolerance_rad", 0.05),
      rclcpp::Parameter("warmup_timeout_s", 1.2),
      rclcpp::Parameter("request_offboard_timeout_s", 0.3),
      rclcpp::Parameter("disable_offboard_timeout_s", 0.3),
      rclcpp::Parameter("request_arm_timeout_s", 0.3),
      rclcpp::Parameter("armed_hold_duration_s", 0.4),
      rclcpp::Parameter("request_disarm_timeout_s", 0.3),
      rclcpp::Parameter("request_position_mode_timeout_s", 0.3),
      rclcpp::Parameter("landed_confirmation_s", 0.1),
      rclcpp::Parameter("takeoff_timeout_s", 0.3),
      rclcpp::Parameter("landing_timeout_s", 0.3),
      rclcpp::Parameter("hover_altitude_tolerance_m", 0.05),
      rclcpp::Parameter("hover_vertical_velocity_tolerance_mps", 0.05),
      rclcpp::Parameter("hover_settle_time_s", 0.0),
    });
    state_machine_ = std::make_shared<DroneStateMachine>(options);
    command_pub_ = client_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
      "command", commandQoS());
    land_request_pub_ = client_node_->create_publisher<std_msgs::msg::Empty>(
      "land_request", rclcpp::QoS(1));
    takeoff_client_ = rclcpp_action::create_client<Takeoff>(client_node_, "takeoff");
    land_client_ = rclcpp_action::create_client<Land>(client_node_, "land");
    arm_client_ = rclcpp_action::create_client<Arm>(client_node_, "arm");
    disarm_client_ = rclcpp_action::create_client<Disarm>(client_node_, "disarm");
    enable_offboard_client_ = rclcpp_action::create_client<EnableOffboard>(
      client_node_, "enable_offboard");
    disable_offboard_client_ = rclcpp_action::create_client<DisableOffboard>(
      client_node_, "disable_offboard");
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
    ASSERT_TRUE(arm_client_->wait_for_action_server(2s));
    ASSERT_TRUE(disarm_client_->wait_for_action_server(2s));
    ASSERT_TRUE(enable_offboard_client_->wait_for_action_server(2s));
    ASSERT_TRUE(disable_offboard_client_->wait_for_action_server(2s));
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    takeoff_client_.reset();
    land_client_.reset();
    arm_client_.reset();
    disarm_client_.reset();
    enable_offboard_client_.reset();
    disable_offboard_client_.reset();
    command_pub_.reset();
    land_request_pub_.reset();
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
    Land::Goal goal{};
    auto future = land_client_->async_send_goal(goal);
    if (future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<Arm>> sendArmGoal()
  {
    Arm::Goal goal{};
    auto future = arm_client_->async_send_goal(goal);
    if (future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<Disarm>> sendDisarmGoal()
  {
    Disarm::Goal goal{};
    auto future = disarm_client_->async_send_goal(goal);
    if (future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<EnableOffboard>> sendEnableOffboardGoal()
  {
    EnableOffboard::Goal goal{};
    auto future = enable_offboard_client_->async_send_goal(goal);
    if (future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<DisableOffboard>> sendDisableOffboardGoal()
  {
    DisableOffboard::Goal goal{};
    auto future = disable_offboard_client_->async_send_goal(goal);
    if (future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult waitTakeoffResult(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<Takeoff>> & handle)
  {
    auto future = takeoff_client_->async_get_result(handle);
    EXPECT_EQ(future.wait_for(4s), std::future_status::ready);
    return future.get();
  }

  rclcpp_action::ClientGoalHandle<Land>::WrappedResult waitLandResult(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<Land>> & handle)
  {
    auto future = land_client_->async_get_result(handle);
    EXPECT_EQ(future.wait_for(4s), std::future_status::ready);
    return future.get();
  }

  rclcpp_action::ClientGoalHandle<Arm>::WrappedResult waitArmResult(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<Arm>> & handle)
  {
    auto future = arm_client_->async_get_result(handle);
    EXPECT_EQ(future.wait_for(4s), std::future_status::ready);
    return future.get();
  }

  rclcpp_action::ClientGoalHandle<Disarm>::WrappedResult waitDisarmResult(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<Disarm>> & handle)
  {
    auto future = disarm_client_->async_get_result(handle);
    EXPECT_EQ(future.wait_for(4s), std::future_status::ready);
    return future.get();
  }

  rclcpp_action::ClientGoalHandle<EnableOffboard>::WrappedResult waitEnableOffboardResult(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<EnableOffboard>> & handle)
  {
    auto future = enable_offboard_client_->async_get_result(handle);
    EXPECT_EQ(future.wait_for(4s), std::future_status::ready);
    return future.get();
  }

  rclcpp_action::ClientGoalHandle<DisableOffboard>::WrappedResult waitDisableOffboardResult(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<DisableOffboard>> & handle)
  {
    auto future = disable_offboard_client_->async_get_result(handle);
    EXPECT_EQ(future.wait_for(4s), std::future_status::ready);
    return future.get();
  }

  void completeTakeoff(const std::shared_ptr<rclcpp_action::ClientGoalHandle<Takeoff>> & handle)
  {
    ASSERT_TRUE(waitFor([this]() {
      return fake_px4_->setModeCount(1.0F, 6.0F) > 0 &&
             fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) > 0;
    }, 3s));
    fake_px4_->publishLandDetected(false);
    fake_px4_->publishLocalPosition(-2.0F, 0.0F, 0.0F);
    auto result = waitTakeoffResult(handle);
    ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
    ASSERT_TRUE(result.result->success);
    ASSERT_EQ(result.result->message, "Takeoff complete");
  }

  void publishLandRequest()
  {
    std_msgs::msg::Empty msg{};
    land_request_pub_->publish(msg);
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  rclcpp::Node::SharedPtr fake_node_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<DroneStateMachine> state_machine_;
  std::unique_ptr<FakePx4Peer> fake_px4_;
  rclcpp_action::Client<Takeoff>::SharedPtr takeoff_client_;
  rclcpp_action::Client<Land>::SharedPtr land_client_;
  rclcpp_action::Client<Arm>::SharedPtr arm_client_;
  rclcpp_action::Client<Disarm>::SharedPtr disarm_client_;
  rclcpp_action::Client<EnableOffboard>::SharedPtr enable_offboard_client_;
  rclcpp_action::Client<DisableOffboard>::SharedPtr disable_offboard_client_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr land_request_pub_;
  rclcpp::Subscription<DroneHeartbeat>::SharedPtr heartbeat_sub_;
  mutable std::mutex heartbeat_mutex_;
  std::vector<DroneHeartbeat> heartbeats_;
  std::thread spin_thread_;
};

class DroneStateMachineBehaviorPrefixedTopics : public DroneStateMachineBehavior
{
protected:
  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    fake_node_ = std::make_shared<rclcpp::Node>("fake_px4_peer_prefixed");
    client_node_ = std::make_shared<rclcpp::Node>("mullet_core_action_client_prefixed");
    fake_px4_ = std::make_unique<FakePx4Peer>(fake_node_, "/uav_2");

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("vehicle_instance", 2),
      rclcpp::Parameter("px4_namespace_prefix", "uav"),
      rclcpp::Parameter("control_rate_hz", 50.0),
      rclcpp::Parameter("offboard_warmup_s", 1.0),
      rclcpp::Parameter("command_retry_interval_s", 0.1),
      rclcpp::Parameter("command_timeout_s", 0.25),
      rclcpp::Parameter("takeoff_altitude_m", 2.0),
      rclcpp::Parameter("takeoff_acceptance_m", 0.05),
      rclcpp::Parameter("takeoff_yaw_tolerance_rad", 0.05),
      rclcpp::Parameter("warmup_timeout_s", 1.2),
      rclcpp::Parameter("request_offboard_timeout_s", 0.3),
      rclcpp::Parameter("request_arm_timeout_s", 0.3),
      rclcpp::Parameter("takeoff_timeout_s", 0.3),
      rclcpp::Parameter("landing_timeout_s", 0.3),
      rclcpp::Parameter("hover_altitude_tolerance_m", 0.05),
      rclcpp::Parameter("hover_vertical_velocity_tolerance_mps", 0.05),
      rclcpp::Parameter("hover_settle_time_s", 0.0),
    });
    state_machine_ = std::make_shared<DroneStateMachine>(options);
    command_pub_ = client_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
      "command", commandQoS());
    takeoff_client_ = rclcpp_action::create_client<Takeoff>(client_node_, "takeoff");
    land_client_ = rclcpp_action::create_client<Land>(client_node_, "land");
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
  }
};

class DroneStateMachineBehaviorLongRetryInterval : public DroneStateMachineBehavior
{
protected:
  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    fake_node_ = std::make_shared<rclcpp::Node>("fake_px4_peer_long_retry");
    client_node_ = std::make_shared<rclcpp::Node>("mullet_core_action_client_long_retry");
    fake_px4_ = std::make_unique<FakePx4Peer>(fake_node_);

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("control_rate_hz", 50.0),
      rclcpp::Parameter("offboard_warmup_s", 1.0),
      rclcpp::Parameter("command_retry_interval_s", 2.0),
      rclcpp::Parameter("command_timeout_s", 0.25),
      rclcpp::Parameter("takeoff_altitude_m", 2.0),
      rclcpp::Parameter("takeoff_acceptance_m", 0.05),
      rclcpp::Parameter("takeoff_yaw_tolerance_rad", 0.05),
      rclcpp::Parameter("warmup_timeout_s", 1.2),
      rclcpp::Parameter("request_offboard_timeout_s", 0.3),
      rclcpp::Parameter("disable_offboard_timeout_s", 0.3),
      rclcpp::Parameter("request_arm_timeout_s", 0.3),
      rclcpp::Parameter("armed_hold_duration_s", 0.4),
      rclcpp::Parameter("request_disarm_timeout_s", 0.3),
      rclcpp::Parameter("request_position_mode_timeout_s", 0.3),
      rclcpp::Parameter("landed_confirmation_s", 0.1),
      rclcpp::Parameter("takeoff_timeout_s", 0.3),
      rclcpp::Parameter("landing_timeout_s", 0.3),
      rclcpp::Parameter("hover_altitude_tolerance_m", 0.05),
      rclcpp::Parameter("hover_vertical_velocity_tolerance_mps", 0.05),
      rclcpp::Parameter("hover_settle_time_s", 0.0),
    });
    state_machine_ = std::make_shared<DroneStateMachine>(options);
    command_pub_ = client_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
      "command", commandQoS());
    land_request_pub_ = client_node_->create_publisher<std_msgs::msg::Empty>(
      "land_request", rclcpp::QoS(1));
    takeoff_client_ = rclcpp_action::create_client<Takeoff>(client_node_, "takeoff");
    land_client_ = rclcpp_action::create_client<Land>(client_node_, "land");
    arm_client_ = rclcpp_action::create_client<Arm>(client_node_, "arm");
    disarm_client_ = rclcpp_action::create_client<Disarm>(client_node_, "disarm");
    enable_offboard_client_ = rclcpp_action::create_client<EnableOffboard>(
      client_node_, "enable_offboard");
    disable_offboard_client_ = rclcpp_action::create_client<DisableOffboard>(
      client_node_, "disable_offboard");
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
    ASSERT_TRUE(arm_client_->wait_for_action_server(2s));
    ASSERT_TRUE(disarm_client_->wait_for_action_server(2s));
    ASSERT_TRUE(enable_offboard_client_->wait_for_action_server(2s));
    ASSERT_TRUE(disable_offboard_client_->wait_for_action_server(2s));
  }
};

TEST_F(DroneStateMachineBehavior, HeartbeatPublishesBeforePx4TelemetryReady)
{
  ASSERT_TRUE(waitFor([this]() {return !heartbeats().empty();}, 2s));

  const auto heartbeat = latestHeartbeat();
  EXPECT_EQ(heartbeat.schema_version, 1U);
  EXPECT_EQ(heartbeat.drone_id, "uav_0");
  EXPECT_EQ(heartbeat.vehicle_instance, 0U);
  EXPECT_EQ(heartbeat.target_system, 1U);
  EXPECT_EQ(heartbeat.ros_namespace, "/");
  EXPECT_EQ(heartbeat.takeoff_action_path, "/takeoff");
  EXPECT_TRUE(heartbeat.core_alive);
  EXPECT_FALSE(heartbeat.has_vehicle_status);
  EXPECT_FALSE(heartbeat.has_local_position);
  EXPECT_FALSE(heartbeat.local_position_valid);
  EXPECT_EQ(heartbeat.state_label, "INIT");
}

TEST_F(DroneStateMachineBehavior, HeartbeatSequenceIncrementsMonotonically)
{
  ASSERT_TRUE(waitFor([this]() {return heartbeats().size() >= 3U;}, 2s));

  const auto samples = heartbeats();
  for (std::size_t i = 1; i < samples.size(); ++i) {
    EXPECT_GT(samples[i].sequence, samples[i - 1].sequence);
  }
}

TEST_F(DroneStateMachineBehavior, HeartbeatReportsPreviousStateAfterTransition)
{
  publishReadyAndWait();

  ASSERT_TRUE(waitFor([this]() {
    const auto heartbeat = latestHeartbeat();
    return heartbeat.state_label == "READY" && heartbeat.previous_state_label == "INIT";
  }, 2s));
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
  EXPECT_FALSE(heartbeat.local_position_valid);
  EXPECT_DOUBLE_EQ(heartbeat.x, 0.0);
  EXPECT_DOUBLE_EQ(heartbeat.y, 0.0);
  EXPECT_DOUBLE_EQ(heartbeat.z, 0.0);
  EXPECT_DOUBLE_EQ(heartbeat.yaw, 0.0);
}

TEST_F(DroneStateMachineBehavior, HeartbeatActionAvailabilityChangesWithTelemetry)
{
  ASSERT_TRUE(waitFor([this]() {return !heartbeats().empty();}, 2s));
  EXPECT_FALSE(latestHeartbeat().can_takeoff);
  EXPECT_FALSE(latestHeartbeat().can_enable_offboard);

  publishReadyAndWait();

  ASSERT_TRUE(waitFor([this]() {
    const auto heartbeat = latestHeartbeat();
    return heartbeat.state_label == "READY" && heartbeat.can_takeoff &&
           heartbeat.can_enable_offboard && heartbeat.can_arm;
  }, 2s));
}

TEST_F(DroneStateMachineBehavior, TakeoffRejectedBeforePx4TelemetryReady)
{
  auto handle = sendTakeoffGoal();

  EXPECT_EQ(handle, nullptr);
}

TEST_F(DroneStateMachineBehavior, ArmRejectedBeforeLandedTelemetryConfirmed)
{
  EXPECT_EQ(sendArmGoal(), nullptr);

  fake_px4_->publishStatus();
  fake_px4_->publishLocalPosition(0.0F, 0.0F, 0.0F);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(sendArmGoal(), nullptr);

  fake_px4_->publishLandDetected(false);
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(sendArmGoal(), nullptr);
}

TEST_F(DroneStateMachineBehavior, ArmFromReadyRequestsOffboardBeforeArmAndHolds)
{
  publishReadyAndWait();

  auto handle = sendArmGoal();
  ASSERT_NE(handle, nullptr);
  auto result = waitArmResult(handle);

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(result.result->success);
  EXPECT_EQ(result.result->message, "Vehicle armed");
  EXPECT_GT(fake_px4_->setModeCount(1.0F, 6.0F), 0U);
  EXPECT_GT(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM), 0U);
  EXPECT_TRUE(fake_px4_->positionSetpointBeforeFirstArm(0.0F));
}

TEST_F(DroneStateMachineBehavior, ArmFromOffboardHoldDoesNotRequestOffboardAgain)
{
  publishReadyAndWait();

  auto enable = sendEnableOffboardGoal();
  ASSERT_NE(enable, nullptr);
  auto enable_result = waitEnableOffboardResult(enable);
  ASSERT_EQ(enable_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  const auto offboard_count = fake_px4_->setModeCount(1.0F, 6.0F);

  auto arm = sendArmGoal();
  ASSERT_NE(arm, nullptr);
  auto arm_result = waitArmResult(arm);

  EXPECT_EQ(arm_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(fake_px4_->setModeCount(1.0F, 6.0F), offboard_count);
}

TEST_F(
  DroneStateMachineBehaviorLongRetryInterval,
  ArmFromReadyRecoversAfterDisableOffboardWithoutRetryTimerCarryover)
{
  publishReadyAndWait();

  auto enable = sendEnableOffboardGoal();
  ASSERT_NE(enable, nullptr);
  auto enable_result = waitEnableOffboardResult(enable);
  ASSERT_EQ(enable_result.code, rclcpp_action::ResultCode::SUCCEEDED);

  auto disable = sendDisableOffboardGoal();
  ASSERT_NE(disable, nullptr);
  auto disable_result = waitDisableOffboardResult(disable);
  ASSERT_EQ(disable_result.code, rclcpp_action::ResultCode::SUCCEEDED);

  auto arm = sendArmGoal();
  ASSERT_NE(arm, nullptr);
  auto arm_result = waitArmResult(arm);

  EXPECT_EQ(arm_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(arm_result.result->success);
  EXPECT_EQ(arm_result.result->message, "Vehicle armed");
  EXPECT_GT(fake_px4_->setModeCount(1.0F, 6.0F), 1U);
}

TEST_F(DroneStateMachineBehavior, DisarmFromArmedHoldDirectDisarmsAndKeepsOffboard)
{
  publishReadyAndWait();

  auto arm = sendArmGoal();
  ASSERT_NE(arm, nullptr);
  ASSERT_EQ(waitArmResult(arm).code, rclcpp_action::ResultCode::SUCCEEDED);

  const auto before_position_mode_count = fake_px4_->setModeCount(1.0F, 3.0F);
  auto disarm = sendDisarmGoal();
  ASSERT_NE(disarm, nullptr);
  auto result = waitDisarmResult(disarm);

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->success);
  EXPECT_EQ(result.result->message, "Vehicle disarmed");
  EXPECT_EQ(fake_px4_->setModeCount(1.0F, 3.0F), before_position_mode_count);
  EXPECT_GT(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM), 1U);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);

  auto disable = sendDisableOffboardGoal();
  ASSERT_NE(disable, nullptr);
  auto disable_result = waitDisableOffboardResult(disable);

  EXPECT_EQ(disable_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(disable_result.result->success);
  EXPECT_EQ(disable_result.result->message, "Offboard disabled");
  EXPECT_GT(fake_px4_->setModeCount(1.0F, 3.0F), before_position_mode_count);
}

TEST_F(DroneStateMachineBehavior, DisarmWhileAirborneIsRejectedWithoutLanding)
{
  publishReadyAndWait();
  auto takeoff = sendTakeoffGoal();
  ASSERT_NE(takeoff, nullptr);
  completeTakeoff(takeoff);
  fake_px4_->publishLandDetected(false);

  auto before_disarm_count = fake_px4_->commandCount(
    VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM);
  auto before_land_count = fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND);
  auto disarm = sendDisarmGoal();
  EXPECT_EQ(disarm, nullptr);

  EXPECT_EQ(
    fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM),
    before_disarm_count);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), before_land_count);
}

TEST_F(DroneStateMachineBehavior, LostLandedStateStartsRecoveryAndRejectsDisarm)
{
  publishReadyAndWait();

  auto arm = sendArmGoal();
  ASSERT_NE(arm, nullptr);
  ASSERT_EQ(waitArmResult(arm).code, rclcpp_action::ResultCode::SUCCEEDED);
  fake_px4_->publishLandDetected(false);
  std::this_thread::sleep_for(150ms);

  const auto before_disarm_count = fake_px4_->commandCount(
    VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM);
  EXPECT_EQ(sendDisarmGoal(), nullptr);
  EXPECT_EQ(
    fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM),
    before_disarm_count);
  EXPECT_GT(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
}

TEST_F(
  DroneStateMachineBehaviorPrefixedTopics,
  NonZeroVehicleInstanceUsesConfiguredNamespacePrefixForPx4Topics)
{
  auto before_telemetry = sendTakeoffGoal();
  EXPECT_EQ(before_telemetry, nullptr);

  publishReadyAndWait();

  auto handle = sendTakeoffGoal();
  ASSERT_NE(handle, nullptr);
  EXPECT_TRUE(waitFor([this]() {
    return fake_px4_->setModeCount(1.0F, 6.0F) > 0 &&
           fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) > 0;
  }, 3s));
}

TEST_F(DroneStateMachineBehavior, EnableOffboardRejectedBeforePx4TelemetryReady)
{
  auto handle = sendEnableOffboardGoal();

  EXPECT_EQ(handle, nullptr);
}

TEST_F(DroneStateMachineBehavior, DisableOffboardRejectedBeforePx4TelemetryReady)
{
  auto handle = sendDisableOffboardGoal();

  EXPECT_EQ(handle, nullptr);
}

TEST_F(DroneStateMachineBehavior, DisableOffboardRejectsOutsideOffboardHold)
{
  publishReadyAndWait();

  EXPECT_EQ(sendDisableOffboardGoal(), nullptr);

  auto takeoff = sendTakeoffGoal();
  ASSERT_NE(takeoff, nullptr);
  EXPECT_EQ(sendDisableOffboardGoal(), nullptr);

  auto cancel_future = takeoff_client_->async_cancel_goal(takeoff);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto takeoff_result = waitTakeoffResult(takeoff);
  EXPECT_EQ(takeoff_result.code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(DroneStateMachineBehavior, EnableOffboardRejectsWhenAlreadyOffboardOrArmed)
{
  publishReadyAndWait();
  fake_px4_->setOffboard(true);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(sendEnableOffboardGoal(), nullptr);

  fake_px4_->setOffboard(false);
  fake_px4_->setArmed(true);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(sendEnableOffboardGoal(), nullptr);
}

TEST_F(DroneStateMachineBehavior, EnableOffboardHappyPathStaysReadyWithOffboardActive)
{
  publishReadyAndWait();

  auto handle = sendEnableOffboardGoal();
  ASSERT_NE(handle, nullptr);
  auto result = waitEnableOffboardResult(handle);

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(result.result->success);
  EXPECT_EQ(result.result->message, "Offboard enabled");
  ASSERT_TRUE(waitFor([this]() {
    const auto heartbeat = latestHeartbeat();
    return heartbeat.state_label == "READY" && heartbeat.offboard && !heartbeat.armed &&
           heartbeat.can_disable_offboard;
  }, 2s));
  EXPECT_GT(fake_px4_->setModeCount(1.0F, 6.0F), 0U);
  EXPECT_GT(fake_px4_->positionSetpointCountAtZ(0.0F), 0U);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM), 0U);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
}

TEST_F(DroneStateMachineBehavior, EnableOffboardTimeoutAbortsWithoutLandingRecovery)
{
  publishReadyAndWait();
  fake_px4_->setAutoOffboard(false);

  auto handle = sendEnableOffboardGoal();
  ASSERT_NE(handle, nullptr);
  auto result = waitEnableOffboardResult(handle);

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_FALSE(result.result->success);
  EXPECT_EQ(result.result->message, "enable_offboard request timed out");
  EXPECT_GT(fake_px4_->setModeCount(1.0F, 6.0F), 0U);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM), 0U);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
}

TEST_F(DroneStateMachineBehavior, EnableOffboardCancelReturnsReadyWithoutArmOrLand)
{
  publishReadyAndWait();

  auto handle = sendEnableOffboardGoal();
  ASSERT_NE(handle, nullptr);
  auto cancel_future = enable_offboard_client_->async_cancel_goal(handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto result = waitEnableOffboardResult(handle);

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_FALSE(result.result->success);
  EXPECT_EQ(result.result->message, "EnableOffboard canceled before completion");
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM), 0U);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
  auto second = sendEnableOffboardGoal();
  ASSERT_NE(second, nullptr);
  auto second_cancel_future = enable_offboard_client_->async_cancel_goal(second);
  ASSERT_EQ(second_cancel_future.wait_for(2s), std::future_status::ready);
  auto second_result = waitEnableOffboardResult(second);
  EXPECT_EQ(second_result.code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(DroneStateMachineBehavior, LandRequestPreemptsActiveEnableOffboardWithoutLanding)
{
  publishReadyAndWait();
  fake_px4_->setAutoOffboard(false);

  auto handle = sendEnableOffboardGoal();
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([this]() {return fake_px4_->setModeCount(1.0F, 6.0F) > 0;}));
  publishLandRequest();

  auto result = waitEnableOffboardResult(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_FALSE(result.result->success);
  EXPECT_EQ(result.result->message, "EnableOffboard preempted by emergency land request");
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM), 0U);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);

  auto second = sendEnableOffboardGoal();
  ASSERT_NE(second, nullptr);
  auto cancel_future = enable_offboard_client_->async_cancel_goal(second);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto second_result = waitEnableOffboardResult(second);
  EXPECT_EQ(second_result.code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(DroneStateMachineBehavior, LandRequestDuringEnableOffboardPreservesAcceptedOffboardHold)
{
  publishReadyAndWait();
  fake_px4_->setAutoOffboard(false);

  auto handle = sendEnableOffboardGoal();
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([this]() {return fake_px4_->setModeCount(1.0F, 6.0F) > 0;}));

  fake_px4_->setOffboard(true);
  publishLandRequest();

  auto result = waitEnableOffboardResult(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_FALSE(result.result->success);
  EXPECT_EQ(result.result->message, "EnableOffboard preempted by emergency land request");
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM), 0U);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);

  auto takeoff = sendTakeoffGoal();
  ASSERT_NE(takeoff, nullptr);
  ASSERT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) > 0;
  }, 2s));
  fake_px4_->publishLocalPosition(-2.0F, 0.0F, 0.0F);
  auto takeoff_result = waitTakeoffResult(takeoff);
  EXPECT_EQ(takeoff_result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(DroneStateMachineBehavior, EnableOffboardRejectsOutsideReady)
{
  publishReadyAndWait();
  fake_px4_->setAutoOffboard(false);

  auto takeoff = sendTakeoffGoal();
  ASSERT_NE(takeoff, nullptr);
  EXPECT_EQ(sendEnableOffboardGoal(), nullptr);

  auto cancel_future = takeoff_client_->async_cancel_goal(takeoff);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto takeoff_result = waitTakeoffResult(takeoff);
  ASSERT_EQ(takeoff_result.code, rclcpp_action::ResultCode::CANCELED);

  fake_px4_->setAutoOffboard(true);
  auto enable = sendEnableOffboardGoal();
  ASSERT_NE(enable, nullptr);
  auto enable_result = waitEnableOffboardResult(enable);
  ASSERT_EQ(enable_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(sendEnableOffboardGoal(), nullptr);
}

TEST_F(DroneStateMachineBehavior, OffboardHoldDoesNotReissueOffboardModeAfterSuccess)
{
  publishReadyAndWait();

  auto handle = sendEnableOffboardGoal();
  ASSERT_NE(handle, nullptr);
  auto result = waitEnableOffboardResult(handle);
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);

  const auto offboard_request_count = fake_px4_->setModeCount(1.0F, 6.0F);
  std::this_thread::sleep_for(350ms);
  EXPECT_EQ(fake_px4_->setModeCount(1.0F, 6.0F), offboard_request_count);
}

TEST_F(DroneStateMachineBehavior, DisableOffboardHappyPathRequestsPositionMode)
{
  publishReadyAndWait();

  auto enable = sendEnableOffboardGoal();
  ASSERT_NE(enable, nullptr);
  auto enable_result = waitEnableOffboardResult(enable);
  ASSERT_EQ(enable_result.code, rclcpp_action::ResultCode::SUCCEEDED);

  auto disable = sendDisableOffboardGoal();
  ASSERT_NE(disable, nullptr);
  auto disable_result = waitDisableOffboardResult(disable);

  EXPECT_EQ(disable_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(disable_result.result->success);
  EXPECT_EQ(disable_result.result->message, "Offboard disabled");
  ASSERT_TRUE(waitFor([this]() {
    const auto heartbeat = latestHeartbeat();
    return heartbeat.state_label == "READY" && !heartbeat.offboard && heartbeat.can_enable_offboard;
  }, 2s));
  EXPECT_GT(fake_px4_->setModeCount(1.0F, 3.0F), 0U);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM), 0U);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
}

TEST_F(DroneStateMachineBehavior, DisableOffboardTimeoutReturnsToUsableOffboardHold)
{
  publishReadyAndWait();

  auto enable = sendEnableOffboardGoal();
  ASSERT_NE(enable, nullptr);
  auto enable_result = waitEnableOffboardResult(enable);
  ASSERT_EQ(enable_result.code, rclcpp_action::ResultCode::SUCCEEDED);

  fake_px4_->setAutoPositionMode(false);
  auto disable = sendDisableOffboardGoal();
  ASSERT_NE(disable, nullptr);
  auto disable_result = waitDisableOffboardResult(disable);

  EXPECT_EQ(disable_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_FALSE(disable_result.result->success);
  EXPECT_EQ(disable_result.result->message, "disable_offboard request timed out");
  EXPECT_GT(fake_px4_->setModeCount(1.0F, 3.0F), 0U);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM), 0U);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);

  auto takeoff = sendTakeoffGoal();
  ASSERT_NE(takeoff, nullptr);
  ASSERT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) > 0;
  }, 2s));
  fake_px4_->publishLocalPosition(-2.0F, 0.0F, 0.0F);
  auto takeoff_result = waitTakeoffResult(takeoff);
  EXPECT_EQ(takeoff_result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(DroneStateMachineBehavior, DisableOffboardCancelReturnsToUsableOffboardHold)
{
  publishReadyAndWait();

  auto enable = sendEnableOffboardGoal();
  ASSERT_NE(enable, nullptr);
  auto enable_result = waitEnableOffboardResult(enable);
  ASSERT_EQ(enable_result.code, rclcpp_action::ResultCode::SUCCEEDED);

  fake_px4_->setAutoPositionMode(false);
  auto disable = sendDisableOffboardGoal();
  ASSERT_NE(disable, nullptr);
  ASSERT_TRUE(waitFor([this]() {return fake_px4_->setModeCount(1.0F, 3.0F) > 0;}));
  auto cancel_future = disable_offboard_client_->async_cancel_goal(disable);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto disable_result = waitDisableOffboardResult(disable);

  EXPECT_EQ(disable_result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_FALSE(disable_result.result->success);
  EXPECT_EQ(disable_result.result->message, "DisableOffboard canceled before completion");

  auto takeoff = sendTakeoffGoal();
  ASSERT_NE(takeoff, nullptr);
  ASSERT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) > 0;
  }, 2s));
  fake_px4_->publishLocalPosition(-2.0F, 0.0F, 0.0F);
  auto takeoff_result = waitTakeoffResult(takeoff);
  EXPECT_EQ(takeoff_result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(DroneStateMachineBehavior, DisableOffboardUnexpectedArmStartsLandingRecovery)
{
  publishReadyAndWait();

  auto enable = sendEnableOffboardGoal();
  ASSERT_NE(enable, nullptr);
  auto enable_result = waitEnableOffboardResult(enable);
  ASSERT_EQ(enable_result.code, rclcpp_action::ResultCode::SUCCEEDED);

  fake_px4_->setAutoPositionMode(false);
  auto disable = sendDisableOffboardGoal();
  ASSERT_NE(disable, nullptr);
  ASSERT_TRUE(waitFor([this]() {return fake_px4_->setModeCount(1.0F, 3.0F) > 0;}));
  fake_px4_->setArmed(true);

  auto disable_result = waitDisableOffboardResult(disable);
  EXPECT_EQ(disable_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_FALSE(disable_result.result->success);
  EXPECT_EQ(disable_result.result->message, "disable_offboard aborted: vehicle armed unexpectedly");
  EXPECT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > 0;
  }, 2s));
}

TEST_F(DroneStateMachineBehavior, OffboardHoldExitsIfPx4LeavesOffboard)
{
  publishReadyAndWait();

  auto handle = sendEnableOffboardGoal();
  ASSERT_NE(handle, nullptr);
  auto result = waitEnableOffboardResult(handle);
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);

  const auto offboard_request_count = fake_px4_->setModeCount(1.0F, 6.0F);
  fake_px4_->setOffboard(false);
  ASSERT_TRUE(waitFor([this, offboard_request_count]() {
    return fake_px4_->setModeCount(1.0F, 6.0F) == offboard_request_count;
  }, 500ms));
  std::this_thread::sleep_for(250ms);
  EXPECT_EQ(fake_px4_->setModeCount(1.0F, 6.0F), offboard_request_count);
}

TEST_F(DroneStateMachineBehavior, OffboardHoldInvalidLocalPositionReturnsToInit)
{
  publishReadyAndWait();

  auto handle = sendEnableOffboardGoal();
  ASSERT_NE(handle, nullptr);
  auto result = waitEnableOffboardResult(handle);
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);

  fake_px4_->publishInvalidLocalPosition();
  ASSERT_TRUE(waitFor([this]() {
    return sendTakeoffGoal() == nullptr;
  }, 500ms));
}

TEST_F(DroneStateMachineBehavior, OffboardHoldUnexpectedGroundArmEntersArmedHold)
{
  publishReadyAndWait();

  auto handle = sendEnableOffboardGoal();
  ASSERT_NE(handle, nullptr);
  auto result = waitEnableOffboardResult(handle);
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);

  fake_px4_->setArmed(true);
  ASSERT_TRUE(waitFor([this]() {
    const auto heartbeat = latestHeartbeat();
    return heartbeat.state_label == "ARMED_HOLD" && heartbeat.armed && heartbeat.offboard &&
           heartbeat.can_disarm;
  }, 2s));
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
}

TEST_F(DroneStateMachineBehavior, LandFromDisarmedOffboardHoldIsRejected)
{
  publishReadyAndWait();

  auto enable = sendEnableOffboardGoal();
  ASSERT_NE(enable, nullptr);
  auto enable_result = waitEnableOffboardResult(enable);
  ASSERT_EQ(enable_result.code, rclcpp_action::ResultCode::SUCCEEDED);

  EXPECT_EQ(sendLandGoal(), nullptr);
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
}

TEST_F(DroneStateMachineBehavior, TakeoffFromOffboardHoldPublishesTakeoffSetpointBeforeArm)
{
  publishReadyAndWait();

  auto enable = sendEnableOffboardGoal();
  ASSERT_NE(enable, nullptr);
  auto enable_result = waitEnableOffboardResult(enable);
  ASSERT_EQ(enable_result.code, rclcpp_action::ResultCode::SUCCEEDED);

  auto takeoff = sendTakeoffGoal();
  ASSERT_NE(takeoff, nullptr);
  ASSERT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) > 0;
  }, 2s));
  EXPECT_TRUE(fake_px4_->positionSetpointBeforeFirstArm(-2.0F));
  fake_px4_->publishLocalPosition(-2.0F, 0.0F, 0.0F);
  auto takeoff_result = waitTakeoffResult(takeoff);
  EXPECT_EQ(takeoff_result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(DroneStateMachineBehavior, TakeoffHappyPathEmitsOffboardAndArmThenSucceeds)
{
  publishReadyAndWait();

  auto handle = sendTakeoffGoal();
  ASSERT_NE(handle, nullptr);
  completeTakeoff(handle);

  EXPECT_GT(fake_px4_->setModeCount(1.0F, 6.0F), 0U);
  EXPECT_GT(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM), 0U);
}

TEST_F(DroneStateMachineBehavior, DuplicateTakeoffGoalWhileActiveIsRejected)
{
  publishReadyAndWait();

  auto first = sendTakeoffGoal();
  ASSERT_NE(first, nullptr);
  auto second = sendTakeoffGoal();
  EXPECT_EQ(second, nullptr);

  auto cancel_future = takeoff_client_->async_cancel_goal(first);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto result = waitTakeoffResult(first);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(result.result->message, "Takeoff canceled before arming");
}

TEST_F(DroneStateMachineBehavior, CancelBeforeArmingDoesNotRequireLandingRecovery)
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
  EXPECT_EQ(result.result->message, "Takeoff canceled before arming");
  EXPECT_EQ(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
}

TEST_F(DroneStateMachineBehavior, CancelAfterArmRequestContinuesLandingRecovery)
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
  ASSERT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > 0;
  }, 2s));

  fake_px4_->setAutoDisarmOnLand(true);
  auto before = fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND);
  ASSERT_TRUE(waitFor([this, before]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > before;
  }, 2s));
  auto result = waitTakeoffResult(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(result.result->message, "Takeoff canceled; vehicle disarmed after landing");
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
  EXPECT_FALSE(takeoff_result.result->success);
  EXPECT_EQ(takeoff_result.result->message, "Takeoff preempted by land action");
  EXPECT_EQ(land_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(land_result.result->success);
  EXPECT_EQ(
    land_result.result->message,
    "Takeoff canceled before arming; vehicle already disarmed");
}

TEST_F(DroneStateMachineBehavior, LandFromCommandSendsNavLandAndSucceedsAfterDisarm)
{
  publishReadyAndWait();
  auto takeoff = sendTakeoffGoal();
  ASSERT_NE(takeoff, nullptr);
  completeTakeoff(takeoff);

  std_msgs::msg::Float32MultiArray command{};
  command.data = {0.2F, 0.0F, 1.0F};
  command_pub_->publish(command);
  ASSERT_TRUE(waitFor([this]() {return fake_px4_->velocitySetpointCount() > 0;}));

  fake_px4_->setAutoDisarmOnLand(true);
  auto land = sendLandGoal();
  ASSERT_NE(land, nullptr);
  auto result = waitLandResult(land);

  EXPECT_GT(fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND), 0U);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->success);
  EXPECT_EQ(result.result->message, "Vehicle disarmed after landing");
}

TEST_F(DroneStateMachineBehavior, LandGoalWhileAlreadyLandingIsRejected)
{
  publishReadyAndWait();
  auto takeoff = sendTakeoffGoal();
  ASSERT_NE(takeoff, nullptr);
  completeTakeoff(takeoff);

  fake_px4_->setAutoDisarmOnLand(false);
  auto first_land = sendLandGoal();
  ASSERT_NE(first_land, nullptr);
  ASSERT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > 0;
  }));

  auto second_land = sendLandGoal();
  EXPECT_EQ(second_land, nullptr);

  fake_px4_->setAutoDisarmOnLand(true);
  auto before = fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND);
  ASSERT_TRUE(waitFor([this, before]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > before;
  }, 2s));
  auto result = waitLandResult(first_land);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(DroneStateMachineBehavior, LandingTimeoutAbortsActionButKeepsRetryingUntilDisarm)
{
  publishReadyAndWait();
  auto takeoff = sendTakeoffGoal();
  ASSERT_NE(takeoff, nullptr);
  completeTakeoff(takeoff);

  fake_px4_->setAutoDisarmOnLand(false);
  auto land = sendLandGoal();
  ASSERT_NE(land, nullptr);
  ASSERT_TRUE(waitFor([this]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > 0;
  }, 2s));

  auto land_result = waitLandResult(land);
  EXPECT_EQ(land_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(land_result.result->success);
  EXPECT_EQ(land_result.result->message, "Landing timed out while vehicle is still armed");

  const auto before_retry_count = fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND);
  EXPECT_TRUE(waitFor([this, before_retry_count]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > before_retry_count;
  }, 2s));

  fake_px4_->setAutoDisarmOnLand(true);
  const auto before_disarm_count = fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND);
  EXPECT_TRUE(waitFor([this, before_disarm_count]() {
    return fake_px4_->commandCount(VehicleCommand::VEHICLE_CMD_NAV_LAND) > before_disarm_count;
  }, 2s));
}

TEST_F(DroneStateMachineBehavior, TakeoffTimeoutAbortsActionAndKeepsLandingRecovery)
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

}  // namespace
}  // namespace mullet_core
