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
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace mullet_core
{
namespace
{

using namespace std::chrono_literals;

using Land = mullet_core::action::Land;
using Takeoff = mullet_core::action::Takeoff;
using VehicleCommand = px4_msgs::msg::VehicleCommand;
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
  explicit FakePx4Peer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    local_position_pub_ = node_->create_publisher<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", px4QoS());
    vehicle_status_pub_ = node_->create_publisher<VehicleStatus>(
      "/fmu/out/vehicle_status_v1", px4QoS());
    vehicle_command_sub_ = node_->create_subscription<VehicleCommand>(
      "/fmu/in/vehicle_command", px4QoS(),
      [this](const VehicleCommand::SharedPtr msg) {onVehicleCommand(msg);});
    trajectory_sub_ = node_->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", px4QoS(),
      [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        trajectory_setpoints_.push_back(*msg);
      });
    offboard_mode_sub_ = node_->create_subscription<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", px4QoS(),
      [this](const px4_msgs::msg::OffboardControlMode::SharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        offboard_modes_.push_back(*msg);
      });
  }

  void publishReadyTelemetry(float z_ned_m = 0.0F, float heading_rad = 0.0F)
  {
    armed_ = false;
    offboard_ = false;
    publishStatus();
    publishLocalPosition(z_ned_m, heading_rad, 0.0F);
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

  void publishStatus()
  {
    VehicleStatus msg{};
    msg.arming_state = armed_ ? VehicleStatus::ARMING_STATE_ARMED :
      VehicleStatus::ARMING_STATE_DISARMED;
    msg.nav_state = offboard_ ? VehicleStatus::NAVIGATION_STATE_OFFBOARD :
      VehicleStatus::NAVIGATION_STATE_POSCTL;
    vehicle_status_pub_->publish(msg);
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
      [](const px4_msgs::msg::TrajectorySetpoint & msg) {
        return std::isfinite(msg.velocity[0]) && std::isfinite(msg.velocity[1]);
      }));
  }

private:
  void onVehicleCommand(const VehicleCommand::SharedPtr msg)
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      vehicle_commands_.push_back(*msg);
    }
    if (msg->command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
      std::fabs(msg->param2 - 6.0F) < 1e-3F && auto_offboard_.load())
    {
      offboard_ = true;
      publishStatus();
    }
    if (msg->command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM &&
      msg->param1 > 0.5F && auto_arm_.load())
    {
      armed_ = true;
      publishStatus();
    }
    if (msg->command == VehicleCommand::VEHICLE_CMD_NAV_LAND && auto_disarm_on_land_.load()) {
      armed_ = false;
      offboard_ = false;
      publishStatus();
    }
  }

  rclcpp::Node::SharedPtr node_;
  bool armed_{false};
  bool offboard_{false};
  std::atomic_bool auto_arm_{true};
  std::atomic_bool auto_offboard_{true};
  std::atomic_bool auto_disarm_on_land_{false};
  mutable std::mutex mutex_{};
  std::vector<VehicleCommand> vehicle_commands_{};
  std::vector<px4_msgs::msg::TrajectorySetpoint> trajectory_setpoints_{};
  std::vector<px4_msgs::msg::OffboardControlMode> offboard_modes_{};
  rclcpp::Publisher<VehicleLocalPosition>::SharedPtr local_position_pub_;
  rclcpp::Publisher<VehicleStatus>::SharedPtr vehicle_status_pub_;
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

    executor_->add_node(fake_node_);
    executor_->add_node(client_node_);
    executor_->add_node(state_machine_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    ASSERT_TRUE(takeoff_client_->wait_for_action_server(2s));
    ASSERT_TRUE(land_client_->wait_for_action_server(2s));
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    takeoff_client_.reset();
    land_client_.reset();
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

  void publishReadyAndWait()
  {
    for (int i = 0; i < 5; ++i) {
      fake_px4_->publishReadyTelemetry();
      std::this_thread::sleep_for(50ms);
    }
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

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  rclcpp::Node::SharedPtr fake_node_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<DroneStateMachine> state_machine_;
  std::unique_ptr<FakePx4Peer> fake_px4_;
  rclcpp_action::Client<Takeoff>::SharedPtr takeoff_client_;
  rclcpp_action::Client<Land>::SharedPtr land_client_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_pub_;
  std::thread spin_thread_;
};

TEST_F(DroneStateMachineBehavior, TakeoffRejectedBeforePx4TelemetryReady)
{
  auto handle = sendTakeoffGoal();

  EXPECT_EQ(handle, nullptr);
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
