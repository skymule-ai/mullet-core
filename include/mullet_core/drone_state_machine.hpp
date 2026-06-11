#ifndef MULLET_CORE_DRONE_STATE_MACHINE_HPP_
#define MULLET_CORE_DRONE_STATE_MACHINE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "mullet_core/px4_interface.hpp"
#include "mullet_core/types.hpp"
#include "mullet_interfaces/action/arm.hpp"
#include "mullet_interfaces/action/disable_offboard.hpp"
#include "mullet_interfaces/action/disarm.hpp"
#include "mullet_interfaces/action/enable_offboard.hpp"
#include "mullet_interfaces/action/land.hpp"
#include "mullet_interfaces/action/takeoff.hpp"
#include "mullet_interfaces/msg/drone_heartbeat.hpp"

namespace mullet_core::state_machine_detail
{
class ActionLifecycleCoordinator;
}  // namespace mullet_core::state_machine_detail

namespace mullet_core
{

class DroneStateMachine : public rclcpp::Node
{
public:
  explicit DroneStateMachine(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DroneStateMachine() override;

private:
  using DisableOffboard = mullet_interfaces::action::DisableOffboard;
  using GoalHandleDisableOffboard = rclcpp_action::ServerGoalHandle<DisableOffboard>;
  using Arm = mullet_interfaces::action::Arm;
  using GoalHandleArm = rclcpp_action::ServerGoalHandle<Arm>;
  using Disarm = mullet_interfaces::action::Disarm;
  using GoalHandleDisarm = rclcpp_action::ServerGoalHandle<Disarm>;
  using EnableOffboard = mullet_interfaces::action::EnableOffboard;
  using GoalHandleEnableOffboard = rclcpp_action::ServerGoalHandle<EnableOffboard>;
  using Land = mullet_interfaces::action::Land;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;
  using Takeoff = mullet_interfaces::action::Takeoff;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;

  void loadParameters();
  void controlTick();
  void publishHeartbeat();
  void onCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void onLandRequest(const std_msgs::msg::Empty::SharedPtr msg);

  rclcpp_action::GoalResponse handleTakeoffGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Takeoff::Goal> goal);
  rclcpp_action::CancelResponse handleTakeoffCancel(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle);
  void handleTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goal_handle);
  rclcpp_action::GoalResponse handleLandGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Land::Goal> goal);
  rclcpp_action::CancelResponse handleLandCancel(
    const std::shared_ptr<GoalHandleLand> goal_handle);
  void handleLandAccepted(const std::shared_ptr<GoalHandleLand> goal_handle);
  rclcpp_action::GoalResponse handleEnableOffboardGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const EnableOffboard::Goal> goal);
  rclcpp_action::CancelResponse handleEnableOffboardCancel(
    const std::shared_ptr<GoalHandleEnableOffboard> goal_handle);
  void handleEnableOffboardAccepted(
    const std::shared_ptr<GoalHandleEnableOffboard> goal_handle);
  rclcpp_action::GoalResponse handleDisableOffboardGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DisableOffboard::Goal> goal);
  rclcpp_action::CancelResponse handleDisableOffboardCancel(
    const std::shared_ptr<GoalHandleDisableOffboard> goal_handle);
  void handleDisableOffboardAccepted(
    const std::shared_ptr<GoalHandleDisableOffboard> goal_handle);
  rclcpp_action::GoalResponse handleArmGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Arm::Goal> goal);
  rclcpp_action::CancelResponse handleArmCancel(
    const std::shared_ptr<GoalHandleArm> goal_handle);
  void handleArmAccepted(const std::shared_ptr<GoalHandleArm> goal_handle);
  rclcpp_action::GoalResponse handleDisarmGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Disarm::Goal> goal);
  rclcpp_action::CancelResponse handleDisarmCancel(
    const std::shared_ptr<GoalHandleDisarm> goal_handle);
  void handleDisarmAccepted(const std::shared_ptr<GoalHandleDisarm> goal_handle);

  void transitionTo(DroneState new_state, const char * reason);
  void handleInit(const rclcpp::Time & now);
  void handleReady(const rclcpp::Time & now);
  void handleEnableOffboardRequested(const rclcpp::Time & now);
  void handleWarmupOffboardHoldSetpoints(const rclcpp::Time & now);
  void handleRequestOffboardOnly(const rclcpp::Time & now);
  void handleOffboardHold(const rclcpp::Time & now);
  void handleArmRequested(const rclcpp::Time & now);
  void handleArmCommand(const rclcpp::Time & now);
  void handleArmedOffboardHold(const rclcpp::Time & now);
  void handleDisarmCommand(const rclcpp::Time & now);
  void handleReleaseOffboardRequest(const rclcpp::Time & now);
  void handleDisarmedOffboardReleaseFailed(const rclcpp::Time & now);
  void handleDisableOffboardRequested(const rclcpp::Time & now);
  void handleTakeoffRequested(const rclcpp::Time & now);
  void handleWarmup(const rclcpp::Time & now);
  void handleRequestOffboard(const rclcpp::Time & now);
  void handleRequestArm(const rclcpp::Time & now);
  void handleTakeoff(const rclcpp::Time & now);
  void handleHover(const rclcpp::Time & now);
  void handleCommand(const rclcpp::Time & now);
  void handleCancelingToLand(const rclcpp::Time & now);
  void handleLanding(const rclcpp::Time & now);
  void handleFailsafeHold(const rclcpp::Time & now);
  void handleDisarmed(const rclcpp::Time & now);

  bool commandFresh(const rclcpp::Time & now) const;
  bool takeoffSettled(const rclcpp::Time & now);
  bool takeoffYawSettled(float current_heading, float * yaw_error_rad) const;
  bool phaseTimedOut(const rclcpp::Time & now, double timeout_s) const;
  bool haveActiveTakeoffGoal() const;
  bool haveActiveLandGoal() const;
  bool haveActiveEnableOffboardGoal() const;
  bool haveActiveDisableOffboardGoal() const;
  bool haveActiveArmGoal() const;
  bool haveActiveDisarmGoal() const;
  bool activeTakeoffState() const;
  bool landActionAllowedFromState() const;
  bool goalCancelRequested() const;
  bool enableOffboardCancelRequested() const;
  bool disableOffboardCancelRequested() const;
  bool shouldRecoverThroughLanding() const;
  bool landedConfirmed(const rclcpp::Time & now) const;
  bool directDisarmAllowed(const rclcpp::Time & now) const;
  bool anyActionActive() const;
  bool canTakeoff() const;
  bool canLand() const;
  bool canEnableOffboard() const;
  bool canDisableOffboard() const;
  bool canArm(const rclcpp::Time & now) const;
  bool canDisarm(const rclcpp::Time & now) const;
  bool canEmergencyLand() const;
  bool latchOffboardHoldTarget();
  bool latchTakeoffTarget();
  bool configureTakeoffTarget(const Takeoff::Goal & goal);
  void latchHoverTarget();
  void latchHoverTargetPreservingTakeoffYaw();
  void publishTakeoffSetpoint();
  void publishHoverSetpoint();
  void publishTakeoffFeedback(float progress);
  void publishLandFeedback(const rclcpp::Time & now);
  void publishEnableOffboardFeedback(const rclcpp::Time & now);
  void publishDisableOffboardFeedback(const rclcpp::Time & now);
  void publishArmFeedback(const rclcpp::Time & now);
  void publishDisarmFeedback(const rclcpp::Time & now);
  void succeedTakeoffGoal(const char * message);
  void abortTakeoffGoal(const char * message);
  void cancelTakeoffGoal(const char * message);
  void preemptTakeoffForLandAction();
  void succeedLandGoal(const char * message);
  void abortLandGoal(const char * message);
  void succeedEnableOffboardGoal(const char * message);
  void abortEnableOffboardGoal(const char * message);
  void cancelEnableOffboardGoal(const char * message);
  void succeedDisableOffboardGoal(const char * message);
  void abortDisableOffboardGoal(const char * message);
  void cancelDisableOffboardGoal(const char * message);
  void succeedArmGoal(const char * message);
  void abortArmGoal(const char * message);
  void cancelArmGoal(const char * message);
  void succeedDisarmGoal(const char * message);
  void abortDisarmGoal(const char * message);
  void cancelDisarmGoal(const char * message);
  void startLanding(const char * reason);
  void beginLanding(const char * reason, bool request_position_mode);
  void failTakeoffOrLand(const char * reason);
  void finishEnableOffboardAttempt(DroneState fallback_state, const char * reason);
  void finishDisableOffboardAttempt(const char * reason);
  void clearExternalCommandBuffer();
  void resetTakeoffAttemptState();
  void resetVehicleCommandRetryTimer();
  void updateTimingStats(const rclcpp::Time & now);
  void maybeLogTiming(const rclcpp::Time & now);

  static const char * stateName(DroneState state);
  static const char * navStateName(std::uint8_t nav_state);
  static const char * armingStateName(std::uint8_t arming_state);

  ControlConfig config_{};
  std::unique_ptr<Px4Interface> px4_;
  ExternalCommandBuffer command_buffer_{};

  DroneState state_{DroneState::kInit};
  DroneState previous_state_{DroneState::kInit};
  rclcpp::Time state_enter_time_{};
  rclcpp::Time landing_start_time_{};
  rclcpp::Time last_vehicle_command_time_{};
  rclcpp::Time last_tick_time_{};
  rclcpp::Time last_timing_log_time_{};
  double max_tick_jitter_ms_{0.0};
  std::uint64_t missed_deadline_count_{0};
  bool timing_initialized_{false};
  bool position_mode_requested_for_landing_{false};
  bool landing_timeout_reported_{false};
  bool arm_started_from_offboard_hold_{false};
  bool release_offboard_after_disarm_{false};
  rclcpp::Time armed_hold_deadline_{};

  float takeoff_x_ned_m_{0.0F};
  float takeoff_y_ned_m_{0.0F};
  float takeoff_origin_z_ned_m_{0.0F};
  float takeoff_z_ned_m_{-2.0F};
  float hover_x_ned_m_{0.0F};
  float hover_y_ned_m_{0.0F};
  float hover_z_ned_m_{-2.0F};
  float hover_yaw_rad_{0.0F};

  std::string drone_id_{"uav_0"};
  std::string fmu_prefix_{};
  std::uint8_t target_system_{1};
  std::uint64_t heartbeat_sequence_{0};
  std::string command_topic_{"command"};
  std::string land_topic_{"land_request"};

  std::unique_ptr<state_machine_detail::ActionLifecycleCoordinator> action_lifecycle_;
  bool takeoff_arm_requested_{false};
  bool active_takeoff_uses_current_yaw_{true};
  float active_takeoff_target_yaw_rad_{0.0F};
  rclcpp::Time hover_settle_start_time_{};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr land_sub_;
  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_action_server_;
  rclcpp_action::Server<Land>::SharedPtr land_action_server_;
  rclcpp_action::Server<Arm>::SharedPtr arm_action_server_;
  rclcpp_action::Server<Disarm>::SharedPtr disarm_action_server_;
  rclcpp_action::Server<EnableOffboard>::SharedPtr enable_offboard_action_server_;
  rclcpp_action::Server<DisableOffboard>::SharedPtr disable_offboard_action_server_;
  rclcpp::Publisher<mullet_interfaces::msg::DroneHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace mullet_core

#endif  // MULLET_CORE_DRONE_STATE_MACHINE_HPP_
