#ifndef MULLET_CORE_DRONE_STATE_MACHINE_HPP_
#define MULLET_CORE_DRONE_STATE_MACHINE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "mullet_core/action/takeoff.hpp"
#include "mullet_core/px4_interface.hpp"
#include "mullet_core/types.hpp"

namespace mullet_core
{

class DroneStateMachine : public rclcpp::Node
{
public:
  DroneStateMachine();

private:
  using Takeoff = mullet_core::action::Takeoff;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;

  void loadParameters();
  void controlTick();
  void onCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void onLandRequest(const std_msgs::msg::Empty::SharedPtr msg);

  rclcpp_action::GoalResponse handleTakeoffGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Takeoff::Goal> goal);
  rclcpp_action::CancelResponse handleTakeoffCancel(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle);
  void handleTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goal_handle);

  void transitionTo(DroneState new_state, const char * reason);
  void handleInit(const rclcpp::Time & now);
  void handleReady(const rclcpp::Time & now);
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

  bool commandFresh(const rclcpp::Time & now) const;
  bool takeoffSettled(const rclcpp::Time & now);
  bool takeoffYawSettled(float current_heading, float * yaw_error_rad) const;
  bool phaseTimedOut(const rclcpp::Time & now, double timeout_s) const;
  bool haveActiveTakeoffGoal() const;
  bool goalCancelRequested() const;
  bool shouldRecoverThroughLanding() const;
  bool latchTakeoffTarget();
  bool configureTakeoffTarget(const Takeoff::Goal & goal);
  void latchHoverTarget();
  void latchHoverTargetPreservingTakeoffYaw();
  void publishTakeoffSetpoint();
  void publishHoverSetpoint();
  void publishTakeoffFeedback(float progress);
  void succeedTakeoffGoal(const char * message);
  void abortTakeoffGoal(const char * message);
  void cancelTakeoffGoal(const char * message);
  void beginLanding(const char * reason, bool request_position_mode);
  void failTakeoffOrLand(const char * reason);
  void resetTakeoffAttemptState();
  void updateTimingStats(const rclcpp::Time & now);
  void maybeLogTiming(const rclcpp::Time & now);

  static const char * stateName(DroneState state);

  ControlConfig config_{};
  std::unique_ptr<Px4Interface> px4_;
  ExternalCommandBuffer command_buffer_{};

  DroneState state_{DroneState::kInit};
  rclcpp::Time state_enter_time_{};
  rclcpp::Time last_vehicle_command_time_{};
  rclcpp::Time last_tick_time_{};
  rclcpp::Time last_timing_log_time_{};
  double max_tick_jitter_ms_{0.0};
  std::uint64_t missed_deadline_count_{0};
  bool timing_initialized_{false};
  bool position_mode_requested_for_landing_{false};
  bool landing_timeout_reported_{false};

  float takeoff_x_ned_m_{0.0F};
  float takeoff_y_ned_m_{0.0F};
  float takeoff_origin_z_ned_m_{0.0F};
  float takeoff_z_ned_m_{-2.0F};
  float hover_x_ned_m_{0.0F};
  float hover_y_ned_m_{0.0F};
  float hover_z_ned_m_{-2.0F};
  float hover_yaw_rad_{0.0F};

  std::string command_topic_{"command"};
  std::string land_topic_{"land_request"};

  std::shared_ptr<GoalHandleTakeoff> active_takeoff_goal_{};
  bool takeoff_cancel_requested_{false};
  bool takeoff_arm_requested_{false};
  bool active_takeoff_uses_current_yaw_{true};
  float active_takeoff_target_yaw_rad_{0.0F};
  rclcpp::Time hover_settle_start_time_{};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr land_sub_;
  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_action_server_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace mullet_core

#endif  // MULLET_CORE_DRONE_STATE_MACHINE_HPP_
