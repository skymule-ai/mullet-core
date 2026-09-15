#ifndef MULLET_CORE_FLIGHT_DRONE_STATE_MACHINE_HPP_
#define MULLET_CORE_FLIGHT_DRONE_STATE_MACHINE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mullet_core/flight/types.hpp"
#include "mullet_core/px4/offboard_heartbeat_publisher.hpp"
#include "mullet_core/px4/px4_interface.hpp"
#include "mullet_core/telemetry/paddock_heartbeat_publisher.hpp"
#include "mullet_interfaces/action/land.hpp"
#include "mullet_interfaces/action/return_home.hpp"
#include "mullet_interfaces/action/takeoff.hpp"
#include "mullet_interfaces/msg/drone_heartbeat.hpp"
#include "mullet_interfaces/msg/mission_command.hpp"

namespace mullet_core::state_machine_detail
{
class ActionLifecycleCoordinator;
}  // namespace mullet_core::state_machine_detail

namespace mullet_core
{

// Supervises PX4 offboard flight through ROS actions, services, telemetry, and commands.
class DroneStateMachine : public rclcpp::Node
{
public:
  // Creates the supervisor node and its ROS interfaces.
  //
  // Args:
  //   options: ROS node construction options.
  explicit DroneStateMachine(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  // Destroys the node; timers/subscriptions/action servers tear down via their own destructors.
  ~DroneStateMachine() override;

private:
  // Steps inside DroneState::kTransition. The phase decides which transition is running:
  // kWarmup..kAscent are a takeoff, kLanding is a land, kReturnHome is a return home.
  enum class Phase : std::uint8_t
  {
    kWarmup, kRequestOffboard, kRequestArm, kAscent, kLanding, kReturnHome,
  };
  using Land = mullet_interfaces::action::Land;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;
  using ReturnHome = mullet_interfaces::action::ReturnHome;
  using GoalHandleReturnHome = rclcpp_action::ServerGoalHandle<ReturnHome>;
  using Takeoff = mullet_interfaces::action::Takeoff;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;
  using Trigger = std_srvs::srv::Trigger;
  using MissionCommand = mullet_interfaces::msg::MissionCommand;

  // Declares all ROS parameters into `config_`/topic names, clamping each to a safe minimum.
  void loadParameters();
  // Creates the external mission command subscription.
  void initializeSubscriptions();
  // Creates the three ROS action servers and the two Trigger services.
  void initializeActionServers();
  // Runs one control-loop iteration: PX4 overrides, cancel requests, then the per-state tick.
  void controlTick();
  // PX4 outranks the supervisor: an external disarm (kill switch) ends airborne control in
  // IDLE, and losing offboard while airborne (pilot takeover) drops to a passive HOLD.
  void handleExternalOverride();
  // Applies Takeoff/ReturnHome cancel requests latched by the cancel callbacks.
  void handleActionCancellations();
  // Runs the handler for the current FSM state.
  void dispatchState(const rclcpp::Time & now);
  // Assembles the full `/mullet/heartbeat` snapshot (FSM state, PX4 status, position,
  // capability flags) from current node state.
  mullet_interfaces::msg::DroneHeartbeat buildHeartbeatMessage();
  // Validates and latches an external mission command from `command_topic_`.
  void onCommand(const MissionCommand::SharedPtr msg);
  // Trigger service bodies: start_mission (HOLD -> MISSION) and hold (MISSION or return home
  // -> HOLD). Both reject with the matching xRejectReason() as the response message.
  void onStartMission(Trigger::Response & response);
  void onHold(Trigger::Response & response);

  // Goal/cancel/accepted callbacks for the three ROS actions. Goal callbacks reject with a
  // RCLCPP_WARN reason and never mutate state; accepted callbacks run the transition.
  rclcpp_action::GoalResponse handleTakeoffGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Takeoff::Goal> goal);
  rclcpp_action::CancelResponse handleTakeoffCancel(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle);
  void handleTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goal_handle);
  rclcpp_action::GoalResponse handleLandGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Land::Goal> goal);
  // Land is never cancelable once accepted: a committed landing always runs to completion.
  rclcpp_action::CancelResponse handleLandCancel(
    const std::shared_ptr<GoalHandleLand> goal_handle);
  void handleLandAccepted(const std::shared_ptr<GoalHandleLand> goal_handle);
  rclcpp_action::GoalResponse handleReturnHomeGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ReturnHome::Goal> goal);
  rclcpp_action::CancelResponse handleReturnHomeCancel(
    const std::shared_ptr<GoalHandleReturnHome> goal_handle);
  void handleReturnHomeAccepted(const std::shared_ptr<GoalHandleReturnHome> goal_handle);

  // Moves the FSM to `new_state`, logging `reason`; a no-op when already there.
  void transitionTo(DroneState new_state, const char * reason);
  // Enters kTransition (if not already there) at `phase`, restarting phase timing and the PX4
  // command retry timer.
  void enterPhase(Phase phase, const char * reason);
  // kTransition tick: dispatches on `phase_` to the takeoff, landing, or return-home body.
  void handleTransition(const rclcpp::Time & now);
  void handleTakingOff(const rclcpp::Time & now);
  void handleLanding(const rclcpp::Time & now);
  void handleReturnHome();
  void handleMission(const rclcpp::Time & now);

  // Returns whether the last external mission command is still within `command_timeout_s`.
  bool commandFresh(const rclcpp::Time & now) const;
  // Returns whether the vehicle has reached and held the takeoff target for
  // `hover_settle_time_s`, per position/velocity/yaw tolerances in `config_`.
  bool takeoffSettled(const rclcpp::Time & now);
  // Returns whether takeoff yaw has converged; always true when using current-heading takeoff.
  bool takeoffYawSettled(float current_heading, float * yaw_error_rad) const;
  // Returns whether the current phase has been active for at least `timeout_s`.
  bool phaseTimedOut(const rclcpp::Time & now, double timeout_s) const;
  // Returns whether the FSM is in one of the takeoff phases of kTransition.
  bool takingOff() const;
  // Returns whether an in-progress takeoff must end through landing rather than a plain reset
  // to IDLE, because PX4 may already be armed.
  bool shouldRecoverThroughLanding() const;
  // `canX()`/`xRejectReason()` pairs are the single source of truth for command eligibility,
  // shared by the heartbeat's `can_*` capability flags and by the goal/service handlers.
  const char * takeoffRejectReason() const;
  bool canTakeoff() const;
  const char * landRejectReason() const;
  bool canLand() const;
  const char * returnHomeRejectReason() const;
  bool canReturnHome() const;
  const char * startMissionRejectReason() const;
  bool canStartMission() const;
  const char * holdRejectReason() const;
  bool canHold() const;
  // Latches the current PX4 local position as the takeoff origin/target (and home).
  bool latchTakeoffTarget();
  // Applies a Takeoff goal's altitude/yaw onto the just-latched takeoff target.
  bool configureTakeoffTarget(const Takeoff::Goal & goal);
  // Latches the current PX4 local position/heading as the hold target.
  void latchHoverTarget();
  // Like latchHoverTarget(), but keeps the takeoff's target yaw instead of the current heading.
  void latchHoverTargetPreservingTakeoffYaw();
  // Builds the setpoint OffboardHeartbeatPublisher should stream for the current state.
  OffboardHeartbeatPublisher::Setpoint buildOffboardSetpoint() const;
  // MISSION setpoint from the fresh latched command: its horizontal axes in the command's
  // mode, clamped, at the altitude held since entering MISSION.
  OffboardHeartbeatPublisher::Setpoint missionSetpoint() const;
  // Publish feedback for the active goal of each action type (no-op if none is active).
  void publishTakeoffFeedback(float progress);
  void publishLandFeedback(const rclcpp::Time & now);
  void publishReturnHomeFeedback(float distance_m);
  // Enters the land phase, requesting a PX4 position-mode switch before NAV_LAND.
  void startLanding(const char * reason);
  // Aborts the active Takeoff goal with `reason`, then ends the attempt via endTakeoff().
  void failTakeoff(const char * reason);
  // Ends a takeoff attempt whose goal is already resolved: lands if PX4 may be armed, else IDLE.
  void endTakeoff(const char * reason);
  // Resets the PX4 command retry timer to zero, so the next retriable command is sent
  // immediately.
  void resetVehicleCommandRetryTimer();
  // Tracks worst-case tick jitter and counts ticks that ran more than 1.5x the expected period
  // late, for the periodic timing log in maybeLogTiming().
  void updateTimingStats(const rclcpp::Time & now);
  // Logs accumulated jitter/missed-deadline stats at most once every `timing_log_period_s`, then
  // resets the jitter accumulator.
  void maybeLogTiming(const rclcpp::Time & now);

  // Human-readable names for the corresponding enum/PX4 status code, used in logs and the
  // heartbeat message; each returns "UNKNOWN" for an unrecognized value.
  static const char * stateName(DroneState state);
  static const char * phaseName(Phase phase);
  static const char * navStateName(std::uint8_t nav_state);
  static const char * armingStateName(std::uint8_t arming_state);
  // Heartbeat `active_action`: "takeoff", "land" or "return_home" while in kTransition, else "".
  const char * activeActionName() const;

  ControlConfig config_{};
  std::unique_ptr<Px4Interface> px4_;
  std::unique_ptr<OffboardHeartbeatPublisher> offboard_heartbeat_;
  std::unique_ptr<PaddockHeartbeatPublisher> paddock_heartbeat_;
  LatchedCommand latest_command_{};

  DroneState state_{DroneState::kIdle};
  DroneState previous_state_{DroneState::kIdle};
  rclcpp::Time state_enter_time_{};
  rclcpp::Time phase_enter_time_{};
  Phase phase_{Phase::kWarmup};
  rclcpp::Time landing_start_time_{};
  rclcpp::Time last_vehicle_command_time_{};
  rclcpp::Time last_tick_time_{};
  rclcpp::Time last_timing_log_time_{};
  double max_tick_jitter_ms_{0.0};
  std::uint64_t missed_deadline_count_{0};
  bool timing_initialized_{false};
  bool position_mode_requested_for_landing_{false};
  bool landing_timeout_reported_{false};

  // NED targets latched from the real PX4 position before any setpoint is streamed; the -2.0F
  // initializers are arbitrary placeholders, never streamed to PX4 as-is. The takeoff x/y
  // double as home for return home, and hover_* is the hold target (and the return-home
  // carrot while flying home).
  float takeoff_x_ned_m_{0.0F};
  float takeoff_y_ned_m_{0.0F};
  float takeoff_origin_z_ned_m_{0.0F};
  float takeoff_z_ned_m_{-2.0F};
  float hover_x_ned_m_{0.0F};
  float hover_y_ned_m_{0.0F};
  float hover_z_ned_m_{-2.0F};
  float hover_yaw_rad_{0.0F};
  bool home_valid_{false};

  std::string drone_id_{"uav_0"};
  std::uint64_t heartbeat_sequence_{0};
  std::string command_topic_{"command"};

  std::unique_ptr<state_machine_detail::ActionLifecycleCoordinator> action_lifecycle_;
  bool takeoff_arm_requested_{false};
  bool active_takeoff_uses_current_yaw_{true};
  float active_takeoff_target_yaw_rad_{0.0F};
  rclcpp::Time hover_settle_start_time_{};

  rclcpp::Subscription<MissionCommand>::SharedPtr command_sub_;
  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_action_server_;
  rclcpp_action::Server<Land>::SharedPtr land_action_server_;
  rclcpp_action::Server<ReturnHome>::SharedPtr return_home_action_server_;
  rclcpp::Service<Trigger>::SharedPtr start_mission_service_;
  rclcpp::Service<Trigger>::SharedPtr hold_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace mullet_core

#endif  // MULLET_CORE_FLIGHT_DRONE_STATE_MACHINE_HPP_
