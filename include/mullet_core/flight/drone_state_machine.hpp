#pragma once

#include <array>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mullet_core/flight/config.hpp"
#include "mullet_core/flight/detail/action_server.hpp"
#include "mullet_core/flight/detail/tick_stats.hpp"
#include "mullet_core/flight/flight_states.hpp"
#include "mullet_core/flight/mission_command.hpp"
#include "mullet_core/px4/offboard_heartbeat_publisher.hpp"
#include "mullet_core/px4/px4_interface.hpp"
#include "mullet_core/telemetry/localization_status_publisher.hpp"
#include "mullet_core/telemetry/paddock_heartbeat_publisher.hpp"
#include "mullet_interfaces/action/land.hpp"
#include "mullet_interfaces/action/return_home.hpp"
#include "mullet_interfaces/action/takeoff.hpp"
#include "mullet_interfaces/msg/drone_heartbeat.hpp"
#include "mullet_interfaces/msg/mission_command.hpp"

namespace mullet_core
{

// Supervises PX4 offboard flight through ROS actions, services, telemetry, and commands.
class DroneStateMachine : public rclcpp::Node
{
public:
  explicit DroneStateMachine(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DroneStateMachine() override;

private:
  using Land = mullet_interfaces::action::Land;
  using ReturnHome = mullet_interfaces::action::ReturnHome;
  using Takeoff = mullet_interfaces::action::Takeoff;
  using Trigger = std_srvs::srv::Trigger;
  using MissionCommand = mullet_interfaces::msg::MissionCommand;

  // Returned by ticks and applied outside std::visit, so the visited alternative is never
  // replaced under the reference a tick holds.
  struct Transition { FlightState next; const char * reason; };

  void controlTick();
  // PX4 outranks the supervisor: external disarm -> IDLE, offboard lost while airborne -> HOLD.
  void handleExternalOverride();
  void handleActionCancellations();
  mullet_interfaces::msg::DroneHeartbeat buildHeartbeatMessage();
  void onCommand(const MissionCommand::SharedPtr msg);
  void onStartMission(Trigger::Response & response);
  void onHold(Trigger::Response & response);
  const char * takeoffGoalRejectReason(const Takeoff::Goal & goal) const;
  void onTakeoffAccepted(const Takeoff::Goal & goal);
  void onLandAccepted(const Land::Goal & goal);
  void onReturnHomeAccepted(const ReturnHome::Goal & goal);

  // Single source of truth for command eligibility: the heartbeat's can_* flags and the
  // goal/service handlers both read these pairs.
  const char * takeoffRejectReason() const; bool canTakeoff() const;
  const char * landRejectReason() const; bool canLand() const;
  const char * returnHomeRejectReason() const; bool canReturnHome() const;
  const char * startMissionRejectReason() const; bool canStartMission() const;
  const char * holdRejectReason() const; bool canHold() const;

  // Whether the takeoff target has been held for hover_settle_time_s within tolerance.
  bool takeoffSettled(TakingOff & s, const rclcpp::Time & now);
  // TakingOff for an accepted goal at the current position; nullopt (logged) if it is unusable.
  std::optional<TakingOff> makeTakeoff(const Takeoff::Goal & goal) const;
  // `fallback` with every field PX4 reports valid replaced by the live value, so a GPS dropout
  // holds the last good target instead of the EKF origin.
  Pose currentPose(Pose fallback) const;
  OffboardHeartbeatPublisher::Setpoint buildOffboardSetpoint() const;
  OffboardHeartbeatPublisher::Setpoint missionSetpoint(const Mission & mission) const;
  void publishTakeoffFeedback(const TakingOff & s, float progress);
  void publishLandFeedback(const Landing & s, const rclcpp::Time & now);
  void publishReturnHomeFeedback(float distance_m);
  // Aborts the active Takeoff goal, then returns endTakeoff()'s transition.
  Transition failTakeoff(const TakingOff & takeoff, const char * reason);
  // Landing if PX4 may be armed, else Idle; returned, not applied, so it is safe inside a tick.
  Transition endTakeoff(const TakingOff & takeoff, const char * reason);

  // Logs and updates previous_state_ only when the wire state changes. Never call inside
  // std::visit over state_: return a Transition instead.
  void transitionTo(FlightState next, const char * reason);

  // Per-state tick: mutates the alternative in place and returns the transition to apply.
  std::optional<Transition> tick(Idle &, const rclcpp::Time &) { return std::nullopt; }
  std::optional<Transition> tick(TakingOff & s, const rclcpp::Time & t);
  std::optional<Transition> tick(Landing & s, const rclcpp::Time & t);
  std::optional<Transition> tick(ReturningHome & s, const rclcpp::Time & t);
  std::optional<Transition> tick(Hold &, const rclcpp::Time &) { return std::nullopt; }
  std::optional<Transition> tick(Mission & s, const rclcpp::Time & t);

  ControlConfig config_{};
  std::unique_ptr<Px4Interface> px4_;
  std::unique_ptr<OffboardHeartbeatPublisher> offboard_heartbeat_;
  std::unique_ptr<PaddockHeartbeatPublisher> paddock_heartbeat_;
  std::unique_ptr<LocalizationStatusPublisher> localization_status_;
  std::optional<LatchedCommand> latest_command_{};
  state_machine_detail::TickStats tick_stats_;
  FlightState state_{Idle{}};
  DroneState previous_state_{DroneState::kIdle};
  // NED x/y of the last accepted takeoff; return home flies here at the latched altitude/yaw.
  std::optional<std::array<float, 2>> home_{};
  std::uint64_t heartbeat_sequence_{0};

  std::unique_ptr<state_machine_detail::ActionServer<Takeoff>> takeoff_action_;
  std::unique_ptr<state_machine_detail::ActionServer<Land>> land_action_;
  std::unique_ptr<state_machine_detail::ActionServer<ReturnHome>> return_home_action_;
  rclcpp::Subscription<MissionCommand>::SharedPtr command_sub_;
  rclcpp::Service<Trigger>::SharedPtr start_mission_service_;
  rclcpp::Service<Trigger>::SharedPtr hold_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace mullet_core
