#ifndef MULLET_CORE_DRONE_STATE_MACHINE_HPP_
#define MULLET_CORE_DRONE_STATE_MACHINE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "mullet_core/px4_interface.hpp"
#include "mullet_core/types.hpp"

namespace mullet_core
{

class DroneStateMachine : public rclcpp::Node
{
public:
  DroneStateMachine();

private:
  void loadParameters();
  void controlTick();
  void onCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void onLandRequest(const std_msgs::msg::Empty::SharedPtr msg);

  void transitionTo(DroneState new_state, const char * reason);
  void handleInit(const rclcpp::Time & now);
  void handleWarmup(const rclcpp::Time & now);
  void handleRequestOffboard(const rclcpp::Time & now);
  void handleRequestArm(const rclcpp::Time & now);
  void handleTakeoff(const rclcpp::Time & now);
  void handleHover(const rclcpp::Time & now);
  void handleCommand(const rclcpp::Time & now);
  void handleLanding(const rclcpp::Time & now);
  void handleFailsafeHold(const rclcpp::Time & now);

  bool commandFresh(const rclcpp::Time & now) const;
  bool takeoffReached() const;
  void latchTakeoffTarget();
  void latchHoverTarget();
  void publishTakeoffSetpoint();
  void publishHoverSetpoint();
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

  float takeoff_x_ned_m_{0.0F};
  float takeoff_y_ned_m_{0.0F};
  float takeoff_z_ned_m_{-2.0F};
  float hover_x_ned_m_{0.0F};
  float hover_y_ned_m_{0.0F};
  float hover_z_ned_m_{-2.0F};
  float hover_yaw_rad_{0.0F};

  std::string command_topic_{"command"};
  std::string land_topic_{"land_request"};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr land_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace mullet_core

#endif  // MULLET_CORE_DRONE_STATE_MACHINE_HPP_
