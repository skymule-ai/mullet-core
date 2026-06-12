#ifndef MULLET_CORE_TYPES_HPP_
#define MULLET_CORE_TYPES_HPP_

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace mullet_core
{

enum class DroneState : std::uint8_t
{
  kInit,
  kReady,
  kEnableOffboardRequested,
  kWarmupOffboardHoldSetpoints,
  kRequestOffboardOnly,
  kArmRequested,
  kArmCommand,
  kArmedHold,
  kDisarmCommand,
  kReleaseOffboardRequest,
  kDisarmedOffboardReleaseFailed,
  kDisableOffboardRequested,
  kTakeoffRequested,
  kWarmupSetpoints,
  kRequestOffboard,
  kRequestArm,
  kTakeoff,
  kHover,
  kCommand,
  kCancelingToLand,
  kLanding,
  kTakeoffFailed,
  kTakeoffCanceled,
  kFailsafeHold,
  kDisarmed,
};

struct VelocityCommand
{
  float vx_ned_mps{0.0F};
  float vy_ned_mps{0.0F};
  float vz_ned_mps{0.0F};
  rclcpp::Time stamp{};
  bool valid{false};
};

struct ExternalCommandBuffer
{
  VelocityCommand latest_velocity{};
  bool land_requested{false};
};

struct ControlConfig
{
  int vehicle_instance{0};
  std::string px4_namespace_prefix{"px4"};
  std::string drone_id{};
  std::string runtime_mode{"unknown"};
  std::string heartbeat_topic{"/mullet/heartbeat"};
  double control_rate_hz{50.0};
  double heartbeat_rate_hz{10.0};
  double offboard_warmup_s{1.0};
  double command_retry_interval_s{1.0};
  double command_timeout_s{0.25};
  double takeoff_acceptance_m{0.25};
  double takeoff_yaw_tolerance_rad{0.174533};
  double hover_yaw_rad{0.0};
  double max_vx_ned_mps{1.0};
  double max_vy_ned_mps{1.0};
  double max_vz_ned_mps{0.5};
  double timing_log_period_s{1.0};
  double min_takeoff_altitude_m{0.5};
  double max_takeoff_altitude_m{10.0};
  double warmup_timeout_s{3.0};
  double request_offboard_timeout_s{5.0};
  double disable_offboard_timeout_s{5.0};
  double request_arm_timeout_s{5.0};
  double armed_hold_duration_s{5.0};
  double request_disarm_timeout_s{5.0};
  double request_position_mode_timeout_s{5.0};
  double landed_confirmation_s{0.5};
  double takeoff_timeout_s{20.0};
  double landing_timeout_s{30.0};
  double hover_altitude_tolerance_m{0.25};
  double hover_vertical_velocity_tolerance_mps{0.25};
  double hover_settle_time_s{1.0};
};

}  // namespace mullet_core

#endif  // MULLET_CORE_TYPES_HPP_
