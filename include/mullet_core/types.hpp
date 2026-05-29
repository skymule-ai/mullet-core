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

struct PositionCommand
{
  float x_ned_m{0.0F};
  float y_ned_m{0.0F};
  float z_ned_m{0.0F};
  float yaw_rad{0.0F};
  rclcpp::Time stamp{};
  bool valid{false};
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
  PositionCommand latest_position{};
  VelocityCommand latest_velocity{};
  bool land_requested{false};
};

struct ControlConfig
{
  int vehicle_instance{0};
  std::string px4_namespace_prefix{"px4"};
  double control_rate_hz{50.0};
  double offboard_warmup_s{1.0};
  double command_retry_interval_s{1.0};
  double command_timeout_s{0.25};
  double takeoff_altitude_m{2.0};
  double takeoff_acceptance_m{0.25};
  double takeoff_yaw_tolerance_rad{0.174533};
  double hover_acceptance_radius_m{0.25};
  double hover_yaw_rad{0.0};
  double max_vx_ned_mps{1.0};
  double max_vy_ned_mps{1.0};
  double max_vz_ned_mps{0.5};
  double timing_log_period_s{1.0};
  double min_takeoff_altitude_m{0.5};
  double max_takeoff_altitude_m{10.0};
  double warmup_timeout_s{3.0};
  double request_offboard_timeout_s{5.0};
  double request_arm_timeout_s{5.0};
  double takeoff_timeout_s{20.0};
  double landing_timeout_s{30.0};
  double hover_altitude_tolerance_m{0.25};
  double hover_vertical_velocity_tolerance_mps{0.25};
  double hover_settle_time_s{1.0};
};

}  // namespace mullet_core

#endif  // MULLET_CORE_TYPES_HPP_
