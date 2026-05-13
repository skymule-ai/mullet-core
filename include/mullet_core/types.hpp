#ifndef MULLET_CORE_TYPES_HPP_
#define MULLET_CORE_TYPES_HPP_

#include <cstdint>

#include <rclcpp/rclcpp.hpp>

namespace mullet_core
{

enum class DroneState : std::uint8_t
{
  kInit,
  kWarmupSetpoints,
  kRequestOffboard,
  kRequestArm,
  kTakeoff,
  kHover,
  kCommand,
  kLanding,
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
  double control_rate_hz{50.0};
  double offboard_warmup_s{1.0};
  double command_retry_interval_s{1.0};
  double command_timeout_s{0.25};
  double takeoff_altitude_m{2.0};
  double takeoff_acceptance_m{0.25};
  double hover_acceptance_radius_m{0.25};
  double hover_yaw_rad{0.0};
  double max_vx_ned_mps{1.0};
  double max_vy_ned_mps{1.0};
  double max_vz_ned_mps{0.5};
  double timing_log_period_s{1.0};
};

}  // namespace mullet_core

#endif  // MULLET_CORE_TYPES_HPP_
