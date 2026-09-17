#ifndef MULLET_CORE_FLIGHT_TYPES_HPP_
#define MULLET_CORE_FLIGHT_TYPES_HPP_

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace mullet_core
{

// States managed by DroneStateMachine. Multi-step work (takeoff, land, return home) all runs
// inside kTransition; which one is decided by the private DroneStateMachine::Phase.
enum class DroneState : std::uint8_t
{
  kIdle,        // On the ground, or otherwise not under supervisor control.
  kTransition,  // Takeoff, land, or return home in progress.
  kHold,        // Airborne, holding a latched position.
  kMission,     // Airborne, following the external mission command stream.
};

// Latest valid mullet_interfaces/msg/MissionCommand, in NED. `mode` is one of its mode
// constants; z is NaN in the planar modes, the position is read only in the TRAJECTORY modes,
// the acceleration is NaN when a command has no feedforward, and the yaw is NaN when the
// command leaves heading to mullet_core.
struct LatchedCommand
{
  std::uint8_t mode{0};
  float x_ned_m{0.0F};
  float y_ned_m{0.0F};
  float z_ned_m{0.0F};
  float vx_ned_mps{0.0F};
  float vy_ned_mps{0.0F};
  float vz_ned_mps{0.0F};
  float ax_ned_mps2{0.0F};
  float ay_ned_mps2{0.0F};
  float az_ned_mps2{0.0F};
  float yaw_rad{0.0F};
  rclcpp::Time stamp{};
  bool valid{false};
};

// Parameters controlling state-machine timing, limits, and PX4 addressing. Populated by
// DroneStateMachine::loadParameters() from ROS parameters, each clamped to a safe minimum.
struct ControlConfig
{
  int vehicle_instance{0};                     // PX4 target system = this + 1; see vehicle_instance.hpp.
  std::string px4_namespace_prefix{"px4"};      // FMU topic namespace prefix for instance > 0.
  std::string drone_id{};                       // Heartbeat drone_id; defaults to "uav_<instance>".
  std::string heartbeat_topic{"/mullet/heartbeat"};
  double control_rate_hz{50.0};                 // controlTick() timer rate; clamped to >= 2 Hz.
  double heartbeat_rate_hz{10.0};                // Paddock heartbeat publish rate; clamped to >= 1 Hz.
  double offboard_warmup_s{1.0};                // Setpoints streamed before requesting OFFBOARD.
  double command_retry_interval_s{1.0};         // Minimum gap between repeated PX4 VehicleCommand retries.
  double command_timeout_s{0.25};               // Max age before an external mission command goes stale.
  double command_loss_hold_s{2.0};              // Stale MISSION stream this long drops to HOLD.
  double takeoff_acceptance_m{0.25};            // Altitude tolerance combined with hover tolerance for takeoffSettled().
  double takeoff_yaw_tolerance_rad{0.174533};   // ~10 degrees; max yaw error for a fixed-yaw takeoff to settle.
  double hover_yaw_rad{0.0};                    // Fallback hold yaw if current heading is non-finite.
  double max_vx_ned_mps{1.0};                   // Clamp applied to MISSION vx commands.
  double max_vy_ned_mps{1.0};                   // Clamp applied to MISSION vy commands.
  double max_vz_ned_mps{0.5};                   // Clamp applied to MISSION vz commands (3D modes).
  double max_accel_ned_mps2{3.0};               // Per-axis clamp applied to MISSION acceleration commands.
  double max_xy_position_lead_m{1.0};           // Farthest a TRAJECTORY position may lead the drone horizontally.
  double max_z_position_lead_m{0.5};            // Farthest a TRAJECTORY_3D position may lead the drone vertically.
  double timing_log_period_s{1.0};              // Interval between periodic tick-jitter log lines.
  double min_takeoff_altitude_m{0.5};           // Lower bound enforced on Takeoff goal altitude_m.
  double max_takeoff_altitude_m{10.0};          // Upper bound enforced on Takeoff goal altitude_m.
  double request_offboard_timeout_s{5.0};       // Timeout waiting for PX4 to enter OFFBOARD mode.
  double request_arm_timeout_s{5.0};            // Timeout waiting for PX4 to report armed.
  double takeoff_timeout_s{20.0};               // Timeout for the ascent phase to settle.
  double landing_timeout_s{30.0};               // Time armed while landing before the Land goal aborts.
  double hover_altitude_tolerance_m{0.25};      // Altitude error tolerance for takeoff settling.
  double hover_vertical_velocity_tolerance_mps{0.25};  // Vertical speed tolerance for takeoff settling.
  double hover_settle_time_s{1.0};              // Time within tolerance required before takeoff is declared settled.
  double return_home_speed_mps{2.0};            // Horizontal speed of the return-home setpoint.
  double return_home_acceptance_m{0.5};         // Horizontal distance from home that ends return home.
};

}  // namespace mullet_core

#endif  // MULLET_CORE_FLIGHT_TYPES_HPP_
