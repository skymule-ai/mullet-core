#include "mullet_core/flight/config.hpp"

#include <rclcpp/rclcpp.hpp>

namespace mullet_core
{

ControlConfig loadConfig(rclcpp::Node & node)
{
  ControlConfig config;

  const auto declare_and_clamp = [&node](const std::string & name, double & value, double min) {
      value = node.declare_parameter<double>(name, value);
      if (value < min) {
        RCLCPP_WARN(
          node.get_logger(), "Parameter '%s' = %.3f below minimum %.3f; clamping",
          name.c_str(), value, min);
        value = min;
      }
    };

  config.vehicle_instance = node.declare_parameter<int>("vehicle_instance", config.vehicle_instance);
  config.px4_namespace_prefix = node.declare_parameter<std::string>(
    "px4_namespace_prefix", config.px4_namespace_prefix);
  declare_and_clamp("control_rate_hz", config.control_rate_hz, 2.0);
  declare_and_clamp("heartbeat_rate_hz", config.heartbeat_rate_hz, 1.0);
  config.heartbeat_topic = node.declare_parameter<std::string>("heartbeat_topic", config.heartbeat_topic);
  config.command_topic = node.declare_parameter<std::string>("command_topic", config.command_topic);
  config.drone_id = node.declare_parameter<std::string>("drone_id", config.drone_id);
  declare_and_clamp("offboard_warmup_s", config.offboard_warmup_s, 1.0);
  declare_and_clamp("command_retry_interval_s", config.command_retry_interval_s, 0.1);
  declare_and_clamp("command_timeout_s", config.command_timeout_s, 0.0);
  declare_and_clamp("command_loss_hold_s", config.command_loss_hold_s, 0.0);
  declare_and_clamp("takeoff_acceptance_m", config.takeoff_acceptance_m, 0.05);
  declare_and_clamp("takeoff_yaw_tolerance_rad", config.takeoff_yaw_tolerance_rad, 0.0);
  config.hover_yaw_rad = node.declare_parameter<double>("hover_yaw_rad", config.hover_yaw_rad);
  declare_and_clamp("max_vx_ned_mps", config.max_vx_ned_mps, 0.0);
  declare_and_clamp("max_vy_ned_mps", config.max_vy_ned_mps, 0.0);
  declare_and_clamp("max_vz_ned_mps", config.max_vz_ned_mps, 0.0);
  declare_and_clamp("max_accel_ned_mps2", config.max_accel_ned_mps2, 0.0);
  declare_and_clamp("max_xy_position_lead_m", config.max_xy_position_lead_m, 0.0);
  declare_and_clamp("max_z_position_lead_m", config.max_z_position_lead_m, 0.0);
  declare_and_clamp("timing_log_period_s", config.timing_log_period_s, 0.25);
  declare_and_clamp("min_takeoff_altitude_m", config.min_takeoff_altitude_m, 0.1);
  declare_and_clamp(
    "max_takeoff_altitude_m", config.max_takeoff_altitude_m,
    config.min_takeoff_altitude_m);
  declare_and_clamp("request_offboard_timeout_s", config.request_offboard_timeout_s, 0.1);
  declare_and_clamp("request_arm_timeout_s", config.request_arm_timeout_s, 0.1);
  declare_and_clamp("takeoff_timeout_s", config.takeoff_timeout_s, 0.1);
  declare_and_clamp("landing_timeout_s", config.landing_timeout_s, 0.1);
  declare_and_clamp("hover_altitude_tolerance_m", config.hover_altitude_tolerance_m, 0.05);
  declare_and_clamp(
    "hover_vertical_velocity_tolerance_mps",
    config.hover_vertical_velocity_tolerance_mps, 0.01);
  declare_and_clamp("hover_settle_time_s", config.hover_settle_time_s, 0.0);
  declare_and_clamp("return_home_speed_mps", config.return_home_speed_mps, 0.1);
  declare_and_clamp("return_home_acceptance_m", config.return_home_acceptance_m, 0.05);

  config.drone_id = config.drone_id.empty() ?
    "uav_" + std::to_string(config.vehicle_instance) : config.drone_id;

  return config;
}

}  // namespace mullet_core
