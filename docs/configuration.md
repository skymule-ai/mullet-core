# Configuration

Configuration is loaded only from the packaged
`config/mullet_core_params.yaml`. The launch file has no arguments or
parameter overrides. Configure each drone's copy of this YAML before launch;
run separate `mullet_core` deployments for multiple drones.

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `vehicle_instance` | `0` | PX4 instance ID. Instance `0` uses `/fmu/...`; instance `N` uses `/<px4_namespace_prefix>_N/fmu/...`. |
| `px4_namespace_prefix` | `uav` | FMU topic namespace prefix, also the node namespace for instance `N > 0`. |
| `drone_id` | `""` | Heartbeat `drone_id`; empty means `uav_<vehicle_instance>`. |
| `heartbeat_topic` | `/mullet/heartbeat` | Paddock heartbeat topic. |
| `heartbeat_rate_hz` | `10.0` | Heartbeat publish rate. Clamped to at least `1.0`. |
| `control_rate_hz` | `50.0` | State machine and setpoint publish rate. Clamped to at least `2.0`. |
| `offboard_warmup_s` | `1.0` | Proof-of-life warmup before requesting Offboard. Clamped to at least `1.0`. |
| `command_retry_interval_s` | `1.0` | Retry interval for PX4 vehicle commands. Clamped to at least `0.1`. |
| `command_timeout_s` | `0.25` | Freshness window for `MISSION` commands. |
| `command_loss_hold_s` | `2.0` | A `MISSION` stream stale this long (counted from the last command or entering `MISSION`) drops to `HOLD`. |
| `takeoff_acceptance_m` | `0.25` | Takeoff altitude acceptance used with hover altitude tolerance. |
| `takeoff_yaw_tolerance_rad` | `0.174533` | Yaw error tolerance for explicit-yaw takeoff completion. |
| `hover_yaw_rad` | `0.0` | Fallback hold yaw when current heading is unavailable. |
| `max_vx_ned_mps` | `1.0` (YAML ships `2.0`) | `MISSION` command clamp for NED X velocity. Clamped to at least `0`. |
| `max_vy_ned_mps` | `1.0` (YAML ships `2.0`) | `MISSION` command clamp for NED Y velocity. Clamped to at least `0`. |
| `max_vz_ned_mps` | `0.5` (YAML ships `0.75`) | `MISSION` command clamp for NED Z velocity (3D modes). Clamped to at least `0`. |
| `max_accel_ned_mps2` | `3.0` | `MISSION` per-axis clamp for NED acceleration. Clamped to at least `0`. |
| `max_xy_position_lead_m` | `1.0` | `TRAJECTORY` modes: farthest the position setpoint may lead the drone horizontally. Clamped to at least `0`. |
| `max_z_position_lead_m` | `0.5` | `TRAJECTORY_3D`: farthest the position setpoint may lead the drone vertically. Clamped to at least `0`. |
| `timing_log_period_s` | `1.0` | Timing diagnostic log period. |
| `min_takeoff_altitude_m` | `0.5` | Minimum accepted takeoff altitude. |
| `max_takeoff_altitude_m` | `10.0` | Maximum accepted takeoff altitude. |
| `request_offboard_timeout_s` | `5.0` | Timeout while waiting for PX4 to report Offboard during takeoff. |
| `request_arm_timeout_s` | `5.0` | Timeout while waiting for PX4 to report armed during takeoff. |
| `takeoff_timeout_s` | `20.0` | Timeout for the ascent phase to settle. |
| `landing_timeout_s` | `30.0` | User-facing land action timeout. Landing retries continue afterward. |
| `hover_altitude_tolerance_m` | `0.25` | Takeoff altitude settle tolerance. |
| `hover_vertical_velocity_tolerance_mps` | `0.25` | Takeoff vertical velocity settle tolerance. |
| `hover_settle_time_s` | `1.0` | Duration altitude, vertical velocity, and yaw must remain settled. |
| `return_home_speed_mps` | `2.0` | Horizontal speed of the return-home setpoint. Tune per airframe. |
| `return_home_acceptance_m` | `0.5` | Horizontal distance from home that ends return home. |
| `command_topic` | `command` | `MISSION` command topic, `mullet_interfaces/msg/MissionCommand`. |
