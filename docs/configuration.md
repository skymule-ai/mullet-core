# Configuration

Configuration is loaded through ROS 2 parameters. The launch file loads
`mullet_core_params_<instance>.yaml` when present and falls back to
`mullet_core_params_default.yaml`.

## Launch Arguments

| Argument | Default | Description |
| --- | --- | --- |
| `px4_instances` | `0` | Comma-separated explicit PX4 instances. Overrides `num_robots` and `px4_instance_offset` when non-empty. |
| `num_robots` | `1` | Number of nodes to spawn when `px4_instances` is empty. |
| `px4_instance_offset` | `0` | First PX4 instance when `px4_instances` is empty. |
| `config_dir` | package `config/` | Directory containing parameter YAML files. |

Instance values must be unique and in `[0, 254]`.

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `vehicle_instance` | `0` | PX4 instance ID. Instance `0` uses `/fmu/...`; instance `N` uses `/px4_N/fmu/...`. |
| `control_rate_hz` | `50.0` | State machine and setpoint publish rate. Clamped to at least `2.0`. |
| `offboard_warmup_s` | `1.0` | Proof-of-life warmup before requesting Offboard. Clamped to at least `1.0`. |
| `command_retry_interval_s` | `1.0` | Retry interval for PX4 vehicle commands. Clamped to at least `0.1`. |
| `command_timeout_s` | `0.25` | Freshness window for external velocity commands. |
| `takeoff_acceptance_m` | `0.25` | Takeoff altitude acceptance used with hover altitude tolerance. |
| `takeoff_yaw_tolerance_rad` | `0.174533` | Yaw error tolerance for explicit-yaw takeoff completion. |
| `hover_yaw_rad` | `0.0` | Fallback hover yaw when current heading is unavailable. |
| `max_vx_ned_mps` | `1.0` | External command clamp for NED X velocity. |
| `max_vy_ned_mps` | `1.0` | External command clamp for NED Y velocity. |
| `max_vz_ned_mps` | `0.5` | External command clamp for NED Z velocity. |
| `timing_log_period_s` | `1.0` | Timing diagnostic log period. |
| `min_takeoff_altitude_m` | `0.5` | Minimum accepted action takeoff altitude. |
| `max_takeoff_altitude_m` | `10.0` | Maximum accepted action takeoff altitude. |
| `warmup_timeout_s` | `3.0` | Timeout for Offboard proof-of-life warmup phases. |
| `request_offboard_timeout_s` | `5.0` | Timeout while waiting for PX4 to report Offboard. |
| `disable_offboard_timeout_s` | `5.0` | Timeout while waiting for PX4 to leave Offboard during disable. |
| `request_arm_timeout_s` | `5.0` | Timeout while waiting for PX4 to report armed. |
| `armed_hold_duration_s` | `5.0` | Time this node holds armed-on-ground Offboard before requesting disarm. PX4 may disarm earlier via native PX4 parameters. |
| `request_disarm_timeout_s` | `5.0` | Timeout while waiting for PX4 to report disarmed after a direct ground disarm request. |
| `request_position_mode_timeout_s` | `5.0` | Timeout while waiting for PX4 to leave Offboard after ground disarm. |
| `landed_confirmation_s` | `0.5` | Required continuous `vehicle_land_detected.landed=true` duration before direct disarm or ground arm. |
| `takeoff_timeout_s` | `20.0` | Timeout while waiting for takeoff completion. |
| `landing_timeout_s` | `30.0` | User-facing land action timeout. Landing recovery continues afterward. |
| `hover_altitude_tolerance_m` | `0.25` | Hover/takeoff altitude settle tolerance. |
| `hover_vertical_velocity_tolerance_mps` | `0.25` | Vertical velocity settle tolerance. |
| `hover_settle_time_s` | `1.0` | Duration altitude, vertical velocity, and yaw must remain settled. |
| `command_topic` | `command` | External velocity command topic. |
| `land_topic` | `land_request` | Emergency land request topic. |

## Deprecated Compatibility Parameters

The following parameters are still accepted to avoid breaking existing launch
files, but they are not part of the active behavior:

| Parameter | Compatibility default | Replacement |
| --- | --- | --- |
| `takeoff_altitude_m` | `2.0` | Use the `Takeoff` action goal `altitude_m`. |
| `hover_acceptance_radius_m` | `0.25` | Use `hover_altitude_tolerance_m` and command freshness rules. |

Do not add these deprecated parameters to new YAML files.
