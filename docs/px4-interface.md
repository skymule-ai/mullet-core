# PX4 Interface

`Px4Interface` owns PX4 topic naming, QoS, telemetry caching, and vehicle command
publication. The state machine asks for higher-level operations such as arm,
Offboard mode, Position mode, and land.

## Topics

PX4 topics use best-effort, transient-local QoS with keep-last depth `1`.

| Direction | Topic for instance `0` | Topic for instance `N > 0` |
| --- | --- | --- |
| publish | `/fmu/in/offboard_control_mode` | `/px4_N/fmu/in/offboard_control_mode` |
| publish | `/fmu/in/trajectory_setpoint` | `/px4_N/fmu/in/trajectory_setpoint` |
| publish | `/fmu/in/vehicle_command` | `/px4_N/fmu/in/vehicle_command` |
| subscribe | `/fmu/out/vehicle_local_position` | `/px4_N/fmu/out/vehicle_local_position` |
| subscribe | `/fmu/out/vehicle_status_v1` | `/px4_N/fmu/out/vehicle_status_v1` |
| subscribe | `/fmu/out/vehicle_land_detected` | `/px4_N/fmu/out/vehicle_land_detected` |

## Vehicle Commands

| Operation | PX4 command |
| --- | --- |
| Arm | `VEHICLE_CMD_COMPONENT_ARM_DISARM`, `param1=1` |
| Disarm | `VEHICLE_CMD_COMPONENT_ARM_DISARM`, `param1=0` |
| Offboard mode | `VEHICLE_CMD_DO_SET_MODE`, `param1=1`, `param2=6` |
| Position mode | `VEHICLE_CMD_DO_SET_MODE`, `param1=1`, `param2=3` |
| Land | `VEHICLE_CMD_NAV_LAND` |

`target_system` is `vehicle_instance + 1`. Valid PX4 instances are `[0, 254]`,
which map to PX4 system IDs `[1, 255]` without unsigned wraparound.

## Setpoints

Position control publishes `OffboardControlMode(position=true)` and a
`TrajectorySetpoint` with finite position and yaw. Velocity control publishes
`OffboardControlMode(velocity=true)` and a `TrajectorySetpoint` with finite NED
velocity and yaw. Unused trajectory fields are set to NaN.

## External Command Stream

The compatibility command topic uses:

```text
std_msgs/msg/Float32MultiArray [vx_ned_mps, vy_ned_mps, mode]
```

Only `mode=1` is accepted. Non-finite values are rejected before entering the
command buffer. Accepted commands are interpreted in the PX4 NED world frame and
clamped by `max_vx_ned_mps`, `max_vy_ned_mps`, and `max_vz_ned_mps`.
