# PX4 Interface

`Px4Interface` owns PX4 topic naming, QoS, telemetry caching, and vehicle command
publication. The state machine asks for higher-level operations such as arm,
Offboard mode, Position mode, and land.

`Px4Interface` retains the latest local-position, vehicle-status, and land-detected
messages. `isFailsafe()` maps PX4's native `VehicleStatus.failsafe` flag; it is
reported on the heartbeat without driving state transitions. The heartbeat's
lat/lon/alt are derived from `VehicleLocalPosition`'s EKF origin
(`ref_lat`/`ref_lon`/`ref_alt`, valid when `xy_global` and `z_global`), so there is
no global-position subscription. The dedicated Offboard heartbeat publisher owns
fixed-rate setpoint streaming.

## Topics

PX4 topics use best-effort, transient-local QoS with keep-last depth `1`.

| Direction | Topic for instance `0` | Topic for instance `N > 0` (`<prefix>` = `px4_namespace_prefix`, `uav` by default) |
| --- | --- | --- |
| publish | `/fmu/in/offboard_control_mode` | `/<prefix>_N/fmu/in/offboard_control_mode` |
| publish | `/fmu/in/trajectory_setpoint` | `/<prefix>_N/fmu/in/trajectory_setpoint` |
| publish | `/fmu/in/vehicle_command` | `/<prefix>_N/fmu/in/vehicle_command` |
| subscribe | `/fmu/out/vehicle_local_position` | `/<prefix>_N/fmu/out/vehicle_local_position` |
| subscribe | `/fmu/out/vehicle_status_v1` | `/<prefix>_N/fmu/out/vehicle_status_v1` |
| subscribe | `/fmu/out/vehicle_land_detected` | `/<prefix>_N/fmu/out/vehicle_land_detected` |

## Vehicle Commands

| Operation | PX4 command |
| --- | --- |
| Arm | `VEHICLE_CMD_COMPONENT_ARM_DISARM`, `param1=1` |
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

`MISSION` streams the compatibility command topic:

```text
std_msgs/msg/Float32MultiArray [vx_ned_mps, vy_ned_mps, mode]
```

Only `mode=1` is accepted. Non-finite values are rejected before entering the
latest command. Accepted commands are interpreted in the PX4 NED world frame,
clamped by `max_vx_ned_mps` and `max_vy_ned_mps`, and streamed with zero vertical
velocity.
