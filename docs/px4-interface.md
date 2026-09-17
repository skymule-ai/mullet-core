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
| subscribe | `/fmu/out/vehicle_local_position_v1` | `/<prefix>_N/fmu/out/vehicle_local_position_v1` |
| subscribe | `/fmu/out/vehicle_status_v1` | `/<prefix>_N/fmu/out/vehicle_status_v1` |
| subscribe | `/fmu/out/vehicle_land_detected` | `/<prefix>_N/fmu/out/vehicle_land_detected` |

PX4 names the topic of a message whose `MESSAGE_VERSION` is above 0 `<topic>_v<version>`.
The node builds every name from the px4_msgs constant (`px4Topic(prefix, suffix, version)`), so the
names follow the px4_msgs branch the build uses, which must match the firmware. The table shows PX4
1.17 (`px4_msgs` `release/1.17`), where `VehicleLocalPosition` became v1; on 1.16 it was
`vehicle_local_position`.

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

Every tick publishes a `TrajectorySetpoint` and an `OffboardControlMode` that
enables position, velocity and acceleration control for each field with a finite
component. Unset components are NaN, so axes can mix:

| State | position | velocity | acceleration |
| --- | --- | --- | --- |
| hold, takeoff, return home | x, y, z | NaN | NaN |
| `MISSION`, `TRAJECTORY` | x, y, z (held) | vx, vy, NaN | ax, ay, NaN or NaN |
| `MISSION`, `VELOCITY` | NaN, NaN, z | vx, vy, NaN | ax, ay, NaN or NaN |
| `MISSION`, `ACCELERATION` | NaN, NaN, z | NaN | ax, ay, NaN |
| `MISSION`, `TRAJECTORY_3D` | x, y, z | vx, vy, vz | ax, ay, az or NaN |
| `MISSION`, `VELOCITY_3D` | NaN | vx, vy, vz | ax, ay, az or NaN |
| `MISSION`, `ACCELERATION_3D` | NaN | NaN | ax, ay, az |

## External Command Stream

`MISSION` streams `mullet_interfaces/msg/MissionCommand` from `command_topic`.
Invalid commands (unset mode, finite z in a planar mode, or non-finite fields the
mode needs) are rejected before entering the latest command. Accepted commands are
interpreted in the PX4 NED world frame; velocities are clamped by `max_vx_ned_mps`,
`max_vy_ned_mps` and `max_vz_ned_mps`, accelerations per axis by
`max_accel_ned_mps2`. In the `TRAJECTORY` modes the position is clamped to lead
the drone by at most `max_xy_position_lead_m` horizontally and `max_z_position_lead_m`
vertically, measured from the drone's current position. In the planar modes the z position is the altitude latched on
entering `MISSION`; in the 3D modes the command drives z, and a stale stream holds
the altitude the drone has at that moment.
