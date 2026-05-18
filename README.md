# mullet_core

`mullet_core` is a C++ ROS 2 package for drone-only PX4 offboard supervision. The package folder is `mullet-core`, but the ROS 2 package name is `mullet_core`.

## Control Boundary

PX4 remains the hard real-time flight controller. It owns attitude stabilization, motor mixing, inner-loop control, and emergency failsafe execution.

`mullet_core` is a deterministic soft real-time offboard supervisor. It owns explicit takeoff action handling, continuous offboard proof-of-life publication after takeoff is requested, setpoint streaming, mode/arm/land transitions, external command validation, and stale-command fallback behavior.

High-level planners, mission generation, optimization, and expensive diagnostics should run in separate nodes, not inside this control loop.

## Runtime Architecture

```text
mullet_core_node
├── Px4Interface
│   ├── publishes OffboardControlMode, TrajectorySetpoint, VehicleCommand
│   ├── subscribes VehicleLocalPosition, VehicleStatus
│   └── handles PX4 topic prefix and target_system
├── DroneStateMachine
│   ├── fixed-rate control timer
│   ├── state transitions and guards
│   └── command timeout and velocity clamps
└── ExternalCommandBuffer
    ├── latest velocity command
    ├── land request flag
    └── timestamps / validity flags
```

## State Flow

```text
INIT
READY
TAKEOFF_REQUESTED
WARMUP_SETPOINTS
REQUEST_OFFBOARD
REQUEST_ARM
TAKEOFF
HOVER
COMMAND
CANCELING_TO_LAND
LANDING
TAKEOFF_FAILED
TAKEOFF_CANCELED
FAILSAFE_HOLD
DISARMED
```

Launch is passive: reaching `READY` means PX4 status and local position are available, but the node will not request Offboard, arm, or take off until a takeoff action goal is accepted.

`REQUEST_OFFBOARD` and `REQUEST_ARM` are intentionally separate for clearer real-hardware logs and safer debugging.

## Takeoff Action

Takeoff is commanded through the `takeoff` action:

```text
mullet_core/action/Takeoff
goal: altitude_m, yaw_rad, use_current_yaw
result: success, message
feedback: state, current_altitude_m, target_altitude_m, altitude_error_m, progress
```

Goals are accepted only from `READY`, with valid PX4 telemetry, a disarmed vehicle, finite yaw when requested, and an altitude inside `min_takeoff_altitude_m` / `max_takeoff_altitude_m`.

Explicit-yaw goals (`use_current_yaw=false`) complete only after the vehicle is within
`takeoff_yaw_tolerance_rad` of the requested yaw while altitude and vertical velocity
are settled for `hover_settle_time_s`. Goals using the current yaw keep the existing
altitude/vertical-settle completion behavior.

Cancellation before this node sends an arm request returns to `READY`. Cancellation after an arm request, even before PX4 reports armed, requests Position mode as a best-effort stabilization step, commands `NAV_LAND`, and waits for disarm.

## Topics

PX4 topics use PX4-compatible QoS: best effort, transient local, keep last depth 1.

| Direction | Topic |
| --- | --- |
| publish | `/fmu/in/offboard_control_mode` |
| publish | `/fmu/in/trajectory_setpoint` |
| publish | `/fmu/in/vehicle_command` |
| subscribe | `/fmu/out/vehicle_local_position` |
| subscribe | `/fmu/out/vehicle_status_v1` |

PX4 instances must be in `[0, 254]` so `target_system = instance + 1` maps to
PX4 system IDs `[1, 255]` without wraparound. For PX4 instance `N > 0`, topics
are prefixed with `/px4_N` and `target_system` is `N + 1`.

## External Commands

The first implementation keeps compatibility with the existing command stream:

```text
std_msgs/msg/Float32MultiArray [vx_ned_mps, vy_ned_mps, mode]
```

Only `mode=1` is accepted. Commands are interpreted in PX4 NED world frame, clamped by parameters, and rejected if stale or non-finite. Long term, replace this with a typed command message or `geometry_msgs/msg/TwistStamped`.

## Build

From a ROS 2 environment with `px4_msgs` available:

```bash
cd ws
colcon build --packages-select mullet_core --symlink-install
```

## Launch

```bash
source ws/install/setup.bash
ros2 launch mullet_core mullet_core.launch.py px4_instances:=0
```

Multi-instance example:

```bash
ros2 launch mullet_core mullet_core.launch.py px4_instances:=1,2
```

The launch file rejects duplicate `px4_instances` within one launch invocation.
Each node also validates that its own `vehicle_instance` is in `[0, 254]`.

## Takeoff Action Client Example

```
ros2 action send_goal /px4_1/takeoff mullet_core/action/Takeoff "{altitude_m: 2.0, yaw_rad: 0.0, use_current_yaw: true}" --feedback
```

## Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `vehicle_instance` | int | `0` | PX4 instance ID (0–254); determines topic prefix and target system |
| `control_rate_hz` | double | `50.0` | State machine and setpoint publish rate |
| `min_takeoff_altitude_m` | float | `0.5` | Minimum acceptable takeoff altitude |
| `max_takeoff_altitude_m` | float | `100.0` | Maximum acceptable takeoff altitude |
| `takeoff_yaw_tolerance_rad` | float | `0.1` | Yaw error threshold for completion |
| `hover_settle_time_s` | float | `2.0` | Duration altitude and velocity must be settled before completion |
| `velocity_clamp_mps` | float | `5.0` | Maximum NED velocity magnitude from external commands |
| `command_timeout_s` | float | `1.0` | Time before stale velocity command is dropped |


## Safety Notes

PX4 offboard mode requires a continuous proof-of-life stream before and during offboard operation. `mullet_core` publishes at 50 Hz by default to provide margin against scheduling jitter.

Stopping this node must result in a configured PX4 offboard-loss failsafe, not undefined behavior. Before real hardware use, configure and test PX4 failsafe parameters such as offboard loss timeout, RC fallback action, geofence, and position-estimate failsafe.
