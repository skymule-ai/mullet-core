# mullet_core

`mullet_core` is a C++ ROS 2 package for drone-only PX4 offboard supervision. The package folder is `mullet-core`, but the ROS 2 package name is `mullet_core`.

## Control Boundary

PX4 remains the hard real-time flight controller. It owns attitude stabilization, motor mixing, inner-loop control, and emergency failsafe execution.

`mullet_core` is a deterministic soft real-time offboard supervisor. It owns continuous offboard proof-of-life publication, setpoint streaming, mode/arm/land transitions, external command validation, and stale-command fallback behavior.

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
WARMUP_SETPOINTS
REQUEST_OFFBOARD
REQUEST_ARM
TAKEOFF
HOVER
COMMAND
LANDING
FAILSAFE_HOLD
DISARMED
```

`REQUEST_OFFBOARD` and `REQUEST_ARM` are intentionally separate for clearer real-hardware logs and safer debugging.

## Topics

PX4 topics use PX4-compatible QoS: best effort, transient local, keep last depth 1.

| Direction | Topic |
| --- | --- |
| publish | `/fmu/in/offboard_control_mode` |
| publish | `/fmu/in/trajectory_setpoint` |
| publish | `/fmu/in/vehicle_command` |
| subscribe | `/fmu/out/vehicle_local_position` |
| subscribe | `/fmu/out/vehicle_status_v1` |

For PX4 instance `N > 0`, topics are prefixed with `/px4_N` and `target_system` is `N + 1`.

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

## Safety Notes

PX4 offboard mode requires a continuous proof-of-life stream before and during offboard operation. `mullet_core` publishes at 50 Hz by default to provide margin against scheduling jitter.

Stopping this node must result in a configured PX4 offboard-loss failsafe, not undefined behavior. Before real hardware use, configure and test PX4 failsafe parameters such as offboard loss timeout, RC fallback action, geofence, and position-estimate failsafe.
