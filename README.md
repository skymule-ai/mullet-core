# mullet_core

`mullet_core` is a C++ ROS 2 package for drone-only PX4 offboard supervision. The package folder is `mullet-core`, but the ROS 2 package name is `mullet_core`.

## Documentation

Detailed maintainer documentation lives in `docs/`:

- `docs/state-machine.md`: states, transitions, PX4 overrides, and guardrails
- `docs/actions.md`: action and service contracts
- `docs/configuration.md`: parameters and defaults
- `docs/px4-interface.md`: PX4 topics, QoS, commands, and instance mapping
- `docs/testing.md`: build and test commands

## Control Boundary

PX4 remains the hard real-time flight controller. It owns attitude stabilization, motor mixing, inner-loop control, and emergency failsafe execution. The emergency stop is the RC controller's kill switch, not this node.

`mullet_core` is a deterministic soft real-time offboard supervisor. It owns takeoff, land, and return-home sequencing, continuous offboard proof-of-life publication, setpoint streaming, mode/arm/land commands, and external command validation. It never fights PX4: an external disarm or a switch out of Offboard ends whatever it was doing.

High-level planners, mission generation, optimization, and expensive diagnostics should run in separate nodes, not inside this control loop. A mission node drives the drone by publishing `MissionCommand` setpoints on `command` while the state is `MISSION`.

## Runtime Architecture

```text
mullet_core_node
├── DroneStateMachine
│   ├── fixed-rate control timer at control_rate_hz
│   ├── 4 states (IDLE, TRANSITION, HOLD, MISSION) and PX4 override rules
│   ├── three action servers (Takeoff, Land, ReturnHome)
│   ├── two Trigger services (start_mission, hold)
│   └── MISSION command freshness and velocity clamps
├── Px4Interface
│   ├── publishes VehicleCommand (arm, mode set, land)
│   ├── subscribes VehicleLocalPosition, VehicleStatus, VehicleLandDetected
│   └── handles PX4 topic prefix and target_system
├── OffboardHeartbeatPublisher
│   ├── publishes OffboardControlMode and TrajectorySetpoint
│   ├── owns its own timer at control_rate_hz (minimum 2 Hz)
│   └── pulls the current setpoint from the state machine each tick
└── PaddockHeartbeatPublisher
    ├── publishes DroneHeartbeat on /mullet/heartbeat
    └── owns its own timer at heartbeat_rate_hz
```

The Offboard proof-of-life stream is owned by `OffboardHeartbeatPublisher` on its
own timer, so it does not depend on any individual state handler remembering to
publish.

## State Flow

```text
IDLE --takeoff--> TRANSITION --settled--> HOLD <--start_mission / hold--> MISSION
HOLD or MISSION --return_home--> TRANSITION --arrived--> HOLD
anything armed --land--> TRANSITION --disarmed--> IDLE
```

Launch is passive: `IDLE` never requests Offboard, arms, or takes off until a `Takeoff` goal is accepted. `TRANSITION` sequences internally through phases (`warmup`, `request_offboard`, `request_arm`, `ascent`, `landing`, `return_home`); phases are logged, and the heartbeat reports only which transition is running as `active_action`.

See `docs/state-machine.md` for the full transition table and `docs/actions.md` for each command.

## Topics

PX4 topics use PX4-compatible QoS: best effort, transient local, keep last depth 1.

| Direction | Topic | Owner |
| --- | --- | --- |
| publish | `/fmu/in/offboard_control_mode` | `OffboardHeartbeatPublisher` |
| publish | `/fmu/in/trajectory_setpoint` | `OffboardHeartbeatPublisher` |
| publish | `/fmu/in/vehicle_command` | `Px4Interface` |
| subscribe | `/fmu/out/vehicle_local_position` | `Px4Interface` |
| subscribe | `/fmu/out/vehicle_status_v1` | `Px4Interface` |
| subscribe | `/fmu/out/vehicle_land_detected` | `Px4Interface` |

PX4 instances must be in `[0, 254]` so `target_system = instance + 1` maps to
PX4 system IDs `[1, 255]` without wraparound. Instance `0` uses the bare `/fmu/...`
topics above. For instance `N > 0`, topics are prefixed with
`/<px4_namespace_prefix>_N` and `target_system` is `N + 1`. The packaged
`px4_namespace_prefix` is `uav`, so instance `1` uses `/uav_1/fmu/...` and
`target_system` `2`.

## Heartbeat

The node publishes `mullet_interfaces/msg/DroneHeartbeat` on `heartbeat_topic`
(`/mullet/heartbeat`) at `heartbeat_rate_hz`, default 10 Hz. It carries
`schema_version` `3`.

One message is a full snapshot for ground-station consumers: FSM state and label,
the running transition (`active_action`), PX4 nav/arming state and label,
telemetry-presence and validity flags, local position and yaw, WGS84 lat/lon/alt,
the command paths, and a `can_*` capability flag per command. The `can_*` flags come
from the same predicates that accept or reject a real command, so a client that
respects them will not be rejected on eligibility grounds.

`px4_failsafe` mirrors PX4's own `VehicleStatus.failsafe` flag. It is reported for
observability and does not drive any state transition in this node.

The message and the actions are defined in `mullet-interfaces`, which is where a
consumer proposes a new field or action. This node is the producer: a field becomes
real once `buildHeartbeatMessage()` assigns it, and this node owns the
`schema_version` bump that goes with it.

## External Commands

`MISSION` follows `mullet_interfaces/msg/MissionCommand` on `command_topic`. Its `mode` names what PX4 tracks:

- `VELOCITY`: horizontal NED velocity, clamped by `max_vx_ned_mps`/`max_vy_ned_mps`, with an optional acceleration feedforward (both axes or neither).
- `ACCELERATION`: horizontal NED acceleration only.

Accelerations are clamped per axis by `max_accel_ned_mps2`. The z components must be NaN: the altitude stays at the value latched on entering `MISSION`. A NaN yaw faces the commanded velocity in `VELOCITY` and keeps the hold heading in `ACCELERATION`. Commands with an unset mode, a finite z, or missing fields for their mode are rejected. Commands older than `command_timeout_s` are ignored and the drone holds its last position; commands outside `MISSION` are latched but never streamed.

## Build

From a ROS 2 environment with `px4_msgs` and `mullet_interfaces` available:

```bash
colcon build --packages-select mullet_core --symlink-install
```

## Launch

```bash
source install/setup.bash
ros2 launch mullet_core mullet_core.launch.py
```
The launch file reads its complete configuration from
`config/mullet_core_params.yaml`; it accepts no launch arguments.
Set `vehicle_instance` and `px4_namespace_prefix` in that YAML before launch.
Run a separate deployment with its own YAML configuration for each drone.

## Drone Deployment

`docker/` runs `mullet_core` and the PX4 uXRCE-DDS agent on the drone computer, on top of
`mullet-ros2-base`. The image build compiles `mullet_interfaces` (from the sibling
`mullet-interfaces` checkout) and `mullet_core`, and runs this package's tests: a failing
test means no image.

```bash
cd docker
cp .env.example .env   # ROS_DOMAIN_ID, PX4 UART device and baud
docker compose up -d --build
```

Both containers restart on boot. `config/mullet_core_params.yaml` is mounted read-only:
edit it, then `docker compose restart mullet_core`. PX4's `UXRCE_DDS_DOM_ID` must equal
`ROS_DOMAIN_ID`, and the base image's `px4_msgs` must match the firmware's messages.

## Command Examples

Names are namespaced the same way as PX4 topics: bare for `vehicle_instance: 0`,
prefixed with `/<px4_namespace_prefix>_N` otherwise.

```bash
ros2 action send_goal /uav_1/takeoff mullet_interfaces/action/Takeoff "{altitude_m: 2.0, yaw_rad: 0.0, use_current_yaw: true}" --feedback
ros2 service call /uav_1/start_mission std_srvs/srv/Trigger
ros2 service call /uav_1/hold std_srvs/srv/Trigger
ros2 action send_goal /uav_1/return_home mullet_interfaces/action/ReturnHome "{}" --feedback
ros2 action send_goal /uav_1/land mullet_interfaces/action/Land "{}" --feedback
```

## Parameters

See `docs/configuration.md` for the complete parameter reference.

## Safety Notes

PX4 offboard mode requires a continuous proof-of-life stream before and during offboard operation. `mullet_core` publishes at 50 Hz by default to provide margin against scheduling jitter.

Stopping this node must result in a configured PX4 offboard-loss failsafe, not undefined behavior. Before real hardware use, configure and test PX4 failsafe parameters such as offboard loss timeout, RC fallback action, RC kill switch, geofence, and position-estimate failsafe.
