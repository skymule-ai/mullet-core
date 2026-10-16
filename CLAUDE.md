# CLAUDE.md

Guidance for AI coding sessions working in this repository.

## What this is

`mullet_core` is a C++17 soft-real-time PX4 offboard supervisor for multicopter drones (single ROS 2 `ament_cmake` package). It is meant to be checked out inside a colcon workspace's `src/`, alongside two sibling repos:

- `mullet-interfaces` — shared ROS 2 action/msg IDL (`Takeoff`, `Land`, `ReturnHome` actions; `DroneHeartbeat` msg). `start_mission` and `hold` are `std_srvs/srv/Trigger` services with no IDL there. Required at build time (`<depend>mullet_interfaces</depend>`). It is the stack's wire contract, and this repo is its only producer: consumers propose a field or action there, and it becomes real when this node populates or serves it. Its `README.md` describes that request path.
- `mullet-paddock` — the ground-station/fleet-management gateway that consumes `/mullet/heartbeat`. Unrelated to this repo's own build; only relevant when changing the heartbeat message contract.

**Control Boundary**: PX4 is the hard-real-time flight controller. `mullet_core` is the soft-real-time offboard supervisor sitting above it — it decides *what* the vehicle should do (state machine, action goals) and streams setpoints to PX4, but never does hard-real-time control itself. See `README.md` for the full architecture diagram and safety notes.

## Architecture

One ROS 2 node, `mullet_core_node` (class `DroneStateMachine`,
`include/mullet_core/flight/drone_state_machine.hpp` + `src/flight/*.cpp`):

- A 4-state FSM (`DroneState` in `include/mullet_core/flight/types.hpp`) driven by a `control_timer_` at `control_rate_hz` (default 50 Hz) calling `controlTick()`. The states are `kIdle`, `kTransition`, `kHold`, `kMission`; their wire labels (`IDLE`, `TRANSITION`, `HOLD`, `MISSION`) come from `stateName()` in `src/flight/labels.cpp`, the only place those strings exist. `controlTick()` first applies the PX4 overrides in `handleExternalOverride()`: an external disarm (RC kill switch) goes to IDLE, losing Offboard while airborne goes to a passive HOLD.
- `kTransition` sequences internally through the private `DroneStateMachine::Phase` enum (`kWarmup`, `kRequestOffboard`, `kRequestArm`, `kAscent` for takeoff; `kLanding`; `kReturnHome`). The phase decides which transition is running; it leaves the node only as the heartbeat's `active_action` (`takeoff`/`land`/`return_home`, from `activeActionName()`). When a behavior question is about *sequencing within* a transition, the answer is in the phase, not in a state.
- Three `rclcpp_action` servers (Takeoff, Land, ReturnHome) and two `std_srvs/srv/Trigger` services (`start_mission`, `hold`). A private header-only typed action slot in `src/flight/detail/action_lifecycle_coordinator.hpp` owns each active goal and its cancellation/terminal state.
- `Px4Interface` (`include/mullet_core/px4/px4_interface.hpp`) — PX4 `VehicleCommand` publishing (arm/mode-set/land) and the telemetry subscriptions (`VehicleLocalPosition`, `VehicleStatus`, `VehicleLandDetected`) that drive FSM predicates.
- `OffboardHeartbeatPublisher` (`include/mullet_core/px4/offboard_heartbeat_publisher.hpp`) — the PX4-facing OFFBOARD keep-alive. It owns a wall timer at `control_rate_hz` (minimum 2 Hz) and pulls the current setpoint from its builder on each tick. It is the sole publisher of `OffboardControlMode` and `TrajectorySetpoint`; `Px4Interface` publishes `VehicleCommand` only. Setpoint questions belong here, command questions belong there.
- `PaddockHeartbeatPublisher` (`include/mullet_core/telemetry/paddock_heartbeat_publisher.hpp`) — owns the `/mullet/heartbeat` (`DroneHeartbeat`) publisher and its own timer at `heartbeat_rate_hz` (default 10 Hz); it only owns *when/where* the heartbeat is sent. The message content itself (FSM state, active transition, PX4 status, local and global position, capability flags) is assembled by `DroneStateMachine::buildHeartbeatMessage()`, since that requires deep FSM-internal state. It stamps `schema_version` `3`. Two field-level facts worth not re-deriving: `px4_failsafe` mirrors PX4's `VehicleStatus.failsafe` for observability only and drives no transition, and lat/lon/alt come from `VehicleLocalPosition`'s EKF origin through `localToGlobal()` in `src/flight/detail/helpers.cpp` — there is no global-position subscription.
- `LocalizationStatusPublisher` (`include/mullet_core/telemetry/localization_status_publisher.hpp`) — subscribes `/fmu/out/estimator_status_flags` and republishes it at 1 Hz as `LocalizationStatus` on `/mullet/localization_status` (keyed by `drone_id`), through the pure `toLocalizationStatus()`. Read-only, independent of the FSM, silent until the first flags arrive.
- The heartbeat's `can_*` capability flags and the `handleXGoal()` rejection checks both read the same `canX()`/`xRejectReason()` pairs in `src/flight/predicates.cpp`. Change eligibility there once; never add a second rule in a goal handler.
- `include/mullet_core/px4/topic_utils.hpp` — shared free functions (`px4Topic`, `px4NowUs`, `px4Qos`) for PX4 topic-name prefixing, message timestamps, and the PX4 QoS profile, used by both `Px4Interface` and `OffboardHeartbeatPublisher` so this isn't duplicated.

For FSM transitions, action semantics, PX4 topic tables, and parameters, `docs/README.md` and its linked docs (`state-machine.md`, `actions.md`, `configuration.md`, `px4-interface.md`, `testing.md`) are the source of truth — there's an explicit Maintenance Rule there: update docs in the same change as behavior.

## Downstream contract

`mullet-paddock` consumes this node's `DroneHeartbeat`. Two things there are hard-coded against this repo and break silently rather than loudly:

- `frontend/src/features/drones/stateMachine.ts` hard-codes the `DroneState` wire labels. A label that isn't in its list makes the operator UI render "Unknown state". Changing `stateName()` means changing that file in the same PR.
- `gateway/mullet_control_gateway/adapters/ros2_adapter.py` reads heartbeat fields through `getattr(heartbeat, ..., default)`, so a renamed or dropped field silently becomes the default instead of raising.

Adding a heartbeat field means editing three repos in order: `mullet-interfaces` declares it in the `.msg`, this one populates it in `buildHeartbeatMessage()` and bumps `schema_version`, and `mullet-paddock` reads it (`models.py`, `droneTypes.ts`). The middle step is the one that makes the field real — a field declared in the IDL and never assigned here reaches consumers as a default they cannot distinguish from a genuine zero. The same order applies to a new action: define it in `mullet-interfaces`, serve it here with its acceptance predicate, then call it. The `mullet-stack-explorer` agent traces these consumers across sibling checkouts.

## Build & test

```bash
colcon build --packages-select mullet_core --symlink-install
colcon test --packages-select mullet_core
colcon test-result --verbose

# focused gtest by name
colcon test --packages-select mullet_core --ctest-args -R test_state_machine_helpers
# focused pytest
colcon test --packages-select mullet_core --pytest-args -k "launch"
```

`launch/mullet_core.launch.py` takes no launch arguments. All configuration comes from the single packaged `config/mullet_core_params.yaml`, which the launch file parses itself to derive the node namespace. Multi-drone means one deployment per drone, each with its own YAML — not one launch spawning several nodes.

Four gtest binaries (`test_vehicle_instance`, `test_state_machine_helpers`, `test_drone_state_machine_behavior`, `test_localization_status`) plus one pytest suite (`test_mullet_core_launch.py`, tests YAML-derived launch configuration). `test_drone_state_machine_behavior.cpp` is a black-box behavioral suite that drives the node through a fake PX4 peer over real ROS 2 topics — it does not touch internal classes directly, so it keeps validating new components automatically as long as topic names/message types/QoS are preserved.

There is no ROS 2 on the host: build and test inside the `ghcr.io/skymule-ai/px4-sitl` image with `../mullet-sim/ws/install` (built `px4_msgs`) sourced as an underlay and the sibling `mullet-interfaces` built alongside.

No linter or CI is configured in this repo (no `.clang-format`, `.clang-tidy`, or `.github/workflows`) — don't assume tooling that isn't there.

## Code style

Existing conventions (follow these for any C++ you touch):

- Header guards `#ifndef MULLET_CORE_..._HPP_` / `#define ...` (not `#pragma once`).
- Namespace `mullet_core`; `mullet_core::state_machine_detail` is reserved for private state-machine helpers.
- PascalCase classes (`DroneStateMachine`, `Px4Interface`), camelCase methods (`loadParameters()`, `controlTick()`), `snake_case_` member variables with a trailing underscore (`state_`, `command_buffer_`), `k`-prefixed PascalCase for enum values and free constants (`DroneState::kInit`, `kMinVehicleInstance`).
- `std::unique_ptr` for ownership, explicit constructors, `override` on virtual overrides, `-Wall -Wextra -Wpedantic` must stay clean.

New policies for code written or touched going forward:

- **YAGNI/KISS via "ponytail programming"** — before writing any new code, walk this ladder in order and stop at the first `yes`:
  1. Does this need to exist at all? → no: skip it.
  2. Does it already exist in this codebase? → reuse it, don't rewrite.
  3. Does the C++ stdlib do it? → use it.
  4. Does a native platform/OS feature do it? → use it.
  5. Does an already-installed dependency do it? → use it.
  6. Can it be one line? → make it one line.
  7. Only if none of the above apply: write the minimum new code that works.
- **Docstrings**: Google-style docstring comments, always in English, on new public APIs (classes, public methods, free functions) wherever the *why* isn't obvious from the name/signature. This codebase has no prior docstring convention — existing undocumented code relies on `docs/*.md` for behavior — so this is a going-forward policy for new/touched public surface, not a mandate to retroactively document everything.
