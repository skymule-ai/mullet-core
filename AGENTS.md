# AGENTS.md (Repo Guide for Coding Agents)

This repo is `mullet_core`, a C++17 ROS 2 `ament_cmake` package for soft
real-time PX4 offboard supervision of multicopter drones.

## Quick Orientation

- ROS 2 package name: `mullet_core` (repo folder is `mullet-core`).
- Main node: `mullet_core_node`.
- Core class: `DroneStateMachine` in `include/mullet_core/flight/drone_state_machine.hpp`
  and `src/flight/*.cpp`.
- PX4 interface: `Px4Interface` owns PX4 command publishing and telemetry
  subscriptions.
- Detailed behavior docs live in `docs/`; update them with behavior changes.
- `CLAUDE.md` has deeper architecture notes and should stay consistent with this
  file.

## Repo Layout

- `include/mullet_core/flight/`: public state-machine types and node header.
- `include/mullet_core/px4/`: public PX4 interface, topic utilities, and offboard publisher.
- `include/mullet_core/telemetry/`: public paddock telemetry publisher.
- `src/flight/`: state-machine implementation. State handlers stay in `states.cpp`;
  target/setpoint logic is in `setpoints.cpp`, labels in `labels.cpp`, and external
  command parsing in `commands.cpp`. `src/flight/detail/` holds private,
  uninstalled helpers, including the typed ROS action slots.
- `src/px4/`, `src/telemetry/`, `src/node/`: PX4 implementation, telemetry
  implementation, and node entrypoint.
- `launch/`: ROS 2 launch file and packaged-config loading.
- `config/`: packaged parameter YAML.
- `docs/`: maintainer docs for actions, state machine, configuration, PX4 topics,
  and testing.
- `test/`: gtests plus launch parser pytest.
- `mullet-agents/`: agent/skill support files; not part of the ROS package build.

## Build / Test Commands

Run from a ROS 2 workspace with `px4_msgs` and `mullet_interfaces` available:

```sh
colcon build --packages-select mullet_core --symlink-install
colcon test --packages-select mullet_core
colcon test-result --verbose
```

Focused tests:

```sh
colcon test --packages-select mullet_core --ctest-args -R test_state_machine_helpers
colcon test --packages-select mullet_core --pytest-args -k "launch"
colcon test-result --verbose
```

No project clang-format, clang-tidy, or GitHub Actions workflow is configured.
Do not assume tooling that is not present.

## Simulation / Integration

For SITL work, use the sibling repo:

```sh
cd /home/mattia/WORKSPACE/MULLET/mullet-sim
./run.sh
./stop.sh
```

PX4 instance topic prefixes must match the simulator convention:

- instance `0`: `/fmu/...`
- instance `N`: `/px4_N/fmu/...`

For multi-vehicle testing, starting instances at `1` keeps every vehicle under a
named prefix.

## Coding Rules

- Keep changes small and avoid drive-by formatting.
- Do not revert existing user changes in a dirty worktree.
- Use C++17 and keep `-Wall -Wextra -Wpedantic` clean.
- Header guards use `MULLET_CORE_..._HPP_`; do not introduce `#pragma once`.
- Namespace is `mullet_core`; use `mullet_core::state_machine_detail` only for
  private state-machine helpers when needed.
- Naming: PascalCase classes, camelCase methods, `snake_case_` members with a
  trailing underscore, `k`-prefixed PascalCase enum values/constants.
- Prefer `std::unique_ptr` for ownership, explicit constructors, and `override`
  on virtual overrides.
- Add Google-style comments on public APIs when the reason is not obvious from
  the name/signature.

## Design Defaults

- PX4 is the hard real-time flight controller; this package is only the
  soft-real-time offboard supervisor.
- Keep high-rate control paths boring and deterministic.
- Before adding code, check whether existing code, the C++ standard library, or
  an already-installed dependency already covers it.
- Do not add abstractions, config, or scaffolding for speculative future needs.
- Preserve docs and tests with behavior changes, especially for state
  transitions, action contracts, PX4 topics, parameters, and launch parsing.
