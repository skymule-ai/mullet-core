# Testing

Run tests from a ROS 2 environment with `px4_msgs` available.

## Build

```bash
cd ws
colcon build --packages-select mullet_core --symlink-install
```

## Package Tests

```bash
cd ws
colcon test --packages-select mullet_core
colcon test-result --verbose
```

## Focused Tests

CTest/gtest by name:

```bash
cd ws
colcon test --packages-select mullet_core --ctest-args -R test_state_machine_helpers
colcon test-result --verbose
```

Launch parser pytest:

```bash
cd ws
colcon test --packages-select mullet_core --pytest-args -k "launch"
colcon test-result --verbose
```

## Coverage Areas

- Vehicle instance bounds and `target_system` mapping.
- Helper behavior: retry timing, angle wrapping, and the local-to-WGS84 projection.
- The operator workflow end to end: takeoff, `start_mission`, `hold`, return
  home, land, and the heartbeat fields each step publishes (state, active action,
  capability flags, global position).
- Takeoff cancellation and failure on either side of the arm request, Land
  preempting a takeoff, committed Land cancellation rejection, and landing timeout.
- PX4 overrides: external disarm returns to `IDLE`, Offboard loss drops to
  `HOLD` without landing, and a stale `MISSION` stream holds in place, then drops to `HOLD`.
- EKF2 flags to `LocalizationStatus` (outdoor, indoor, 3-axis mag in flight).
- Launch parsing of the packaged YAML (`vehicle_instance`, namespace prefix).

There is no ROS 2 on a plain host checkout; the SITL image
(`ghcr.io/skymule-ai/px4-sitl`) has ROS and a built `px4_msgs` underlay in the
sibling `mullet-sim/ws`.
