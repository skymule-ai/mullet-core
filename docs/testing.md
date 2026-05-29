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
- Helper behavior such as retry timing, angle wrapping, and state predicates.
- Action behavior around takeoff, landing, cancellation, timeouts, Offboard hold,
  DisableOffboard, and emergency land preemption.
- Launch parsing for explicit instance lists, duplicate rejection, malformed
  tokens, and offset ranges.
