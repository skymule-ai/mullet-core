# State Machine

`DroneStateMachine` is a deterministic soft real-time supervisor around PX4
Offboard. PX4 remains responsible for hard real-time flight control,
stabilization, mixing, and failsafe execution.

## States

```text
INIT
READY
ENABLE_OFFBOARD_REQUESTED
WARMUP_OFFBOARD_HOLD_SETPOINTS
REQUEST_OFFBOARD_ONLY
ARM_REQUESTED
ARM_COMMAND
ARMED_HOLD
DISARM_COMMAND
RELEASE_OFFBOARD_REQUEST
DISARMED_OFFBOARD_RELEASE_FAILED
DISABLE_OFFBOARD_REQUESTED
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

## Primary Flow

| From | To | Trigger |
| --- | --- | --- |
| `INIT` | `READY` | PX4 vehicle status exists and local position has valid XY/Z. |
| `READY` | `ENABLE_OFFBOARD_REQUESTED` | `enable_offboard` goal is accepted. |
| `ENABLE_OFFBOARD_REQUESTED` | `WARMUP_OFFBOARD_HOLD_SETPOINTS` | Next control tick commits the goal. |
| `WARMUP_OFFBOARD_HOLD_SETPOINTS` | `REQUEST_OFFBOARD_ONLY` | `offboard_warmup_s` elapses. |
| `REQUEST_OFFBOARD_ONLY` | `READY` | PX4 reports Offboard mode. |
| `READY` | `ARM_REQUESTED` | `arm` goal accepted with confirmed landed telemetry. |
| `READY` | `ARM_COMMAND` | `arm` goal accepted while already Offboard. |
| `ARM_COMMAND` | `ARMED_HOLD` | PX4 reports armed. |
| `ARMED_HOLD` | `DISARM_COMMAND` | `disarm` goal accepted or armed-hold timer expires with confirmed landed telemetry. |
| `DISARM_COMMAND` | `READY` | PX4 reports disarmed; Offboard remains active until explicitly disabled. |
| `RELEASE_OFFBOARD_REQUEST` | `READY` | PX4 leaves Offboard mode. |
| `RELEASE_OFFBOARD_REQUEST` | `DISARMED_OFFBOARD_RELEASE_FAILED` | PX4 stays Offboard past `request_position_mode_timeout_s`. |
| `READY` | `DISABLE_OFFBOARD_REQUESTED` | `disable_offboard` goal is accepted while PX4 is Offboard and disarmed. |
| `DISABLE_OFFBOARD_REQUESTED` | `READY` | PX4 leaves Offboard mode. |
| `READY` | `TAKEOFF_REQUESTED` | `takeoff` goal is accepted. |
| `READY` | `REQUEST_ARM` | `takeoff` goal is accepted while PX4 is already Offboard and disarmed. |
| `TAKEOFF_REQUESTED` | `WARMUP_SETPOINTS` | Next control tick commits the goal. |
| `WARMUP_SETPOINTS` | `REQUEST_OFFBOARD` | Offboard proof-of-life warmup completes. |
| `REQUEST_OFFBOARD` | `REQUEST_ARM` | PX4 reports Offboard mode. |
| `REQUEST_ARM` | `TAKEOFF` | PX4 reports armed. |
| `TAKEOFF` | `HOVER` | Altitude, vertical velocity, and yaw settle. |
| `HOVER` | `COMMAND` | A fresh external velocity command is available. |
| `COMMAND` | `FAILSAFE_HOLD` | External command becomes stale. |
| `FAILSAFE_HOLD` | `COMMAND` | A fresh external velocity command returns. |
| `LANDING` | `DISARMED` | PX4 reports disarmed. |
| `DISARMED` | `READY` or `INIT` | Depends on current local-position validity. |

## Guardrails

- Launch is passive. Reaching `READY` does not arm, request Offboard, or take
  off without an accepted action goal.
- Offboard proof-of-life is streamed before and during Offboard control.
- `REQUEST_OFFBOARD` and `REQUEST_ARM` are separate states for clearer logs and
  safer debugging.
- Cancellation before an arm request returns to a terminal takeoff-canceled path
  without landing recovery.
- Cancellation after an arm request enters landing recovery because PX4 may arm
  asynchronously.
- `LANDING` keeps retrying `NAV_LAND` until PX4 reports disarmed, even if the
  user-facing land action times out.
- Disarmed `READY` while PX4 is Offboard ignores external velocity commands and
  rejects land goals because no landing action is required.
- Direct disarm is a ground-only shutdown guarded by recent, continuously
  landed `vehicle_land_detected` telemetry. Airborne or uncertain disarm
  requests are rejected; use `Land` or emergency land for landing recovery.

## Source References

- State enum: `include/mullet_core/types.hpp`
- Tick dispatch and top-level preemption: `src/drone_state_machine.cpp`
- State handlers: `src/drone_state_machine_states.cpp`
- State predicates and names: `src/drone_state_machine_predicates.cpp`
