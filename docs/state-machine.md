# State Machine

`DroneStateMachine` is a deterministic soft real-time supervisor around PX4
Offboard. PX4 remains responsible for hard real-time flight control,
stabilization, mixing, and failsafe execution.

## States

```text
IDLE        on the ground, or otherwise not under supervisor control
TRANSITION  takeoff, land, or return home in progress
HOLD        airborne, holding a latched position
MISSION     airborne, following the external mission command stream
```

Which transition is running is decided by the private `Phase`
(`warmup`, `request_offboard`, `request_arm`, `ascent` for takeoff; `landing`;
`return_home`) and published as the heartbeat's `active_action`
(`takeoff`, `land`, `return_home`).

## Transitions

| From | To | Trigger |
| --- | --- | --- |
| `IDLE` | `TRANSITION` (takeoff) | `Takeoff` goal: telemetry valid, disarmed. |
| `TRANSITION` (takeoff) | `HOLD` | Altitude, vertical velocity, and yaw settle. |
| `TRANSITION` (takeoff) | `IDLE` | Failure, cancel, or `Land` before the arm request. |
| `TRANSITION` (takeoff) | `TRANSITION` (land) | Failure, cancel, or `Land` after the arm request. |
| `HOLD` | `MISSION` | `start_mission` service. |
| `MISSION` | `HOLD` | `hold` service. |
| `HOLD`, `MISSION` | `TRANSITION` (return home) | `ReturnHome` goal. |
| `TRANSITION` (return home) | `HOLD` | Within `return_home_acceptance_m` of home, or `hold` / cancel (holds in place). |
| anything armed, not landing | `TRANSITION` (land) | `Land` goal. |
| `TRANSITION` (land) | `IDLE` | PX4 reports disarmed. |

PX4 overrides, checked first on every tick while the vehicle is airborne under
Offboard control (`HOLD`, `MISSION`, takeoff ascent, return home):

| Event | Result |
| --- | --- |
| PX4 reports disarmed (RC kill switch) | Active goals abort, `IDLE`. |
| PX4 leaves Offboard (pilot takeover or PX4 failsafe) | Active goals abort, `HOLD`, re-latching the current position every tick. |

## Guardrails

- Launch is passive. `IDLE` never arms, requests Offboard, or takes off without
  an accepted `Takeoff` goal.
- Offboard proof-of-life (the takeoff setpoint) is streamed through warmup
  before Offboard is requested.
- The supervisor never fights PX4 or the pilot. After a mode switch away from
  Offboard it only keeps the hold target on the vehicle, so re-entering Offboard
  holds in place instead of flying back to a stale setpoint. Position loss and
  other failsafes are PX4's to handle.
- Cancelling a takeoff before the arm request returns to `IDLE`; after it, the
  vehicle lands, because PX4 may arm asynchronously.
- Landing keeps retrying `NAV_LAND` until PX4 reports disarmed, even after the
  user-facing land action times out.
- Return home slides the hold target toward home at `return_home_speed_mps`, so
  PX4 never receives a setpoint step it would chase at full speed. It holds the
  current altitude.
- `MISSION` streams the mission command (horizontal velocity or acceleration)
  while it is fresh, at the altitude latched on entering `MISSION`. When the
  stream goes stale the drone holds its last position and stays in `MISSION`
  (`command_stream_valid=false`); nothing leaves `MISSION` automatically.
- The emergency stop is the RC controller's kill switch. There is no emergency
  land topic.

## Source References

- State enum: `include/mullet_core/flight/types.hpp`
- Tick dispatch and PX4 overrides: `src/flight/state_machine.cpp`
- Transition bodies (takeoff, landing, return home, mission): `src/flight/states.cpp`
- Command eligibility: `src/flight/predicates.cpp`
- Target latching, takeoff settling, and setpoint assembly: `src/flight/setpoints.cpp`
- State, phase, and PX4 labels: `src/flight/labels.cpp`
- Action goal ownership and terminal results: `src/flight/detail/action_lifecycle_coordinator.hpp`
