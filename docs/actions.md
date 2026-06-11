# Actions

The node exposes six ROS 2 action servers. Goal acceptance is intentionally
strict so action clients cannot bypass PX4 telemetry checks or landing recovery.

## EnableOffboard

Action: `mullet_interfaces/action/EnableOffboard`

```text
goal: empty
result: bool success, string message
feedback: string state, bool offboard, float32 elapsed_s
```

Accepted only from `READY` when PX4 telemetry and local position are valid, the
vehicle is disarmed, and PX4 is not already in Offboard. The node latches the
current finite local position and yaw, streams that fixed hold target through
warmup, requests Offboard, and succeeds with `Offboard enabled` when PX4 reports
Offboard.

After success the node remains in `OFFBOARD_HOLD`, keeps publishing the latched
hold setpoint, and does not reissue Offboard mode, arm, land, or accept external
velocity control.

## Arm

Action: `mullet_interfaces/action/Arm`

```text
goal: empty
result: bool success, string message
feedback: string state, bool armed, bool offboard, float32 elapsed_s
```

Accepted from `READY` or disarmed `OFFBOARD_HOLD` only when PX4 status, local
position, and recent `vehicle_land_detected` telemetry are valid and
`landed=true` has remained continuous for `landed_confirmation_s`. From `READY`
the node latches a ground hold target, streams Offboard proof-of-life, requests
Offboard, then requests arm. From `OFFBOARD_HOLD` it reuses the existing hold
target and requests arm without re-requesting Offboard.

The action succeeds as soon as PX4 reports armed. The node then enters
`ARMED_OFFBOARD_HOLD`, continues publishing the hold setpoint, and starts
`armed_hold_duration_s`. When that timer expires, the node direct-disarms only
if landed telemetry is still confirmed, then requests Position mode. PX4 may
disarm earlier due to native PX4 parameters such as `COM_DISARM_PRFLT` or
`COM_DISARM_LAND`; in that case the node skips its own disarm command and still
releases Offboard.

## Disarm

Action: `mullet_interfaces/action/Disarm`

```text
goal: empty
result: bool success, string message
feedback: string state, bool armed, bool landed,
          float32 elapsed_s
```

Already-disarmed `READY` and disarmed `OFFBOARD_HOLD` succeed immediately.
From `ARMED_OFFBOARD_HOLD`, direct disarm is allowed only while PX4 reports armed
and recent land-detected telemetry has continuously reported landed for
`landed_confirmation_s`; after disarm the node keeps streaming the hold setpoint
until PX4 leaves Offboard.

Airborne, landing-active, takeoff-active, or uncertain disarm requests are
rejected. `Disarm` is not a landing command; clients must use the `Land` action
or emergency land topic for airborne recovery. If Position-mode release after
ground disarm times out, the node enters `DISARMED_OFFBOARD_RELEASE_FAILED`,
keeps safe Offboard hold publications active, and rejects new arm/takeoff
attempts until explicit recovery.

## DisableOffboard

Action: `mullet_interfaces/action/DisableOffboard`

```text
goal: empty
result: bool success, string message
feedback: string state, bool offboard, float32 elapsed_s
```

Accepted only from `OFFBOARD_HOLD` when telemetry and local position are valid,
the vehicle is disarmed, and PX4 is currently Offboard. The node keeps streaming
the hold setpoint while it retries Position mode requests. It succeeds with
`Offboard disabled` when PX4 leaves Offboard.

Timeout aborts the action with `disable_offboard request timed out`. If telemetry
is still valid, the vehicle is disarmed, and PX4 remains Offboard, the state
returns to `OFFBOARD_HOLD`. Unexpected arming during disable aborts the action
and starts landing recovery.

## Takeoff

Action: `mullet_interfaces/action/Takeoff`

```text
goal: float32 altitude_m, float32 yaw_rad, bool use_current_yaw
result: bool success, string message
feedback: string state, float32 current_altitude_m,
          float32 target_altitude_m, float32 altitude_error_m,
          float32 progress
```

Accepted from `READY`, or from `OFFBOARD_HOLD` when PX4 is still Offboard and
disarmed. Goals require valid PX4 telemetry, a disarmed vehicle, finite yaw when
explicit yaw is requested, and an altitude in
`[min_takeoff_altitude_m, max_takeoff_altitude_m]`.

Explicit-yaw goals complete only after altitude, vertical velocity, and yaw are
settled for `hover_settle_time_s`. Goals using current yaw keep the existing
altitude and vertical-velocity settle behavior.

## Land

Action: `mullet_interfaces/action/Land`

```text
goal: empty
result: bool success, string message
feedback: string state, bool armed, float32 elapsed_s
```

Accepted from `HOVER` or `COMMAND` when PX4 reports armed. It is also accepted
during active takeoff states and preempts the active takeoff goal. Landing is
committed once accepted: cancel requests are rejected, external velocity
commands are ignored, and takeoff goals are rejected.

The action succeeds with `Vehicle disarmed after landing` only after PX4 reports
disarmed. If `landing_timeout_s` expires first, the action aborts with
`Landing timed out while vehicle is still armed`, but the `LANDING` state keeps
retrying `NAV_LAND`.

## Emergency Land Topic

Topic: `land_request` (`std_msgs/msg/Empty`)

This topic remains an emergency escape hatch. It can preempt broader states such
as takeoff. A request received while already landing is idempotent and resets the
landing command retry timer.
