# Actions and Services

The node serves three ROS 2 actions and two `std_srvs/srv/Trigger` services.
Acceptance comes from the `xRejectReason()` predicates in
`src/flight/predicates.cpp` — the same functions behind the heartbeat's `can_*`
flags, so a flag and an acceptance decision cannot disagree.

| Command | Kind | Accepted from |
| --- | --- | --- |
| `takeoff` | `mullet_interfaces/action/Takeoff` | `IDLE`, disarmed, telemetry valid |
| `land` | `mullet_interfaces/action/Land` | anything armed, or a takeoff in progress; not while already landing |
| `return_home` | `mullet_interfaces/action/ReturnHome` | `HOLD` or `MISSION`, after a takeoff latched home, PX4 in Offboard |
| `start_mission` | `std_srvs/srv/Trigger` | `HOLD`, PX4 in Offboard |
| `hold` | `std_srvs/srv/Trigger` | `MISSION`, or a return home in progress |

## Takeoff

```text
goal: float32 altitude_m, float32 yaw_rad, bool use_current_yaw
result: bool success, string message
feedback: string state, float32 current_altitude_m,
          float32 target_altitude_m, float32 altitude_error_m,
          float32 progress
```

Goals also require finite yaw when explicit yaw is requested and an altitude in
`[min_takeoff_altitude_m, max_takeoff_altitude_m]`. The node latches the current
position as home and the takeoff origin, streams the takeoff setpoint through
warmup, requests Offboard, arms, and succeeds with `Takeoff complete` once
altitude, vertical velocity, and (for explicit-yaw goals) yaw are settled for
`hover_settle_time_s`. The state is then `HOLD`.

Cancel before the arm request returns to `IDLE`. Cancel after it resolves the
goal as canceled and lands.

## Land

```text
goal: empty
result: bool success, string message
feedback: string state, bool armed, float32 elapsed_s
```

The only land command. It preempts a takeoff, a return home, or a mission. A
takeoff that has not requested arm yet is simply ended: the goal succeeds at
once and the state returns to `IDLE`. Otherwise the node requests Position mode,
then retries `NAV_LAND`, and succeeds with `Vehicle disarmed after landing` when
PX4 reports disarmed. Landing is committed once accepted: cancel requests are
rejected.

If `landing_timeout_s` expires first, the action aborts with
`Landing timed out while vehicle is still armed`, but the node keeps retrying
`NAV_LAND` until disarm.

## ReturnHome

```text
goal: empty
result: bool success, string message
feedback: float32 distance_m
```

Flies back above the takeoff point at the current altitude by sliding the hold
target toward it at `return_home_speed_mps`. Succeeds with `Arrived home` within
`return_home_acceptance_m`, in `HOLD`; the operator lands from there. Cancel, or
the `hold` service, stops in place and holds.

## start_mission and hold

`start_mission` moves `HOLD` to `MISSION`, where the node streams the latest
fresh velocity command from the `command` topic. `hold` moves `MISSION`, or a
return home, back to `HOLD` at the current position. A rejected call returns
`success=false` with the reject reason as the message.

## Emergency stop

There is none here. The RC controller's kill switch disarms PX4; the node sees
the disarm and goes to `IDLE`.
