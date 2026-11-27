# Map-Frame Missions (Design Note)

Status: proposed, 2026-11-27. Not implemented. Work items: #48 (this design),
#49 (behaviour when the localizer is lost), skymule-ai/mullet-interfaces#6,
skymule-ai/mullet-localization#16, skymule-ai/mullet-mapping#11,
skymule-ai/ergodic_mppi#8.

## Goal

Fly a mission drawn on a prior map: waypoints, a search domain, later
obstacles, all in the map's coordinates, indoors or without GPS. The map comes
from mullet-mapping: built offline from a walk (FAST-LIO2 + loop closure), and
`map_localizer` measures on board where FAST-LIO2's drifting world sits in it.

## Decision: Correct the Targets, Never the Estimate

There are two places the map correction can go (mullet-mapping
`Documentation/design.rst`, Options A and B):

- **A, chosen:** PX4 keeps flying on FAST-LIO2's odometry alone, so its
  estimate is smooth and never jumps. mullet_core converts the mission's map
  targets into PX4's frame with the latest correction. A correction update moves
  the setpoint a few cm and the position loop follows it.
- **B, not now:** ev_bridge sends PX4 the map-corrected pose. Every correction
  then moves EKF2's estimate (innovation gates, possible resets, heading steps),
  and map_localizer joins the flight-critical path: a 1-5 s search stall would
  starve EKF2 of vision.

With A, a crashed, slow or lost map_localizer only freezes the correction: the
drone keeps flying on FAST-LIO2, which drifts by 0.2-0.5 % of the distance.
B stays documented in mullet-mapping and is reconsidered only if phase 2 hand
flights show corrections consistently under ~5 cm and something needs PX4
itself in map coordinates.

## Frames

| Frame | What | Published by |
| --- | --- | --- |
| `map` | the prior map: FAST-LIO2's world of the mapping walk, after loop closure. z up, gravity-aligned | fixed, offline |
| `camera_init` | FAST-LIO2's world of this flight; drifts in `map` | map_localizer: `map -> camera_init` on `/<ns>/tf`, matched at 1 Hz, re-sent at 10 Hz |
| `body` | the MID-360's IMU | FAST-LIO2: `camera_init -> body`, 10 Hz |
| `base_link` | PX4's body (FLU in ROS) | ev_bridge extrinsic; on TF as `body -> base_link` once mullet-localization#16 lands |
| PX4 local NED | EKF2's world, the frame of every setpoint this node sends | PX4 |

The chain `map -> base_link` is all TF. The last step, from `camera_init` to
PX4 local NED, is not a fixed transform:

- `outdoor-vio` (`EKF2_EV_CTRL 5`, compass on): EKF2 learns the rotation from
  the vision frame to NED while GPS and compass are fused, and keeps it.
- `indoor` (`EKF2_EV_CTRL 13`, no compass): vision heading, but PX4 does not
  yet align yaw on it (mullet-px4 README, mullet-localization#2).
- In every profile EKF2 also fuses other sensors (baro height, flow), so its
  position is not FAST-LIO2's.

So this node does not compose a chain of assumed transforms. It measures the
one transform it needs.

## The Alignment: Measured, Not Assumed

`T_ned_map`, the pose of `map` in PX4 local NED. Both frames are
gravity-aligned, so it has four degrees of freedom: a yaw `psi` and a
translation `t`.

```text
F = diag(1, -1, -1)        FLU world -> FRD world (180 deg about x), as ev_bridge flips
Rz(psi)                    rotation about NED down by psi (clockwise seen from above)

map pose  (p_map, yaw_map)  of base_link, from TF at a FAST-LIO2 odometry stamp
NED pose  (p_ned, yaw_ned)  from VehicleLocalPosition, the sample nearest that stamp

psi_k = yaw_ned + yaw_map                      (yaw_map counter-clockwise, yaw_ned clockwise)
t_k   = p_ned - Rz(psi_k) F p_map
```

Each pair gives one sample `(psi_k, t_k)`. The applied alignment:

1. **Filter.** Take the median of the last ~1 s of samples. A single sample
   carries EKF2's lag (about 27 ms on mullet1, so 5 cm at 2 m/s) and the
   FAST-LIO2/EKF2 disagreement.
2. **Rate-limit.** Move the applied alignment toward the filtered one by at
   most `map_align_max_rate_mps` (for example 0.1 m/s) and
   `map_align_max_rate_dps` (for example 1 deg/s). A map correction then moves
   the setpoints gradually, never as a step.
3. **Settle.** The alignment is valid once map_localizer reports TRACKING and
   the last N samples agree within a tolerance. Before that it does not exist.

Why measure it locally, at the drone:

- It absorbs everything between the map and what PX4 believes: the map
  correction, EKF2's learned rotation, EKF2's own offsets. To put the drone
  physically on a map point, the target has to be expressed in PX4's belief,
  and this is exactly that.
- The correction's roll and pitch are not zero: up to 1.07 deg in the step 9
  replay (the 2026-10-21 loop bag). A 4-DoF transform measured where the drone
  is keeps that error proportional to how far a target leads the drone, not to
  the distance from the map's origin: 1 deg over a 3 m lead is 5 cm.
  `max_xy_position_lead_m` and `max_z_position_lead_m` already bound the lead.

## Converting a Command

A `MissionCommand` with `frame = FRAME_MAP` (mullet-interfaces#6) is converted
before the existing validation and clamps, which then run unchanged in NED:

```text
p_ned   = Rz(psi) F p_map + t          NaN components stay NaN (planar modes keep z NaN)
v_ned   = Rz(psi) F v_map
a_ned   = Rz(psi) F a_map
yaw_ned = psi - yaw_map                NaN stays NaN
```

`FRAME_NED` (the default, 0) is today's behaviour, untouched.

A map command arriving while there is no valid alignment is rejected like any
invalid command. The stream goes stale and the existing rules apply: hold in
place in `MISSION`, then `HOLD` after `command_loss_hold_s`. A mission
framework is also expected to check before it starts (ergodic_mppi#8).

## What Mission Frameworks Get

The applied alignment, published as `geometry_msgs/TransformStamped`
(`ned -> map`) whenever it is valid. A framework converts its observation
(`VehicleLocalPosition`, with velocities) into `map` with the inverse and sends
its commands in `FRAME_MAP`. Both directions use the same transform, so a
command computed from an observation round-trips exactly.

## What Stays in NED

Takeoff, hold, return home and land. Return home flies to the takeoff point
latched in NED, which is right for "back where I started": the map is not
needed to get home, and a lost localizer cannot block it.

## When the Localizer Degrades

- **HOLDING:** a match failed. The correction is at most a few seconds old and
  the alignment keeps converting.
- **LOST:** no passing match for 5 s, or repeated large steps. The alignment
  freezes at its last value and keeps converting. FAST-LIO2's drift is the only
  error that grows. What the drone should do then (continue, hold, return home)
  is #49.
- **map_localizer not running:** no alignment. Map commands are rejected; NED
  missions fly as today.

## Validation

1. Pure functions with unit tests: the sample, the filter, the rate limit, the
   conversion (FLU/FRD flips, yaw signs, NaN handling).
2. Replay: the 2026-10-21 loop bag with map_localizer, and `VehicleLocalPosition`
   from the same bag. A fixed map target, converted over the whole loop, must
   stay on the same physical point: its NED position must follow the drift that
   loop closure measured (up to 2.7 m on that bag) while staying smooth.
3. SIH on mullet1, then a hand-carried walk, then flight.

## Open Points

- **Time pairing.** `VehicleLocalPosition.timestamp` is PX4 time and FAST-LIO2
  stamps ROS time. Pair them through the PX4 timesync offset, or through
  `VehicleOdometry` echoed by EKF2.
- **Constants.** The filter window, rate limits and settle tolerance come from
  the replay.
- **Paddock.** Whether the heartbeat should carry the alignment state (valid,
  map_localizer state) for the operator.
- **ESDF.** Obstacles from mullet-mapping's static ESDF are in `map` too. They
  belong to the planner (ergodic_mppi), which plans in `map`, not to this node.
