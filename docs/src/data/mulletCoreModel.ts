export type DroneState =
  | "INIT"
  | "READY"
  | "TAKEOFF_REQUESTED"
  | "WARMUP_SETPOINTS"
  | "REQUEST_OFFBOARD"
  | "REQUEST_ARM"
  | "TAKEOFF"
  | "HOVER"
  | "COMMAND"
  | "CANCELING_TO_LAND"
  | "LANDING"
  | "TAKEOFF_FAILED"
  | "TAKEOFF_CANCELED"
  | "FAILSAFE_HOLD"
  | "DISARMED";

export type SourceRef = {
  file: string;
  symbol?: string;
  note?: string;
};

export type ActionOwnership = {
  takeoffGoal: string;
  landGoal: string;
  cancellation: string;
  terminalResult: string;
  vehicleControl: string;
};

export type StateNode = {
  id: DroneState;
  phase: "startup" | "takeoff" | "flight" | "recovery" | "terminal";
  summary: string;
  entry: string[];
  during: string[];
  exits: string[];
  px4Traffic: string[];
  actionOwnership: ActionOwnership;
  sourceRefs: SourceRef[];
};

export type Transition = {
  from: DroneState;
  to: DroneState;
  event: string;
  guard: string;
  nodeResponse: string;
  px4Command?: string;
  actionResult?: string;
  sourceRefs: SourceRef[];
};

export type ScenarioStep = {
  event: string;
  fromState: DroneState;
  toState: DroneState;
  nodeResponse: string;
  px4Command?: string;
  actionResult?: string;
  actionOwnership: ActionOwnership;
  safetyNote?: string;
  sourceRefs: SourceRef[];
};

export type Scenario = {
  id: string;
  title: string;
  question: string;
  summary: string;
  steps: ScenarioStep[];
};

export type FileGroup = {
  title: string;
  purpose: string;
  files: Array<{
    path: string;
    role: string;
    sourceRefs: SourceRef[];
  }>;
};

export type InterfaceItem = {
  name: string;
  kind: "action" | "topic" | "parameter" | "launch" | "timing" | "multi-instance";
  direction?: "publish" | "subscribe" | "server" | "input";
  summary: string;
  details: string[];
  sourceRefs: SourceRef[];
};

export const states: StateNode[] = [
  {
    id: "INIT",
    phase: "startup",
    summary: "Passive startup while waiting for PX4 status and a valid local position.",
    entry: ["Node construction", "DISARMED when local position becomes invalid"],
    during: ["Does not arm, change mode, or publish takeoff setpoints"],
    exits: ["READY once vehicle status exists and local position has valid XY/Z"],
    px4Traffic: ["Subscribes to vehicle status and local position"],
    actionOwnership: {
      takeoffGoal: "None",
      landGoal: "None",
      cancellation: "None",
      terminalResult: "None",
      vehicleControl: "No active vehicle-control command",
    },
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleInit" },
      { file: "src/px4_interface.cpp", symbol: "localPositionValid" },
    ],
  },
  {
    id: "READY",
    phase: "startup",
    summary: "Telemetry is usable and the node is waiting for an explicit takeoff action.",
    entry: ["INIT receives valid PX4 telemetry", "DISARMED recovers with valid local position"],
    during: ["Accepts valid Takeoff goals", "Rejects Land goals because the vehicle is not flying"],
    exits: ["TAKEOFF_REQUESTED after a Takeoff goal is accepted and target latch succeeds"],
    px4Traffic: ["No mode, arm, or land command is sent from this state"],
    actionOwnership: {
      takeoffGoal: "No active goal until handleTakeoffAccepted commits one",
      landGoal: "None",
      cancellation: "None",
      terminalResult: "Rejected goals receive no accepted action handle",
      vehicleControl: "Passive supervision only",
    },
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleReady" },
      { file: "src/drone_state_machine_actions.cpp", symbol: "handleTakeoffGoal" },
      { file: "src/drone_state_machine_actions.cpp", symbol: "handleTakeoffAccepted" },
    ],
  },
  {
    id: "TAKEOFF_REQUESTED",
    phase: "takeoff",
    summary: "A Takeoff goal is active and target coordinates/yaw have been latched.",
    entry: ["Takeoff goal accepted from READY", "configureTakeoffTarget succeeds"],
    during: ["Publishes initial takeoff feedback with progress 0.0"],
    exits: ["WARMUP_SETPOINTS on the next control tick"],
    px4Traffic: ["No vehicle command yet"],
    actionOwnership: {
      takeoffGoal: "Active",
      landGoal: "Can accept a Land goal to preempt takeoff",
      cancellation: "Accepted cancellation returns to READY if no arm request was sent",
      terminalResult: "No terminal Takeoff result yet",
      vehicleControl: "Prepared takeoff target, no PX4 mode transition yet",
    },
    sourceRefs: [
      { file: "src/drone_state_machine_actions.cpp", symbol: "handleTakeoffAccepted" },
      { file: "src/drone_state_machine_states.cpp", symbol: "handleTakeoffRequested" },
    ],
  },
  {
    id: "WARMUP_SETPOINTS",
    phase: "takeoff",
    summary: "Publishes offboard proof-of-life and position setpoints before requesting Offboard mode.",
    entry: ["TAKEOFF_REQUESTED commits the goal"],
    during: ["Streams OffboardControlMode(position) and TrajectorySetpoint at control_rate_hz"],
    exits: ["REQUEST_OFFBOARD after offboard_warmup_s", "TAKEOFF_FAILED on warmup timeout"],
    px4Traffic: ["OffboardControlMode(position=true)", "TrajectorySetpoint(position target)"],
    actionOwnership: {
      takeoffGoal: "Active",
      landGoal: "Can preempt",
      cancellation: "Accepted; pre-arm cancellation cancels Takeoff without landing recovery",
      terminalResult: "Takeoff may be canceled or aborted if warmup fails",
      vehicleControl: "Setpoint stream only",
    },
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleWarmup" },
      { file: "src/drone_state_machine_commands.cpp", symbol: "publishTakeoffSetpoint" },
    ],
  },
  {
    id: "REQUEST_OFFBOARD",
    phase: "takeoff",
    summary: "Retries PX4 Offboard mode requests while continuing the takeoff setpoint stream.",
    entry: ["Warmup duration completes"],
    during: ["Sends DO_SET_MODE main=1 sub=6 when retry interval is due"],
    exits: ["REQUEST_ARM when PX4 reports Offboard", "TAKEOFF_FAILED on timeout"],
    px4Traffic: ["VehicleCommand DO_SET_MODE Offboard", "Position setpoint stream continues"],
    actionOwnership: {
      takeoffGoal: "Active",
      landGoal: "Can preempt",
      cancellation: "Accepted; may require landing recovery if arm request was already sent elsewhere",
      terminalResult: "Takeoff aborts if Offboard request times out",
      vehicleControl: "Requests PX4 navigation-state change",
    },
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleRequestOffboard" },
      { file: "src/px4_interface.cpp", symbol: "setOffboardMode" },
    ],
  },
  {
    id: "REQUEST_ARM",
    phase: "takeoff",
    summary: "Retries arm requests once Offboard mode is confirmed.",
    entry: ["PX4 reports NAVIGATION_STATE_OFFBOARD"],
    during: ["Sends ARM_DISARM arm command when retry interval is due"],
    exits: [
      "TAKEOFF when PX4 reports armed",
      "CANCELING_TO_LAND on arm timeout or Offboard loss after arm request",
    ],
    px4Traffic: ["VehicleCommand COMPONENT_ARM_DISARM param1=1", "Position setpoint stream continues"],
    actionOwnership: {
      takeoffGoal: "Active",
      landGoal: "Can preempt",
      cancellation: "Accepted; after takeoff_arm_requested_ is set, recovery goes through landing",
      terminalResult: "Takeoff aborts on arm timeout or Offboard loss",
      vehicleControl: "Vehicle may arm asynchronously after cancellation or timeout",
    },
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleRequestArm" },
      { file: "src/px4_interface.cpp", symbol: "arm" },
      { file: "src/drone_state_machine_predicates.cpp", symbol: "shouldRecoverThroughLanding" },
    ],
  },
  {
    id: "TAKEOFF",
    phase: "takeoff",
    summary: "Vehicle is armed and climbing/settling at the latched takeoff target.",
    entry: ["PX4 reports armed"],
    during: ["Publishes position setpoint and feedback until altitude, vertical velocity, and yaw settle"],
    exits: ["HOVER on settled target", "CANCELING_TO_LAND on timeout or Offboard loss"],
    px4Traffic: ["Position setpoint stream continues"],
    actionOwnership: {
      takeoffGoal: "Active until settled target succeeds",
      landGoal: "Can preempt",
      cancellation: "Accepted; recovery goes through landing",
      terminalResult: "Takeoff succeeds at settled target or aborts on failure",
      vehicleControl: "Offboard position control owns the takeoff setpoint",
    },
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleTakeoff" },
      { file: "src/drone_state_machine_commands.cpp", symbol: "takeoffSettled" },
    ],
  },
  {
    id: "HOVER",
    phase: "flight",
    summary: "Takeoff action has succeeded and the node holds the current hover target.",
    entry: ["Takeoff settles and succeeds", "Fresh command expires from COMMAND through FAILSAFE_HOLD recovery"],
    during: ["Publishes position hold setpoints"],
    exits: ["COMMAND when a fresh velocity command arrives", "LANDING when a Land goal or emergency land request arrives"],
    px4Traffic: ["OffboardControlMode(position=true)", "TrajectorySetpoint(position hover target)"],
    actionOwnership: {
      takeoffGoal: "Completed successfully",
      landGoal: "May be accepted if PX4 reports armed",
      cancellation: "No active Takeoff cancellation path",
      terminalResult: "Takeoff result has already been sent",
      vehicleControl: "Offboard position hold",
    },
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleHover" },
      { file: "src/drone_state_machine_commands.cpp", symbol: "publishHoverSetpoint" },
    ],
  },
  {
    id: "COMMAND",
    phase: "flight",
    summary: "A fresh external velocity command is being streamed to PX4 in NED frame.",
    entry: ["HOVER or FAILSAFE_HOLD sees a fresh command"],
    during: ["Clamps velocity, points yaw along horizontal velocity, and publishes velocity setpoints"],
    exits: ["FAILSAFE_HOLD when the command becomes stale", "LANDING on Land goal or emergency land request"],
    px4Traffic: ["OffboardControlMode(velocity=true)", "TrajectorySetpoint(velocity command)"],
    actionOwnership: {
      takeoffGoal: "Completed",
      landGoal: "May be accepted if PX4 reports armed",
      cancellation: "None",
      terminalResult: "No Takeoff action remains active",
      vehicleControl: "External velocity command owns setpoint generation until stale",
    },
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleCommand" },
      { file: "src/drone_state_machine_commands.cpp", symbol: "onCommand" },
    ],
  },
  {
    id: "CANCELING_TO_LAND",
    phase: "recovery",
    summary: "Best-effort stabilization path used when cancellation occurs after an arm request.",
    entry: ["Takeoff cancel requested after arm request or while armed"],
    during: ["Latches/publishes hover if position is valid, requests Position mode once, sends NAV_LAND"],
    exits: ["LANDING immediately after the landing command path is entered"],
    px4Traffic: ["VehicleCommand DO_SET_MODE Position", "VehicleCommand NAV_LAND"],
    actionOwnership: {
      takeoffGoal: "Still active until canceled/aborted during landing recovery",
      landGoal: "Usually none unless a Land goal caused preemption",
      cancellation: "Takeoff cancel has been accepted",
      terminalResult: "Terminal Takeoff result is deferred until landing/disarm or timeout",
      vehicleControl: "Transfers from Offboard toward PX4-managed landing",
    },
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleCancelingToLand" },
      { file: "src/drone_state_machine_actions.cpp", symbol: "beginLanding" },
    ],
  },
  {
    id: "LANDING",
    phase: "recovery",
    summary: "Committed landing recovery; NAV_LAND is retried until disarm or recovery timeout handling.",
    entry: ["Land action", "Emergency land request", "Failure after arm request", "Cancel after arm request"],
    during: ["Publishes Land feedback and retries NAV_LAND on command_retry_interval_s"],
    exits: ["DISARMED when PX4 reports disarmed"],
    px4Traffic: ["VehicleCommand NAV_LAND"],
    actionOwnership: {
      takeoffGoal: "May already be aborted/canceled or may complete when disarm/timeout occurs",
      landGoal: "Active until disarm success or landing timeout abort",
      cancellation: "Land cancellation is always rejected",
      terminalResult: "Land can abort on timeout while LANDING continues retrying NAV_LAND",
      vehicleControl: "PX4-managed landing is continuously requested",
    },
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleLanding" },
      { file: "src/drone_state_machine_actions.cpp", symbol: "handleLandCancel" },
    ],
  },
  {
    id: "TAKEOFF_FAILED",
    phase: "terminal",
    summary: "Transient terminal state for pre-arm takeoff failures that do not require landing recovery.",
    entry: ["Takeoff failure occurs before arming and before an arm request"],
    during: ["Resets takeoff attempt state"],
    exits: ["READY on the next control tick when landing recovery is not required"],
    px4Traffic: ["No landing command unless recovery is required"],
    actionOwnership: {
      takeoffGoal: "Aborted before entering or while entering this state",
      landGoal: "None",
      cancellation: "None",
      terminalResult: "Takeoff result is aborted",
      vehicleControl: "No active flight command",
    },
    sourceRefs: [
      { file: "src/drone_state_machine.cpp", symbol: "controlTick" },
      { file: "src/drone_state_machine_actions.cpp", symbol: "failTakeoffOrLand" },
    ],
  },
  {
    id: "TAKEOFF_CANCELED",
    phase: "terminal",
    summary: "Transient terminal state for pre-arm takeoff cancellation.",
    entry: ["Takeoff cancel accepted before arm request"],
    during: ["Resets takeoff attempt state"],
    exits: ["READY on the next control tick when landing recovery is not required"],
    px4Traffic: ["No NAV_LAND because the node has not requested arming"],
    actionOwnership: {
      takeoffGoal: "Canceled before entering this state",
      landGoal: "None",
      cancellation: "Consumed",
      terminalResult: "Takeoff result is canceled",
      vehicleControl: "No active flight command",
    },
    sourceRefs: [
      { file: "src/drone_state_machine.cpp", symbol: "controlTick" },
      { file: "src/action_lifecycle_coordinator.cpp", symbol: "cancelTakeoff" },
    ],
  },
  {
    id: "FAILSAFE_HOLD",
    phase: "recovery",
    summary: "Position hold after an external velocity command becomes stale.",
    entry: ["COMMAND detects command_timeout_s has elapsed"],
    during: ["Publishes hover setpoint if local position is valid"],
    exits: ["COMMAND when fresh command returns", "LANDING if local position is invalid"],
    px4Traffic: ["Position hold setpoint", "NAV_LAND if local position is invalid"],
    actionOwnership: {
      takeoffGoal: "Completed",
      landGoal: "May be accepted only through emergency/state rules if landing starts",
      cancellation: "None",
      terminalResult: "No active Takeoff result path",
      vehicleControl: "Holds latest valid position as stale-command fallback",
    },
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleFailsafeHold" },
      { file: "src/drone_state_machine_commands.cpp", symbol: "commandFresh" },
    ],
  },
  {
    id: "DISARMED",
    phase: "terminal",
    summary: "Landing/recovery observed PX4 disarmed and clears command/takeoff attempt state.",
    entry: ["LANDING sees PX4 arming state DISARMED"],
    during: ["Completes active Land goal as succeeded and cancels any active Takeoff goal"],
    exits: ["READY with valid local position", "INIT if local position is invalid"],
    px4Traffic: ["No further vehicle commands in this state"],
    actionOwnership: {
      takeoffGoal: "Canceled if still active",
      landGoal: "Succeeded if active",
      cancellation: "Cleared",
      terminalResult: "Vehicle-disarmed results are sent before transition",
      vehicleControl: "No active vehicle command",
    },
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleLanding" },
      { file: "src/drone_state_machine_states.cpp", symbol: "handleDisarmed" },
    ],
  },
];

export const transitions: Transition[] = [
  {
    from: "INIT",
    to: "READY",
    event: "PX4 telemetry becomes ready",
    guard: "hasVehicleStatus() && localPositionValid()",
    nodeResponse: "Marks the supervisor ready without commanding PX4.",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleInit" }],
  },
  {
    from: "READY",
    to: "TAKEOFF_REQUESTED",
    event: "Takeoff goal accepted",
    guard: "No active land/takeoff goal, vehicle disarmed, telemetry valid, goal altitude/yaw valid",
    nodeResponse: "Accepts the action goal and latches takeoff position/yaw target.",
    actionResult: "Takeoff goal becomes active; no terminal result yet",
    sourceRefs: [{ file: "src/drone_state_machine_actions.cpp", symbol: "handleTakeoffAccepted" }],
  },
  {
    from: "TAKEOFF_REQUESTED",
    to: "WARMUP_SETPOINTS",
    event: "Next control tick",
    guard: "Takeoff goal is committed",
    nodeResponse: "Publishes feedback and begins offboard proof-of-life warmup.",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleTakeoffRequested" }],
  },
  {
    from: "WARMUP_SETPOINTS",
    to: "REQUEST_OFFBOARD",
    event: "Warmup completes",
    guard: "elapsed >= offboard_warmup_s",
    nodeResponse: "Continues setpoint streaming and prepares to request Offboard.",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleWarmup" }],
  },
  {
    from: "WARMUP_SETPOINTS",
    to: "TAKEOFF_FAILED",
    event: "Warmup timeout before arm request",
    guard: "phaseTimedOut(now, warmup_timeout_s) && !shouldRecoverThroughLanding()",
    nodeResponse: "Aborts the Takeoff action and enters the pre-arm terminal failure state.",
    actionResult: "Takeoff aborted: takeoff warmup timed out",
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleWarmup" },
      { file: "src/drone_state_machine_actions.cpp", symbol: "failTakeoffOrLand" },
    ],
  },
  {
    from: "WARMUP_SETPOINTS",
    to: "TAKEOFF_CANCELED",
    event: "Takeoff cancel before arm request",
    guard: "goalCancelRequested() && !shouldRecoverThroughLanding()",
    nodeResponse: "Cancels the Takeoff action without landing recovery because no arm request was sent.",
    actionResult: "Takeoff canceled: Takeoff canceled before arming",
    sourceRefs: [{ file: "src/drone_state_machine.cpp", symbol: "controlTick" }],
  },
  {
    from: "REQUEST_OFFBOARD",
    to: "REQUEST_ARM",
    event: "PX4 reports Offboard",
    guard: "px4_->isOffboard()",
    nodeResponse: "Moves to the explicit arm request phase.",
    px4Command: "Retries DO_SET_MODE Offboard until this guard is true",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleRequestOffboard" }],
  },
  {
    from: "REQUEST_OFFBOARD",
    to: "TAKEOFF_FAILED",
    event: "Offboard request timeout before arm request",
    guard: "phaseTimedOut(now, request_offboard_timeout_s) && !shouldRecoverThroughLanding()",
    nodeResponse: "Aborts the Takeoff action and enters the pre-arm terminal failure state.",
    actionResult: "Takeoff aborted: offboard request timed out",
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleRequestOffboard" },
      { file: "src/drone_state_machine_actions.cpp", symbol: "failTakeoffOrLand" },
    ],
  },
  {
    from: "REQUEST_ARM",
    to: "TAKEOFF",
    event: "PX4 reports armed",
    guard: "px4_->isArmed()",
    nodeResponse: "Starts takeoff-settle monitoring.",
    px4Command: "Retries COMPONENT_ARM_DISARM arm until this guard is true",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleRequestArm" }],
  },
  {
    from: "REQUEST_ARM",
    to: "CANCELING_TO_LAND",
    event: "Arm-phase failure after arm request",
    guard: "phaseTimedOut(now, request_arm_timeout_s) || !px4_->isOffboard()",
    nodeResponse: "Aborts the Takeoff action and starts landing recovery through the cancel-to-land path.",
    px4Command: "DO_SET_MODE Position, then NAV_LAND",
    actionResult: "Takeoff aborted with landing recovery",
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleRequestArm" },
      { file: "src/drone_state_machine_actions.cpp", symbol: "failTakeoffOrLand" },
    ],
  },
  {
    from: "TAKEOFF",
    to: "HOVER",
    event: "Takeoff target settles",
    guard: "Altitude, vertical velocity, and requested yaw are within tolerances for hover_settle_time_s",
    nodeResponse: "Latches hover target and succeeds the Takeoff action.",
    actionResult: "Takeoff succeeded: Takeoff complete",
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleTakeoff" },
      { file: "src/drone_state_machine_commands.cpp", symbol: "takeoffSettled" },
    ],
  },
  {
    from: "TAKEOFF",
    to: "CANCELING_TO_LAND",
    event: "Takeoff failure while armed",
    guard: "phaseTimedOut(now, takeoff_timeout_s) || !px4_->isOffboard()",
    nodeResponse: "Aborts the Takeoff action and starts landing recovery through the cancel-to-land path.",
    px4Command: "DO_SET_MODE Position, then NAV_LAND",
    actionResult: "Takeoff aborted with landing recovery",
    sourceRefs: [
      { file: "src/drone_state_machine_states.cpp", symbol: "handleTakeoff" },
      { file: "src/drone_state_machine_actions.cpp", symbol: "failTakeoffOrLand" },
    ],
  },
  {
    from: "TAKEOFF_FAILED",
    to: "READY",
    event: "Terminal failure handled",
    guard: "!shouldRecoverThroughLanding()",
    nodeResponse: "Resets takeoff attempt state and returns to passive ready.",
    sourceRefs: [{ file: "src/drone_state_machine.cpp", symbol: "controlTick" }],
  },
  {
    from: "TAKEOFF_CANCELED",
    to: "READY",
    event: "Terminal cancel handled",
    guard: "!shouldRecoverThroughLanding()",
    nodeResponse: "Resets takeoff attempt state and returns to passive ready.",
    sourceRefs: [{ file: "src/drone_state_machine.cpp", symbol: "controlTick" }],
  },
  {
    from: "HOVER",
    to: "COMMAND",
    event: "Fresh velocity command arrives",
    guard: "commandFresh(now)",
    nodeResponse: "Switches from position hold to velocity setpoint publishing.",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleHover" }],
  },
  {
    from: "COMMAND",
    to: "FAILSAFE_HOLD",
    event: "External command becomes stale",
    guard: "latest command age >= command_timeout_s",
    nodeResponse: "Latches current hover target and holds position.",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleCommand" }],
  },
  {
    from: "FAILSAFE_HOLD",
    to: "COMMAND",
    event: "Fresh velocity command returns",
    guard: "commandFresh(now)",
    nodeResponse: "Restores velocity setpoint mode.",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleFailsafeHold" }],
  },
  {
    from: "FAILSAFE_HOLD",
    to: "LANDING",
    event: "Local position becomes invalid",
    guard: "!localPositionValid()",
    nodeResponse: "Starts landing recovery and sends NAV_LAND.",
    px4Command: "VehicleCommand NAV_LAND",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleFailsafeHold" }],
  },
  {
    from: "REQUEST_ARM",
    to: "CANCELING_TO_LAND",
    event: "Takeoff cancel after arm request",
    guard: "goalCancelRequested() && shouldRecoverThroughLanding()",
    nodeResponse: "Requests Position mode before committing to landing recovery.",
    px4Command: "DO_SET_MODE Position, then NAV_LAND",
    actionResult: "Takeoff cancellation result is deferred until landing/disarm or timeout",
    sourceRefs: [
      { file: "src/drone_state_machine.cpp", symbol: "controlTick" },
      { file: "src/drone_state_machine_actions.cpp", symbol: "beginLanding" },
    ],
  },
  {
    from: "CANCELING_TO_LAND",
    to: "LANDING",
    event: "Landing command path starts",
    guard: "Always after canceling-to-land handler runs",
    nodeResponse: "Transitions to committed LANDING.",
    px4Command: "VehicleCommand NAV_LAND",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleCancelingToLand" }],
  },
  {
    from: "LANDING",
    to: "DISARMED",
    event: "PX4 reports disarmed",
    guard: "px4_->isDisarmed()",
    nodeResponse: "Completes active Land goal, cancels residual Takeoff goal, clears command buffer.",
    actionResult: "Land succeeded; residual Takeoff canceled if still active",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleLanding" }],
  },
  {
    from: "DISARMED",
    to: "READY",
    event: "Post-disarm telemetry remains valid",
    guard: "localPositionValid()",
    nodeResponse: "Returns to passive ready state.",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleDisarmed" }],
  },
  {
    from: "DISARMED",
    to: "INIT",
    event: "Post-disarm local position is invalid",
    guard: "!localPositionValid()",
    nodeResponse: "Waits for telemetry again.",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleDisarmed" }],
  },
];

const inactiveOwnership: ActionOwnership = {
  takeoffGoal: "None",
  landGoal: "None",
  cancellation: "None",
  terminalResult: "None",
  vehicleControl: "Passive or reset state",
};

export const scenarios: Scenario[] = [
  {
    id: "happy-takeoff",
    title: "Takeoff succeeds and holds hover",
    question: "What happens on the normal takeoff path?",
    summary: "A valid action goal moves through warmup, Offboard request, arm request, takeoff settle, then hover.",
    steps: [
      {
        event: "Operator sends a valid Takeoff goal",
        fromState: "READY",
        toState: "TAKEOFF_REQUESTED",
        nodeResponse: "The goal is accepted and the takeoff target is latched from current local position.",
        actionOwnership: {
          takeoffGoal: "Active",
          landGoal: "None",
          cancellation: "None",
          terminalResult: "Pending",
          vehicleControl: "Target latched, no PX4 mode command yet",
        },
        sourceRefs: [{ file: "src/drone_state_machine_actions.cpp", symbol: "handleTakeoffGoal" }],
      },
      {
        event: "Warmup and Offboard request complete",
        fromState: "WARMUP_SETPOINTS",
        toState: "REQUEST_ARM",
        nodeResponse: "The node streams setpoints continuously and retries Offboard mode until PX4 accepts.",
        px4Command: "DO_SET_MODE Offboard",
        actionOwnership: {
          takeoffGoal: "Active",
          landGoal: "Can preempt",
          cancellation: "Accepted if requested",
          terminalResult: "Pending",
          vehicleControl: "Offboard proof-of-life and position target stream",
        },
        safetyNote: "PX4 Offboard requires the setpoint stream before and during Offboard operation.",
        sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleRequestOffboard" }],
      },
      {
        event: "PX4 arms and takeoff settles",
        fromState: "TAKEOFF",
        toState: "HOVER",
        nodeResponse: "The node succeeds the Takeoff action and latches a hover target.",
        actionResult: "Takeoff succeeded: Takeoff complete",
        actionOwnership: {
          takeoffGoal: "Completed successfully",
          landGoal: "May now be accepted from HOVER",
          cancellation: "No active cancellation",
          terminalResult: "Sent",
          vehicleControl: "Position hold",
        },
        sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleTakeoff" }],
      },
    ],
  },
  {
    id: "cancel-before-arm",
    title: "Cancel takeoff before arming",
    question: "What happens if I cancel takeoff before arming?",
    summary: "The Takeoff action is canceled without NAV_LAND because this node has not requested arming.",
    steps: [
      {
        event: "Client requests Takeoff cancellation during early takeoff",
        fromState: "WARMUP_SETPOINTS",
        toState: "TAKEOFF_CANCELED",
        nodeResponse: "The node cancels the active Takeoff goal, resets attempt state, and avoids landing recovery.",
        actionResult: "Takeoff canceled: Takeoff canceled before arming",
        actionOwnership: {
          takeoffGoal: "Canceled",
          landGoal: "None",
          cancellation: "Consumed",
          terminalResult: "Canceled result sent",
          vehicleControl: "No arm request had been sent, so no landing command is needed",
        },
        safetyNote: "This is intentionally different from canceling after arm request.",
        sourceRefs: [
          { file: "src/drone_state_machine.cpp", symbol: "controlTick" },
          { file: "src/drone_state_machine_predicates.cpp", symbol: "shouldRecoverThroughLanding" },
        ],
      },
      {
        event: "Terminal cancel state is handled",
        fromState: "TAKEOFF_CANCELED",
        toState: "READY",
        nodeResponse: "The node returns to READY if local position is still valid.",
        actionOwnership: inactiveOwnership,
        sourceRefs: [{ file: "src/drone_state_machine.cpp", symbol: "controlTick" }],
      },
    ],
  },
  {
    id: "cancel-after-arm-request",
    title: "Cancel after arm request",
    question: "What happens if PX4 arms just after cancellation?",
    summary: "Once an arm request was sent, cancellation recovers through Position mode and repeated NAV_LAND.",
    steps: [
      {
        event: "Takeoff cancel arrives after arm was requested",
        fromState: "REQUEST_ARM",
        toState: "CANCELING_TO_LAND",
        nodeResponse: "The node treats the vehicle as potentially armed and starts landing recovery.",
        px4Command: "DO_SET_MODE Position",
        actionOwnership: {
          takeoffGoal: "Still active",
          landGoal: "None",
          cancellation: "Accepted",
          terminalResult: "Deferred",
          vehicleControl: "Stabilize through Position mode before landing",
        },
        safetyNote: "The arm acknowledgement can race with cancellation, so recovery assumes landing may be required.",
        sourceRefs: [
          { file: "src/drone_state_machine.cpp", symbol: "controlTick" },
          { file: "src/drone_state_machine_states.cpp", symbol: "handleCancelingToLand" },
        ],
      },
      {
        event: "Landing recovery commits",
        fromState: "CANCELING_TO_LAND",
        toState: "LANDING",
        nodeResponse: "The node sends NAV_LAND and enters committed landing.",
        px4Command: "NAV_LAND",
        actionOwnership: {
          takeoffGoal: "Active until disarm or timeout resolves it",
          landGoal: "None unless a Land goal was also accepted",
          cancellation: "Pending recovery result",
          terminalResult: "Deferred",
          vehicleControl: "PX4-managed landing is requested repeatedly",
        },
        sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleLanding" }],
      },
      {
        event: "PX4 reports disarmed",
        fromState: "LANDING",
        toState: "DISARMED",
        nodeResponse: "The node cancels the Takeoff goal and clears command buffers.",
        actionResult: "Takeoff canceled: Takeoff canceled; vehicle disarmed after landing",
        actionOwnership: {
          takeoffGoal: "Canceled",
          landGoal: "Succeeded if active",
          cancellation: "Complete",
          terminalResult: "Sent",
          vehicleControl: "No active flight command",
        },
        sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleLanding" }],
      },
    ],
  },
  {
    id: "land-preempts-takeoff",
    title: "Land action preempts takeoff",
    question: "Can Land interrupt an active Takeoff?",
    summary: "A Land goal can interrupt active takeoff states and becomes the committed recovery path.",
    steps: [
      {
        event: "Land goal arrives during active takeoff",
        fromState: "TAKEOFF",
        toState: "LANDING",
        nodeResponse: "The node aborts the active Takeoff goal and accepts Land as the owner of recovery.",
        px4Command: "NAV_LAND",
        actionResult: "Takeoff aborted: Takeoff preempted by land action",
        actionOwnership: {
          takeoffGoal: "Aborted",
          landGoal: "Active",
          cancellation: "Land cancel will be rejected",
          terminalResult: "Takeoff result sent; Land pending",
          vehicleControl: "Committed landing",
        },
        sourceRefs: [
          { file: "src/drone_state_machine_actions.cpp", symbol: "handleLandGoal" },
          { file: "src/drone_state_machine_actions.cpp", symbol: "preemptTakeoffForLandAction" },
        ],
      },
      {
        event: "Vehicle disarms",
        fromState: "LANDING",
        toState: "DISARMED",
        nodeResponse: "The Land action succeeds only after PX4 reports disarmed.",
        actionResult: "Land succeeded: Vehicle disarmed after landing",
        actionOwnership: {
          takeoffGoal: "Already aborted",
          landGoal: "Succeeded",
          cancellation: "None",
          terminalResult: "Land success sent",
          vehicleControl: "No active flight command",
        },
        sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleLanding" }],
      },
    ],
  },
  {
    id: "stale-command",
    title: "External command becomes stale",
    question: "What happens when command input stops?",
    summary: "Velocity command mode drops into position hold instead of continuing stale velocity.",
    steps: [
      {
        event: "Command age reaches command_timeout_s",
        fromState: "COMMAND",
        toState: "FAILSAFE_HOLD",
        nodeResponse: "The node latches a hover target and publishes position setpoints.",
        actionOwnership: {
          takeoffGoal: "Completed",
          landGoal: "None",
          cancellation: "None",
          terminalResult: "No action result changes",
          vehicleControl: "Position hold fallback",
        },
        safetyNote: "Non-finite values and mode values other than 1 are rejected before they enter the command buffer.",
        sourceRefs: [
          { file: "src/drone_state_machine_commands.cpp", symbol: "onCommand" },
          { file: "src/drone_state_machine_states.cpp", symbol: "handleCommand" },
        ],
      },
      {
        event: "Fresh command returns",
        fromState: "FAILSAFE_HOLD",
        toState: "COMMAND",
        nodeResponse: "Velocity setpoint publishing resumes.",
        actionOwnership: {
          takeoffGoal: "Completed",
          landGoal: "None",
          cancellation: "None",
          terminalResult: "No action result changes",
          vehicleControl: "External velocity command",
        },
        sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleFailsafeHold" }],
      },
    ],
  },
  {
    id: "landing-timeout",
    title: "Landing timeout while still armed",
    question: "Does the node keep retrying NAV_LAND after the action aborts?",
    summary: "The Land action can abort on timeout while the LANDING state continues retrying NAV_LAND.",
    steps: [
      {
        event: "Landing has not disarmed before landing_timeout_s",
        fromState: "LANDING",
        toState: "LANDING",
        nodeResponse: "The node aborts active action goals but remains in LANDING.",
        px4Command: "NAV_LAND retries continue",
        actionResult: "Land aborted: Landing timed out while vehicle is still armed",
        actionOwnership: {
          takeoffGoal: "Canceled or aborted depending on cancellation state",
          landGoal: "Aborted",
          cancellation: "Consumed if present",
          terminalResult: "Timeout result sent",
          vehicleControl: "LANDING continues retrying NAV_LAND",
        },
        safetyNote: "This is the key divergence: user-facing action completion does not stop landing recovery.",
        sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleLanding" }],
      },
      {
        event: "PX4 eventually reports disarmed",
        fromState: "LANDING",
        toState: "DISARMED",
        nodeResponse: "The node clears buffers and returns through DISARMED even though the Land action already aborted.",
        actionOwnership: {
          takeoffGoal: "None",
          landGoal: "Already aborted",
          cancellation: "None",
          terminalResult: "No duplicate terminal action result",
          vehicleControl: "Recovery complete",
        },
        sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleLanding" }],
      },
    ],
  },
];

export const fileGroups: FileGroup[] = [
  {
    title: "State Machine Core",
    purpose: "Owns control ticks, state transitions, state handlers, predicates, timing, and helpers.",
    files: [
      {
        path: "include/mullet_core/drone_state_machine.hpp",
        role: "Node API and private state for actions, timers, targets, buffers, and transitions.",
        sourceRefs: [{ file: "include/mullet_core/drone_state_machine.hpp" }],
      },
      {
        path: "src/drone_state_machine.cpp",
        role: "Node construction, parameter loading, subscriptions/action servers, and controlTick dispatch.",
        sourceRefs: [{ file: "src/drone_state_machine.cpp", symbol: "controlTick" }],
      },
      {
        path: "src/drone_state_machine_states.cpp",
        role: "Per-state behavior and transition calls.",
        sourceRefs: [{ file: "src/drone_state_machine_states.cpp" }],
      },
      {
        path: "src/drone_state_machine_commands.cpp",
        role: "External command validation, target latching, setpoint publication, and takeoff settle checks.",
        sourceRefs: [{ file: "src/drone_state_machine_commands.cpp" }],
      },
    ],
  },
  {
    title: "Actions and Lifecycle",
    purpose: "Defines ROS action contracts and the single owner for active goal handles.",
    files: [
      {
        path: "action/Takeoff.action",
        role: "Takeoff goal/result/feedback contract.",
        sourceRefs: [{ file: "action/Takeoff.action" }],
      },
      {
        path: "action/Land.action",
        role: "Managed landing goal/result/feedback contract.",
        sourceRefs: [{ file: "action/Land.action" }],
      },
      {
        path: "src/action_lifecycle_coordinator.*",
        role: "Stores active Takeoff/Land handles and sends terminal action results.",
        sourceRefs: [{ file: "src/action_lifecycle_coordinator.cpp" }],
      },
    ],
  },
  {
    title: "PX4 Boundary",
    purpose: "Keeps PX4 topics, QoS, command encoding, telemetry, and instance prefixes isolated.",
    files: [
      {
        path: "include/mullet_core/px4_interface.hpp",
        role: "Boundary class for PX4 publishers, subscribers, and status helpers.",
        sourceRefs: [{ file: "include/mullet_core/px4_interface.hpp" }],
      },
      {
        path: "src/px4_interface.cpp",
        role: "Publishes OffboardControlMode, TrajectorySetpoint, VehicleCommand and receives PX4 telemetry.",
        sourceRefs: [{ file: "src/px4_interface.cpp" }],
      },
      {
        path: "include/mullet_core/vehicle_instance.hpp",
        role: "Validates PX4 instance IDs and maps instance N to target_system N + 1.",
        sourceRefs: [{ file: "include/mullet_core/vehicle_instance.hpp" }],
      },
    ],
  },
  {
    title: "Launch, Config, Tests",
    purpose: "Defines runtime parameters, multi-instance spawning, and regression behavior.",
    files: [
      {
        path: "launch/mullet_core.launch.py",
        role: "Creates one node per PX4 instance and loads per-instance or fallback config.",
        sourceRefs: [{ file: "launch/mullet_core.launch.py" }],
      },
      {
        path: "config/mullet_core_params_default.yaml",
        role: "Default control, timeout, command, hover, and safety clamp parameters.",
        sourceRefs: [{ file: "config/mullet_core_params_default.yaml" }],
      },
      {
        path: "test/test_drone_state_machine_behavior.cpp",
        role: "Behavior tests using a fake PX4 peer and action clients.",
        sourceRefs: [{ file: "test/test_drone_state_machine_behavior.cpp" }],
      },
    ],
  },
];

export const interfaces: InterfaceItem[] = [
  {
    name: "takeoff",
    kind: "action",
    direction: "server",
    summary: "Explicit takeoff request with altitude and yaw behavior.",
    details: [
      "Goal: altitude_m, yaw_rad, use_current_yaw",
      "Feedback: state, current altitude, target altitude, error, progress",
      "Accepted only from READY with valid telemetry and disarmed PX4",
    ],
    sourceRefs: [
      { file: "action/Takeoff.action" },
      { file: "src/drone_state_machine_actions.cpp", symbol: "handleTakeoffGoal" },
    ],
  },
  {
    name: "land",
    kind: "action",
    direction: "server",
    summary: "Managed landing request that is intentionally non-cancelable after acceptance.",
    details: [
      "Accepted from HOVER, COMMAND, and active takeoff states",
      "Preempts active Takeoff goals",
      "Succeeds only when PX4 reports disarmed",
    ],
    sourceRefs: [
      { file: "action/Land.action" },
      { file: "src/drone_state_machine_actions.cpp", symbol: "handleLandGoal" },
    ],
  },
  {
    name: "command",
    kind: "topic",
    direction: "subscribe",
    summary: "Temporary velocity command stream in NED frame.",
    details: [
      "Type: std_msgs/msg/Float32MultiArray",
      "Format: [vx_ned_mps, vy_ned_mps, mode]",
      "Only finite values with mode=1 are accepted",
    ],
    sourceRefs: [{ file: "src/drone_state_machine_commands.cpp", symbol: "onCommand" }],
  },
  {
    name: "land_request",
    kind: "topic",
    direction: "subscribe",
    summary: "Emergency escape hatch that starts landing without the Land action protocol.",
    details: [
      "Type: std_msgs/msg/Empty",
      "Can preempt takeoff",
      "Idempotent if already landing",
    ],
    sourceRefs: [
      { file: "src/drone_state_machine_commands.cpp", symbol: "onLandRequest" },
      { file: "src/drone_state_machine.cpp", symbol: "controlTick" },
    ],
  },
  {
    name: "/fmu/in/offboard_control_mode",
    kind: "topic",
    direction: "publish",
    summary: "PX4 proof-of-life and control-mode selector.",
    details: ["Best effort, transient local, depth 1", "Position mode for takeoff/hover, velocity mode for COMMAND"],
    sourceRefs: [{ file: "src/px4_interface.cpp", symbol: "publishOffboardControlMode" }],
  },
  {
    name: "/fmu/in/trajectory_setpoint",
    kind: "topic",
    direction: "publish",
    summary: "PX4 position or velocity setpoint stream.",
    details: ["Unused fields are NaN", "Yaw is held or pointed along horizontal velocity"],
    sourceRefs: [
      { file: "src/px4_interface.cpp", symbol: "publishPositionSetpoint" },
      { file: "src/px4_interface.cpp", symbol: "publishVelocitySetpoint" },
    ],
  },
  {
    name: "/fmu/in/vehicle_command",
    kind: "topic",
    direction: "publish",
    summary: "PX4 vehicle commands for Offboard, Position mode, arming, and landing.",
    details: ["target_system is vehicle_instance + 1", "NAV_LAND is retried while LANDING remains active"],
    sourceRefs: [{ file: "src/px4_interface.cpp", symbol: "publishVehicleCommand" }],
  },
  {
    name: "mullet_core.launch.py",
    kind: "launch",
    direction: "input",
    summary: "Launches one node per explicit or offset PX4 instance.",
    details: [
      "px4_instances overrides num_robots and px4_instance_offset",
      "Duplicate and out-of-range instances are rejected",
      "Instance 0 has no namespace; instance N uses /px4_N",
    ],
    sourceRefs: [{ file: "launch/mullet_core.launch.py" }],
  },
  {
    name: "control and timeout parameters",
    kind: "parameter",
    direction: "input",
    summary: "Control loop rate, warmup, retries, command timeout, takeoff/landing timeouts, and clamps.",
    details: [
      "control_rate_hz defaults to 50",
      "command_retry_interval_s controls repeated vehicle commands",
      "command_timeout_s controls stale command fallback",
    ],
    sourceRefs: [
      { file: "config/mullet_core_params_default.yaml" },
      { file: "src/drone_state_machine.cpp", symbol: "loadParameters" },
    ],
  },
  {
    name: "multi-instance mapping",
    kind: "multi-instance",
    direction: "input",
    summary: "Maps PX4 instance N to namespace /px4_N and target_system N + 1.",
    details: [
      "Valid vehicle_instance range is [0, 254]",
      "Instance 0 uses /fmu/... topics",
      "Instances above 0 use /px4_N/fmu/... topics",
    ],
    sourceRefs: [
      { file: "include/mullet_core/vehicle_instance.hpp" },
      { file: "src/drone_state_machine.cpp", symbol: "DroneStateMachine" },
    ],
  },
];

export const actionLifecycleNotes = [
  {
    title: "Takeoff owner",
    body: "ActionLifecycleCoordinator stores exactly one active Takeoff goal. Duplicate accepted callbacks abort the rejected handle.",
    sourceRefs: [{ file: "src/action_lifecycle_coordinator.cpp", symbol: "acceptTakeoffGoal" }],
  },
  {
    title: "Land owner",
    body: "A Land goal is accepted only when no Land goal is already active, then cancellation is always rejected.",
    sourceRefs: [
      { file: "src/drone_state_machine_actions.cpp", symbol: "handleLandGoal" },
      { file: "src/drone_state_machine_actions.cpp", symbol: "handleLandCancel" },
    ],
  },
  {
    title: "Terminal result timing",
    body: "Landing timeout can abort actions while LANDING continues retrying NAV_LAND until PX4 reports disarmed.",
    sourceRefs: [{ file: "src/drone_state_machine_states.cpp", symbol: "handleLanding" }],
  },
];
