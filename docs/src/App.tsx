import {
  Boxes,
  Cable,
  CheckCircle2,
  CircleDot,
  FileCode2,
  GitBranch,
  type LucideIcon,
  PlayCircle,
  RadioTower,
  Workflow,
} from "lucide-react";
import { useMemo, useState } from "react";

import {
  actionLifecycleNotes,
  fileGroups,
  interfaces,
  scenarios,
  states,
  transitions,
  type ActionOwnership,
  type DroneState,
  type InterfaceItem,
  type Scenario,
  type ScenarioStep,
  type SourceRef,
  type StateNode,
} from "./data/mulletCoreModel";

type TabId = "state-map" | "scenarios" | "actions" | "interfaces" | "files";

type StatePosition = {
  x: number;
  y: number;
};

const STATE_NODE_WIDTH = 164;
const STATE_NODE_HEIGHT = 56;
const STATE_NODE_EDGE_GAP = 6;

const tabs: Array<{ id: TabId; label: string; icon: LucideIcon }> = [
  { id: "state-map", label: "State Map", icon: Workflow },
  { id: "scenarios", label: "Scenarios", icon: PlayCircle },
  { id: "actions", label: "Actions", icon: GitBranch },
  { id: "interfaces", label: "Interfaces", icon: Cable },
  { id: "files", label: "Files", icon: FileCode2 },
];

const statePositions: Record<DroneState, StatePosition> = {
  INIT: { x: 90, y: 80 },
  READY: { x: 250, y: 80 },
  TAKEOFF_REQUESTED: { x: 430, y: 80 },
  WARMUP_SETPOINTS: { x: 640, y: 80 },
  REQUEST_OFFBOARD: { x: 850, y: 80 },
  REQUEST_ARM: { x: 1060, y: 80 },
  TAKEOFF: { x: 1060, y: 230 },
  HOVER: { x: 850, y: 230 },
  COMMAND: { x: 640, y: 230 },
  FAILSAFE_HOLD: { x: 430, y: 230 },
  CANCELING_TO_LAND: { x: 250, y: 380 },
  LANDING: { x: 520, y: 380 },
  DISARMED: { x: 790, y: 380 },
  TAKEOFF_FAILED: { x: 1010, y: 380 },
  TAKEOFF_CANCELED: { x: 1010, y: 500 },
};

const phaseLabels: Record<StateNode["phase"], string> = {
  startup: "Startup",
  takeoff: "Takeoff",
  flight: "Flight",
  recovery: "Recovery",
  terminal: "Terminal",
};

const interfaceKinds: Array<InterfaceItem["kind"] | "all"> = [
  "all",
  "action",
  "topic",
  "parameter",
  "launch",
  "timing",
  "multi-instance",
];

function App() {
  const [activeTab, setActiveTab] = useState<TabId>("state-map");
  const [selectedStateId, setSelectedStateId] = useState<DroneState>("LANDING");
  const [selectedScenarioId, setSelectedScenarioId] = useState(scenarios[0].id);
  const [selectedStepIndex, setSelectedStepIndex] = useState(0);
  const [interfaceFilter, setInterfaceFilter] = useState<InterfaceItem["kind"] | "all">("all");

  const stateById = useMemo(() => new Map(states.map((state) => [state.id, state])), []);
  const selectedState = stateById.get(selectedStateId) ?? states[0];
  const selectedScenario = scenarios.find((scenario) => scenario.id === selectedScenarioId) ?? scenarios[0];
  const selectedStep = selectedScenario.steps[selectedStepIndex] ?? selectedScenario.steps[0];
  const filteredInterfaces =
    interfaceFilter === "all"
      ? interfaces
      : interfaces.filter((item) => item.kind === interfaceFilter);

  function selectScenario(nextScenario: Scenario) {
    setSelectedScenarioId(nextScenario.id);
    setSelectedStepIndex(0);
  }

  return (
    <main className="app-shell">
      <Hero />
      <nav className="tab-bar" aria-label="Documentation sections">
        {tabs.map((tab) => {
          const Icon = tab.icon;
          return (
            <button
              className={tab.id === activeTab ? "tab active" : "tab"}
              key={tab.id}
              onClick={() => setActiveTab(tab.id)}
              type="button"
            >
              <Icon size={18} aria-hidden="true" />
              <span>{tab.label}</span>
            </button>
          );
        })}
      </nav>

      {activeTab === "state-map" && (
        <StateMapView
          selectedState={selectedState}
          selectedStateId={selectedStateId}
          onSelectState={setSelectedStateId}
        />
      )}
      {activeTab === "scenarios" && (
        <ScenarioView
          selectedScenario={selectedScenario}
          selectedStep={selectedStep}
          selectedStepIndex={selectedStepIndex}
          onSelectScenario={selectScenario}
          onSelectStep={setSelectedStepIndex}
        />
      )}
      {activeTab === "actions" && <ActionLifecycleView />}
      {activeTab === "interfaces" && (
        <InterfacesView
          filteredInterfaces={filteredInterfaces}
          interfaceFilter={interfaceFilter}
          onSetInterfaceFilter={setInterfaceFilter}
        />
      )}
      {activeTab === "files" && <FileExplorerView />}
    </main>
  );
}

function Hero() {
  return (
    <header className="hero">
      <div>
        <p className="eyebrow">PX4 Offboard Supervisor</p>
        <h1>mullet_core</h1>
        <p className="hero-copy">
          Interactive maintainer documentation for state transitions, ROS actions, PX4 command timing,
          command fallback, landing recovery, and package structure.
        </p>
      </div>
      <div className="hero-metrics" aria-label="Repository summary">
        <Metric icon={CircleDot} label="States" value={String(states.length)} />
        <Metric icon={GitBranch} label="Transitions" value={String(transitions.length)} />
        <Metric icon={PlayCircle} label="Scenarios" value={String(scenarios.length)} />
        <Metric icon={RadioTower} label="PX4 Boundary" value="Isolated" />
      </div>
    </header>
  );
}

function Metric({
  icon: Icon,
  label,
  value,
}: {
  icon: LucideIcon;
  label: string;
  value: string;
}) {
  return (
    <div className="metric">
      <Icon size={18} aria-hidden="true" />
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

function StateMapView({
  selectedState,
  selectedStateId,
  onSelectState,
}: {
  selectedState: StateNode;
  selectedStateId: DroneState;
  onSelectState: (state: DroneState) => void;
}) {
  const relatedTransitions = transitions.filter(
    (transition) => transition.from === selectedStateId || transition.to === selectedStateId,
  );

  return (
    <section className="workspace state-workspace">
      <div className="map-surface">
        <div className="section-heading">
          <div>
            <p className="eyebrow">Clickable Graph</p>
            <h2>State-machine flow</h2>
          </div>
          <PhaseLegend />
        </div>
        <StateGraph selectedStateId={selectedStateId} onSelectState={onSelectState} />
      </div>

      <aside className="details-panel">
        <StateDetails state={selectedState} />
        <div className="divider" />
        <h3>Related transitions</h3>
        <div className="transition-list">
          {relatedTransitions.map((transition) => (
            <button
              type="button"
              className="transition-row"
              key={`${transition.from}-${transition.to}-${transition.event}`}
              onClick={() => onSelectState(transition.to)}
            >
              <span>
                {transition.from} → {transition.to}
              </span>
              <strong>{transition.event}</strong>
            </button>
          ))}
        </div>
      </aside>
    </section>
  );
}

function StateGraph({
  selectedStateId,
  onSelectState,
}: {
  selectedStateId: DroneState;
  onSelectState: (state: DroneState) => void;
}) {
  return (
    <svg className="state-graph" viewBox="0 0 1160 590" role="img" aria-label="mullet_core state graph">
      <defs>
        <marker id="arrow" markerHeight="10" markerWidth="10" orient="auto" refX="8" refY="3">
          <path d="M0,0 L0,6 L9,3 z" fill="currentColor" />
        </marker>
      </defs>
      {transitions.map((transition) => {
        const from = statePositions[transition.from];
        const to = statePositions[transition.to];
        const active = transition.from === selectedStateId || transition.to === selectedStateId;
        const isLoop = transition.from === transition.to;
        const start = isLoop ? from : connectionPoint(from, to, STATE_NODE_EDGE_GAP);
        const end = isLoop ? to : connectionPoint(to, from, STATE_NODE_EDGE_GAP);
        const midX = (start.x + end.x) / 2;
        const midY = (start.y + end.y) / 2;
        return (
          <g
            className={active ? "edge active" : "edge"}
            key={`${transition.from}-${transition.to}-${transition.event}`}
          >
            {isLoop ? (
              <path
                d={`M ${from.x + 60} ${from.y - 12} C ${from.x + 115} ${from.y - 95}, ${
                  from.x - 115
                } ${from.y - 95}, ${from.x - 60} ${from.y - 12}`}
                markerEnd="url(#arrow)"
              />
            ) : (
              <path
                d={`M ${start.x} ${start.y} C ${midX} ${start.y}, ${midX} ${end.y}, ${end.x} ${end.y}`}
                markerEnd="url(#arrow)"
              />
            )}
            {active && (
              <text x={midX} y={midY - 12} className="edge-label">
                {transition.event}
              </text>
            )}
          </g>
        );
      })}

      {states.map((state) => {
        const position = statePositions[state.id];
        const selected = state.id === selectedStateId;
        return (
          <g
            className={`state-node ${state.phase} ${selected ? "selected" : ""}`}
            key={state.id}
            onClick={() => onSelectState(state.id)}
            onKeyDown={(event) => {
              if (event.key === "Enter" || event.key === " ") {
                onSelectState(state.id);
              }
            }}
            role="button"
            tabIndex={0}
            aria-label={`Select ${state.id}`}
          >
            <rect
              x={position.x - STATE_NODE_WIDTH / 2}
              y={position.y - STATE_NODE_HEIGHT / 2}
              width={STATE_NODE_WIDTH}
              height={STATE_NODE_HEIGHT}
              rx="8"
            />
            <text x={position.x} y={position.y - 4} textAnchor="middle" className="node-title">
              {state.id}
            </text>
            <text x={position.x} y={position.y + 15} textAnchor="middle" className="node-phase">
              {phaseLabels[state.phase]}
            </text>
          </g>
        );
      })}

      <g className="boundary-label">
        <rect x="28" y="26" width="1104" height="524" rx="18" />
        <text x="52" y="540">
          Vehicle control continues after some action terminal results, especially inside LANDING.
        </text>
      </g>
    </svg>
  );
}

function connectionPoint(from: StatePosition, to: StatePosition, gap: number): StatePosition {
  const dx = to.x - from.x;
  const dy = to.y - from.y;
  if (dx === 0 && dy === 0) {
    return from;
  }

  const halfWidth = STATE_NODE_WIDTH / 2;
  const halfHeight = STATE_NODE_HEIGHT / 2;
  const xScale = dx === 0 ? Number.POSITIVE_INFINITY : halfWidth / Math.abs(dx);
  const yScale = dy === 0 ? Number.POSITIVE_INFINITY : halfHeight / Math.abs(dy);
  const edgeScale = Math.min(xScale, yScale);
  const centerDistance = Math.hypot(dx, dy);
  const gapScale = gap / centerDistance;
  const scale = edgeScale + gapScale;

  return {
    x: from.x + dx * scale,
    y: from.y + dy * scale,
  };
}

function PhaseLegend() {
  return (
    <div className="phase-legend" aria-label="State phases">
      {(Object.keys(phaseLabels) as StateNode["phase"][]).map((phase) => (
        <span className={`legend-pill ${phase}`} key={phase}>
          {phaseLabels[phase]}
        </span>
      ))}
    </div>
  );
}

function StateDetails({ state }: { state: StateNode }) {
  return (
    <article>
      <div className="detail-title">
        <span className={`phase-badge ${state.phase}`}>{phaseLabels[state.phase]}</span>
        <h2>{state.id}</h2>
      </div>
      <p className="summary">{state.summary}</p>
      <InfoList title="Entry" items={state.entry} />
      <InfoList title="During" items={state.during} />
      <InfoList title="Exits" items={state.exits} />
      <InfoList title="PX4 traffic" items={state.px4Traffic} />
      <OwnershipBlock ownership={state.actionOwnership} />
      <SourceRefs refs={state.sourceRefs} />
    </article>
  );
}

function ScenarioView({
  selectedScenario,
  selectedStep,
  selectedStepIndex,
  onSelectScenario,
  onSelectStep,
}: {
  selectedScenario: Scenario;
  selectedStep: ScenarioStep;
  selectedStepIndex: number;
  onSelectScenario: (scenario: Scenario) => void;
  onSelectStep: (stepIndex: number) => void;
}) {
  return (
    <section className="workspace scenario-workspace">
      <div className="scenario-index">
        <div className="section-heading">
          <div>
            <p className="eyebrow">Question-first Flows</p>
            <h2>Scenarios</h2>
          </div>
        </div>
        <div className="scenario-list">
          {scenarios.map((scenario) => (
            <button
              className={scenario.id === selectedScenario.id ? "scenario-card active" : "scenario-card"}
              key={scenario.id}
              onClick={() => onSelectScenario(scenario)}
              type="button"
            >
              <span>{scenario.question}</span>
              <strong>{scenario.title}</strong>
              <small>{scenario.summary}</small>
            </button>
          ))}
        </div>
      </div>

      <div className="scenario-main">
        <div className="scenario-header">
          <p className="eyebrow">Selected Scenario</p>
          <h2>{selectedScenario.title}</h2>
          <p>{selectedScenario.summary}</p>
        </div>

        <div className="stepper">
          {selectedScenario.steps.map((step, index) => (
            <button
              className={index === selectedStepIndex ? "step-dot active" : "step-dot"}
              key={`${step.fromState}-${step.toState}-${step.event}`}
              onClick={() => onSelectStep(index)}
              type="button"
              aria-label={`Step ${index + 1}: ${step.event}`}
            >
              {index + 1}
            </button>
          ))}
        </div>

        <ScenarioStepDetails step={selectedStep} />
      </div>
    </section>
  );
}

function ScenarioStepDetails({ step }: { step: ScenarioStep }) {
  return (
    <article className="step-details">
      <div className="state-pair">
        <span>{step.fromState}</span>
        <GitBranch size={20} aria-hidden="true" />
        <span>{step.toState}</span>
      </div>
      <h3>{step.event}</h3>
      <p>{step.nodeResponse}</p>
      <div className="fact-grid">
        {step.px4Command && <Fact label="PX4 command" value={step.px4Command} />}
        {step.actionResult && <Fact label="Action result" value={step.actionResult} />}
        {step.safetyNote && <Fact label="Safety note" value={step.safetyNote} />}
      </div>
      <OwnershipBlock ownership={step.actionOwnership} />
      <SourceRefs refs={step.sourceRefs} />
    </article>
  );
}

function ActionLifecycleView() {
  const actionStates = states.filter(
    (state) =>
      state.actionOwnership.takeoffGoal !== "None" ||
      state.actionOwnership.landGoal !== "None" ||
      state.id === "LANDING" ||
      state.id === "DISARMED",
  );

  return (
    <section className="workspace action-workspace">
      <div>
        <div className="section-heading">
          <div>
            <p className="eyebrow">ROS Action Ownership</p>
            <h2>Control state and action lifecycle</h2>
          </div>
        </div>
        <div className="note-grid">
          {actionLifecycleNotes.map((note) => (
            <article className="note" key={note.title}>
              <CheckCircle2 size={20} aria-hidden="true" />
              <h3>{note.title}</h3>
              <p>{note.body}</p>
              <SourceRefs refs={note.sourceRefs} compact />
            </article>
          ))}
        </div>
      </div>

      <div className="ownership-table">
        <div className="table-row table-head">
          <span>State</span>
          <span>Takeoff</span>
          <span>Land</span>
          <span>Terminal result</span>
          <span>Vehicle control</span>
        </div>
        {actionStates.map((state) => (
          <div className="table-row" key={state.id}>
            <strong>{state.id}</strong>
            <span>{state.actionOwnership.takeoffGoal}</span>
            <span>{state.actionOwnership.landGoal}</span>
            <span>{state.actionOwnership.terminalResult}</span>
            <span>{state.actionOwnership.vehicleControl}</span>
          </div>
        ))}
      </div>
    </section>
  );
}

function InterfacesView({
  filteredInterfaces,
  interfaceFilter,
  onSetInterfaceFilter,
}: {
  filteredInterfaces: InterfaceItem[];
  interfaceFilter: InterfaceItem["kind"] | "all";
  onSetInterfaceFilter: (kind: InterfaceItem["kind"] | "all") => void;
}) {
  return (
    <section className="workspace interface-workspace">
      <div className="section-heading full">
        <div>
          <p className="eyebrow">Boundary Reference</p>
          <h2>Actions, topics, launch, params, and PX4 mapping</h2>
        </div>
        <div className="filter-row" aria-label="Interface filters">
          {interfaceKinds.map((kind) => (
            <button
              className={interfaceFilter === kind ? "filter active" : "filter"}
              key={kind}
              onClick={() => onSetInterfaceFilter(kind)}
              type="button"
            >
              {kind}
            </button>
          ))}
        </div>
      </div>
      <div className="interface-grid">
        {filteredInterfaces.map((item) => (
          <article className="interface-card" key={`${item.kind}-${item.name}`}>
            <div className="interface-topline">
              <span className="kind">{item.kind}</span>
              {item.direction && <span className="direction">{item.direction}</span>}
            </div>
            <h3>{item.name}</h3>
            <p>{item.summary}</p>
            <InfoList title="Details" items={item.details} />
            <SourceRefs refs={item.sourceRefs} compact />
          </article>
        ))}
      </div>
    </section>
  );
}

function FileExplorerView() {
  return (
    <section className="workspace files-workspace">
      <div className="section-heading full">
        <div>
          <p className="eyebrow">Package Explorer</p>
          <h2>Where behavior lives</h2>
        </div>
      </div>
      <div className="file-groups">
        {fileGroups.map((group) => (
          <section className="file-group" key={group.title}>
            <div className="file-group-title">
              <Boxes size={20} aria-hidden="true" />
              <div>
                <h3>{group.title}</h3>
                <p>{group.purpose}</p>
              </div>
            </div>
            <div className="file-list">
              {group.files.map((file) => (
                <article className="file-row" key={file.path}>
                  <code>{file.path}</code>
                  <p>{file.role}</p>
                  <SourceRefs refs={file.sourceRefs} compact />
                </article>
              ))}
            </div>
          </section>
        ))}
      </div>
    </section>
  );
}

function InfoList({ title, items }: { title: string; items: string[] }) {
  return (
    <div className="info-list">
      <h4>{title}</h4>
      <ul>
        {items.map((item) => (
          <li key={item}>{item}</li>
        ))}
      </ul>
    </div>
  );
}

function OwnershipBlock({ ownership }: { ownership: ActionOwnership }) {
  return (
    <div className="ownership-block">
      <h4>Action ownership</h4>
      <div className="ownership-grid">
        <Fact label="Takeoff" value={ownership.takeoffGoal} />
        <Fact label="Land" value={ownership.landGoal} />
        <Fact label="Cancellation" value={ownership.cancellation} />
        <Fact label="Terminal result" value={ownership.terminalResult} />
        <Fact label="Vehicle control" value={ownership.vehicleControl} />
      </div>
    </div>
  );
}

function Fact({ label, value }: { label: string; value: string }) {
  return (
    <div className="fact">
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

function SourceRefs({ refs, compact = false }: { refs: SourceRef[]; compact?: boolean }) {
  return (
    <div className={compact ? "source-refs compact" : "source-refs"}>
      <h4>Source refs</h4>
      <div>
        {refs.map((ref) => (
          <code key={`${ref.file}-${ref.symbol ?? "file"}-${ref.note ?? ""}`}>
            {ref.file}
            {ref.symbol ? `::${ref.symbol}` : ""}
            {ref.note ? ` (${ref.note})` : ""}
          </code>
        ))}
      </div>
    </div>
  );
}

export default App;
