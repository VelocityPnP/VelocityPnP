import { useCallback, useEffect, useMemo, useState } from "react";
import { api, waitForOperation } from "./api";
import type {
  BoardDesign, CalibrationSnapshot, CalibrationStep, Device, MachineEvent,
  MachineStatus, Pose, Residuals,
} from "./types";
import "./styles.css";

type Tab = "machine" | "calibration" | "boards";

const EMPTY_POSE: Pose = { x_mm: 0, y_mm: 0, z_mm: 31.5, rotation_deg: 0 };

export default function App() {
  const [tab, setTab] = useState<Tab>("machine");
  const [status, setStatus] = useState<MachineStatus | null>(null);
  const [devices, setDevices] = useState<Device[]>([]);
  const [events, setEvents] = useState<MachineEvent[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  const refresh = useCallback(async () => {
    try {
      setStatus(await api.status());
      setError(null);
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    }
  }, []);

  useEffect(() => {
    void refresh();
    void api.devices().then(setDevices).catch(() => undefined);
    const timer = window.setInterval(() => void refresh(), 1000);
    const source = new EventSource("/api/v1/events");
    const kinds = ["machine.connected", "machine.disconnected", "machine.preflight_completed", "machine.homing_started", "machine.homing_completed", "machine.position", "machine.actuator", "machine.fault", "machine.emergency_stopped", "control.lease_acquired"];
    for (const kind of kinds) {
      source.addEventListener(kind, (message) => {
        const event = message as MessageEvent;
        setEvents((current) => [{ sequence: Number(event.lastEventId), at: new Date().toISOString(), kind, payload: JSON.parse(event.data) }, ...current].slice(0, 20));
        void refresh();
      });
    }
    return () => { window.clearInterval(timer); source.close(); };
  }, [refresh]);

  const run = async (task: () => Promise<unknown>) => {
    setBusy(true);
    setError(null);
    try { await task(); await refresh(); }
    catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setBusy(false); }
  };

  return (
    <div className="app-shell">
      <aside className="sidebar">
        <div className="brand"><span className="brand-mark">V</span><div>Velocity<span>PnP</span></div></div>
        <nav aria-label="Primary navigation">
          <NavButton active={tab === "machine"} onClick={() => setTab("machine")} icon="⌁">Machine</NavButton>
          <NavButton active={tab === "calibration"} onClick={() => setTab("calibration")} icon="◎">Calibration</NavButton>
          <NavButton active={tab === "boards"} onClick={() => setTab("boards")} icon="▦">Boards</NavButton>
        </nav>
        <div className="sidebar-foot">
          <div className={`state-dot state-${status?.state ?? "disconnected"}`} />
          <div><strong>{pretty(status?.state ?? "offline")}</strong><small>{status?.simulated ? "Simulation" : "Physical machine"}</small></div>
        </div>
      </aside>
      <main>
        <header className="topbar">
          <div><p className="eyebrow">LumenPnP V3.0.4</p><h1>{tab === "machine" ? "Machine control" : tab === "calibration" ? "Calibration lab" : "Board library"}</h1></div>
          <button className="estop" onClick={() => {
            if (window.confirm("Emergency stop immediately? The controller may require a reset.")) void run(async () => waitForOperation(await api.emergencyStop()));
          }}>EMERGENCY STOP</button>
        </header>
        {error && <div className="alert"><span>!</span>{error}<button onClick={() => setError(null)}>Dismiss</button></div>}
        {tab === "machine" && status && <MachinePanel status={status} devices={devices} events={events} busy={busy} run={run} />}
        {tab === "calibration" && <CalibrationPanel simulated={status?.simulated ?? true} setError={setError} />}
        {tab === "boards" && <BoardsPanel setError={setError} />}
      </main>
    </div>
  );
}

function NavButton({ active, icon, children, onClick }: { active: boolean; icon: string; children: React.ReactNode; onClick: () => void }) {
  return <button className={active ? "nav-active" : ""} onClick={onClick}><span>{icon}</span>{children}</button>;
}

function MachinePanel({ status, devices, events, busy, run }: {
  status: MachineStatus; devices: Device[]; events: MachineEvent[]; busy: boolean;
  run: (task: () => Promise<unknown>) => Promise<void>;
}) {
  const lease = status.control_lease;
  const pose = status.pose ?? EMPTY_POSE;
  const machineAction = (action: () => Promise<Parameters<typeof waitForOperation>[0]>) => run(async () => waitForOperation(await action()));
  const jog = (dx: number, dy: number, dz = 0) => {
    if (!lease) return;
    void machineAction(() => api.jog(lease.id, {
      x_mm: pose.x_mm + dx,
      y_mm: pose.y_mm + dy,
      z_mm: pose.z_mm + dz,
    }));
  };

  return <div className="content-grid">
    <section className="card hero-card">
      <div className="card-heading"><div><p className="eyebrow">Control sequence</p><h2>Bring the machine online</h2></div><StateBadge state={status.state} /></div>
      <div className="sequence">
        <SequenceStep number="1" title="Connect" done={status.state !== "disconnected"}>
          {status.state === "disconnected" ? <button disabled={busy} onClick={() => void machineAction(api.connect)}>{status.simulated ? "Connect simulator" : "Connect machine"}</button> :
            <button className="secondary" disabled={busy} onClick={() => void machineAction(api.disconnect)}>Disconnect</button>}
        </SequenceStep>
        <SequenceStep number="2" title="Preflight" done={!(["disconnected", "preflight"] as string[]).includes(status.state)}>
          <button disabled={busy || status.state !== "preflight"} onClick={() => void machineAction(api.preflight)}>Confirm workspace clear</button>
        </SequenceStep>
        <SequenceStep number="3" title="Control lease" done={Boolean(lease)}>
          <button disabled={busy || status.state === "disconnected" || Boolean(lease)} onClick={() => void run(api.lease)}>Acquire 15 min lease</button>
        </SequenceStep>
        <SequenceStep number="4" title="Home" done={status.homed}>
          <button disabled={busy || !lease || !["unhomed", "ready"].includes(status.state)} onClick={() => {
            if (lease && window.confirm("Nozzles level, envelope clear, and power isolation within reach?")) void machineAction(() => api.home(lease.id));
          }}>Home machine</button>
        </SequenceStep>
      </div>
    </section>

    <section className="card pose-card">
      <div className="card-heading"><div><p className="eyebrow">Machine coordinates</p><h2>Position</h2></div><small>mm / degrees</small></div>
      <div className="dro">
        {(["x_mm", "y_mm", "z_mm", "rotation_deg"] as const).map((key) => <div key={key}><span>{key === "rotation_deg" ? "θ" : key[0].toUpperCase()}</span><strong>{pose[key].toFixed(3)}</strong></div>)}
      </div>
      <div className="jog-grid">
        <button disabled={!lease || status.state !== "ready"} onClick={() => jog(0, 1)}>Y+</button>
        <button disabled={!lease || status.state !== "ready"} onClick={() => jog(-1, 0)}>X−</button>
        <button disabled={!lease || status.state !== "ready"} onClick={() => jog(1, 0)}>X+</button>
        <button disabled={!lease || status.state !== "ready"} onClick={() => jog(0, -1)}>Y−</button>
      </div>
      <p className="hint">Conservative 1 mm jogs. Bounds and homing are enforced by the daemon.</p>
    </section>

    <section className="card devices-card">
      <div className="card-heading"><div><p className="eyebrow">Auto-detection</p><h2>Attached devices</h2></div><span className="count">{devices.length}</span></div>
      <div className="device-list">{devices.map((device) => <div className="device" key={device.path}>
        <span className="device-icon">{device.kind === "camera" ? "◉" : "↯"}</span>
        <div><strong>{device.product ?? device.kind}</strong><small>{device.suggested_role ? pretty(device.suggested_role) : device.path}</small></div>
        <code>{device.vendor_id}:{device.product_id}</code>
      </div>)}</div>
    </section>

    <section className="card events-card">
      <div className="card-heading"><div><p className="eyebrow">Audit stream</p><h2>Recent activity</h2></div></div>
      <div className="event-list">{events.length === 0 ? <p className="empty">Events appear here as operations run.</p> : events.map((event) =>
        <div key={`${event.sequence}-${event.kind}`}><time>{new Date(event.at).toLocaleTimeString()}</time><strong>{pretty(event.kind)}</strong></div>)}</div>
    </section>
  </div>;
}

function SequenceStep({ number, title, done, children }: { number: string; title: string; done: boolean; children: React.ReactNode }) {
  return <div className={`sequence-step ${done ? "step-done" : ""}`}><span>{done ? "✓" : number}</span><strong>{title}</strong>{children}</div>;
}

function StateBadge({ state }: { state: string }) { return <span className={`badge state-${state}`}>{pretty(state)}</span>; }

function CalibrationPanel({ simulated, setError }: { simulated: boolean; setError: (value: string | null) => void }) {
  const [steps, setSteps] = useState<CalibrationStep[]>([]);
  const [snapshots, setSnapshots] = useState<CalibrationSnapshot[]>([]);
  const [active, setActive] = useState<CalibrationSnapshot | null>(null);
  const [metrics, setMetrics] = useState<Residuals>({ sample_count: 30, rms_mm: 0, p95_mm: 0, max_mm: 0 });

  const load = useCallback(async () => {
    try {
      const [workflow, values] = await Promise.all([api.workflow(), api.calibrations()]);
      setSteps(workflow); setSnapshots(values); setActive((current) => values.find((v) => v.id === current?.id) ?? values[0] ?? null);
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
  }, [setError]);
  useEffect(() => { void load(); }, [load]);

  const completed = useMemo(() => new Set(active?.completed_steps ?? []), [active]);
  const canRun = (step: CalibrationStep) => step.dependencies.every((dependency) => completed.has(dependency));
  const record = async (step: CalibrationStep) => {
    if (!active) return;
    try {
      const needsMetrics = ["visual_homing", "motion_geometry", "nozzle_runout", "validation"].includes(step.id);
      const updated = await api.completeCalibration(active.id, step.id, needsMetrics ? metrics : null, { source: "operator_ui" });
      setActive(updated); await load();
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
  };

  return <div className="calibration-layout">
    <section className="card calibration-summary">
      <p className="eyebrow">Revisioned measurements</p><h2>{active ? `Snapshot ${active.id.slice(0, 8)}` : "No calibration snapshot"}</h2>
      <p>Every result retains provenance and invalidates dependent measurements when changed.</p>
      {simulated && <div className="notice">Simulation results can exercise the workflow but can never mark a machine production-ready.</div>}
      <button onClick={() => void api.createCalibration("3286336D3331").then((value) => { setActive(value); void load(); }).catch((reason) => setError(String(reason)))}>New snapshot</button>
      <div className="metric-form">
        <label>Samples<input type="number" min="1" value={metrics.sample_count} onChange={(e) => setMetrics({ ...metrics, sample_count: Number(e.target.value) })} /></label>
        <label>RMS mm<input type="number" min="0" step="0.001" value={metrics.rms_mm} onChange={(e) => setMetrics({ ...metrics, rms_mm: Number(e.target.value) })} /></label>
        <label>P95 mm<input type="number" min="0" step="0.001" value={metrics.p95_mm} onChange={(e) => setMetrics({ ...metrics, p95_mm: Number(e.target.value) })} /></label>
        <label>Max mm<input type="number" min="0" step="0.001" value={metrics.max_mm} onChange={(e) => setMetrics({ ...metrics, max_mm: Number(e.target.value) })} /></label>
      </div>
      <small>Measurement entry is enabled now; automated capture replaces manual entry as each CV stage lands.</small>
    </section>
    <section className="workflow">{steps.map((step, index) => {
      const done = completed.has(step.id); const ready = Boolean(active) && canRun(step);
      return <article className={`workflow-step ${done ? "workflow-done" : ready ? "workflow-ready" : "workflow-locked"}`} key={step.id}>
        <div className="workflow-index">{done ? "✓" : String(index + 1).padStart(2, "0")}</div>
        <div><div className="step-labels">{step.requires_motion && <span>Motion</span>}{step.requires_fixture && <span>Fixture</span>}</div><h3>{step.title}</h3><p>{step.description}</p>
          {step.dependencies.length > 0 && <small>Requires {step.dependencies.map(pretty).join(", ")}</small>}</div>
        <button disabled={!ready || done} onClick={() => void record(step)}>{done ? "Recorded" : ready ? "Record result" : "Locked"}</button>
      </article>;
    })}</section>
    {snapshots.length > 1 && <p className="snapshot-count">{snapshots.length} calibration snapshots retained</p>}
  </div>;
}

function BoardsPanel({ setError }: { setError: (value: string | null) => void }) {
  const [boards, setBoards] = useState<BoardDesign[]>([]);
  const [pcb, setPcb] = useState<File | null>(null);
  const [bom, setBom] = useState<File | null>(null);
  const load = useCallback(() => void api.boards().then(setBoards).catch((reason) => setError(String(reason))), [setError]);
  useEffect(load, [load]);
  const upload = async () => {
    if (!pcb) return;
    try { await api.importBoard(pcb, bom); setPcb(null); setBom(null); load(); }
    catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
  };
  return <div className="boards-layout">
    <section className="card import-card"><p className="eyebrow">KiCad-native workflow</p><h2>Import a board</h2><p>Use the PCB as the source of placement geometry. A CSV BOM is optional enrichment.</p>
      <label className="file-drop"><span>{pcb?.name ?? "Choose .kicad_pcb"}</span><input type="file" accept=".kicad_pcb" onChange={(e) => setPcb(e.target.files?.[0] ?? null)} /></label>
      <label className="file-drop optional"><span>{bom?.name ?? "Optional BOM (.csv)"}</span><input type="file" accept=".csv,text/csv" onChange={(e) => setBom(e.target.files?.[0] ?? null)} /></label>
      <button disabled={!pcb} onClick={() => void upload()}>Import and validate</button>
    </section>
    <section className="board-list">{boards.length === 0 ? <div className="card empty-board"><strong>No boards imported</strong><p>Your validated designs will appear here.</p></div> : boards.map((board) => {
      const active = board.placements.filter((p) => p.enabled && !p.fiducial).length;
      return <article className="card board-card" key={board.id}><div className="board-preview"><span>PCB</span><i /></div><div><p className="eyebrow">KiCad {board.source_version ?? "unknown"}</p><h2>{board.name}</h2><div className="board-stats"><span><strong>{active}</strong> placements</span><span><strong>{board.placements.filter((p) => p.fiducial).length}</strong> fiducials</span><span><strong>{formatSize(board)}</strong> size</span></div>{board.warnings.map((warning) => <p className="warning" key={warning}>⚠ {warning}</p>)}</div></article>;
    })}</section>
  </div>;
}

export function pretty(value: string): string {
  return value.replaceAll(".", " ").replaceAll("_", " ").replace(/\b\w/g, (char) => char.toUpperCase());
}

function formatSize(board: BoardDesign): string {
  return board.width_mm != null && board.height_mm != null ? `${board.width_mm.toFixed(1)} × ${board.height_mm.toFixed(1)} mm` : "Unknown";
}
