import { useCallback, useEffect, useMemo, useState } from "react";
import { api, waitForOperation } from "./api";
import type {
  AutomatedCalibrationRun, BoardDesign, BoardRegistrationRun, CalibrationArtifactPlan,
  CalibrationSnapshot, CalibrationStep, Device, MachineEvent, MachineStatus, Pose,
  SensorReading, VisualHomingResult, WorkspaceBoardCandidate, WorkspaceBoardDiscoveryReport,
  WorkspaceChangeSetReport, WorkspaceInventoryBaseline, WorkspaceScanReport,
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
    const kinds = ["machine.connected", "machine.disconnected", "machine.preflight_completed", "machine.homing_started", "machine.homing_completed", "machine.position", "machine.actuator", "machine.sensor", "machine.visual_home_applied", "machine.fine_z_home_applied", "machine.fault", "machine.emergency_stopped", "control.lease_acquired", "control.lease_renewed"];
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
          <div><p className="eyebrow">LumenPnP V3.0.4</p><h1>{tab === "machine" ? "Machine control" : tab === "calibration" ? "Machine calibration" : "Board library"}</h1></div>
          <button className="estop" onClick={() => {
            if (window.confirm("Emergency stop immediately? The controller may require a reset.")) void run(async () => waitForOperation(await api.emergencyStop()));
          }}>EMERGENCY STOP</button>
        </header>
        {error && <div className="alert"><span>!</span>{error}<button onClick={() => setError(null)}>Dismiss</button></div>}
        {tab === "machine" && status && <MachinePanel status={status} devices={devices} events={events} busy={busy} run={run} />}
        {tab === "calibration" && status && <CalibrationPanel status={status} devices={devices} setError={setError} />}
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
  const [readings, setReadings] = useState<Record<string, SensorReading>>({});
  const [visualHome, setVisualHome] = useState<VisualHomingResult | null>(null);
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
  const actuatorAllowed = Boolean(lease)
    && ["preflight", "unhomed", "ready"].includes(status.state);
  const setActuator = (actuatorId: string, enabled: boolean) => {
    if (!lease) return;
    if (enabled && actuatorId.startsWith("vacuum_")
      && !window.confirm(`Enable ${pretty(actuatorId)}? Use Off immediately after the check.`)) return;
    void machineAction(() => api.actuate(lease.id, actuatorId, enabled));
  };
  const readSensor = (sensorId: string) => {
    if (!lease) return;
    void run(async () => {
      const reading = await api.readSensor(lease.id, sensorId);
      setReadings((current) => ({ ...current, [sensorId]: reading }));
    });
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

    <section className="card hardware-check-card">
      <div className="card-heading"><div><p className="eyebrow">Initial validation</p><h2>Actuators and datum</h2></div></div>
      <div className="hardware-checks">
        <div><strong>Ring lights</strong><span><button disabled={busy || !actuatorAllowed} onClick={() => setActuator("lights", true)}>On</button><button disabled={busy || !actuatorAllowed} onClick={() => setActuator("lights", false)}>Off</button></span></div>
        {(["vacuum_1", "vacuum_2"] as const).map((sensorId) => <div key={sensorId}><strong>{pretty(sensorId)}</strong><code>{readings[sensorId]?.raw_value ?? "—"} raw</code><span><button disabled={busy || !actuatorAllowed} onClick={() => setActuator(sensorId, true)}>On</button><button disabled={busy || !actuatorAllowed} onClick={() => setActuator(sensorId, false)}>Off</button><button disabled={busy || !actuatorAllowed} onClick={() => readSensor(sensorId)}>Read</button></span></div>)}
      </div>
      <button className="datum-observe" disabled={busy || !lease || status.state !== "ready"} onClick={() => {
        if (lease && window.confirm("Move to the primary datum seed and capture an observation?")) void run(async () => setVisualHome(await api.visualHome(lease.id)));
      }}>Observe primary datum</button>
      {visualHome && <p className="hint">Datum error {visualHome.initial.error_x_px.toFixed(2)}, {visualHome.initial.error_y_px.toFixed(2)} px; proposed move {visualHome.initial.correction_x_mm.toFixed(3)}, {visualHome.initial.correction_y_mm.toFixed(3)} mm. Coordinate reset not applied.</p>}
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

function CalibrationPanel({ status, devices, setError }: {
  status: MachineStatus; devices: Device[]; setError: (value: string | null) => void;
}) {
  const [steps, setSteps] = useState<CalibrationStep[]>([]);
  const [snapshots, setSnapshots] = useState<CalibrationSnapshot[]>([]);
  const [active, setActive] = useState<CalibrationSnapshot | null>(null);
  const [activeRun, setActiveRun] = useState<AutomatedCalibrationRun | null>(null);
  const [artifactPlan, setArtifactPlan] = useState<CalibrationArtifactPlan | null>(null);
  const [starting, setStarting] = useState(false);

  const load = useCallback(async () => {
    try {
      const [workflow, values] = await Promise.all([api.workflow(), api.calibrations()]);
      setSteps(workflow); setSnapshots(values); setActive((current) => values.find((v) => v.id === current?.id) ?? values[0] ?? null);
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
  }, [setError]);
  useEffect(() => { void load(); }, [load]);

  useEffect(() => {
    if (!active) { setActiveRun(null); return; }
    void api.calibrationRuns(active.id)
      .then((runs) => setActiveRun(runs[0] ?? null))
      .catch((reason) => setError(reason instanceof Error ? reason.message : String(reason)));
  }, [active?.id, setError]);

  useEffect(() => {
    if (!active) { setArtifactPlan(null); return; }
    void api.calibrationPlan(active.id)
      .then(setArtifactPlan)
      .catch((reason) => setError(reason instanceof Error ? reason.message : String(reason)));
  }, [active?.id, setError]);

  useEffect(() => {
    if (!activeRun || !["pending", "running"].includes(activeRun.status)) return;
    const timer = window.setInterval(() => {
      void api.calibrationRun(activeRun.id).then((run) => {
        setActiveRun(run);
        if (!["pending", "running"].includes(run.status)) void load();
      }).catch((reason) => setError(reason instanceof Error ? reason.message : String(reason)));
    }, 1000);
    return () => window.clearInterval(timer);
  }, [activeRun?.id, activeRun?.status, load, setError]);

  const completed = useMemo(() => new Set(active?.completed_steps ?? []), [active]);
  const createSnapshot = async () => {
    const serial = devices.find((device) => device.kind === "serial_controller")?.serial ?? "unassigned";
    try {
      const snapshot = await api.createCalibration(serial);
      setSnapshots((current) => [snapshot, ...current]);
      setActiveRun(null);
      setActive(snapshot);
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
  };
  const start = async () => {
    if (!active) return;
    if (!window.confirm(
      "Start automated calibration? Confirm both nozzles are level, the travel envelope is clear, power isolation is within reach, and both nozzle tubes are seated, routed normally, and free of snags. The bottom camera will identify each tip before any rotation.",
    )) return;
    setStarting(true); setError(null);
    try {
      const lease = status.control_lease
        ? await api.renewLease(status.control_lease.id, 3600)
        : await api.lease("VelocityPnP calibration", 3600);
      const started = await api.startCalibration(active.id, lease.id);
      setActiveRun(started.run);
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setStarting(false); }
  };
  const recalibrateHeadGeometry = async () => {
    if (!active) return;
    if (!window.confirm(
      "Recalibrate head geometry from the validated parent snapshot? Confirm both nozzles are level, the travel envelope is clear, power isolation is within reach, and both nozzle tubes are seated, routed normally, and free of snags. The machine will perform one live visual alignment, then measure camera transfer, nozzle offsets, runout, and height models.",
    )) return;
    setStarting(true); setError(null);
    try {
      const lease = status.control_lease
        ? await api.renewLease(status.control_lease.id, 3600)
        : await api.lease("VelocityPnP head geometry", 3600);
      const started = await api.startHeadGeometryCalibration(active.id, lease.id);
      setActiveRun(started.run);
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setStarting(false); }
  };
  const vacuumProbe = async (nozzleId: "N1" | "N2") => {
    if (!active) return;
    if (!window.confirm(
      `Probe ${nozzleId} on the datum-board sealing target? Confirm the board is fixed, the target and nozzle tips are clean, both nozzles are level, the travel envelope is clear, and power isolation is within reach. X/Y motion occurs only while both nozzles are level; the selected nozzle then lowers at fixed X/Y and does not move below working Z0.`,
    )) return;
    setStarting(true); setError(null);
    try {
      const lease = status.control_lease
        ? await api.renewLease(status.control_lease.id, 900)
        : await api.lease("VelocityPnP vacuum probe", 900);
      await waitForOperation(await api.vacuumProbe(active.id, lease.id, nozzleId));
      const snapshot = await api.calibration(active.id);
      setActive(snapshot);
      setSnapshots((current) => current.map((value) => value.id === snapshot.id ? snapshot : value));
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setStarting(false); }
  };
  const fineHome = async () => {
    if (!active) return;
    if (!window.confirm(
      "Fine-home XYZ from the datum PCB? Confirm the board is fixed, the sealing target and nozzle tips are clean, both nozzles are level, the travel envelope is clear, and power isolation is within reach. The machine will visually refine X/Y, identify and center N2, reproduce the validated vacuum contact, and apply a bounded Z coordinate reset.",
    )) return;
    setStarting(true); setError(null);
    try {
      const lease = status.control_lease
        ? await api.renewLease(status.control_lease.id, 900)
        : await api.lease("VelocityPnP fine home", 900);
      await waitForOperation(await api.fineHome(active.id, lease.id));
      await load();
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setStarting(false); }
  };
  const characterizeVacuum = async (nozzleId: "N1" | "N2") => {
    if (!active) return;
    if (!window.confirm(
      `Characterize ${nozzleId} on the datum-board sealing target? Confirm the board is fixed, the target and nozzle tips are clean, both nozzles are level, the travel envelope is clear, and power isolation is within reach. The pump will be on only during the narrowed contact search and sealed sampling.`,
    )) return;
    setStarting(true); setError(null);
    try {
      const lease = status.control_lease
        ? await api.renewLease(status.control_lease.id, 900)
        : await api.lease("VelocityPnP vacuum characterization", 900);
      await waitForOperation(await api.characterizeVacuum(active.id, lease.id, nozzleId));
      const snapshot = await api.calibration(active.id);
      setActive(snapshot);
      setSnapshots((current) => current.map((value) => value.id === snapshot.id ? snapshot : value));
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setStarting(false); }
  };
  const calibrateXyDirectionResponse = async () => {
    if (!active) return;
    if (!window.confirm(
      "Measure local X/Y direction response on the homing fiducial? Confirm both nozzles are level, the datum board is fixed, the travel envelope is clear, and power isolation is reachable. The head will approach the datum from both sides at several bounded distances and speeds.",
    )) return;
    setStarting(true); setError(null);
    try {
      const lease = status.control_lease
        ? await api.renewLease(status.control_lease.id, 900)
        : await api.lease("VelocityPnP XY direction response", 900);
      await waitForOperation(await api.calibrateXyDirectionResponse(active.id, lease.id));
      const snapshot = await api.calibration(active.id);
      setActive(snapshot);
      setSnapshots((current) => current.map((value) => value.id === snapshot.id ? snapshot : value));
      setArtifactPlan(await api.calibrationPlan(active.id));
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setStarting(false); }
  };
  const calibrateInstalledGeometry = async () => {
    if (!active) return;
    if (!window.confirm(
      "Calibrate the cameras and installed plate geometry? Confirm both nozzles are level, all installed hardware is secure, the travel envelope is clear, and power isolation is reachable. The machine will scan widely. Components, feeders, and fixtures can obscure plate features; unavailable targets will be skipped.",
    )) return;
    setStarting(true); setError(null);
    try {
      const lease = status.control_lease
        ? await api.renewLease(status.control_lease.id, 3600)
        : await api.lease("VelocityPnP installed geometry", 3600);
      const started = await api.startInstalledGeometryCalibration(active.id, lease.id);
      setActiveRun(started.run);
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setStarting(false); }
  };
  const recomputeDerived = async () => {
    if (!active) return;
    setStarting(true); setError(null);
    try {
      const result = await api.recomputeDerivedCalibration(active.id);
      setActive(result.snapshot);
      setSnapshots((current) => current.map((value) => value.id === result.snapshot.id ? result.snapshot : value));
      setArtifactPlan(await api.calibrationPlan(active.id));
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setStarting(false); }
  };
  const capturePlannedGeometry = async () => {
    if (!active || !artifactPlan) return;
    const supported = new Set([
      "nozzle_axis_runout_n1", "nozzle_axis_runout_n2",
      "nozzle_height_n1", "nozzle_height_n2",
    ]);
    const targets = artifactPlan.items
      .filter((item) => item.action === "recapture" && supported.has(item.artifact))
      .map((item) => item.artifact);
    if (targets.length === 0) return;
    if (!window.confirm(
      `Capture only ${targets.map(pretty).join(", ")}? Confirm both nozzles are level, the envelope is clear, tube routing is normal, and power isolation is reachable.`,
    )) return;
    setStarting(true); setError(null);
    try {
      const lease = status.control_lease
        ? await api.renewLease(status.control_lease.id, 3600)
        : await api.lease("VelocityPnP targeted calibration", 3600);
      const started = await api.startArtifactCalibration(active.id, lease.id, targets);
      setActiveRun(started.run);
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setStarting(false); }
  };

  const runActive = Boolean(activeRun && ["pending", "running"].includes(activeRun.status));
  const machineReady = ["unhomed", "ready"].includes(status.state);
  const canRecalibrateHeadGeometry = Boolean(
    active?.parent_id && status.state === "ready" && status.homed,
  );
  const canVacuumProbe = Boolean(active && completed.has("nozzle_height_model") && status.state === "ready" && status.homed);
  const canCharacterizeVacuum = (nozzleId: "N1" | "N2") => Boolean(
    canVacuumProbe && active?.values.vacuum_probe?.[nozzleId]?.passed,
  );
  const canCalibrateXyDirectionResponse = Boolean(
    active?.artifacts.top_camera_working_plane_map
      && status.state === "ready" && status.homed,
  );
  const canCalibrateInstalledGeometry = Boolean(
    active?.artifacts.top_camera_working_plane_map
      && active?.artifacts.bottom_camera_local_map
      && active?.artifacts.tooling_n1
      && status.state === "ready" && status.homed,
  );

  return <div className="calibration-layout">
    <section className="card calibration-summary">
      <p className="eyebrow">Machine-measured revision</p><h2>{active ? `Snapshot ${active.id.slice(0, 8)}` : "No calibration snapshot"}</h2>
      <p>The machine executes supported steps, retains raw evidence and residuals, and invalidates downstream measurements when an upstream result changes.</p>
      {status.simulated && <div className="notice">Simulation can exercise the runner but can never mark a machine production-ready.</div>}
      {!active ? <button onClick={() => void createSnapshot()}>Create calibration</button> : <>
        <button disabled={starting || runActive || !machineReady} onClick={() => void start()}>{runActive ? "Calibration running" : activeRun ? "Run calibration again" : "Run initial calibration"}</button>
        <button className="secondary" disabled={starting || runActive || !canRecalibrateHeadGeometry} onClick={() => void recalibrateHeadGeometry()}>Recalibrate head geometry</button>
        <button className="secondary" disabled={starting || runActive} onClick={() => void createSnapshot()}>New revision</button>
        <button className="secondary" disabled={starting || runActive || !canVacuumProbe} onClick={() => void vacuumProbe("N1")}>Probe N1 vacuum</button>
        <button className="secondary" disabled={starting || runActive || !canVacuumProbe} onClick={() => void vacuumProbe("N2")}>Probe N2 vacuum</button>
        <button className="secondary" disabled={starting || runActive || !canCharacterizeVacuum("N1")} onClick={() => void fineHome()}>Fine-home XYZ</button>
        <button className="secondary" disabled={starting || runActive || !canCharacterizeVacuum("N1")} onClick={() => void characterizeVacuum("N1")}>Characterize N1 vacuum</button>
        <button className="secondary" disabled={starting || runActive || !canCharacterizeVacuum("N2")} onClick={() => void characterizeVacuum("N2")}>Characterize N2 vacuum</button>
        <button className="secondary" disabled={starting || runActive || !canCalibrateXyDirectionResponse} onClick={() => void calibrateXyDirectionResponse()}>Measure XY direction response</button>
        <button className="secondary" disabled={starting || runActive || !canCalibrateInstalledGeometry} onClick={() => void calibrateInstalledGeometry()}>Calibrate cameras and plates</button>
        <button className="secondary" disabled={starting || runActive} onClick={() => void recomputeDerived()}>Recompute derived transforms</button>
        <button className="secondary" disabled={starting || runActive || !artifactPlan?.items.some((item) => item.action === "recapture" && ["nozzle_axis_runout_n1", "nozzle_axis_runout_n2", "nozzle_height_n1", "nozzle_height_n2"].includes(item.artifact))} onClick={() => void capturePlannedGeometry()}>Capture planned nozzle geometry</button>
        {!machineReady && <small>Complete connection and preflight before starting calibration.</small>}
      </>}
      {active && status.calibration_revision === active.id && <div className="notice">Fine XYZ home is active for this calibration revision.</div>}
      {artifactPlan && <div className="run-result">
        <p>Selective plan: <strong>{artifactPlan.requires_motion ? `${artifactPlan.estimated_duration_seconds} s estimated motion` : "no motion"}</strong></p>
        {artifactPlan.items.filter((item) => item.action !== "reuse").map((item) =>
          <p key={item.artifact}>{pretty(item.artifact)}: <strong>{pretty(item.action)}</strong></p>)}
        {artifactPlan.items.every((item) => item.action === "reuse") && <p>All requested artifacts can be reused.</p>}
      </div>}
      {active?.values.vacuum_probe && <div className="run-result">
        {(["N1", "N2"] as const).map((nozzleId) => {
          const report = active.values.vacuum_probe?.[nozzleId];
          return report && <p key={nozzleId}>{nozzleId} {report.tip_id}: <strong>{report.passed ? `signal at Z${report.contact_nozzle_z_mm?.toFixed(3)}` : "no seal in safe interval"}</strong>{report.target_centering_residual_mm !== undefined && `; target residual ${report.target_centering_residual_mm.toFixed(3)} mm`}{report.height_model_applied && `; Z correction ${report.height_correction_x_mm?.toFixed(3)}, ${report.height_correction_y_mm?.toFixed(3)} mm`}</p>;
        })}
      </div>}
      {active?.values.vacuum_sensor_characterization && <div className="run-result">
        {(["N1", "N2"] as const).map((nozzleId) => {
          const report = active.values.vacuum_sensor_characterization?.[nozzleId];
          return report && <p key={nozzleId}>{nozzleId} {report.tip_id}: <strong>{report.passed ? "reference seal characterized" : "distributions overlap"}</strong>; signal {report.median_signal_delta_counts.toFixed(1)} counts; robust gap {report.robust_distribution_gap_counts.toFixed(1)} counts; acquire/release {report.acquire_threshold_raw_value.toFixed(1)} / {report.release_threshold_raw_value.toFixed(1)}</p>;
        })}
      </div>}
      {active?.values.local_xy_direction_response && <div className="run-result">
        {active.values.local_xy_direction_response.axes.map((axis) =>
          <p key={axis.estimate.axis}>{axis.estimate.axis.toUpperCase()}: <strong>{axis.compensation_enabled ? "holdout-validated compensation" : axis.estimate.effect_resolved ? "compensation rejected by holdout" : "no resolved direction effect"}</strong>; gap {axis.estimate.primary_axis_gap_mm.toFixed(4)} mm ± {axis.estimate.primary_axis_confidence_interval_95_mm.toFixed(4)} mm; holdout {axis.holdout.direction_gap_mm.toFixed(4)} mm; {axis.estimate.sample_pairs} pairs</p>)}
      </div>}
      {activeRun && <div className="calibration-run">
        <div className="run-heading"><strong>{pretty(activeRun.status)}</strong><span>{pretty(activeRun.phase)}</span></div>
        {activeRun.phase === "visual_homing_validation" && <p>{activeRun.visual_homing_samples.length} / {activeRun.config.visual_homing_sample_count} homing cycles measured</p>}
        {activeRun.phase === "head_geometry_calibration" && <p>Mapping cameras and measuring nozzle runout and Z-dependent axis shift</p>}
        {activeRun.phase === "top_camera_planar_calibration" && <p>Measuring the top-camera image plane with the installed datum fiducial</p>}
        {activeRun.phase === "bottom_camera_planar_calibration" && <p>Measuring the bottom-camera image plane with the installed N1 tip</p>}
        {activeRun.phase === "installed_plate_geometry" && <p>Registering installed plates, then scanning candidate holes; occupied and ambiguous targets are skipped</p>}
        <div className="calibration-checks">{activeRun.checks.map((check) => <div key={check.id} className={`check-${check.status}`}><span>{check.status === "passed" ? "✓" : check.status === "failed" ? "!" : "·"}</span><div><strong>{check.title}</strong><small>{check.message}</small></div></div>)}</div>
        {activeRun.visual_homing_report && <p className="run-result">Independent return p95 <strong>{activeRun.visual_homing_report.independent_return.p95_mm.toFixed(4)} mm</strong></p>}
        {activeRun.head_geometry_report && <div className="run-result">
          <p>Bottom camera <strong>{activeRun.head_geometry_report.top_camera_center.controller_x_mm.toFixed(3)}, {activeRun.head_geometry_report.top_camera_center.controller_y_mm.toFixed(3)} mm</strong></p>
          {activeRun.head_geometry_report.camera_ray_transfer && <p>Camera-ray transfer <strong>{activeRun.head_geometry_report.camera_ray_transfer.transfer_correction_x_mm.toFixed(3)}, {activeRun.head_geometry_report.camera_ray_transfer.transfer_correction_y_mm.toFixed(3)} mm</strong></p>}
          <p>N1 to N2 <strong>{activeRun.head_geometry_report.n1_to_n2_x_mm.toFixed(3)}, {activeRun.head_geometry_report.n1_to_n2_y_mm.toFixed(3)} mm</strong></p>
          {activeRun.head_geometry_report.nozzles.map((nozzle) => <p key={nozzle.nozzle_id}>{nozzle.nozzle_id} {nozzle.tip_id} runout <strong>{nozzle.runout_radius_mm.toFixed(4)} mm</strong>; fit p95 <strong>{nozzle.fit_residuals.p95_mm.toFixed(4)} mm</strong></p>)}
          {(activeRun.head_geometry_report.nozzle_height_models ?? []).map((model) => <p key={model.nozzle_id}>{model.nozzle_id} axis slope <strong>{model.axis_slope_x_mm_per_mm.toFixed(5)}, {model.axis_slope_y_mm_per_mm.toFixed(5)} mm/mm</strong>; fit p95 <strong>{model.fit_residuals.p95_mm.toFixed(4)} mm</strong></p>)}
        </div>}
        {activeRun.installed_geometry_report && <div className="run-result">
          <p>Top camera <strong>{pretty(activeRun.installed_geometry_report.top_camera.model.kind)}</strong>; holdout RMS <strong>{activeRun.installed_geometry_report.top_camera.holdout_residuals.rms_mm.toFixed(4)} mm</strong>; {activeRun.installed_geometry_report.top_camera.detected_positions} / {activeRun.installed_geometry_report.top_camera.attempted_positions} positions visible</p>
          <p>Bottom camera <strong>{pretty(activeRun.installed_geometry_report.bottom_camera.model.kind)}</strong>; holdout RMS <strong>{activeRun.installed_geometry_report.bottom_camera.holdout_residuals.rms_mm.toFixed(4)} mm</strong>; {activeRun.installed_geometry_report.bottom_camera.detected_positions} / {activeRun.installed_geometry_report.bottom_camera.attempted_positions} positions visible</p>
          <p>Installed plates <strong>{activeRun.installed_geometry_report.plate_geometry.qualified_plate_count} qualified</strong>; {activeRun.installed_geometry_report.plate_geometry.detected_targets} / {activeRun.installed_geometry_report.plate_geometry.attempted_targets} candidate holes visible</p>
          {activeRun.installed_geometry_report.plate_geometry.registrations.map((registration) => <p key={registration.plate_id}>{pretty(registration.workspace_position)} {pretty(registration.role)} registration <strong>{registration.passed ? "passed" : "not found"}</strong>{registration.offset_x_mm !== null && registration.offset_y_mm !== null ? ` at ${registration.offset_x_mm.toFixed(3)} mm X, ${registration.offset_y_mm.toFixed(3)} mm Y` : ""}{registration.seeded_from_plate_id ? `; seeded from ${pretty(registration.seeded_from_plate_id)}` : ""}</p>)}
          {activeRun.installed_geometry_report.plate_geometry.limitations.map((limitation) => <p key={limitation}>{limitation}</p>)}
        </div>}
        {activeRun.failure && <div className="notice">{activeRun.failure}</div>}
      </div>}
      <small>Machine measurements complete each automated step. Reciprocal camera geometry supplies the working-plane transfer. Manual target observations are diagnostic overrides only.</small>
    </section>
    <section className="workflow">{steps.map((step, index) => {
      const done = completed.has(step.id); const automated = step.automation === "automated";
      return <article className={`workflow-step ${done ? "workflow-done" : automated ? "workflow-ready" : "workflow-locked"}`} key={step.id}>
        <div className="workflow-index">{done ? "✓" : String(index + 1).padStart(2, "0")}</div>
        <div><div className="step-labels">{automated && <span>Automated</span>}{step.requires_motion && <span>Motion</span>}{step.requires_fixture && <span>Fixture</span>}</div><h3>{step.title}</h3><p>{step.description}</p>
          {step.dependencies.length > 0 && <small>Requires {step.dependencies.map(pretty).join(", ")}</small>}</div>
        <button disabled>{done ? "Measured" : automated ? "Use controls above" : "Not implemented"}</button>
      </article>;
    })}</section>
    {snapshots.length > 1 && <p className="snapshot-count">{snapshots.length} calibration snapshots retained</p>}
  </div>;
}

function BoardsPanel({ setError }: { setError: (value: string | null) => void }) {
  const [boards, setBoards] = useState<BoardDesign[]>([]);
  const [scans, setScans] = useState<WorkspaceScanReport[]>([]);
  const [selectedScan, setSelectedScan] = useState("");
  const [discovery, setDiscovery] = useState<WorkspaceBoardDiscoveryReport | null>(null);
  const [discovering, setDiscovering] = useState(false);
  const [inventoryBusy, setInventoryBusy] = useState(false);
  const [baseline, setBaseline] = useState<WorkspaceInventoryBaseline | null>(null);
  const [changes, setChanges] = useState<WorkspaceChangeSetReport | null>(null);
  const [registration, setRegistration] = useState<BoardRegistrationRun | null>(null);
  const [registrationBusy, setRegistrationBusy] = useState(false);
  const [pcb, setPcb] = useState<File | null>(null);
  const [bom, setBom] = useState<File | null>(null);
  const load = useCallback(async () => {
    try {
      const [boardValues, scanValues] = await Promise.all([
        api.boards(), api.workspaceScans(),
      ]);
      const eligible = scanValues
        .filter((scan) => scan.status === "succeeded"
          && (scan.height_map !== null || scan.refinement_composite?.accepted === true))
        .sort((first, second) => second.updated_at.localeCompare(first.updated_at));
      setBoards(boardValues);
      setScans(eligible);
      setSelectedScan((current) =>
        eligible.some((scan) => scan.id === current) ? current : eligible[0]?.id ?? ""
      );
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    }
  }, [setError]);
  useEffect(() => { void load(); }, [load]);
  const upload = async () => {
    if (!pcb) return;
    try { await api.importBoard(pcb, bom); setPcb(null); setBom(null); load(); }
    catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
  };
  const discover = async () => {
    if (!selectedScan) return;
    setDiscovering(true);
    setDiscovery(null);
    try {
      setDiscovery(await api.discoverWorkspaceObjects(selectedScan));
      await load();
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setDiscovering(false);
    }
  };
  const setInventoryBaseline = async () => {
    if (!selectedScan) return;
    setInventoryBusy(true);
    try {
      setBaseline(await api.setWorkspaceInventoryBaseline(selectedScan));
      setChanges(null);
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setInventoryBusy(false);
    }
  };
  const compareInventory = async () => {
    if (!selectedScan) return;
    setInventoryBusy(true);
    try {
      setChanges(await api.compareWorkspaceObservation(selectedScan));
      await load();
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setInventoryBusy(false);
    }
  };
  const registerBoard = async (candidate: WorkspaceBoardCandidate, boardId: string) => {
    if (!selectedScan) return;
    if (!window.confirm(
      "Register this PCB from its KiCad fiducials? Confirm the PCB is fixed, both nozzles are level, the travel envelope is clear, and power isolation is reachable. The machine will visual-home, then move the top camera over each predicted board fiducial.",
    )) return;
    setRegistrationBusy(true); setError(null); setRegistration(null);
    try {
      const machine = await api.status();
      const lease = machine.control_lease
        ? await api.renewLease(machine.control_lease.id, 900)
        : await api.lease("VelocityPnP board registration", 900);
      const started = await api.startBoardRegistration(
        selectedScan, candidate.candidate_index, boardId, lease.id,
      );
      await waitForOperation(started.operation);
      setRegistration(await api.boardRegistration(started.registration.id));
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setRegistrationBusy(false);
    }
  };
  const selectedReport = scans.find((scan) => scan.id === selectedScan);
  const hasProposals = discovery !== null || selectedReport?.board_discovery != null;
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
    <section className="card board-discovery-card">
      <div className="card-heading"><div><p className="eyebrow">Workspace inventory</p><h2>Analyze workspace objects</h2></div>{scans.length > 0 && <span className="count">{scans.length}</span>}</div>
      <p>Select a completed height-aware scan. Plate-aligned proposals use known fixtures and retain uncertain objects for later classification. Analysis causes no machine motion.</p>
      <div className="board-discovery-controls">
        <select value={selectedScan} onChange={(event) => { setSelectedScan(event.target.value); setDiscovery(null); setChanges(null); }}>
          {scans.length === 0 && <option value="">No eligible scans</option>}
          {scans.map((scan) => <option key={scan.id} value={scan.id}>{new Date(scan.updated_at).toLocaleString()} · {pretty(scan.mode)}{scan.board_discovery ? ` · ${scan.board_discovery.candidate_count} prior candidates` : ""}</option>)}
        </select>
        <button disabled={!selectedScan || discovering} onClick={() => void discover()}>{discovering ? "Analyzing…" : "Analyze objects"}</button>
        <button disabled={!selectedScan || !hasProposals || inventoryBusy} onClick={() => void setInventoryBaseline()}>Use as baseline</button>
        <button disabled={!selectedScan || !hasProposals || inventoryBusy} onClick={() => void compareInventory()}>Compare to baseline</button>
      </div>
      {baseline && <p>Baseline saved from scan <strong>{baseline.scan_id.slice(0, 8)}</strong>. Future compatible scans can reuse unchanged inventory.</p>}
      {changes && <p>{pretty(changes.detection_mode)} found <strong>{changes.added_count}</strong> added, <strong>{changes.removed_count}</strong> removed, <strong>{changes.modified_count}</strong> modified, and <strong>{changes.unchanged_count}</strong> unchanged. <strong>{changes.target_regions.length}</strong> regions need refinement.</p>}
      {registration && <div className="run-result">
        <p><strong>{registration.passed ? "Board registered" : "Registration failed"}</strong>{registration.selected_rotation_deg != null ? ` at ${registration.selected_rotation_deg.toFixed(3)}°` : ""}{registration.fit_residuals ? ` with ${registration.fit_residuals.max_mm.toFixed(3)} mm maximum residual` : ""}.</p>
        <p>{registration.message}</p>
        {registration.hypotheses.flatMap((hypothesis) => hypothesis.observations).map((observation) => <a key={`${registration.id}-${observation.artifact_name}`} href={`/api/v1/board-registrations/${registration.id}/artifacts/${observation.artifact_name}`} target="_blank" rel="noreferrer">{observation.reference} evidence</a>)}
      </div>}
      {discovery && <div className="board-discovery-result">
        <img src={`/api/v1/workspace-scans/${selectedScan}/artifacts/mosaic-board-candidates.png`} alt="Workspace mosaic with semantic object proposals" />
        <div><p><strong>{discovery.candidates.length}</strong> raised-surface proposals. Known fixtures are purple, boards are green, feeders are blue, and unresolved objects are amber. Matched boards show enabled top-side placements in cyan and KiCad fiducials in yellow.</p>
          {discovery.warnings.map((warning) => <p className="warning" key={warning}>⚠ {warning}</p>)}
          <div className="candidate-list">{discovery.candidates.map((candidate) => <WorkspaceCandidateCard key={candidate.candidate_index} candidate={candidate} scanId={selectedScan} registering={registrationBusy} onRegister={registerBoard} />)}</div>
        </div>
      </div>}
    </section>
  </div>;
}

function WorkspaceCandidateCard({ candidate, scanId, registering, onRegister }: {
  candidate: WorkspaceBoardCandidate;
  scanId: string;
  registering: boolean;
  onRegister: (candidate: WorkspaceBoardCandidate, boardId: string) => void;
}) {
  const design = candidate.design_matches[0];
  const visualDesign = design?.registration_method === "visual_outline"
    && design.center_x_mm != null
    && design.center_y_mm != null ? design : null;
  return <div>
    <strong>{pretty(candidate.classification?.class ?? "unknown")} {candidate.candidate_index + 1}</strong>
    <span>Proposal {candidate.width_mm.toFixed(1)} × {candidate.height_mm.toFixed(1)} mm at {candidate.center_x_mm.toFixed(1)}, {candidate.center_y_mm.toFixed(1)}{candidate.plate_role ? ` · ${pretty(candidate.plate_role)} plate` : ""}</span>
    {visualDesign && <span>Visual PCB at {visualDesign.center_x_mm!.toFixed(3)}, {visualDesign.center_y_mm!.toFixed(3)} mm · {visualDesign.rotation_deg.toFixed(3)}° · {visualDesign.dimension_error_mm.toFixed(3)} mm error · {((visualDesign.edge_support_fraction ?? 0) * 100).toFixed(0)}% edge support</span>}
    <span>{((candidate.classification?.confidence ?? 0) * 100).toFixed(0)}% class confidence{candidate.known_fixture_id ? ` · ${pretty(candidate.known_fixture_id)}` : design ? ` · ${design.board_name}` : " · review required"}{design?.orientation_ambiguous ? " · orientation needs fiducials" : ""}</span>
    {candidate.evidence_artifact_name && <a href={`/api/v1/workspace-scans/${scanId}/artifacts/${candidate.evidence_artifact_name}`} target="_blank" rel="noreferrer">View normalized evidence</a>}
    {visualDesign && <button disabled={registering} onClick={() => onRegister(candidate, visualDesign.board_id)}>{registering ? "Registering…" : "Register from fiducials"}</button>}
  </div>;
}

export function pretty(value: string): string {
  return value.replaceAll(".", " ").replaceAll("_", " ").replace(/\b\w/g, (char) => char.toUpperCase());
}

function formatSize(board: BoardDesign): string {
  return board.width_mm != null && board.height_mm != null ? `${board.width_mm.toFixed(1)} × ${board.height_mm.toFixed(1)} mm` : "Unknown";
}
