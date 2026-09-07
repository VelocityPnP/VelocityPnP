import { useCallback, useEffect, useMemo, useState } from "react";
import { api, waitForOperation } from "./api";
import { DialogProvider, useDialog } from "./Dialog";
import type {
  AutomatedCalibrationRun, BoardDesign, BoardRegistrationRun, CalibrationArtifactPlan,
  CalibrationSnapshot, CalibrationStep, Device, Job, JobExecutionRun, MachineEvent,
  MachineStatus, Pose,
  SensorReading, StripFeederLiveHoldoutRun, StripFeederPickupQualificationRun,
  StripFeederRegistration,
  StripFeederVisionReport,
  WorkspaceBoardCandidate, WorkspaceBoardDiscoveryReport, WorkspaceChangeSetReport,
  WorkspaceInventoryBaseline, WorkspaceScanReport,
} from "./types";
import "./styles.css";

type Tab = "machine" | "calibration" | "boards" | "jobs";

const EMPTY_POSE: Pose = { x_mm: 0, y_mm: 0, z_mm: 31.5, rotation_deg: 0 };

export function canStartJob(
  status: Pick<MachineStatus, "state" | "homed" | "calibration_revision">,
  busy: boolean,
  unfinishedExecution: boolean,
): boolean {
  return !busy
    && !unfinishedExecution
    && status.state === "ready"
    && status.homed
    && Boolean(status.calibration_revision);
}

export function completedJobStepCount(
  steps: Array<Pick<JobExecutionRun["steps"][number], "status">>,
): number {
  return steps.filter((step) => ["succeeded", "skipped"].includes(step.status)).length;
}

export function canResolveJobStep(
  runStatus: JobExecutionRun["status"],
  stepStatus: JobExecutionRun["steps"][number]["status"],
): boolean {
  return runStatus === "needs_attention" && stepStatus === "needs_attention";
}

export default function App() {
  return <DialogProvider><AppContent /></DialogProvider>;
}

function AppContent() {
  const [tab, setTab] = useState<Tab>("machine");
  const [status, setStatus] = useState<MachineStatus | null>(null);
  const [devices, setDevices] = useState<Device[]>([]);
  const [events, setEvents] = useState<MachineEvent[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  const refresh = useCallback(async () => {
    try {
      setStatus(await api.status());
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    }
  }, []);

  useEffect(() => {
    void refresh();
    void api.devices().then(setDevices).catch((reason) =>
      setError(reason instanceof Error ? reason.message : String(reason)));
    const timer = window.setInterval(() => void refresh(), 1000);
    const source = new EventSource("/api/v1/events");
    const kinds = ["machine.connected", "machine.disconnected", "machine.preflight_completed", "machine.homing_started", "machine.homing_completed", "machine.motion_started", "machine.motion_completed", "machine.motion_failed", "machine.position", "machine.actuator", "machine.sensor", "machine.visual_home_applied", "machine.fine_z_home_applied", "machine.fault", "machine.emergency_stopped", "control.lease_acquired", "control.lease_renewed"];
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
          <NavButton active={tab === "jobs"} onClick={() => setTab("jobs")} icon="▶">Jobs</NavButton>
        </nav>
        <div className="sidebar-foot">
          <div className={`state-dot state-${status?.state ?? "disconnected"}`} />
          <div><strong>{pretty(status?.state ?? "loading")}</strong><small>{status == null ? "Awaiting status" : status.simulated ? "Simulation" : "Physical machine"}</small></div>
        </div>
      </aside>
      <main>
        <header className="topbar">
          <div><p className="eyebrow">LumenPnP V3.0.2</p><h1>{tab === "machine" ? "Machine control" : tab === "calibration" ? "Machine calibration" : tab === "boards" ? "Board library" : "Job execution"}</h1></div>
          <div className="topbar-actions">
            <div className="machine-summary"><span className={`state-dot state-${status?.state ?? "disconnected"}`} /><span>{pretty(status?.state ?? "loading")}</span></div>
            <button className="estop" onClick={() => void run(async () => waitForOperation(await api.emergencyStop()))}>EMERGENCY STOP</button>
          </div>
        </header>
        {error && <div className="alert" role="alert"><span>!</span>{error}<button onClick={() => setError(null)}>Dismiss</button></div>}
        {!status && (tab === "machine" || tab === "calibration" || tab === "jobs") && <section className="card loading-card"><p className="eyebrow">Machine service</p><h2>Loading status…</h2><p>Waiting for the VelocityPnP daemon.</p></section>}
        {tab === "machine" && status && <MachinePanel status={status} devices={devices} events={events} busy={busy} run={run} openCalibration={() => setTab("calibration")} />}
        {tab === "calibration" && status && <CalibrationPanel status={status} devices={devices} setError={setError} />}
        {tab === "boards" && <BoardsPanel status={status} setError={setError} />}
        {tab === "jobs" && status && <JobsPanel status={status} setError={setError} />}
      </main>
    </div>
  );
}

function NavButton({ active, icon, children, onClick }: { active: boolean; icon: string; children: React.ReactNode; onClick: () => void }) {
  return <button aria-current={active ? "page" : undefined} className={active ? "nav-active" : ""} onClick={onClick}><span>{icon}</span>{children}</button>;
}

function JobsPanel({ status, setError }: {
  status: MachineStatus;
  setError: (value: string | null) => void;
}) {
  const requestDialog = useDialog();
  const [jobs, setJobs] = useState<Job[]>([]);
  const [executions, setExecutions] = useState<JobExecutionRun[]>([]);
  const [busy, setBusy] = useState(false);
  const load = useCallback(async () => {
    try {
      const [jobValues, executionValues] = await Promise.all([
        api.jobs(), api.jobExecutions(),
      ]);
      setJobs(jobValues);
      setExecutions(executionValues.sort((left, right) =>
        right.updated_at.localeCompare(left.updated_at)));
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    }
  }, [setError]);
  useEffect(() => {
    void load();
    const timer = window.setInterval(() => void load(), 1000);
    return () => window.clearInterval(timer);
  }, [load]);
  const lease = async (clientName: string) => {
    const machine = await api.status();
    return machine.control_lease
      ? api.renewLease(machine.control_lease.id, 900)
      : api.lease(clientName, 900);
  };
  const act = async (action: () => Promise<unknown>) => {
    setBusy(true); setError(null);
    try { await action(); await load(); }
    catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setBusy(false); }
  };
  const start = (job: Job) => void act(async () => {
    if (status.state !== "ready" || !status.homed) {
      throw new Error("Connect, complete preflight, and home the machine first.");
    }
    if (!status.calibration_revision) throw new Error("Fine XYZ home is required.");
    if (await requestDialog({
      title: `Start ${job.name}?`,
      message: "Confirm the PCB and feeders are fixed, both nozzles are level, the travel envelope is clear, and power isolation is reachable.",
      confirmLabel: "Start job",
    }) === null) return;
    const control = await lease("VelocityPnP job execution");
    await api.startJobExecution(job.id, control.id, status.calibration_revision);
  });
  const resume = (run: JobExecutionRun) => void act(async () => {
    if (await requestDialog({
      title: "Resume job?",
      message: "Confirm fine XYZ home is current, the PCB and feeders are fixed, both nozzles are level, the travel envelope is clear, and power isolation is reachable.",
      confirmLabel: "Resume job",
    }) === null) return;
    const control = await lease("VelocityPnP job resume");
    await api.resumeJobExecution(run.id, control.id);
  });
  const resolve = (
    run: JobExecutionRun, stepIndex: number, action: "retry" | "skip",
  ) => void act(async () => {
    const note = await requestDialog({
      title: action === "skip" ? "Skip this placement?" : "Retry this placement?",
      message: action === "skip"
        ? "Record why it is safe to continue without this placement."
        : "You can add a note for the execution record before the retry.",
      confirmLabel: action === "skip" ? "Skip placement" : "Retry placement",
      tone: action === "skip" ? "danger" : "default",
      input: {
        label: action === "skip" ? "Reason" : "Execution note",
        placeholder: action === "skip" ? "Reason for the skip" : "Optional note",
        required: action === "skip",
      },
    });
    if (note === null) return;
    await api.resolveJobExecutionStep(run.id, stepIndex, action, note);
  });
  const cancel = (run: JobExecutionRun) => void act(async () => {
    if (await requestDialog({
      title: "Cancel this job?",
      message: "The execution cannot be resumed after it is cancelled.",
      confirmLabel: "Cancel job",
      tone: "danger",
    }) === null) return;
    await api.cancelJobExecution(run.id);
  });
  const unfinished = executions.some((run) =>
    !["succeeded", "completed_with_skips", "cancelled"].includes(run.status));
  const startAllowed = canStartJob(status, busy, unfinished);
  const startBlockedMessage = unfinished
    ? "Finish or cancel the current execution before another job starts."
    : status.state !== "ready" || !status.homed
      ? "Connect, complete preflight, and home the machine before a job starts."
      : !status.calibration_revision
        ? "Fine-home XYZ before a job starts."
        : null;

  return <div className="page-stack">
    <section className="panel">
      <div className="section-heading"><div><p className="eyebrow">Queue</p><h2>Jobs</h2></div></div>
      {startBlockedMessage && <div className="notice compact-notice">{startBlockedMessage}</div>}
      {jobs.length === 0 && <p>No jobs are stored.</p>}
      <div className="card-grid">{jobs.map((job) => <article className="card" key={job.id}>
        <h3>{job.name}</h3>
        <p>{job.placement_references.length || "All"} selected placements</p>
        <button disabled={!startAllowed} onClick={() => start(job)}>Start job</button>
      </article>)}</div>
    </section>
    <section className="panel">
      <div className="section-heading"><div><p className="eyebrow">Persistent state</p><h2>Executions</h2></div></div>
      {executions.length === 0 && <p>No job executions are stored.</p>}
      {executions.map((run) => <article className="workflow-step" key={run.id}>
        <div className="workflow-index">{completedJobStepCount(run.steps)}/{run.steps.length}</div>
        <div>
          <h3>{jobs.find((job) => job.id === run.job_id)?.name ?? run.job_id}</h3>
          <p><strong>{pretty(run.status)}</strong>{run.message ? ` — ${run.message}` : ""}</p>
          {run.steps.map((step) => <small key={step.step_index} className="job-step">
            {step.placement_reference}: {pretty(step.status)}
            {step.outcome_disposition ? ` (${pretty(step.outcome_disposition)})` : ""}
            {step.board_contacted ? "; board contacted" : ""}
            {step.message ? ` — ${step.message}` : ""}
            {canResolveJobStep(run.status, step.status) && !step.board_contacted && <button disabled={busy} onClick={() => resolve(run, step.step_index, "retry")}>Retry</button>}
            {canResolveJobStep(run.status, step.status) && <button disabled={busy} onClick={() => resolve(run, step.step_index, "skip")}>Skip</button>}
          </small>)}
        </div>
        <div>
          {(run.status === "pending" || run.status === "running") && <button disabled={busy || run.pause_requested} onClick={() => void act(() => api.pauseJobExecution(run.id))}>Pause</button>}
          {run.status === "paused" && <button disabled={busy} onClick={() => resume(run)}>Resume</button>}
          {!["succeeded", "completed_with_skips", "cancelled"].includes(run.status) && <button disabled={busy} onClick={() => cancel(run)}>Cancel</button>}
        </div>
      </article>)}
    </section>
  </div>;
}

function MachinePanel({ status, devices, events, busy, run, openCalibration }: {
  status: MachineStatus; devices: Device[]; events: MachineEvent[]; busy: boolean;
  run: (task: () => Promise<unknown>) => Promise<void>;
  openCalibration: () => void;
}) {
  const requestDialog = useDialog();
  const [readings, setReadings] = useState<Record<string, SensorReading>>({});
  const lease = status.control_lease;
  const pose = status.pose ?? EMPTY_POSE;
  const controllerSettings = status.controller?.settings;
  const xyFeedLimit = controllerSettings
    ? Math.min(
      controllerSettings.max_feed_units_per_second.X ?? Number.POSITIVE_INFINITY,
      controllerSettings.max_feed_units_per_second.Y ?? Number.POSITIVE_INFINITY,
    ) * 60
    : null;
  const xyMicrosteps = controllerSettings
    ? `${controllerSettings.microsteps.X ?? "—"}/${controllerSettings.microsteps.Y ?? "—"}`
    : null;
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
    void machineAction(() => api.actuate(lease.id, actuatorId, enabled));
  };
  const home = async () => {
    if (!lease) return;
    if (await requestDialog({
      title: status.homed ? "Home the machine again?" : "Home the machine?",
      message: "Confirm both nozzles are level, the travel envelope is clear, and power isolation is within reach.",
      confirmLabel: status.homed ? "Home again" : "Start homing",
    }) === null) return;
    void machineAction(() => api.home(lease.id));
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
          <button disabled={busy || !["unhomed", "ready"].includes(status.state) || Boolean(lease)} onClick={() => void run(api.lease)}>Acquire 15 min lease</button>
        </SequenceStep>
        <SequenceStep number="4" title="Home machine" done={status.homed && Boolean(status.calibration_revision)}>
          <div className="sequence-action">
            {!status.homed || status.calibration_revision
              ? <button disabled={busy || !lease || !["unhomed", "ready"].includes(status.state)} onClick={() => void home()}>{status.homed ? "Home again" : "Home machine"}</button>
              : <button disabled={busy} onClick={openCalibration}>Continue to fine-home XYZ</button>}
            <small>{!status.homed
              ? "First establish the controller axis references."
              : status.calibration_revision
                ? "Coarse and fine XYZ homing are complete."
                : "Coarse home is complete. Fine-home XYZ against the datum is still required."}</small>
          </div>
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
      {status.controller && <p className="hint">
        Active XY limit {xyFeedLimit && Number.isFinite(xyFeedLimit) ? `${xyFeedLimit.toFixed(0)} mm/min` : "not reported"};
        microsteps X/Y {xyMicrosteps}; interpolation {controllerSettings?.interpolation.X && controllerSettings?.interpolation.Y ? "on" : "not reported"}.
      </p>}
    </section>

    <section className="card hardware-check-card">
      <div className="card-heading"><div><p className="eyebrow">Initial validation</p><h2>Manual actuator checks</h2></div></div>
      <div className="hardware-checks">
        <div><strong>Ring lights</strong><span><button disabled={busy || !actuatorAllowed} onClick={() => setActuator("lights", true)}>On</button><button disabled={busy || !actuatorAllowed} onClick={() => setActuator("lights", false)}>Off</button></span></div>
        {(["vacuum_1", "vacuum_2"] as const).map((sensorId) => <div key={sensorId}><strong>{pretty(sensorId)}</strong><code>{readings[sensorId]?.raw_value ?? "—"} raw</code><span><button disabled={busy || !actuatorAllowed} onClick={() => setActuator(sensorId, true)}>On</button><button disabled={busy || !actuatorAllowed} onClick={() => setActuator(sensorId, false)}>Off</button><button disabled={busy || !actuatorAllowed} onClick={() => readSensor(sensorId)}>Read</button></span></div>)}
      </div>
      <p className="hint">Vacuum outputs stay on until you select Off. Use these controls only for short checks.</p>
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
  const requestDialog = useDialog();
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
    if (await requestDialog({
      title: "Start automated calibration?",
      message: "Confirm both nozzles are level, the travel envelope is clear, power isolation is within reach, and both nozzle tubes are seated, routed normally, and free of snags. The bottom camera will identify each tip before any rotation.",
      confirmLabel: "Start calibration",
    }) === null) return;
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
    if (await requestDialog({
      title: "Recalibrate head geometry?",
      message: "Confirm both nozzles are level, the travel envelope is clear, power isolation is within reach, and both nozzle tubes are seated, routed normally, and free of snags. The machine will use the validated parent snapshot and then measure camera transfer, nozzle offsets, runout, and height models.",
      confirmLabel: "Start recalibration",
    }) === null) return;
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
    if (await requestDialog({
      title: `Probe ${nozzleId} vacuum?`,
      message: "Confirm the datum board is fixed, the sealing target and nozzle tips are clean, both nozzles are level, the travel envelope is clear, and power isolation is within reach. X/Y motion occurs only while both nozzles are level. The selected nozzle then lowers at fixed X/Y and does not move below working Z0.",
      confirmLabel: "Start probe",
    }) === null) return;
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
    if (await requestDialog({
      title: "Fine-home XYZ?",
      message: "Confirm the datum PCB is fixed, the sealing target and nozzle tips are clean, both nozzles are level, the travel envelope is clear, and power isolation is within reach. The machine will visually refine X/Y, identify and center N2, reproduce the validated vacuum contact, and apply a bounded Z coordinate reset.",
      confirmLabel: "Start fine homing",
    }) === null) return;
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
    if (await requestDialog({
      title: `Characterize ${nozzleId} vacuum?`,
      message: "Confirm the datum board is fixed, the sealing target and nozzle tips are clean, both nozzles are level, the travel envelope is clear, and power isolation is within reach. The pump will be on only during the narrowed contact search and sealed sampling.",
      confirmLabel: "Start characterization",
    }) === null) return;
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
    if (await requestDialog({
      title: "Measure X/Y direction response?",
      message: "Confirm both nozzles are level, the datum board is fixed, the travel envelope is clear, and power isolation is reachable. The head will approach the datum from both sides at several bounded distances and speeds.",
      confirmLabel: "Start measurement",
    }) === null) return;
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
    if (await requestDialog({
      title: "Calibrate cameras and plates?",
      message: "Confirm both nozzles are level, all installed hardware is secure, the travel envelope is clear, and power isolation is reachable. The machine will scan widely. Components, feeders, and fixtures can obscure plate features; unavailable targets will be skipped.",
      confirmLabel: "Start calibration",
    }) === null) return;
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
    if (await requestDialog({
      title: "Capture planned geometry?",
      message: `Capture only ${targets.map(pretty).join(", ")}. Confirm both nozzles are level, the envelope is clear, tube routing is normal, and power isolation is reachable.`,
      confirmLabel: "Start capture",
    }) === null) return;
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
        <div className="button-cluster">
          <button disabled={starting || runActive || !machineReady} onClick={() => void start()}>{runActive ? "Calibration running" : activeRun ? "Run calibration again" : "Run initial calibration"}</button>
          <button className="secondary" disabled={starting || runActive || !canRecalibrateHeadGeometry} onClick={() => void recalibrateHeadGeometry()}>Recalibrate head geometry</button>
          <button className="secondary" disabled={starting || runActive} onClick={() => void createSnapshot()}>New revision</button>
          <button className="secondary" disabled={starting || runActive || !canVacuumProbe} onClick={() => void vacuumProbe("N1")}>Probe N1 vacuum</button>
          <button className="secondary" disabled={starting || runActive || !canVacuumProbe} onClick={() => void vacuumProbe("N2")}>Probe N2 vacuum</button>
          <button className="secondary" disabled={starting || runActive || !canCharacterizeVacuum("N2")} onClick={() => void fineHome()}>Fine-home XYZ</button>
          <button className="secondary" disabled={starting || runActive || !canCharacterizeVacuum("N1")} onClick={() => void characterizeVacuum("N1")}>Characterize N1 vacuum</button>
          <button className="secondary" disabled={starting || runActive || !canCharacterizeVacuum("N2")} onClick={() => void characterizeVacuum("N2")}>Characterize N2 vacuum</button>
          <button className="secondary" disabled={starting || runActive || !canCalibrateXyDirectionResponse} onClick={() => void calibrateXyDirectionResponse()}>Measure XY direction response</button>
          <button className="secondary" disabled={starting || runActive || !canCalibrateInstalledGeometry} onClick={() => void calibrateInstalledGeometry()}>Calibrate cameras and plates</button>
          <button className="secondary" disabled={starting || runActive} onClick={() => void recomputeDerived()}>Recompute derived transforms</button>
          <button className="secondary" disabled={starting || runActive || !artifactPlan?.items.some((item) => item.action === "recapture" && ["nozzle_axis_runout_n1", "nozzle_axis_runout_n2", "nozzle_height_n1", "nozzle_height_n2"].includes(item.artifact))} onClick={() => void capturePlannedGeometry()}>Capture planned nozzle geometry</button>
        </div>
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
        {active.values.local_xy_direction_response.axes.flatMap((axis) => axis.feeds?.length
          ? axis.feeds.map((feed) =>
            <p key={`${axis.axis}-${feed.feed_mm_per_min}`}>{axis.axis?.toUpperCase()} at {feed.feed_mm_per_min.toFixed(0)} mm/min: <strong>{feed.qualified ? feed.compensation_enabled ? "qualified compensation" : "qualified without compensation" : "not qualified"}</strong>; gap {feed.estimate.primary_axis_gap_mm.toFixed(4)} mm ± {feed.estimate.primary_axis_confidence_interval_95_mm.toFixed(4)} mm; holdout {feed.holdout.direction_gap_mm.toFixed(4)} mm; {feed.estimate.sample_pairs} pairs</p>)
          : axis.estimate && axis.holdout
            ? [<p key={axis.estimate.axis}>{axis.estimate.axis.toUpperCase()}: legacy direction response; recalibration required</p>]
            : [])}
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
      const ready = automated && step.dependencies.every((dependency) => completed.has(dependency));
      return <article className={`workflow-step ${done ? "workflow-done" : ready ? "workflow-ready" : automated ? "workflow-waiting" : "workflow-locked"}`} key={step.id}>
        <div className="workflow-index">{done ? "✓" : String(index + 1).padStart(2, "0")}</div>
        <div><div className="step-labels">{automated && <span>Automated</span>}{step.requires_motion && <span>Motion</span>}{step.requires_fixture && <span>Fixture</span>}</div><h3>{step.title}</h3><p>{step.description}</p>
          {step.dependencies.length > 0 && <small>Requires {step.dependencies.map(pretty).join(", ")}</small>}</div>
        <button disabled>{done ? "Measured" : ready ? "Use controls above" : automated ? "Waiting for prerequisites" : "Not implemented"}</button>
      </article>;
    })}</section>
    {snapshots.length > 1 && <p className="snapshot-count">{snapshots.length} calibration snapshots retained</p>}
  </div>;
}

function BoardsPanel({ status, setError }: {
  status: MachineStatus | null;
  setError: (value: string | null) => void;
}) {
  const requestDialog = useDialog();
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
  const [feederVision, setFeederVision] = useState<StripFeederVisionReport | null>(null);
  const [feederVisionBusy, setFeederVisionBusy] = useState(false);
  const [feederDraft, setFeederDraft] = useState<StripFeederRegistration | null>(null);
  const [feederHoldout, setFeederHoldout] = useState<StripFeederLiveHoldoutRun | null>(null);
  const [feederPickup, setFeederPickup] = useState<StripFeederPickupQualificationRun | null>(null);
  const [feederPickupSequence, setFeederPickupSequence] = useState<StripFeederPickupQualificationRun[]>([]);
  const [feederPartKey, setFeederPartKey] = useState("");
  const [feederPartRotation, setFeederPartRotation] = useState("");
  const [pcb, setPcb] = useState<File | null>(null);
  const [bom, setBom] = useState<File | null>(null);
  const precisionMotionReady = Boolean(
    status?.state === "ready" && status.homed && status.calibration_revision,
  );
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
    if (!precisionMotionReady) {
      setError("Connect, home, and fine-home the machine before board registration.");
      return;
    }
    if (await requestDialog({
      title: "Register this PCB?",
      message: "Confirm the PCB is fixed, both nozzles are level, the travel envelope is clear, and power isolation is reachable. The machine will fine-home X/Y and then move the top camera over each predicted board fiducial.",
      confirmLabel: "Start registration",
    }) === null) return;
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
  const analyzeStripFeeder = async (candidate: WorkspaceBoardCandidate) => {
    if (!selectedScan) return;
    setFeederVisionBusy(true); setError(null); setFeederVision(null); setFeederDraft(null); setFeederHoldout(null); setFeederPickup(null);
    try {
      setFeederVision(await api.analyzeStripFeeder(
        selectedScan, candidate.candidate_index,
      ));
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setFeederVisionBusy(false);
    }
  };
  const saveStripFeederDraft = async (proposalIndex: number) => {
    if (!selectedScan || !feederVision) return;
    const rotation = feederPartRotation.trim() === ""
      ? undefined : Number(feederPartRotation);
    if (rotation !== undefined && !Number.isFinite(rotation)) {
      setError("Part rotation must be a number."); return;
    }
    setFeederVisionBusy(true); setError(null); setFeederDraft(null);
    try {
      setFeederDraft(await api.createStripFeederDraft(
        selectedScan,
        feederVision.source_candidate_index,
        proposalIndex,
        feederPartKey.trim() || undefined,
        rotation,
      ));
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setFeederVisionBusy(false);
    }
  };
  const runStripFeederHoldout = async () => {
    if (!feederDraft) return;
    if (!precisionMotionReady) {
      setError("Connect, home, and fine-home the machine before a live holdout.");
      return;
    }
    if (await requestDialog({
      title: "Run a live pocket check?",
      message: "Confirm the tape is fixed, both nozzles are level, the travel envelope is clear, and power isolation is reachable. The machine will fine-home X/Y and make a short safe-Z camera sweep around one predicted pocket. It will not lower a nozzle or index tape.",
      confirmLabel: "Start check",
    }) === null) return;
    setFeederVisionBusy(true); setError(null); setFeederHoldout(null);
    try {
      const machine = await api.status();
      const lease = machine.control_lease
        ? await api.renewLease(machine.control_lease.id, 900)
        : await api.lease("VelocityPnP strip feeder holdout", 900);
      const started = await api.startStripFeederLiveHoldout(
        feederDraft.id, lease.id,
      );
      try { await waitForOperation(started.operation); } catch { /* Load retained rejection. */ }
      const holdout = await api.stripFeederLiveHoldout(started.holdout.id);
      setFeederHoldout(holdout);
      setFeederDraft(await api.stripFeederDraft(feederDraft.id));
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setFeederVisionBusy(false);
    }
  };
  const runStripFeederPickup = async () => {
    if (!feederDraft || feederHoldout?.report?.passed !== true) return;
    if (!precisionMotionReady) {
      setError("Connect, home, and fine-home the machine before a pickup check.");
      return;
    }
    if (await requestDialog({
      title: "Pick and inspect one component?",
      message: "Confirm the cover tape is removed, the carrier tape is fixed, both nozzles are level, the travel envelope is clear, and power isolation is reachable. The machine will use bounded vacuum Z detection, inspect the part with the bottom camera, and return it to the same pocket.",
      confirmLabel: "Start pickup check",
    }) === null) return;
    setFeederVisionBusy(true); setError(null); setFeederPickup(null);
    try {
      const machine = await api.status();
      const lease = machine.control_lease
        ? await api.renewLease(machine.control_lease.id, 900)
        : await api.lease("VelocityPnP strip feeder pickup", 900);
      const started = await api.startStripFeederPickupQualification(
        feederDraft.id, lease.id,
      );
      try { await waitForOperation(started.operation); } catch { /* Load retained failure. */ }
      setFeederPickup(await api.stripFeederPickupQualification(
        started.qualification.id,
      ));
      setFeederDraft(await api.stripFeederDraft(feederDraft.id));
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setFeederVisionBusy(false);
    }
  };
  const runStripFeederPickupSequence = async () => {
    if (!feederDraft || feederHoldout?.report?.passed !== true) return;
    if (!precisionMotionReady) {
      setError("Connect, home, and fine-home the machine before pickup qualification.");
      return;
    }
    const pockets = feederDraft.pockets.filter((pocket) =>
      pocket.status === "available" && pocket.occupancy === "occupied",
    ).slice(0, 3);
    if (pockets.length < 3) {
      setError("The feeder needs three distinct available occupied pockets."); return;
    }
    if (await requestDialog({
      title: "Run three-pocket qualification?",
      message: "Confirm the loose part is removed, the cover tape is open, the carrier tape is fixed, both nozzles are level, the travel envelope is clear, and power isolation is reachable. The machine will use three different feeder pockets.",
      confirmLabel: "Start qualification",
    }) === null) return;
    setFeederVisionBusy(true); setError(null); setFeederPickupSequence([]);
    try {
      const machine = await api.status();
      const lease = machine.control_lease
        ? await api.renewLease(machine.control_lease.id, 900)
        : await api.lease("VelocityPnP feeder pickup sequence", 900);
      const completed: StripFeederPickupQualificationRun[] = [];
      for (const pocket of pockets) {
        const started = await api.startStripFeederPickupQualification(
          feederDraft.id, lease.id, "N1", undefined, pocket.pocket_index,
        );
        try { await waitForOperation(started.operation); } catch { /* Retain failure evidence. */ }
        const run = await api.stripFeederPickupQualification(started.qualification.id);
        completed.push(run); setFeederPickup(run); setFeederPickupSequence([...completed]);
        if (run.report?.passed !== true) break;
      }
      setFeederDraft(await api.stripFeederDraft(feederDraft.id));
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setFeederVisionBusy(false);
    }
  };
  const reanalyzeStripFeederReturn = async () => {
    if (!feederPickup) return;
    setFeederVisionBusy(true); setError(null);
    try {
      setFeederPickup(await api.reanalyzeStripFeederReturn(feederPickup.id));
      if (feederDraft) {
        setFeederDraft(await api.stripFeederDraft(feederDraft.id));
      }
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setFeederVisionBusy(false);
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
        <select value={selectedScan} onChange={(event) => { setSelectedScan(event.target.value); setDiscovery(null); setChanges(null); setFeederVision(null); setFeederDraft(null); setFeederHoldout(null); setFeederPickup(null); }}>
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
      {selectedReport?.strip_feeder_inventory && <div className="run-result">
        <p><strong>{selectedReport.strip_feeder_inventory.populated_lane_count} populated strip feeders</strong>: {selectedReport.strip_feeder_inventory.present_count} present, {selectedReport.strip_feeder_inventory.absent_count} absent, and {selectedReport.strip_feeder_inventory.uncertain_count} uncertain feed positions{selectedReport.strip_feeder_inventory.coverage_complete ? "." : "; counts are lower bounds because tape reaches a scan edge."}</p>
        {selectedReport.strip_feeder_inventory.recommended_rescan_region && <p>Recommended rescan: X {selectedReport.strip_feeder_inventory.recommended_rescan_region.x_min_mm.toFixed(1)}–{selectedReport.strip_feeder_inventory.recommended_rescan_region.x_max_mm.toFixed(1)} mm, Y {selectedReport.strip_feeder_inventory.recommended_rescan_region.y_min_mm.toFixed(1)}–{selectedReport.strip_feeder_inventory.recommended_rescan_region.y_max_mm.toFixed(1)} mm.</p>}
        <a href={`/api/v1/workspace-scans/${selectedScan}/artifacts/${selectedReport.strip_feeder_inventory.preview_artifact_name}`} target="_blank" rel="noreferrer">View combined workspace inventory overlay</a>
      </div>}
      {discovery && !precisionMotionReady && <div className="notice compact-notice">Connect, home, and fine-home the machine to enable board registration. Image analysis remains available.</div>}
      {feederVision && <div className="run-result">
        <p><strong>{feederVision.proposals.length} strip lanes found</strong> in feeder proposal {feederVision.source_candidate_index + 1}. Yellow circles are sprocket detections, cyan lines are proposed pickup centerlines, green boxes are present components, and red marks are absent feed positions.</p>
        <img src={`/api/v1/workspace-scans/${selectedScan}/artifacts/${feederVision.preview_artifact_name}`} alt="Strip feeder lane analysis" />
        {feederVision.rejected_lattices.length > 0 && <p><strong>{feederVision.rejected_lattices.length} pocket-row {feederVision.rejected_lattices.length === 1 ? "lattice" : "lattices"} rejected</strong>. Red lines in the preview mark repeated component-pocket features that were paired with stronger sprocket rows.</p>}
        <div className="board-discovery-controls"><input value={feederPartKey} onChange={(event) => setFeederPartKey(event.target.value)} placeholder="Part key (optional)" /><input value={feederPartRotation} onChange={(event) => setFeederPartRotation(event.target.value)} placeholder="Part rotation ° (optional)" /></div>
        {feederVision.proposals.map((proposal) => <div className="feeder-proposal" key={proposal.proposal_index}><p>Lane {proposal.proposal_index + 1}: <strong>{proposal.tape_width_mm?.toFixed(0) ?? "unknown"} mm tape</strong>; {proposal.component_pitch_mm?.toFixed(0) ?? "unknown"} mm component pitch; {proposal.component_size_mm ? `${proposal.component_size_mm[0].toFixed(1)} × ${proposal.component_size_mm[1].toFixed(1)} mm parts; ` : "unknown part size; "}{proposal.present_count} present and {proposal.absent_count} absent; {proposal.sprocket_center_count} sprocket holes; {proposal.sprocket_pitch_residual_p95_mm.toFixed(3)} mm lattice p95; {(proposal.confidence * 100).toFixed(0)}% confidence{proposal.review_required ? "; review required" : "; draft gate passed"}.</p><button disabled={feederVisionBusy || proposal.tape_width_mm == null || proposal.component_pitch_mm == null || proposal.pick_centerline_machine_mm == null} onClick={() => void saveStripFeederDraft(proposal.proposal_index)}>Save lane as draft</button></div>)}
        {feederDraft && <div className="feeder-draft"><p><strong>{feederDraft.name} saved</strong> as {pretty(feederDraft.status)}. {feederDraft.pockets.filter((pocket) => pocket.status === "available").length} occupied pockets are available. Live pocket occupancy and pickup qualification are required before production use.</p><p><strong>Feeder gate:</strong> {feederDraft.status === "ready" ? "passed" : feederDraft.status === "placement_validation_required" ? "one guarded inspected placement is required" : "pickup qualification is required"}. The three-pocket return gate is {feederDraft.pickup_sequence_passed ? "passed" : "not passed"}.</p>{feederDraft.measured_part_size_mm && <p>Bottom vision measured the part as <strong>{feederDraft.measured_part_size_mm[0].toFixed(3)} × {feederDraft.measured_part_size_mm[1].toFixed(3)} mm</strong> at {feederDraft.measured_part_rotation_deg?.toFixed(2) ?? "unknown"}°.</p>}{!precisionMotionReady && <p className="notice compact-notice">Connect, home, and fine-home the machine to enable live feeder checks.</p>}<div className="button-cluster"><button disabled={feederVisionBusy || !precisionMotionReady} onClick={() => void runStripFeederHoldout()}>{feederVisionBusy ? "Checking…" : "Run live pocket holdout"}</button>{feederHoldout?.report?.passed && <button disabled={feederVisionBusy || !precisionMotionReady} onClick={() => void runStripFeederPickup()}>{feederVisionBusy ? "Qualifying…" : "Pick, inspect, and return one part"}</button>}{feederHoldout?.report?.passed && <button disabled={feederVisionBusy || !precisionMotionReady} onClick={() => void runStripFeederPickupSequence()}>{feederVisionBusy ? "Qualifying…" : "Run three-pocket qualification"}</button>}</div>{feederPickupSequence.length > 0 && <p><strong>{feederPickupSequence.filter((run) => run.report?.passed).length}/{feederPickupSequence.length} sequence cycles passed.</strong> The sequence stops on its first failure.</p>}</div>}
        {feederHoldout?.report && <div><p><strong>{feederHoldout.report.passed ? "Live holdout passed" : "Live holdout rejected"}</strong>: {feederHoldout.report.sprocket_center_count} sprocket centers, {feederHoldout.report.correction_magnitude_mm.toFixed(3)} mm local correction, {pretty(feederHoldout.report.occupancy)} at {(feederHoldout.report.occupancy_confidence * 100).toFixed(0)}% confidence.</p><a href={`/api/v1/strip-feeder-live-holdouts/${feederHoldout.id}/artifacts/${feederHoldout.report.artifact_name}`} target="_blank" rel="noreferrer">View live holdout evidence</a>{feederHoldout.report.findings.map((finding) => <p key={finding}>{finding}</p>)}</div>}
        {feederPickup?.report && <div><p><strong>{feederPickup.report.passed ? "Pickup qualification passed" : "Pickup qualification failed"}</strong>: pocket {feederPickup.pocket_index}; {feederPickup.report.pickup_nozzle_z_mm != null ? `pickup Z ${feederPickup.report.pickup_nozzle_z_mm.toFixed(3)} mm; ` : "no pickup Z; "}{feederPickup.report.vacuum_held_after_transfer ? "vacuum held; " : "vacuum hold failed; "}{pretty(feederPickup.report.disposition)}.</p>{feederPickup.report.alignment && <p>Bottom alignment {feederPickup.report.alignment.passed ? "passed" : "failed"}{feederPickup.report.alignment.final_centering_residual_mm != null ? ` with ${feederPickup.report.alignment.final_centering_residual_mm.toFixed(3)} mm center residual` : ""}{feederPickup.report.alignment.final_rotation_residual_deg != null ? ` and ${Math.abs(feederPickup.report.alignment.final_rotation_residual_deg).toFixed(2)}° axis residual` : ""}.</p>}{feederPickup.report.nozzle_release_verification && <p>Post-release nozzle check {feederPickup.report.nozzle_release_verification.passed ? "passed" : "failed"} after {feederPickup.report.nozzle_release_verification.attempt_count} attempt{feederPickup.report.nozzle_release_verification.attempt_count === 1 ? "" : "s"}.</p>}{feederPickup.report.return_verification?.return_center_shift_mm != null && <p>Return center shifted {feederPickup.report.return_verification.return_center_shift_mm.toFixed(3)} mm between the before and after views{feederPickup.report.return_verification.appearance_similarity != null ? `; package appearance similarity is ${feederPickup.report.return_verification.appearance_similarity.toFixed(3)}` : ""}.</p>}<button disabled={feederVisionBusy} onClick={() => void reanalyzeStripFeederReturn()}>{feederVisionBusy ? "Checking…" : "Reanalyze retained return images"}</button>{feederPickup.report.inspection && <a href={`/api/v1/strip-feeder-pickup-qualifications/${feederPickup.id}/artifacts/${feederPickup.report.inspection.artifact_name}`} target="_blank" rel="noreferrer">View bottom-camera pickup evidence</a>}{feederPickup.report.nozzle_release_verification?.artifact_names.at(-1) && <a href={`/api/v1/strip-feeder-pickup-qualifications/${feederPickup.id}/artifacts/${feederPickup.report.nozzle_release_verification.artifact_names.at(-1)}`} target="_blank" rel="noreferrer">View post-release nozzle evidence</a>}{feederPickup.report.return_verification && <a href={`/api/v1/strip-feeder-pickup-qualifications/${feederPickup.id}/artifacts/${feederPickup.report.return_verification.artifact_name}`} target="_blank" rel="noreferrer">View top-camera return evidence</a>}{feederPickup.report.findings.map((finding) => <p key={finding}>{finding}</p>)}</div>}
        {feederVision.warnings.map((warning) => <p className="warning" key={warning}>⚠ {warning}</p>)}
      </div>}
      {discovery && <div className="board-discovery-result">
        <img src={`/api/v1/workspace-scans/${selectedScan}/artifacts/mosaic-board-candidates.png`} alt="Workspace mosaic with semantic object proposals" />
        <div><p><strong>{discovery.candidates.length}</strong> raised-surface proposals. Known fixtures are purple, boards are green, feeders are blue, and unresolved objects are amber. Matched boards show enabled top-side placements in cyan and KiCad fiducials in yellow.</p>
          {discovery.warnings.map((warning) => <p className="warning" key={warning}>⚠ {warning}</p>)}
          <div className="candidate-list">{discovery.candidates.map((candidate) => <WorkspaceCandidateCard key={candidate.candidate_index} candidate={candidate} scanId={selectedScan} registering={registrationBusy} registrationAllowed={precisionMotionReady} analyzingFeeder={feederVisionBusy} onRegister={registerBoard} onAnalyzeFeeder={analyzeStripFeeder} />)}</div>
        </div>
      </div>}
    </section>
  </div>;
}

function WorkspaceCandidateCard({ candidate, scanId, registering, registrationAllowed, analyzingFeeder, onRegister, onAnalyzeFeeder }: {
  candidate: WorkspaceBoardCandidate;
  scanId: string;
  registering: boolean;
  registrationAllowed: boolean;
  analyzingFeeder: boolean;
  onRegister: (candidate: WorkspaceBoardCandidate, boardId: string) => void;
  onAnalyzeFeeder: (candidate: WorkspaceBoardCandidate) => void;
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
    {visualDesign && <button disabled={registering || !registrationAllowed} onClick={() => onRegister(candidate, visualDesign.board_id)}>{registering ? "Registering…" : "Register from fiducials"}</button>}
    {candidate.classification?.class === "feeder" && !candidate.known_fixture_id && <button disabled={analyzingFeeder} onClick={() => onAnalyzeFeeder(candidate)}>{analyzingFeeder ? "Analyzing…" : "Analyze strip lanes"}</button>}
  </div>;
}

export function pretty(value: string): string {
  return value.replaceAll(".", " ").replaceAll("_", " ").replace(/\b\w/g, (char) => char.toUpperCase());
}

function formatSize(board: BoardDesign): string {
  return board.width_mm != null && board.height_mm != null ? `${board.width_mm.toFixed(1)} × ${board.height_mm.toFixed(1)} mm` : "Unknown";
}
