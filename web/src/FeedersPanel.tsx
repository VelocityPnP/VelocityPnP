import { useCallback, useEffect, useMemo, useState } from "react";
import { api, waitForOperation } from "./api";
import { useDialog } from "./Dialog";
import {
  latestFeederLanes, workspaceComposition, workspaceFeederBanks,
  workspaceFeederLaneTargets, workspaceFeederSlotTargets,
} from "./WorkspacePanel";
import type {
  MachineStatus, Part, StripFeederMapRegistration, StripFeederRegistration,
  StripFeederRevisionSummary, WorkspaceBoardDiscoveryReport,
  WorkspaceScanSummary,
} from "./types";

interface FeedersPanelProps {
  status: MachineStatus;
  setError: (value: string | null) => void;
  openWorkspace: (targetId: string) => void;
}

interface FeederListItem {
  id: string;
  label: string;
  feeder: StripFeederMapRegistration | null;
}

function displayStatus(feeder: StripFeederMapRegistration | null): string {
  if (!feeder) return "Empty";
  if (feeder.vision_review_required) return "Needs review";
  if (feeder.status === "ready") return "Ready";
  if (!feeder.part_key) return "Tape detected";
  return "Qualification required";
}

function titleCase(value: string): string {
  return value.replaceAll("_", " ").replace(/\b\w/g, (letter) => letter.toUpperCase());
}

export function FeedersPanel({ status, setError, openWorkspace }: FeedersPanelProps) {
  const requestDialog = useDialog();
  const [scans, setScans] = useState<WorkspaceScanSummary[]>([]);
  const [discovery, setDiscovery] = useState<WorkspaceBoardDiscoveryReport | null>(null);
  const [mapFeeders, setMapFeeders] = useState<StripFeederMapRegistration[]>([]);
  const [registrations, setRegistrations] = useState<StripFeederRegistration[]>([]);
  const [parts, setParts] = useState<Part[]>([]);
  const [selectedId, setSelectedId] = useState("");
  const [revisions, setRevisions] = useState<StripFeederRevisionSummary[]>([]);
  const [partKey, setPartKey] = useState("");
  const [partRotation, setPartRotation] = useState("0");
  const [liveHoldoutPassed, setLiveHoldoutPassed] = useState(false);
  const [busy, setBusy] = useState(false);
  const [progress, setProgress] = useState("");

  const load = useCallback(async () => {
    try {
      const [scanValues, mapValues, registrationValues, partValues] = await Promise.all([
        api.workspaceScanSummaries(), api.stripFeederPocketMap(),
        api.stripFeederRegistrations(), api.parts(),
      ]);
      setScans(scanValues);
      setMapFeeders(mapValues);
      setRegistrations(registrationValues);
      setParts(partValues.sort((left, right) => left.key.localeCompare(right.key)));
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    }
  }, [setError]);

  useEffect(() => {
    void load();
    const timer = window.setInterval(() => void load(), 3000);
    return () => window.clearInterval(timer);
  }, [load]);

  const base = useMemo(() => {
    const ordered = [...scans].sort(
      (left, right) => right.updated_at.localeCompare(left.updated_at),
    );
    return workspaceComposition(ordered, status.profile_id).bases[0] ?? null;
  }, [scans, status.profile_id]);

  useEffect(() => {
    setDiscovery(null);
    const summary = base?.board_discovery;
    if (!base || !summary) return;
    let cancelled = false;
    void api.workspaceScanArtifact<WorkspaceBoardDiscoveryReport>(
      base.id, summary.report_artifact_name,
    ).then((value) => {
      if (!cancelled) setDiscovery(value);
    }).catch((reason) => {
      if (!cancelled) setError(reason instanceof Error ? reason.message : String(reason));
    });
    return () => { cancelled = true; };
  }, [base?.board_discovery?.report_artifact_name, base?.id, setError]);

  const feederItems = useMemo(() => {
    const lanes = latestFeederLanes(mapFeeders, status.profile_id, base);
    const banks = workspaceFeederBanks(discovery);
    const laneTargets = workspaceFeederLaneTargets(lanes, banks);
    const items = new Map<string, FeederListItem>();
    for (const slot of workspaceFeederSlotTargets(banks, lanes)) {
      items.set(slot.label, { id: slot.id, label: slot.label, feeder: null });
    }
    for (const lane of laneTargets) {
      items.set(lane.label, {
        id: lane.feeder.id,
        label: lane.label,
        feeder: lane.feeder,
      });
    }
    return [...items.values()].sort((left, right) =>
      left.label.localeCompare(right.label, undefined, { numeric: true })
    );
  }, [base, discovery, mapFeeders, status.profile_id]);

  useEffect(() => {
    setSelectedId((current) => feederItems.some((item) => item.id === current)
      ? current
      : feederItems.find((item) => item.feeder !== null)?.id
        ?? feederItems[0]?.id
        ?? "");
  }, [feederItems]);

  const selected = feederItems.find((item) => item.id === selectedId) ?? null;
  const registration = registrations.find(
    (candidate) => candidate.id === selected?.feeder?.id,
  ) ?? null;

  useEffect(() => {
    setPartKey(registration?.part_key ?? "");
    setPartRotation(registration?.part_rotation_deg?.toString() ?? "0");
    setRevisions([]);
    if (!registration) return;
    let cancelled = false;
    void api.stripFeederRevisions(registration.id).then((values) => {
      if (!cancelled) setRevisions(values);
    }).catch((reason) => {
      if (!cancelled) setError(reason instanceof Error ? reason.message : String(reason));
    });
    return () => { cancelled = true; };
  }, [registration?.id, setError]);

  useEffect(() => {
    setLiveHoldoutPassed(false);
    if (!registration?.latest_live_holdout_id) return;
    let cancelled = false;
    void api.stripFeederLiveHoldout(registration.latest_live_holdout_id)
      .then((run) => {
        if (!cancelled) setLiveHoldoutPassed(run.report?.passed === true);
      })
      .catch(() => {
        if (!cancelled) setLiveHoldoutPassed(false);
      });
    return () => { cancelled = true; };
  }, [registration?.latest_live_holdout_id]);

  const act = async (action: () => Promise<void>) => {
    setBusy(true);
    setError(null);
    try {
      await action();
      await load();
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setBusy(false);
      setProgress("");
    }
  };

  const saveAssignment = () => void act(async () => {
    if (!registration) return;
    const rotation = partKey ? Number(partRotation) : null;
    if (rotation !== null && !Number.isFinite(rotation)) {
      throw new Error("Part rotation must be a number.");
    }
    await api.updateStripFeederAssignment(
      registration.id, partKey || null, rotation,
    );
  });

  const runLiveCheck = () => void act(async () => {
    if (!registration) return;
    if (status.state !== "ready" || !status.homed || !status.calibration_revision) {
      throw new Error("Home and fine-home the machine before a live check.");
    }
    if (await requestDialog({
      title: `Check ${selected?.label}?`,
      message: "Confirm that the tape is fixed, both nozzles are level, the travel path is clear, and power isolation is reachable.",
      confirmLabel: "Start live check",
    }) === null) return;
    setProgress("Running live pocket check…");
    const lease = await api.acquireOrRenewLease("VelocityPnP feeder live check", 900);
    const started = await api.startStripFeederLiveHoldout(registration.id, lease.id);
    await waitForOperation(started.operation, (operation) => {
      setProgress(operation.message ?? "Running live pocket check…");
    });
  });

  const runQualification = () => void act(async () => {
    if (!registration) return;
    if (!liveHoldoutPassed) {
      throw new Error("Run a successful live pocket check first.");
    }
    if (status.state !== "ready" || !status.homed || !status.calibration_revision) {
      throw new Error("Home and fine-home the machine before qualification.");
    }
    const pockets = registration.pockets.filter((pocket) =>
      pocket.status === "available" && pocket.occupancy === "occupied"
    ).slice(0, 3);
    if (pockets.length < 3) {
      throw new Error("The feeder needs three available occupied pockets.");
    }
    if (await requestDialog({
      title: `Qualify ${selected?.label}?`,
      message: "Confirm that the cover tape is open, the carrier tape is fixed, both nozzles are level, the travel path is clear, and power isolation is reachable. The machine will pick, inspect, and return parts from three pockets.",
      confirmLabel: "Start qualification",
    }) === null) return;
    const lease = await api.acquireOrRenewLease("VelocityPnP feeder qualification", 900);
    for (const [index, pocket] of pockets.entries()) {
      setProgress(`Qualifying pocket ${index + 1} of ${pockets.length}…`);
      const started = await api.startStripFeederPickupQualification(
        registration.id, lease.id, "N1", undefined, pocket.pocket_index,
      );
      await waitForOperation(started.operation, (operation) => {
        setProgress(operation.message ?? `Qualifying pocket ${index + 1}…`);
      });
      const completed = await api.stripFeederPickupQualification(
        started.qualification.id,
      );
      if (completed.report?.passed !== true) {
        throw new Error(completed.message ?? "Feeder qualification failed.");
      }
    }
  });

  const occupied = selected?.feeder?.pockets.filter(
    (pocket) => pocket.occupancy === "occupied",
  ).length ?? 0;
  const empty = selected?.feeder?.pockets.filter(
    (pocket) => pocket.occupancy === "empty",
  ).length ?? 0;
  const review = selected?.feeder?.pockets.filter(
    (pocket) => pocket.occupancy === "uncertain" || pocket.occupancy_review_required,
  ).length ?? 0;

  return <div className="feeders-layout">
    <section className="panel feeder-lane-list">
      <div className="section-heading">
        <div><p className="eyebrow">Detected hardware</p><h2>Feeder lanes</h2></div>
        <span className="badge">{feederItems.length}</span>
      </div>
      {!base && <div className="notice">Run a full workspace scan first.</div>}
      {base && !discovery && <div className="notice">Analyze workspace objects to find empty lanes.</div>}
      {feederItems.map((item) => <button
        className={`feeder-lane-row ${selectedId === item.id ? "is-selected" : ""}`}
        key={item.id}
        onClick={() => setSelectedId(item.id)}
        type="button"
      >
        <span><strong>{item.label}</strong><small>{item.feeder?.part_key ?? "No part assigned"}</small></span>
        <span className={`badge feeder-state-${item.feeder?.status ?? "empty"}`}>
          {displayStatus(item.feeder)}
        </span>
      </button>)}
      {base && feederItems.length === 0 && <p>No feeder lanes were recognized.</p>}
    </section>

    <section className="panel feeder-detail">
      {!selected && <div className="empty-board"><strong>Select a feeder lane.</strong></div>}
      {selected && <>
        <div className="section-heading">
          <div><p className="eyebrow">Stable lane identity</p><h2>{selected.label}</h2></div>
          <span className={`badge feeder-state-${selected.feeder?.status ?? "empty"}`}>
            {displayStatus(selected.feeder)}
          </span>
        </div>
        {!selected.feeder ? <>
          <div className="notice">This lane is recognized. No carrier tape is registered.</div>
          <button onClick={() => openWorkspace(selected.id)} type="button">
            Load tape and rescan in Workspace
          </button>
        </> : <>
          <dl className="workspace-facts feeder-summary">
            <div><dt>Assigned part</dt><dd>{selected.feeder.part_key ?? "Not assigned"}</dd></div>
            <div><dt>Tape width / pitch</dt><dd>{selected.feeder.tape_width_mm.toFixed(0)} / {selected.feeder.tape_pitch_mm.toFixed(1)} mm</dd></div>
            <div><dt>Pockets</dt><dd>{occupied} occupied · {empty} empty · {review} review</dd></div>
            <div><dt>Active revision</dt><dd>{selected.feeder.active_revision
              ? new Date(selected.feeder.active_revision.created_at).toLocaleString()
              : "Initial survey"}</dd></div>
            <div><dt>Qualification</dt><dd>{titleCase(selected.feeder.status)}</dd></div>
            <div><dt>Pickup Z</dt><dd>{selected.feeder.pickup_z_mm === null
              ? "Not measured"
              : `${selected.feeder.pickup_z_mm.toFixed(3)} mm`}</dd></div>
          </dl>
          <div className="feeder-action-row">
            <button onClick={() => openWorkspace(selected.id)} type="button">
              Open live controls
            </button>
            <button
              className="secondary"
              disabled={busy || !registration?.part_key}
              onClick={runLiveCheck}
              type="button"
            >Run live check</button>
            <button
              className="secondary"
              disabled={busy || !liveHoldoutPassed}
              onClick={runQualification}
              type="button"
            >Qualify three pockets</button>
          </div>
          {progress && <div className="notice" role="status">{progress}</div>}
          <section className="feeder-assignment">
            <h3>Part assignment</h3>
            <div className="form-grid compact-form-grid">
              <label><span>Part</span><select
                disabled={busy}
                onChange={(event) => setPartKey(event.target.value)}
                value={partKey}
              >
                <option value="">No part assigned</option>
                {parts.map((part) => <option key={part.id} value={part.key}>
                  {part.key} · {part.package}
                </option>)}
              </select></label>
              <label><span>Part rotation</span><input
                disabled={busy || !partKey}
                inputMode="decimal"
                onChange={(event) => setPartRotation(event.target.value)}
                value={partRotation}
              /></label>
            </div>
            <button
              disabled={busy || !registration}
              onClick={saveAssignment}
              type="button"
            >{busy ? "Working…" : "Save assignment"}</button>
            <small>Changing the assigned part disables this feeder until it is qualified again.</small>
          </section>
          <details className="feeder-history">
            <summary>Scan history ({selected.feeder.revision_count})</summary>
            {revisions.length === 0 && <p>No rescan revisions are stored.</p>}
            {revisions.map((revision) => <article key={`${revision.source_scan_id}-${revision.created_at}`}>
              <strong>{new Date(revision.created_at).toLocaleString()}</strong>
              <span>{revision.pocket_count} pockets · {revision.occupied_count} occupied · {revision.empty_count} empty · {revision.review_count} review</span>
              <small>{revision.model_available ? "Model evidence used" : "Model evidence unavailable"}</small>
            </article>)}
          </details>
        </>}
      </>}
    </section>
  </div>;
}
