import { useCallback, useEffect, useMemo, useState } from "react";
import { api, waitForOperation } from "./api";
import { useDialog } from "./Dialog";
import {
  feederScanRegion, formatWorkspaceDuration, latestFeederLanes,
  nearestFeederProposal, workspaceComposition, workspaceFeederBanks,
  workspaceFeederLaneTargets, workspaceFeederSlotTargets,
  workspaceRescanEtaSeconds,
} from "./WorkspacePanel";
import type {
  MachineStatus, Part, StripFeederMapRegistration, StripFeederOccupancyReviewQueue,
  StripFeederPocketReviewLabel, StripFeederRegistration, StripFeederRevisionSummary,
  WorkspaceBoardDiscoveryReport,
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

interface FeederRescanProgress {
  capturedTiles: number;
  plannedTiles: number;
  estimatedDurationSeconds: number;
  startedAtMs: number;
}

interface PocketReviewEdit {
  label?: StripFeederPocketReviewLabel;
  note: string;
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

function reviewLabel(value: StripFeederPocketReviewLabel): string {
  if (value === "component_on_edge") return "On edge";
  return titleCase(value);
}

function sentenceCase(value: string): string {
  return value.length === 0 ? value : value[0].toUpperCase() + value.slice(1);
}

function reviewPocketStateClass(
  pocket: StripFeederOccupancyReviewQueue["pockets"][number],
  pendingLabel: StripFeederPocketReviewLabel | null,
): string {
  if (pocket.unresolved && pendingLabel === null) return "is-unresolved";
  const effectiveLabel = pendingLabel ?? pocket.review?.label;
  if (effectiveLabel === "occupied"
    || (effectiveLabel === undefined && pocket.effective_occupancy === "occupied")) {
    return "is-occupied";
  }
  return "is-empty";
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
  const [rescanProgress, setRescanProgress] = useState<FeederRescanProgress | null>(null);
  const [rescanResult, setRescanResult] = useState("");
  const [reviewQueue, setReviewQueue] = useState<StripFeederOccupancyReviewQueue | null>(null);
  const [reviewPocketIndex, setReviewPocketIndex] = useState<number | null>(null);
  const [reviewNote, setReviewNote] = useState("");
  const [reviewEdits, setReviewEdits] = useState<Record<number, PocketReviewEdit>>({});
  const [reviewBusy, setReviewBusy] = useState(false);

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
  const activeLaneScan = useMemo(() => [...scans]
    .filter((scan) => (scan.status === "pending" || scan.status === "running")
      && scan.parent_scan_id === base?.id)
    .sort((left, right) => right.updated_at.localeCompare(left.updated_at))[0] ?? null,
  [base?.id, scans]);

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
    setRescanProgress(null);
    setRescanResult("");
    setReviewQueue(null);
    setReviewPocketIndex(null);
    setReviewNote("");
    setReviewEdits({});
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
    const activeRevisionCreatedAt = selected?.feeder?.active_revision?.created_at;
    let cancelled = false;
    void api.stripFeederLiveHoldout(registration.latest_live_holdout_id)
      .then((run) => {
        const followsActiveRevision = activeRevisionCreatedAt === undefined
          || new Date(run.created_at).getTime() >= new Date(activeRevisionCreatedAt).getTime();
        if (!cancelled) {
          setLiveHoldoutPassed(run.report?.passed === true && followsActiveRevision);
        }
      })
      .catch(() => {
        if (!cancelled) setLiveHoldoutPassed(false);
      });
    return () => { cancelled = true; };
  }, [
    registration?.latest_live_holdout_id,
    selected?.feeder?.active_revision?.created_at,
  ]);

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

  const runRescan = () => void act(async () => {
    if (!base || !selected?.feeder || !registration) return;
    if (status.state !== "ready" || !status.homed || !status.calibration_revision) {
      throw new Error("Home and fine-home the machine before a feeder rescan.");
    }
    setRescanProgress(null);
    setRescanResult("");
    setProgress("Estimating lane rescan…");
    const parentScan = await api.workspaceScan(base.id);
    if (parentScan.status !== "succeeded") {
      throw new Error("The full workspace scan is not available.");
    }
    if (parentScan.calibration_id !== status.calibration_revision) {
      throw new Error("The workspace scan uses a different calibration. Run a new full workspace scan.");
    }
    const region = feederScanRegion(selected.feeder, parentScan.region);
    const estimate = await api.estimateWorkspaceSurvey(
      parentScan.id, parentScan.calibration_id, region,
    );
    const confirmed = await requestDialog({
      title: `Rescan ${selected.label}?`,
      message: `Scan this complete feeder lane and refresh its pocket occupancy. Estimated scan time is about ${formatWorkspaceDuration(estimate.estimated_duration_seconds)} for ${estimate.planned_tiles} tiles. A short lane-alignment check will run automatically after the scan. Confirm that the tape is fixed, both nozzles are level, the travel path is clear, and power isolation is reachable.`,
      confirmLabel: "Start rescan",
    });
    if (confirmed === null) return;

    const lease = await api.acquireOrRenewLease("VelocityPnP feeder rescan", 3600);
    setProgress("Preparing lane rescan…");
    const started = await api.startWorkspaceSurvey(
      parentScan.id, parentScan.calibration_id, region, lease.id,
    );
    const startedAtMs = Date.now();
    setRescanProgress({
      capturedTiles: started.scan.captured_tiles,
      plannedTiles: started.scan.planned_tiles,
      estimatedDurationSeconds: started.scan.estimated_duration_seconds,
      startedAtMs,
    });
    let progressTimer: number | null = null;
    const refreshProgress = async () => {
      try {
        const scanValue = await api.workspaceScan(started.scan.id);
        setRescanProgress({
          capturedTiles: scanValue.captured_tiles,
          plannedTiles: scanValue.planned_tiles,
          estimatedDurationSeconds: scanValue.estimated_duration_seconds,
          startedAtMs,
        });
      } catch {
        // Keep the latest progress while the operation poll remains active.
      }
    };
    try {
      progressTimer = window.setInterval(() => void refreshProgress(), 500);
      const completed = await waitForOperation(started.operation, (operation) => {
        setProgress(operation.message ?? "Scanning feeder lane…");
      });
      if (completed.status !== "succeeded") {
        throw new Error(completed.message ?? "The feeder rescan did not complete.");
      }
      await refreshProgress();
      setProgress("Detecting tape and pocket occupancy…");
      const discoveryReport = await api.discoverWorkspaceObjects(started.scan.id);
      const laneCenterSum = selected.feeder.pockets.reduce<[number, number]>(
        (sum, pocket) => [
          sum[0] + pocket.center_machine_mm[0],
          sum[1] + pocket.center_machine_mm[1],
        ],
        [0, 0],
      );
      const pocketCount = Math.max(selected.feeder.pockets.length, 1);
      const laneCenter: [number, number] = [
        laneCenterSum[0] / pocketCount,
        laneCenterSum[1] / pocketCount,
      ];
      const candidates = discoveryReport.candidates.filter((candidate) => {
        const classification = candidate.classification;
        return classification?.class === "feeder"
          || (classification?.class === "unknown"
            && classification.evidence.some((evidence) =>
              evidence.suggested_class === "feeder" && evidence.confidence >= .5));
      }).sort((left, right) => Math.hypot(
        left.center_x_mm - laneCenter[0],
        left.center_y_mm - laneCenter[1],
      ) - Math.hypot(
        right.center_x_mm - laneCenter[0],
        right.center_y_mm - laneCenter[1],
      ));
      const candidate = candidates[0];
      if (!candidate) {
        throw new Error("No feeder lane was detected in the rescan.");
      }
      const vision = await api.analyzeStripFeeder(
        started.scan.id, candidate.candidate_index,
      );
      if (!nearestFeederProposal(vision.proposals, laneCenter)) {
        throw new Error("No carrier tape was detected in the selected lane.");
      }
      const occupancy = await api.refreshStripFeederOccupancy(
        started.scan.id, candidate.candidate_index, registration.id,
      );
      const canCheckAlignment = occupancy.observations.some(
        (observation) => observation.occupancy === "occupied"
          && !observation.review_required,
      );
      let alignmentResult = "Lane alignment was not checked because no occupied pocket is ready.";
      if (canCheckAlignment) {
        setProgress("Checking lane alignment…");
        const alignment = await api.startStripFeederLiveHoldout(
          registration.id, lease.id,
        );
        await waitForOperation(alignment.operation, (operation) => {
          setProgress(operation.message ?? "Checking lane alignment…");
        });
        alignmentResult = "Lane alignment passed automatically.";
      }
      setRevisions(await api.stripFeederRevisions(registration.id));
      const modelObservationCount = occupancy.observations.filter(
        (observation) => observation.model_prediction !== null,
      ).length;
      setRescanResult([
        `Rescan complete. ${occupancy.observations.length} pocket states were refreshed.`,
        modelObservationCount > 0
          ? `The source-component model analyzed ${modelObservationCount}.`
          : occupancy.model_available
            ? "The model did not produce usable evidence."
            : "The source-component model was not available.",
        alignmentResult,
      ].join(" "));
    } finally {
      if (progressTimer !== null) window.clearInterval(progressTimer);
      setRescanProgress(null);
    }
  });

  const runLiveCheck = () => void act(async () => {
    if (!registration) return;
    if (status.state !== "ready" || !status.homed || !status.calibration_revision) {
      throw new Error("Home and fine-home the machine before checking lane alignment.");
    }
    if (await requestDialog({
      title: `Recheck ${selected?.label} alignment?`,
      message: "Confirm that the tape is fixed, both nozzles are level, the travel path is clear, and power isolation is reachable.",
      confirmLabel: "Start alignment check",
    }) === null) return;
    setProgress("Checking lane alignment…");
    const lease = await api.acquireOrRenewLease("VelocityPnP feeder alignment", 900);
    const started = await api.startStripFeederLiveHoldout(registration.id, lease.id);
    await waitForOperation(started.operation, (operation) => {
      setProgress(operation.message ?? "Checking lane alignment…");
    });
  });

  const runQualification = () => void act(async () => {
    if (!registration) return;
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
      message: `Confirm that the cover tape is open, the carrier tape is fixed, both nozzles are level, the travel path is clear, and power isolation is reachable. ${liveHoldoutPassed ? "" : "The machine will first check lane alignment. "}The machine will pick, inspect, and return parts from three pockets.`,
      confirmLabel: "Start qualification",
    }) === null) return;
    const lease = await api.acquireOrRenewLease("VelocityPnP feeder qualification", 900);
    if (!liveHoldoutPassed) {
      setProgress("Checking lane alignment…");
      const alignment = await api.startStripFeederLiveHoldout(
        registration.id, lease.id,
      );
      await waitForOperation(alignment.operation, (operation) => {
        setProgress(operation.message ?? "Checking lane alignment…");
      });
    }
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

  const openReview = () => void (async () => {
    if (!registration) return;
    setError(null);
    setReviewBusy(true);
    try {
      const queue = await api.stripFeederOccupancyReview(registration.id);
      setReviewQueue(queue);
      const first = queue.pockets.find((pocket) => pocket.unresolved)
        ?? queue.pockets[0] ?? null;
      setReviewPocketIndex(first?.pocket_index ?? null);
      setReviewNote(first?.review?.note ?? "");
      setReviewEdits({});
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setReviewBusy(false);
    }
  })();

  const selectReviewPocket = (pocketIndex: number) => {
    setReviewPocketIndex(pocketIndex);
    const pocket = reviewQueue?.pockets.find(
      (candidate) => candidate.pocket_index === pocketIndex,
    );
    setReviewNote(reviewEdits[pocketIndex]?.note ?? pocket?.review?.note ?? "");
  };

  const selectAdjacentReviewPocket = (
    pocketIndex: number, direction: -1 | 1, button: HTMLButtonElement,
  ) => {
    if (!reviewQueue || reviewQueue.pockets.length === 0) return;
    const position = reviewQueue.pockets.findIndex(
      (pocket) => pocket.pocket_index === pocketIndex,
    );
    if (position < 0) return;
    const nextPosition = (
      position + direction + reviewQueue.pockets.length
    ) % reviewQueue.pockets.length;
    selectReviewPocket(reviewQueue.pockets[nextPosition].pocket_index);
    button.parentElement
      ?.querySelectorAll<HTMLButtonElement>("button")[nextPosition]
      ?.focus();
  };

  const stagePocketReview = (label: StripFeederPocketReviewLabel) => {
    if (reviewPocketIndex === null) return;
    setReviewEdits((current) => ({
      ...current,
      [reviewPocketIndex]: { label, note: reviewNote },
    }));
  };

  const closePocketReview = () => {
    setReviewQueue(null);
    setReviewPocketIndex(null);
    setReviewNote("");
    setReviewEdits({});
  };

  const saveAndClosePocketReview = () => void (async () => {
    if (!registration || !reviewQueue) return;
    const edits = Object.entries(reviewEdits)
      .filter((entry): entry is [string, PocketReviewEdit & {
        label: StripFeederPocketReviewLabel;
      }] => entry[1].label !== undefined)
      .sort(([left], [right]) => Number(left) - Number(right));
    const incompleteEdit = Object.values(reviewEdits).some(
      (edit) => edit.label === undefined,
    );
    if (incompleteEdit) {
      setError("Select a review result for each edited note.");
      return;
    }
    setError(null);
    if (edits.length === 0) {
      closePocketReview();
      return;
    }
    setReviewBusy(true);
    try {
      for (const [pocketIndex, edit] of edits) {
        await api.reviewStripFeederPocketOccupancy(
          registration.id,
          Number(pocketIndex),
          reviewQueue.lane_revision_id,
          edit.label,
          edit.note,
        );
      }
      await load();
      closePocketReview();
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setReviewBusy(false);
    }
  })();

  const occupied = selected?.feeder?.pockets.filter(
    (pocket) => pocket.occupancy === "occupied",
  ).length ?? 0;
  const empty = selected?.feeder?.pockets.filter(
    (pocket) => pocket.occupancy === "empty",
  ).length ?? 0;
  const review = selected?.feeder?.pockets.filter(
    (pocket) => pocket.occupancy_review_required === true,
  ).length ?? 0;
  const displayedCapturedTiles = rescanProgress?.capturedTiles
    ?? activeLaneScan?.captured_tiles ?? null;
  const displayedPlannedTiles = rescanProgress?.plannedTiles
    ?? activeLaneScan?.planned_tiles ?? null;
  const rescanPercent = displayedPlannedTiles
    ? Math.round((displayedCapturedTiles ?? 0) / displayedPlannedTiles * 100)
    : 0;
  const rescanEtaSeconds = rescanProgress
    ? workspaceRescanEtaSeconds(
      rescanProgress.capturedTiles,
      rescanProgress.plannedTiles,
      rescanProgress.estimatedDurationSeconds,
      (Date.now() - rescanProgress.startedAtMs) / 1000,
    )
    : 0;
  const reviewPocket = reviewQueue?.pockets.find(
    (pocket) => pocket.pocket_index === reviewPocketIndex,
  ) ?? null;
  const currentReviewLabel = reviewPocket?.review?.label
    ?? (reviewPocket?.effective_occupancy === "occupied"
      || reviewPocket?.effective_occupancy === "empty"
      ? reviewPocket.effective_occupancy
      : null);
  const pendingReviewLabel = reviewPocketIndex === null
    ? null
    : reviewEdits[reviewPocketIndex]?.label ?? null;
  const displayedReviewLabel = pendingReviewLabel ?? currentReviewLabel;
  const stagedReviewIndices = new Set(Object.entries(reviewEdits)
    .filter(([, edit]) => edit.label !== undefined)
    .map(([pocketIndex]) => Number(pocketIndex)));
  const displayedUnresolvedCount = reviewQueue?.pockets.filter(
    (pocket) => pocket.unresolved && !stagedReviewIndices.has(pocket.pocket_index),
  ).length ?? 0;
  const reviewMosaicUrl = reviewQueue
    ? `/api/v1/workspace-scans/${reviewQueue.source_scan_id}/artifacts/${encodeURIComponent(reviewQueue.mosaic.artifact_name)}`
    : "";
  const reviewPixel = reviewQueue && reviewPocket ? [
    (reviewPocket.center_machine_mm[0]
      - reviewQueue.mosaic.raster_transform.top_left_x_mm)
      / reviewQueue.mosaic.raster_transform.x_mm_per_pixel,
    (reviewPocket.center_machine_mm[1]
      - reviewQueue.mosaic.raster_transform.top_left_y_mm)
      / reviewQueue.mosaic.raster_transform.y_mm_per_pixel,
  ] as [number, number] : null;
  const reviewZoomWidth = reviewQueue ? Math.min(360, reviewQueue.mosaic.width_px) : 360;
  const reviewZoomHeight = reviewQueue ? Math.min(300, reviewQueue.mosaic.height_px) : 300;

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
      {!selected && <div className="empty-board"><strong>{feederItems.length
        ? "Select a feeder lane"
        : "No feeder lanes available"}</strong></div>}
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
              disabled={busy || !registration}
              onClick={runRescan}
              type="button"
            >Rescan lane</button>
            <button
              className="secondary"
              disabled={busy || reviewBusy || !selected.feeder.active_revision}
              onClick={openReview}
              type="button"
            >{review > 0 ? `Needs review (${review})` : "Review pockets"}</button>
            <button
              className="secondary"
              disabled={busy || !registration?.part_key}
              onClick={runLiveCheck}
              type="button"
            >Recheck lane alignment</button>
            <button
              className="secondary"
              disabled={busy || occupied < 3}
              onClick={runQualification}
              type="button"
            >Qualify three pockets</button>
          </div>
          {reviewQueue && reviewPocket && reviewPixel && <section className="feeder-review">
            <div className="section-heading">
              <div>
                <p className="eyebrow">Promoted lane revision</p>
                <h3>Pocket occupancy review</h3>
              </div>
              <div className="feeder-review-heading-actions">
                <span className="badge">{displayedUnresolvedCount} unresolved</span>
                <button
                  className="secondary compact-button"
                  disabled={reviewBusy}
                  onClick={closePocketReview}
                  type="button"
                >Discard Changes</button>
                <button
                  className="compact-button"
                  disabled={reviewBusy}
                  onClick={saveAndClosePocketReview}
                  type="button"
                >{reviewBusy ? "Saving…" : "Save and Close"}</button>
              </div>
            </div>
            <div className="feeder-review-images">
              <svg
                aria-label="Feeder rescan overview"
                className="feeder-review-overview"
                role="img"
                viewBox={`0 0 ${reviewQueue.mosaic.width_px} ${reviewQueue.mosaic.height_px}`}
              >
                <image
                  height={reviewQueue.mosaic.height_px}
                  href={reviewMosaicUrl}
                  width={reviewQueue.mosaic.width_px}
                />
                {reviewQueue.pockets.map((pocket) => {
                  const x = (pocket.center_machine_mm[0]
                    - reviewQueue.mosaic.raster_transform.top_left_x_mm)
                    / reviewQueue.mosaic.raster_transform.x_mm_per_pixel;
                  const y = (pocket.center_machine_mm[1]
                    - reviewQueue.mosaic.raster_transform.top_left_y_mm)
                    / reviewQueue.mosaic.raster_transform.y_mm_per_pixel;
                  return <circle
                    className={`feeder-review-marker ${pocket.unresolved && !stagedReviewIndices.has(pocket.pocket_index) ? "is-unresolved" : ""} ${stagedReviewIndices.has(pocket.pocket_index) ? "is-pending" : ""} ${pocket.pocket_index === reviewPocketIndex ? "is-selected" : ""}`}
                    cx={x}
                    cy={y}
                    key={pocket.pocket_index}
                    onClick={() => selectReviewPocket(pocket.pocket_index)}
                    r={pocket.pocket_index === reviewPocketIndex ? 10 : 6}
                  />;
                })}
              </svg>
              <svg
                aria-label={`Pocket ${reviewPocket.pocket_index} review image`}
                className="feeder-review-zoom"
                role="img"
                viewBox={`${reviewPixel[0] - reviewZoomWidth / 2} ${reviewPixel[1] - reviewZoomHeight / 2} ${reviewZoomWidth} ${reviewZoomHeight}`}
              >
                <image
                  height={reviewQueue.mosaic.height_px}
                  href={reviewMosaicUrl}
                  width={reviewQueue.mosaic.width_px}
                />
                <line
                  className="feeder-review-crosshair"
                  x1={reviewPixel[0] - reviewZoomWidth / 2}
                  x2={reviewPixel[0] + reviewZoomWidth / 2}
                  y1={reviewPixel[1]}
                  y2={reviewPixel[1]}
                />
                <line
                  className="feeder-review-crosshair"
                  x1={reviewPixel[0]}
                  x2={reviewPixel[0]}
                  y1={reviewPixel[1] - reviewZoomHeight / 2}
                  y2={reviewPixel[1] + reviewZoomHeight / 2}
                />
              </svg>
            </div>
            <div className="feeder-review-pocket-list" aria-label="Review pocket selection">
              {reviewQueue.pockets.map((pocket) => <button
                aria-pressed={pocket.pocket_index === reviewPocketIndex}
                className={`${reviewPocketStateClass(
                  pocket,
                  reviewEdits[pocket.pocket_index]?.label ?? null,
                )} ${stagedReviewIndices.has(pocket.pocket_index) ? "is-pending" : ""}`}
                key={pocket.pocket_index}
                onClick={() => selectReviewPocket(pocket.pocket_index)}
                onKeyDown={(event) => {
                  if (event.key !== "ArrowLeft" && event.key !== "ArrowRight") return;
                  event.preventDefault();
                  selectAdjacentReviewPocket(
                    pocket.pocket_index,
                    event.key === "ArrowLeft" ? -1 : 1,
                    event.currentTarget,
                  );
                }}
                type="button"
              >{pocket.pocket_index}</button>)}
            </div>
            <div className="feeder-review-details">
              <div>
                <strong>Pocket {reviewPocket.pocket_index}</strong>
                <span>{reviewPocket.model_prediction
                  ? `${titleCase(reviewPocket.model_prediction.class)} · ${(reviewPocket.model_prediction.class_confidence * 100).toFixed(1)}%`
                  : "No usable model prediction"}</span>
                <small>{reviewPocket.review
                  ? `Operator: ${reviewLabel(reviewPocket.review.label)}`
                  : `Current: ${titleCase(reviewPocket.effective_occupancy)}`}</small>
              </div>
              {reviewPocket.findings.length > 0
                && <small>{reviewPocket.findings.map(sentenceCase).join(" · ")}</small>}
            </div>
            <label className="feeder-review-note">
              <span>Optional note</span>
              <input
                disabled={reviewBusy}
                maxLength={500}
                onChange={(event) => {
                  const note = event.target.value;
                  setReviewNote(note);
                  if (reviewPocketIndex === null) return;
                  setReviewEdits((current) => ({
                    ...current,
                    [reviewPocketIndex]: {
                      label: current[reviewPocketIndex]?.label
                        ?? currentReviewLabel ?? undefined,
                      note,
                    },
                  }));
                }}
                value={reviewNote}
              />
            </label>
            <div className="feeder-review-decisions">
              {(["occupied", "empty", "component_on_edge", "unusable"] as const)
                .map((label) => <button
                  aria-pressed={displayedReviewLabel === label}
                  className={pendingReviewLabel === label
                    ? "is-pending"
                    : currentReviewLabel === label ? "is-active" : "secondary"}
                  disabled={reviewBusy}
                  key={label}
                  onClick={() => stagePocketReview(label)}
                  type="button"
                >{reviewLabel(label)}</button>)}
            </div>
            <small className="feeder-review-help">
              Occupied and Empty update machine pocket state. On edge and Unusable keep the pocket unavailable.
            </small>
          </section>}
          {progress && <div className="notice" role="status">{progress}</div>}
          {displayedCapturedTiles !== null && displayedPlannedTiles !== null
            && <div className="workspace-selection-progress" role="status">
            <div className="workspace-progress-label">
              <strong>{rescanPercent}%</strong>
              <span>{displayedCapturedTiles} of {displayedPlannedTiles} tiles
                {rescanProgress && rescanEtaSeconds > 0
                  ? ` · about ${formatWorkspaceDuration(rescanEtaSeconds)} left`
                  : ""}</span>
            </div>
            <progress
              value={displayedCapturedTiles}
              max={Math.max(displayedPlannedTiles, 1)}
            />
          </div>}
          {rescanResult && <div className="notice">{rescanResult}</div>}
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
