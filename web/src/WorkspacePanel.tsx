import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { PointerEvent as ReactPointerEvent } from "react";
import { api } from "./api";
import type {
  BoardDesign, BoardInstance, Job, JobExecutionRun, MachineOverlayGeometry,
  MachineStatus, PlacementExecutionRun, PlacementOutcomeRecord, StripFeederMapRegistration,
  WorkspaceBoardDiscoveryReport, WorkspaceDisplayPatch, WorkspaceObjectClass,
  WorkspaceScanReport, WorkspaceScanSummary,
} from "./types";

type WorkspaceLayer =
  | "image" | "objects" | "boards" | "placements" | "feeders" | "machine";

const WORKSPACE_LAYERS: Array<{ id: WorkspaceLayer; label: string }> = [
  { id: "image", label: "Scan image" },
  { id: "objects", label: "Detected objects" },
  { id: "boards", label: "Job boards" },
  { id: "placements", label: "Placement positions" },
  { id: "feeders", label: "Feeder pockets" },
  { id: "machine", label: "Machine position" },
];

interface WorkspacePanelProps {
  status: MachineStatus;
  setError: (value: string | null) => void;
  openMachine: () => void;
  openJobs: () => void;
}

interface ArtifactOption {
  name: string;
  label: string;
}

interface MapPolygon {
  id: string;
  label: string;
  points: Array<[number, number]>;
}

interface WorkspaceComposition {
  bases: WorkspaceScanSummary[];
  overlays: WorkspaceScanSummary[];
}

interface ImageGeometry {
  x: number;
  y: number;
  width: number;
  height: number;
}

export interface WorkspaceViewport {
  x: number;
  y: number;
  width: number;
  height: number;
}

interface PlacementMarker {
  id: string;
  reference: string;
  point: [number, number];
  status: "populated" | "unpopulated";
  current: boolean;
  rotationDeg: number;
}

interface WorkspacePanGesture {
  pointerId: number;
  startClientX: number;
  startClientY: number;
  pixelsPerMachineUnit: number;
  startViewport: WorkspaceViewport;
}

const POPULATED_DISPOSITIONS = new Set([
  "good", "offset", "rotated", "tombstoned", "stacked",
]);

const ACTIVE_JOB_STATUSES = new Set<JobExecutionRun["status"]>([
  "pending", "running", "paused", "needs_attention",
]);

function label(value: string): string {
  return value.replaceAll(".", " ").replaceAll("_", " ")
    .replace(/\b\w/g, (character) => character.toUpperCase());
}

export function selectActiveJobExecution(
  executions: JobExecutionRun[],
): JobExecutionRun | null {
  return [...executions]
    .sort((left, right) => right.updated_at.localeCompare(left.updated_at))
    .find((execution) => ACTIVE_JOB_STATUSES.has(execution.status)) ?? null;
}

function sameBoardRegistration(
  left: BoardInstance,
  right: BoardInstance,
): boolean {
  if (left.board_id !== right.board_id) return false;
  const leftMatrix = left.design_to_machine?.matrix;
  const rightMatrix = right.design_to_machine?.matrix;
  if (!leftMatrix || !rightMatrix
      || leftMatrix.length !== 9 || rightMatrix.length !== 9) return false;
  const originDistance = Math.hypot(
    leftMatrix[2] - rightMatrix[2], leftMatrix[5] - rightMatrix[5],
  );
  const orientationDifference = Math.max(
    Math.abs(leftMatrix[0] - rightMatrix[0]),
    Math.abs(leftMatrix[1] - rightMatrix[1]),
    Math.abs(leftMatrix[3] - rightMatrix[3]),
    Math.abs(leftMatrix[4] - rightMatrix[4]),
  );
  return originDistance <= 5 && orientationDifference <= .05;
}

function jobsShareBoardRegistration(left: Job, right: Job): boolean {
  return left.boards.some((leftBoard) => right.boards.some(
    (rightBoard) => sameBoardRegistration(leftBoard, rightBoard),
  ));
}

export function selectWorkspaceJob(
  jobs: Job[],
  executions: JobExecutionRun[],
  outcomes: PlacementOutcomeRecord[],
): Job | null {
  const active = selectActiveJobExecution(executions);
  const activeJob = jobs.find((job) => job.id === active?.job_id);
  if (activeJob) return activeJob;
  const newestOutcome = [...outcomes]
    .sort((left, right) => right.updated_at.localeCompare(left.updated_at))
    .find((outcome) => jobs.some((job) => job.id === outcome.job_id));
  if (newestOutcome) {
    return jobs.find((job) => job.id === newestOutcome.job_id) ?? null;
  }
  const newestExecution = [...executions]
    .sort((left, right) => right.updated_at.localeCompare(left.updated_at))
    .find((execution) => jobs.some((job) => job.id === execution.job_id));
  return jobs.find((job) => job.id === newestExecution?.job_id) ?? null;
}

export function workspaceArtifactOptions(
  scan: WorkspaceScanReport,
): ArtifactOption[] {
  const options: ArtifactOption[] = [];
  const seen = new Set<string>();
  const add = (name: string | null | undefined, optionLabel: string) => {
    if (!name || seen.has(name)) return;
    seen.add(name);
    options.push({ name, label: optionLabel });
  };

  if (scan.refinement_composite?.accepted) {
    add(scan.refinement_composite.mosaic_artifact_name, "Full scan + refinements");
  }
  add(scan.height_map?.corrected_mosaic_artifact_name, "Height-corrected mosaic");
  add(scan.mosaic_artifact_name, "Primary mosaic");
  for (const layer of scan.mosaic_layers) {
    add(layer.artifact_name, `${label(layer.surface_model)} layer`);
  }
  add(scan.seam_health?.preview_artifact_name, "Seam health");
  add(scan.height_map?.class_preview_artifact_name, "Surface classes");
  add(scan.board_discovery?.preview_artifact_name, "Object analysis preview");
  return options;
}

function applyTransform(
  matrix: number[], xMm: number, yMm: number,
): [number, number] {
  return [
    matrix[0] * xMm + matrix[1] * yMm + matrix[2],
    matrix[3] * xMm + matrix[4] * yMm + matrix[5],
  ];
}

function jobBoardPolygons(job: Job | null, designs: BoardDesign[]): MapPolygon[] {
  if (!job) return [];
  return job.boards.flatMap((board) => {
    const design = designs.find((candidate) => candidate.id === board.board_id);
    const bounds = design?.outline_bounds;
    const matrix = board.design_to_machine?.matrix;
    if (!bounds || !matrix || matrix.length !== 9) return [];
    const points = [
      [bounds.min_x_mm, bounds.min_y_mm],
      [bounds.max_x_mm, bounds.min_y_mm],
      [bounds.max_x_mm, bounds.max_y_mm],
      [bounds.min_x_mm, bounds.max_y_mm],
    ].map(([xMm, yMm]) => applyTransform(matrix, xMm, yMm));
    return [{ id: board.id, label: board.label, points }];
  });
}

export function jobPlacementMarkers(
  job: Job | null,
  execution: JobExecutionRun | null,
  designs: BoardDesign[],
  jobs: Job[] = [],
  outcomes: PlacementOutcomeRecord[] = [],
): PlacementMarker[] {
  if (!job) return [];
  const compatibleJobIds = new Set(
    (jobs.length > 0 ? jobs : [job])
      .filter((candidate) => jobsShareBoardRegistration(job, candidate))
      .map((candidate) => candidate.id),
  );
  const populatedReferences = new Set(outcomes
    .filter((outcome) => compatibleJobIds.has(outcome.job_id)
      && POPULATED_DISPOSITIONS.has(
        outcome.operator_disposition ?? outcome.automatic_disposition,
      ))
    .map((outcome) => outcome.placement_reference));
  for (const step of execution?.steps ?? []) {
    if (step.status === "succeeded") {
      populatedReferences.add(step.placement_reference);
    }
  }
  return job.boards.flatMap((board) => {
    const matrix = board.design_to_machine?.matrix;
    const design = designs.find((candidate) => candidate.id === board.board_id);
    if (!matrix || matrix.length !== 9 || !design) return [];
    return design.placements
      .filter((placement) => placement.enabled && !placement.fiducial)
      .map((placement) => {
        const step = execution?.steps.find(
          (candidate) => candidate.placement_reference === placement.reference,
        );
        return {
          id: `${board.id}-${placement.reference}`,
          reference: placement.reference,
          point: applyTransform(matrix, placement.pose.x_mm, placement.pose.y_mm),
          status: populatedReferences.has(placement.reference)
            ? "populated" : "unpopulated",
          current: step?.step_index === execution?.current_step_index,
          rotationDeg: Math.atan2(matrix[3], matrix[0]) * 180 / Math.PI
            + placement.pose.rotation_deg,
        } satisfies PlacementMarker;
      });
  });
}

function candidateClass(candidate: {
  classification: { class: WorkspaceObjectClass } | null;
}): WorkspaceObjectClass {
  return candidate.classification?.class ?? "unknown";
}

function regionArea(scan: WorkspaceScanSummary): number {
  return (scan.region.x_max_mm - scan.region.x_min_mm)
    * (scan.region.y_max_mm - scan.region.y_min_mm);
}

function regionsOverlap(
  left: WorkspaceScanSummary["region"],
  right: WorkspaceScanSummary["region"],
): boolean {
  return left.x_min_mm < right.x_max_mm
    && left.x_max_mm > right.x_min_mm
    && left.y_min_mm < right.y_max_mm
    && left.y_max_mm > right.y_min_mm;
}

function imageGeometry(scan: Pick<
  WorkspaceScanSummary,
  "region" | "raster_transform" | "mosaic_width_px" | "mosaic_height_px"
>): ImageGeometry {
  if (scan.raster_transform
      && scan.mosaic_width_px !== null
      && scan.mosaic_height_px !== null) {
    return {
      x: scan.raster_transform.top_left_x_mm,
      y: -scan.raster_transform.top_left_y_mm,
      width: Math.abs(
        scan.mosaic_width_px * scan.raster_transform.x_mm_per_pixel,
      ),
      height: Math.abs(
        scan.mosaic_height_px * scan.raster_transform.y_mm_per_pixel,
      ),
    };
  }
  return {
    x: scan.region.x_min_mm,
    y: -scan.region.y_max_mm,
    width: scan.region.x_max_mm - scan.region.x_min_mm,
    height: scan.region.y_max_mm - scan.region.y_min_mm,
  };
}

function displayPatchGeometry(patch: WorkspaceDisplayPatch): ImageGeometry {
  return {
    x: patch.source_raster_transform.top_left_x_mm,
    y: -patch.source_raster_transform.top_left_y_mm,
    width: Math.abs(
      patch.source_width_px * patch.source_raster_transform.x_mm_per_pixel,
    ),
    height: Math.abs(
      patch.source_height_px * patch.source_raster_transform.y_mm_per_pixel,
    ),
  };
}

function displayPatchSvgTransform(patch: WorkspaceDisplayPatch): string {
  const matrix = patch.source_to_base.matrix;
  return `matrix(${matrix[0]} ${-matrix[3]} ${-matrix[1]} ${matrix[4]} ${matrix[2]} ${-matrix[5]})`;
}

export function workspaceComposition(
  scans: WorkspaceScanSummary[],
  profileId: string,
  selectedBaseId?: string,
): WorkspaceComposition {
  const usable = scans.filter((scan) =>
    scan.profile_id === profileId
      && scan.status === "succeeded"
      && scan.mode !== "refinement"
      && scan.display_artifact_name !== null
      && regionArea(scan) > 0
  );
  const maximumArea = Math.max(0, ...usable.map(regionArea));
  const bases = usable
    .filter((scan) => regionArea(scan) >= maximumArea * .5)
    .sort((left, right) => right.updated_at.localeCompare(left.updated_at));
  const base = bases.find((scan) => scan.id === selectedBaseId) ?? bases[0];
  if (!base) return { bases, overlays: [] };
  const nextBase = [...bases]
    .filter((scan) => scan.updated_at > base.updated_at)
    .sort((left, right) => left.updated_at.localeCompare(right.updated_at))[0];
  const overlays = usable
    .filter((scan) => scan.id !== base.id
      && scan.mode !== "reconnaissance"
      && scan.parent_scan_id === base.id
      && (scan.height_map_available
        || scan.refinement_composite?.accepted === true)
      && scan.updated_at > base.updated_at
      && (!nextBase || scan.updated_at < nextBase.created_at)
      && regionArea(scan) < maximumArea * .5
      && regionsOverlap(scan.region, base.region))
    .sort((left, right) => left.updated_at.localeCompare(right.updated_at));
  return { bases, overlays };
}

function stripLaneCoordinate(registration: StripFeederMapRegistration): number {
  const center = registration.pockets.reduce(
    (sum, pocket) => [
      sum[0] + pocket.center_machine_mm[0],
      sum[1] + pocket.center_machine_mm[1],
    ],
    [0, 0],
  ).map((value) => value / Math.max(registration.pockets.length, 1));
  const radians = registration.tape_rotation_deg * Math.PI / 180;
  return -Math.sin(radians) * center[0] + Math.cos(radians) * center[1];
}

function stripAngleDifference(left: number, right: number): number {
  const difference = Math.abs(left - right) % 180;
  return Math.min(difference, 180 - difference);
}

export function latestFeederLanes(
  registrations: StripFeederMapRegistration[],
  profileId: string,
  base: WorkspaceScanSummary | null,
): StripFeederMapRegistration[] {
  if (!base) return [];
  const selected: StripFeederMapRegistration[] = [];
  const candidates = registrations
    .filter((registration) => registration.profile_id === profileId
      && registration.updated_at >= base.updated_at
      && registration.pockets.some((pocket) => {
        const [xMm, yMm] = pocket.center_machine_mm;
        return xMm >= base.region.x_min_mm && xMm <= base.region.x_max_mm
          && yMm >= base.region.y_min_mm && yMm <= base.region.y_max_mm;
      }))
    .sort((left, right) => right.updated_at.localeCompare(left.updated_at));
  for (const candidate of candidates) {
    const sameLane = selected.some((current) =>
      stripAngleDifference(candidate.tape_rotation_deg, current.tape_rotation_deg) < 8
        && Math.abs(stripLaneCoordinate(candidate) - stripLaneCoordinate(current))
          < Math.max(2, Math.min(candidate.tape_width_mm, current.tape_width_mm) * .45)
    );
    if (!sameLane) selected.push(candidate);
  }
  return selected;
}

export function zoomWorkspaceViewport(
  current: WorkspaceViewport,
  full: WorkspaceViewport,
  focus: [number, number],
  scale: number,
): WorkspaceViewport {
  const width = Math.min(full.width, Math.max(full.width / 12, current.width * scale));
  const appliedScale = width / current.width;
  const height = Math.min(full.height, current.height * appliedScale);
  const x = focus[0] - (focus[0] - current.x) * appliedScale;
  const y = focus[1] - (focus[1] - current.y) * appliedScale;
  return {
    x: Math.min(full.x + full.width - width, Math.max(full.x, x)),
    y: Math.min(full.y + full.height - height, Math.max(full.y, y)),
    width,
    height,
  };
}

export function panWorkspaceViewport(
  current: WorkspaceViewport,
  full: WorkspaceViewport,
  deltaX: number,
  deltaY: number,
): WorkspaceViewport {
  return {
    ...current,
    x: Math.min(
      full.x + full.width - current.width,
      Math.max(full.x, current.x + deltaX),
    ),
    y: Math.min(
      full.y + full.height - current.height,
      Math.max(full.y, current.y + deltaY),
    ),
  };
}

export function WorkspacePanel({
  status, setError, openMachine, openJobs,
}: WorkspacePanelProps) {
  const [scanSummaries, setScanSummaries] = useState<WorkspaceScanSummary[]>([]);
  const [scanSummariesLoaded, setScanSummariesLoaded] = useState(false);
  const [selectedScan, setSelectedScan] = useState<WorkspaceScanReport | null>(null);
  const [displayPatches, setDisplayPatches] = useState<WorkspaceDisplayPatch[]>([]);
  const [jobs, setJobs] = useState<Job[]>([]);
  const [executions, setExecutions] = useState<JobExecutionRun[]>([]);
  const [placementOutcomes, setPlacementOutcomes] = useState<
    PlacementOutcomeRecord[]
  >([]);
  const [designs, setDesigns] = useState<BoardDesign[]>([]);
  const [feederRegistrations, setFeederRegistrations] = useState<
    StripFeederMapRegistration[]
  >([]);
  const [machineGeometry, setMachineGeometry] = useState<MachineOverlayGeometry | null>(null);
  const [placementExecution, setPlacementExecution] = useState<PlacementExecutionRun | null>(null);
  const [selectedScanId, setSelectedScanId] = useState("");
  const [artifactName, setArtifactName] = useState("");
  const [viewport, setViewport] = useState<WorkspaceViewport | null>(null);
  const [discovery, setDiscovery] = useState<WorkspaceBoardDiscoveryReport | null>(null);
  const [analyzing, setAnalyzing] = useState(false);
  const [panning, setPanning] = useState(false);
  const [layers, setLayers] = useState<Record<WorkspaceLayer, boolean>>({
    image: true,
    objects: true,
    boards: true,
    placements: true,
    feeders: true,
    machine: true,
  });
  const workspaceMapRef = useRef<SVGSVGElement>(null);
  const panGestureRef = useRef<WorkspacePanGesture | null>(null);

  const loadScanSummaries = useCallback(async () => {
    try {
      const scanValues = await api.workspaceScanSummaries();
      const orderedScans = [...scanValues]
        .sort((left, right) => right.updated_at.localeCompare(left.updated_at));
      const composition = workspaceComposition(orderedScans, status.profile_id);
      setScanSummaries(orderedScans);
      setSelectedScanId((current) =>
        composition.bases.some((scan) => scan.id === current)
          ? current
          : composition.bases[0]?.id ?? ""
      );
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setScanSummariesLoaded(true);
    }
  }, [setError, status.profile_id]);

  useEffect(() => {
    void loadScanSummaries();
    const timer = window.setInterval(() => void loadScanSummaries(), 1000);
    return () => window.clearInterval(timer);
  }, [loadScanSummaries]);

  const loadOperationalData = useCallback(async () => {
    try {
      const [jobValues, executionValues, designValues, outcomeValues]
        = await Promise.all([
          api.jobs(), api.jobExecutions(), api.boards(), api.placementOutcomes(),
      ]);
      setJobs(jobValues);
      setExecutions(executionValues);
      setDesigns(designValues);
      setPlacementOutcomes(outcomeValues);
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    }
  }, [setError]);

  useEffect(() => {
    void loadOperationalData();
    const timer = window.setInterval(() => void loadOperationalData(), 1000);
    return () => window.clearInterval(timer);
  }, [loadOperationalData]);

  const loadFeederData = useCallback(async () => {
    try {
      setFeederRegistrations(await api.stripFeederPocketMap());
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    }
  }, [setError]);

  useEffect(() => {
    void loadFeederData();
    const timer = window.setInterval(() => void loadFeederData(), 2000);
    return () => window.clearInterval(timer);
  }, [loadFeederData]);

  const composition = useMemo(
    () => workspaceComposition(scanSummaries, status.profile_id, selectedScanId),
    [scanSummaries, selectedScanId, status.profile_id],
  );
  const selectedSummary = composition.bases.find(
    (scan) => scan.id === selectedScanId,
  ) ?? null;
  useEffect(() => {
    setSelectedScan(null);
    if (!selectedScanId) return;
    let cancelled = false;
    void api.workspaceScan(selectedScanId).then((scan) => {
      if (!cancelled) setSelectedScan(scan);
    }).catch((reason) => {
      if (!cancelled) setError(reason instanceof Error ? reason.message : String(reason));
    });
    return () => { cancelled = true; };
  }, [selectedScanId, selectedSummary?.status, setError]);

  useEffect(() => {
    setMachineGeometry(null);
    if (!selectedScan?.calibration_id) return;
    let cancelled = false;
    void api.machineOverlayGeometry(selectedScan.calibration_id).then((geometry) => {
      if (!cancelled) setMachineGeometry(geometry);
    }).catch(() => {
      if (!cancelled) setMachineGeometry(null);
    });
    return () => { cancelled = true; };
  }, [selectedScan?.calibration_id]);

  useEffect(() => {
    setDisplayPatches([]);
    if (!selectedScanId || selectedSummary?.status !== "succeeded") return;
    let cancelled = false;
    void api.workspaceDisplayPatches(selectedScanId).then((patches) => {
      if (!cancelled) setDisplayPatches(patches.filter((patch) => patch.accepted));
    }).catch((reason) => {
      if (!cancelled) setError(reason instanceof Error ? reason.message : String(reason));
    });
    return () => { cancelled = true; };
  }, [selectedScanId, selectedSummary?.status, selectedSummary?.updated_at, setError]);

  const artifacts = useMemo(
    () => selectedScan ? workspaceArtifactOptions(selectedScan) : [],
    [selectedScan],
  );
  useEffect(() => {
    setArtifactName((current) =>
      artifacts.some((artifact) => artifact.name === current)
        ? current
        : artifacts[0]?.name ?? ""
    );
  }, [artifacts]);

  useEffect(() => {
    setDiscovery(null);
    if (!selectedScan?.board_discovery) return;
    let cancelled = false;
    void api.workspaceScanArtifact<WorkspaceBoardDiscoveryReport>(
      selectedScan.id, selectedScan.board_discovery.report_artifact_name,
    ).then((report) => {
      if (!cancelled) setDiscovery(report);
    }).catch((reason) => {
      if (!cancelled) setError(reason instanceof Error ? reason.message : String(reason));
    });
    return () => { cancelled = true; };
  }, [selectedScan?.id, selectedScan?.board_discovery?.report_artifact_name, setError]);

  const activeExecution = useMemo(
    () => selectActiveJobExecution(executions),
    [executions],
  );
  const activeJob = jobs.find((job) => job.id === activeExecution?.job_id) ?? null;
  const workspaceJob = useMemo(
    () => selectWorkspaceJob(jobs, executions, placementOutcomes),
    [executions, jobs, placementOutcomes],
  );
  const boardPolygons = useMemo(
    () => jobBoardPolygons(workspaceJob, designs),
    [designs, workspaceJob],
  );
  const placementMarkers = useMemo(
    () => jobPlacementMarkers(
      workspaceJob, activeExecution, designs, jobs, placementOutcomes,
    ),
    [activeExecution, designs, jobs, placementOutcomes, workspaceJob],
  );
  const feederLanes = useMemo(
    () => latestFeederLanes(
      feederRegistrations, status.profile_id, selectedSummary,
    ),
    [feederRegistrations, selectedSummary, status.profile_id],
  );
  const currentStep = activeExecution?.steps.find(
    (step) => step.step_index === activeExecution.current_step_index,
  ) ?? null;
  useEffect(() => {
    setPlacementExecution(null);
    const executionId = currentStep?.execution_id;
    if (!executionId) return;
    let cancelled = false;
    const load = () => {
      void api.placementExecution(executionId).then((execution) => {
        if (!cancelled) setPlacementExecution(execution);
      }).catch(() => {
        if (!cancelled) setPlacementExecution(null);
      });
    };
    load();
    const timer = window.setInterval(load, 1000);
    return () => {
      cancelled = true;
      window.clearInterval(timer);
    };
  }, [currentStep?.execution_id]);
  const completedSteps = activeExecution?.steps.filter(
    (step) => ["succeeded", "skipped"].includes(step.status),
  ).length ?? 0;

  const analyzeObjects = async () => {
    if (!selectedScan) return;
    setAnalyzing(true);
    setError(null);
    try {
      setDiscovery(await api.discoverWorkspaceObjects(selectedScan.id));
      const [scan] = await Promise.all([
        api.workspaceScan(selectedScan.id),
        loadScanSummaries(),
      ]);
      setSelectedScan(scan);
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setAnalyzing(false);
    }
  };

  const toggleLayer = (layer: WorkspaceLayer) => {
    setLayers((current) => ({ ...current, [layer]: !current[layer] }));
  };

  const region = selectedSummary?.region ?? selectedScan?.region ?? null;
  const mapWidth = region ? region.x_max_mm - region.x_min_mm : 1;
  const mapHeight = region ? region.y_max_mm - region.y_min_mm : 1;
  const markerSize = Math.max(mapWidth, mapHeight) * 0.009;
  const machineMarkerSize = Math.max(markerSize, 2.4);
  const placementMarkerRadius = Math.max(
    Math.max(mapWidth, mapHeight) * 0.0024,
    0.65,
  );
  const baseImageGeometry = selectedSummary
    ? imageGeometry(selectedSummary)
    : selectedScan ? imageGeometry(selectedScan) : null;
  const baseArtifactName = artifactName || selectedSummary?.display_artifact_name || "";
  const fullViewport = region ? {
    x: region.x_min_mm,
    y: -region.y_max_mm,
    width: mapWidth,
    height: mapHeight,
  } : null;
  const displayedViewport = viewport ?? fullViewport;
  useEffect(() => setViewport(null), [selectedSummary?.id]);
  const machineOnMap = Boolean(status.pose && region
    && status.pose.x_mm >= region.x_min_mm && status.pose.x_mm <= region.x_max_mm
    && status.pose.y_mm >= region.y_min_mm && status.pose.y_mm <= region.y_max_mm);
  const activeNozzleId = placementExecution?.report?.nozzle_id ?? null;
  const capturedTiles = selectedSummary?.captured_tiles ?? selectedScan?.captured_tiles ?? 0;
  const plannedTiles = selectedSummary?.planned_tiles ?? selectedScan?.planned_tiles ?? 0;
  const selectedStatus = selectedSummary?.status ?? selectedScan?.status ?? null;
  const scanProgress = plannedTiles > 0
    ? capturedTiles / plannedTiles
    : 0;
  const analysisAllowed = selectedStatus === "succeeded"
    && (selectedScan?.height_map !== null
      || selectedScan?.refinement_composite?.accepted === true);
  const zoomPercent = fullViewport && displayedViewport
    ? Math.round(fullViewport.width / displayedViewport.width * 100)
    : 100;
  const zoomAt = (focus: [number, number], scale: number) => {
    if (!fullViewport) return;
    setViewport((current) => zoomWorkspaceViewport(
      current ?? fullViewport, fullViewport, focus, scale,
    ));
  };
  const zoomAtCenter = (scale: number) => {
    if (!displayedViewport) return;
    zoomAt([
      displayedViewport.x + displayedViewport.width / 2,
      displayedViewport.y + displayedViewport.height / 2,
    ], scale);
  };
  useEffect(() => {
    const svg = workspaceMapRef.current;
    if (!svg || !displayedViewport || !fullViewport) return;
    const handleWheel = (event: WheelEvent) => {
      if (!event.shiftKey) return;
      event.preventDefault();
      const matrix = svg.getScreenCTM?.() ?? null;
      let focus: [number, number];
      if (matrix && typeof svg.createSVGPoint === "function") {
        const point = svg.createSVGPoint();
        point.x = event.clientX;
        point.y = event.clientY;
        const machinePoint = point.matrixTransform(matrix.inverse());
        focus = [machinePoint.x, machinePoint.y];
      } else {
        focus = [
          displayedViewport.x + displayedViewport.width / 2,
          displayedViewport.y + displayedViewport.height / 2,
        ];
      }
      setViewport((current) => zoomWorkspaceViewport(
        current ?? fullViewport, fullViewport, focus,
        Math.exp(
          (Math.abs(event.deltaY) >= Math.abs(event.deltaX)
            ? event.deltaY : event.deltaX) * .0015,
        ),
      ));
    };
    svg.addEventListener("wheel", handleWheel, { passive: false });
    return () => svg.removeEventListener("wheel", handleWheel);
  }, [
    displayedViewport?.height, displayedViewport?.width,
    displayedViewport?.x, displayedViewport?.y,
    fullViewport?.height, fullViewport?.width,
    fullViewport?.x, fullViewport?.y,
  ]);

  const startWorkspacePan = (event: ReactPointerEvent<SVGSVGElement>) => {
    if (!event.shiftKey || !displayedViewport || !fullViewport) return;
    const bounds = event.currentTarget.getBoundingClientRect();
    const pixelsPerMachineUnit = Math.min(
      bounds.width / displayedViewport.width,
      bounds.height / displayedViewport.height,
    );
    if (!Number.isFinite(pixelsPerMachineUnit) || pixelsPerMachineUnit <= 0) return;
    event.preventDefault();
    panGestureRef.current = {
      pointerId: event.pointerId,
      startClientX: event.clientX,
      startClientY: event.clientY,
      pixelsPerMachineUnit,
      startViewport: displayedViewport,
    };
    event.currentTarget.setPointerCapture?.(event.pointerId);
    setPanning(true);
  };

  const moveWorkspacePan = (event: ReactPointerEvent<SVGSVGElement>) => {
    const gesture = panGestureRef.current;
    if (!gesture || gesture.pointerId !== event.pointerId || !fullViewport) return;
    event.preventDefault();
    setViewport(panWorkspaceViewport(
      gesture.startViewport,
      fullViewport,
      -(event.clientX - gesture.startClientX) / gesture.pixelsPerMachineUnit,
      -(event.clientY - gesture.startClientY) / gesture.pixelsPerMachineUnit,
    ));
  };

  const stopWorkspacePan = (event: ReactPointerEvent<SVGSVGElement>) => {
    const gesture = panGestureRef.current;
    if (!gesture || gesture.pointerId !== event.pointerId) return;
    panGestureRef.current = null;
    if (event.currentTarget.hasPointerCapture?.(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
    setPanning(false);
  };

  return <div className="workspace-layout">
    <section className="workspace-map-card">
      <div className="workspace-map-heading">
        <div>
          <p className="eyebrow">Machine-coordinate view</p>
          <h2>{selectedSummary ? "Whole workspace" : "Workspace"}</h2>
        </div>
        <div className="workspace-map-badges">
          {displayPatches.length > 0 && <span className="map-status">
            Full scan + {displayPatches.length} aligned updates
          </span>}
          {selectedStatus && <span className={`map-status map-status-${selectedStatus}`}>
            {label(selectedStatus)}
          </span>}
          {activeExecution && <span className={`map-status map-status-${activeExecution.status}`}>
            Job {label(activeExecution.status)}
          </span>}
        </div>
      </div>

      <div className="workspace-toolbar">
        <select
          aria-label="Workspace revision"
          value={selectedScanId}
          onChange={(event) => setSelectedScanId(event.target.value)}
        >
          {!scanSummariesLoaded && <option value="">Loading workspace scans…</option>}
          {scanSummariesLoaded && composition.bases.length === 0
            && <option value="">No full workspace scans</option>}
          {composition.bases.map((scan) => <option key={scan.id} value={scan.id}>
            {new Date(scan.updated_at).toLocaleString()} · Full workspace
          </option>)}
        </select>
        <select
          aria-label="Workspace image layer"
          value={artifactName}
          disabled={artifacts.length === 0}
          onChange={(event) => setArtifactName(event.target.value)}
        >
          {artifacts.length === 0 && <option value="">No image yet</option>}
          {artifacts.map((artifact) => <option key={artifact.name} value={artifact.name}>
            {artifact.label}
          </option>)}
        </select>
        <div className="workspace-layer-toggles" aria-label="Map layers">
          {WORKSPACE_LAYERS.map((layer) =>
            <button
              key={layer.id}
              type="button"
              aria-pressed={layers[layer.id]}
              className={layers[layer.id] ? "layer-active" : ""}
              onClick={() => toggleLayer(layer.id)}
            >{layer.label}</button>
          )}
        </div>
      </div>

      <div className="workspace-map-frame">
        {region ? <svg
          ref={workspaceMapRef}
          className={`workspace-map ${panning ? "is-panning" : ""}`}
          viewBox={displayedViewport
            ? `${displayedViewport.x} ${displayedViewport.y} ${displayedViewport.width} ${displayedViewport.height}`
            : undefined}
          role="img"
          aria-label="Live machine workspace map"
          preserveAspectRatio="xMidYMid meet"
          onPointerDown={startWorkspacePan}
          onPointerMove={moveWorkspacePan}
          onPointerUp={stopWorkspacePan}
          onPointerCancel={stopWorkspacePan}
        >
          <title>Live machine workspace map</title>
          <defs>
            <linearGradient id="workspace-toolhead-shell" x1="0" y1="0" x2="1" y2="1">
              <stop offset="0" stopColor="#34403c" />
              <stop offset=".48" stopColor="#18211e" />
              <stop offset="1" stopColor="#0d1412" />
            </linearGradient>
            <linearGradient id="workspace-toolhead-motor" x1="0" y1="0" x2="0" y2="1">
              <stop offset="0" stopColor="#56635e" />
              <stop offset=".2" stopColor="#26312d" />
              <stop offset="1" stopColor="#111816" />
            </linearGradient>
            <radialGradient id="workspace-camera-lens">
              <stop offset="0" stopColor="#dcf7ff" />
              <stop offset=".22" stopColor="#6ad1ff" />
              <stop offset=".55" stopColor="#18394a" />
              <stop offset="1" stopColor="#071117" />
            </radialGradient>
            {displayPatches.map((patch) => <clipPath
              id={`workspace-patch-${patch.id}`}
              key={patch.id}
              clipPathUnits="userSpaceOnUse"
            >
              <polygon points={patch.clip_polygon_machine_mm.map(
                ([xMm, yMm]) => `${xMm},${-yMm}`,
              ).join(" ")} />
            </clipPath>)}
          </defs>
          <rect
            className="workspace-map-background"
            x={region.x_min_mm}
            y={-region.y_max_mm}
            width={mapWidth}
            height={mapHeight}
          />
          {layers.image && baseArtifactName && baseImageGeometry && selectedSummary && <image
            className="workspace-map-image"
            href={`/api/v1/workspace-scans/${selectedSummary.id}/artifacts/${encodeURIComponent(baseArtifactName)}`}
            x={baseImageGeometry.x}
            y={baseImageGeometry.y}
            width={baseImageGeometry.width}
            height={baseImageGeometry.height}
            preserveAspectRatio="none"
          />}
          {layers.image && displayPatches.map((patch) => {
            const geometry = displayPatchGeometry(patch);
            return <g
              className={`workspace-map-update patch-${patch.kind}`}
              key={patch.id}
              clipPath={`url(#workspace-patch-${patch.id})`}
            >
              <title>{`${patch.object_label}: ${patch.anchor_count} alignment anchors, ${patch.alignment_residuals.p95_mm.toFixed(3)} mm p95 residual`}</title>
              <image
                href={`/api/v1/workspace-scans/${patch.source_scan_id}/artifacts/${encodeURIComponent(patch.artifact_name)}`}
                x={geometry.x}
                y={geometry.y}
                width={geometry.width}
                height={geometry.height}
                transform={displayPatchSvgTransform(patch)}
                preserveAspectRatio="none"
              />
            </g>;
          })}
          {Array.from({ length: 5 }, (_, index) => index + 1).map((step) => <g key={step}>
            <line
              className="workspace-grid-line"
              x1={region.x_min_mm + mapWidth * step / 6}
              x2={region.x_min_mm + mapWidth * step / 6}
              y1={-region.y_max_mm}
              y2={-region.y_min_mm}
            />
            <line
              className="workspace-grid-line"
              x1={region.x_min_mm}
              x2={region.x_max_mm}
              y1={-(region.y_min_mm + mapHeight * step / 6)}
              y2={-(region.y_min_mm + mapHeight * step / 6)}
            />
          </g>)}

          {layers.objects && discovery?.candidates.map((candidate) =>
            <polygon
              key={candidate.candidate_index}
              className={`workspace-object object-${candidateClass(candidate)}`}
              points={candidate.outline_machine_mm.map(
                ([xMm, yMm]) => `${xMm},${-yMm}`,
              ).join(" ")}
            >
              <title>{`${label(candidateClass(candidate))} ${candidate.candidate_index + 1}`}</title>
            </polygon>
          )}

          {layers.boards && boardPolygons.map((board) => <g key={board.id}>
            <polygon
              className="workspace-job-board"
              points={board.points.map(([xMm, yMm]) => `${xMm},${-yMm}`).join(" ")}
            />
            <title>{board.label}</title>
          </g>)}
          {layers.placements && placementMarkers.map((placement) => <g
            className={`workspace-placement-position ${placement.status} ${placement.current ? "current" : ""}`}
            key={placement.id}
            transform={`translate(${placement.point[0]} ${-placement.point[1]}) rotate(${-placement.rotationDeg})`}
          >
            <circle r={placementMarkerRadius} />
            {placement.current && <>
              <line
                x1={-placementMarkerRadius}
                x2={placementMarkerRadius}
              />
              <line
                y1={-placementMarkerRadius}
                y2={placementMarkerRadius}
              />
            </>}
            <title>{`${placement.reference}: ${placement.status}`}</title>
          </g>)}

          {layers.feeders && feederLanes.map((feeder) => <g
            className="workspace-feeder-pockets"
            key={feeder.id}
          >
            <polyline points={feeder.pockets.map(
              (pocket) => `${pocket.center_machine_mm[0]},${-pocket.center_machine_mm[1]}`,
            ).join(" ")} />
            {feeder.pockets.map((pocket) => <circle
              key={pocket.pocket_index}
              className={`${pocket.occupancy} status-${pocket.status} ${pocket.selected ? "selected" : ""}`}
              cx={pocket.center_machine_mm[0]}
              cy={-pocket.center_machine_mm[1]}
              r={Math.max(markerSize * .38, .8)}
            >
              <title>{`${feeder.name} · pocket ${pocket.pocket_index} · ${pocket.occupancy} · ${label(pocket.status)}`}</title>
            </circle>)}
          </g>)}

          {layers.machine && status.pose && machineOnMap && <g
            className={`workspace-machine-overlay ${status.homed ? "is-trusted" : "is-untrusted"}`}
          >
            <g className="workspace-gantry-guide">
              <rect
                x={region.x_min_mm}
                y={-status.pose.y_mm - machineMarkerSize * .35}
                width={mapWidth}
                height={machineMarkerSize * .7}
              />
              <line
                x1={region.x_min_mm}
                x2={region.x_max_mm}
                y1={-status.pose.y_mm}
                y2={-status.pose.y_mm}
              />
              <title>{`Gantry Y position ${status.pose.y_mm.toFixed(3)} mm`}</title>
            </g>
            {machineGeometry && <rect
              className="workspace-camera-footprint"
              x={status.pose.x_mm
                + machineGeometry.camera_footprint_relative_mm.x_min_mm}
              y={-(status.pose.y_mm
                + machineGeometry.camera_footprint_relative_mm.y_max_mm)}
              width={machineGeometry.camera_footprint_relative_mm.x_max_mm
                - machineGeometry.camera_footprint_relative_mm.x_min_mm}
              height={machineGeometry.camera_footprint_relative_mm.y_max_mm
                - machineGeometry.camera_footprint_relative_mm.y_min_mm}
            >
              <title>Calibrated top-camera field of view</title>
            </rect>}
            <g
              className="workspace-toolhead"
              transform={`translate(${status.pose.x_mm} ${-status.pose.y_mm})`}
            >
              {machineGeometry && <g className="workspace-toolhead-model">
                <rect
                  className="workspace-toolhead-carriage"
                  x={-46}
                  y={-22}
                  width={92}
                  height={17}
                  rx={5}
                />
                <path
                  className="workspace-toolhead-shadow"
                  d="M-27-16 H27 Q35-16 38-7 L44 18 V61 Q44 76 32 85 L15 91 H-15 L-32 85 Q-44 76-44 61 V18 L-38-7 Q-35-16-27-16Z"
                  transform="translate(1.8 2.6)"
                />
                <path
                  className="workspace-toolhead-chassis"
                  d="M-27-16 H27 Q35-16 38-7 L44 18 V61 Q44 76 32 85 L15 91 H-15 L-32 85 Q-44 76-44 61 V18 L-38-7 Q-35-16-27-16Z"
                />
                <path
                  className="workspace-toolhead-panel"
                  d="M-25-7 H25 L33 20 V58 Q33 68 25 75 L11 81 H-11 L-25 75 Q-33 68-33 58 V20Z"
                />
                <rect
                  className="workspace-toolhead-spine"
                  x={-6.5}
                  y={15}
                  width={13}
                  height={46}
                  rx={5}
                />
                <circle className="workspace-toolhead-pulley" cx={0} cy={29} r={8} />
                <circle className="workspace-toolhead-pulley-hub" cx={0} cy={29} r={3.2} />
                {[-30, 30].flatMap((x) => [8, 52].map((y) => <circle
                  className="workspace-toolhead-fastener"
                  cx={x}
                  cy={y}
                  key={`${x}-${y}`}
                  r={2.2}
                />))}
                {machineGeometry.nozzles.map((nozzle) => <g
                  className="workspace-toolhead-motor"
                  key={`motor-${nozzle.nozzle_id}`}
                  transform={`translate(${nozzle.offset_machine_mm[0]} ${-nozzle.offset_machine_mm[1]})`}
                >
                  <rect x={-14} y={-17} width={28} height={34} rx={4} />
                  <path d="M-9-17 V17 M-3-17 V17 M3-17 V17 M9-17 V17" />
                  <circle className="motor-collar" r={7.2} />
                  <circle className="motor-shaft" r={3.2} />
                </g>)}
                <title>
                  Approximate LumenPnP V3 toolhead silhouette; calibrated markers are shown above it
                </title>
              </g>}
              <g
                className="workspace-top-camera"
              >
                <circle className="camera-shroud" r={machineMarkerSize * 1.85} />
                <circle className="camera-body" r={machineMarkerSize * .82} />
                <circle className="camera-lens" r={machineMarkerSize * .38} />
                <line x1={-machineMarkerSize * .22} x2={machineMarkerSize * .22} />
                <line y1={-machineMarkerSize * .22} y2={machineMarkerSize * .22} />
                <title>{`Top camera X${status.pose.x_mm.toFixed(3)} Y${status.pose.y_mm.toFixed(3)}`}</title>
              </g>
              {machineGeometry?.nozzles.map((nozzle) => <g
                className={`workspace-nozzle ${nozzle.nozzle_id === activeNozzleId ? "is-active" : ""}`}
                key={nozzle.nozzle_id}
                transform={`translate(${nozzle.offset_machine_mm[0]} ${-nozzle.offset_machine_mm[1]})`}
              >
                <circle className="nozzle-body" r={machineMarkerSize * .64} />
                <circle className="nozzle-tip" r={machineMarkerSize * .2} />
                <text x={0} y={machineMarkerSize * 2.75} textAnchor="middle">
                  {nozzle.nozzle_id}
                </text>
                <title>{`${nozzle.nozzle_id} calibrated axis center${nozzle.nozzle_id === activeNozzleId ? " · active" : ""}`}</title>
              </g>)}
            </g>
          </g>}
          <rect
            className="workspace-region-outline"
            x={region.x_min_mm}
            y={-region.y_max_mm}
            width={mapWidth}
            height={mapHeight}
          />
        </svg> : <div className="workspace-map-empty">
          <strong>{selectedScanId ? "Loading workspace map…" : "No full workspace scan is available"}</strong>
          <span>{selectedScanId
            ? "Loading the full scan and its local updates."
            : "Run a scan that covers the installed workspace."}</span>
        </div>}
        {region && displayedViewport && <div className="workspace-zoom-controls">
          <button type="button" aria-label="Zoom out" onClick={() => zoomAtCenter(1.25)}>−</button>
          <span>{zoomPercent}%</span>
          <button type="button" aria-label="Zoom in" onClick={() => zoomAtCenter(.8)}>+</button>
          <button type="button" onClick={() => setViewport(null)}>Reset</button>
          <small>Shift + scroll to zoom · Shift + drag to pan</small>
        </div>}
        {selectedStatus === "running" && <div className="workspace-live-label">
          <span /> Live scan · {capturedTiles}/{plannedTiles} tiles
        </div>}
        {status.pose && region && !machineOnMap && <div className="workspace-map-note">
          Head position is outside this scan region.
        </div>}
      </div>

      <div className="workspace-legend">
        <div className="workspace-legend-group">
          <strong>Objects</strong>
          <span className="legend-feeder">Feeder</span>
          <span className="legend-board">Board</span>
          <span className="legend-fiducial">Machine fiducial</span>
          <span className="legend-unknown">Unknown</span>
        </div>
        <div className="workspace-legend-group">
          <strong>Position state</strong>
          <span className="legend-populated">Occupied / populated</span>
          <span className="legend-empty">Empty / pending</span>
          <span className="legend-uncertain">Uncertain / review</span>
        </div>
      </div>
    </section>

    <aside className="workspace-rail">
      <section className="card workspace-state-card">
        <div className="card-heading">
          <div><p className="eyebrow">Live machine</p><h2>Operational state</h2></div>
          <span className={`state-dot state-${status.state}`} />
        </div>
        <dl className="workspace-facts">
          <div><dt>State</dt><dd>{label(status.state)}</dd></div>
          <div><dt>Homed</dt><dd>{status.homed ? "Yes" : "No"}</dd></div>
          <div><dt>Fine home</dt><dd>{status.calibration_revision ? "Current" : "Required"}</dd></div>
          <div><dt>Mode</dt><dd>{status.simulated ? "Simulation" : "Physical"}</dd></div>
          {status.pose && <>
            <div><dt>X / Y</dt><dd>{status.pose.x_mm.toFixed(2)} / {status.pose.y_mm.toFixed(2)} mm</dd></div>
            <div><dt>Z</dt><dd>{status.pose.z_mm.toFixed(2)} mm</dd></div>
          </>}
        </dl>
        {(!status.homed || !status.calibration_revision) && <button onClick={openMachine}>
          Open machine setup
        </button>}
      </section>

      <section className={`card workspace-job-card ${activeExecution ? "has-active-job" : ""}`}>
        <div className="card-heading">
          <div><p className="eyebrow">Production</p><h2>{activeJob?.name ?? "No active job"}</h2></div>
          {activeExecution && <span className={`badge state-${activeExecution.status}`}>
            {label(activeExecution.status)}
          </span>}
        </div>
        {activeExecution ? <>
          <div className="workspace-progress-label">
            <strong>{completedSteps}/{activeExecution.steps.length}</strong>
            <span>placements complete</span>
          </div>
          <progress value={completedSteps} max={Math.max(activeExecution.steps.length, 1)} />
          {currentStep && <div className="workspace-current-step">
            <small>Current placement</small>
            <strong>{currentStep.placement_reference}</strong>
            <span>{label(currentStep.status)}</span>
          </div>}
          {activeExecution.message && <p>{activeExecution.message}</p>}
        </> : <p>Start a job when the machine and workspace are ready.</p>}
        <button className="secondary" onClick={openJobs}>
          {activeExecution ? "Open job controls" : "Open jobs"}
        </button>
      </section>

      <section className="card workspace-scan-card">
        <div className="card-heading">
          <h2>Workspace scan</h2>
        </div>
        {selectedScan ? <>
          <div className="workspace-progress-label">
            <strong>{Math.round(scanProgress * 100)}%</strong>
            <span>{capturedTiles}/{plannedTiles} tiles</span>
          </div>
          <progress value={capturedTiles} max={Math.max(plannedTiles, 1)} />
          <dl className="workspace-facts compact">
            <div><dt>Aligned updates</dt><dd>{displayPatches.length}</dd></div>
            <div><dt>Coverage</dt><dd>{selectedScan.covered_fraction === null ? "Pending" : `${(selectedScan.covered_fraction * 100).toFixed(1)}%`}</dd></div>
          </dl>
          {selectedScan.failure && <div className="notice">{selectedScan.failure}</div>}
          <button
            disabled={!analysisAllowed || analyzing}
            onClick={() => void analyzeObjects()}
          >
            {analyzing ? "Analyzing…" : discovery ? "Refresh object analysis" : "Analyze workspace objects"}
          </button>
        </> : <p>{selectedScanId
          ? "The map is ready. Loading detailed scan evidence…"
          : "No full-workspace evidence is stored."}</p>}
      </section>
    </aside>
  </div>;
}
