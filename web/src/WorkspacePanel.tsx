import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type {
  KeyboardEvent as ReactKeyboardEvent, MouseEvent as ReactMouseEvent,
  PointerEvent as ReactPointerEvent,
} from "react";
import { api, waitForOperation } from "./api";
import { useDialog } from "./Dialog";
import type {
  BoardDesign, BoardInstance, Job, JobExecutionRun, MachineOverlayGeometry,
  MachineStatus, PlacementExecutionRun, PlacementOutcomeRecord, Pose,
  StripFeederMapRegistration,
  WorkspaceBoardDiscoveryReport, WorkspaceDisplayPatch, WorkspaceObjectClass,
  WorkspaceScanRegion, WorkspaceScanReport, WorkspaceScanSummary,
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
  boardId: string;
  boardLabel: string;
  boardPoints: Array<[number, number]>;
  status: "populated" | "unpopulated";
  current: boolean;
  rotationDeg: number;
}

type WorkspaceSelectionKind =
  | "feeder" | "board" | "pocket" | "placement" | "point";

interface WorkspaceSelection {
  kind: WorkspaceSelectionKind;
  id: string;
  label: string;
  point: [number, number];
  scanRegion: WorkspaceScanRegion | null;
  objectId: string | null;
}

interface WorkspaceRescanProgress {
  capturedTiles: number;
  plannedTiles: number;
  estimatedDurationSeconds: number;
  startedAtMs: number;
  status: WorkspaceScanReport["status"];
}

interface WorkspacePanGesture {
  pointerId: number;
  startClientX: number;
  startClientY: number;
  pixelsPerMachineUnit: number;
  startViewport: WorkspaceViewport;
}

type WorkspaceActivateEvent =
  | ReactMouseEvent<SVGElement>
  | ReactKeyboardEvent<SVGElement>;

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

function polygonCenter(points: Array<[number, number]>): [number, number] {
  return points.reduce<[number, number]>(
    (sum, point) => [sum[0] + point[0], sum[1] + point[1]],
    [0, 0],
  ).map((value) => value / Math.max(points.length, 1)) as [number, number];
}

function paddedRegion(
  points: Array<[number, number]>,
  parent: WorkspaceScanRegion,
  paddingMm: number,
): WorkspaceScanRegion {
  const xValues = points.map((point) => point[0]);
  const yValues = points.map((point) => point[1]);
  return {
    x_min_mm: Math.max(parent.x_min_mm, Math.min(...xValues) - paddingMm),
    x_max_mm: Math.min(parent.x_max_mm, Math.max(...xValues) + paddingMm),
    y_min_mm: Math.max(parent.y_min_mm, Math.min(...yValues) - paddingMm),
    y_max_mm: Math.min(parent.y_max_mm, Math.max(...yValues) + paddingMm),
  };
}

export function feederScanRegion(
  feeder: StripFeederMapRegistration,
  parent: WorkspaceScanRegion,
): WorkspaceScanRegion {
  const halfWidth = feeder.tape_width_mm / 2;
  return paddedRegion(
    feeder.pockets.map((pocket) => pocket.center_machine_mm),
    parent,
    halfWidth + 6,
  );
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
    const bounds = design.outline_bounds;
    if (!bounds) return [];
    const boardPoints = [
      [bounds.min_x_mm, bounds.min_y_mm],
      [bounds.max_x_mm, bounds.min_y_mm],
      [bounds.max_x_mm, bounds.max_y_mm],
      [bounds.min_x_mm, bounds.max_y_mm],
    ].map(([xMm, yMm]) => applyTransform(matrix, xMm, yMm));
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
          boardId: board.id,
          boardLabel: board.label,
          boardPoints,
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

export function workspaceMoveAnimationDuration(
  from: Pose,
  to: Pose,
  feedMmPerMin: number,
): number {
  const distanceMm = Math.hypot(
    to.x_mm - from.x_mm,
    to.y_mm - from.y_mm,
    to.z_mm - from.z_mm,
  );
  const durationMs = distanceMm / Math.max(feedMmPerMin / 60, 1) * 1000;
  return Math.min(1200, Math.max(100, durationMs));
}

export function formatWorkspaceDuration(seconds: number): string {
  const roundedSeconds = Math.max(1, Math.round(seconds));
  if (roundedSeconds < 60) return `${roundedSeconds} sec`;
  const minutes = Math.floor(roundedSeconds / 60);
  const remainder = roundedSeconds % 60;
  return remainder === 0
    ? `${minutes} min`
    : `${minutes} min ${remainder} sec`;
}

export function workspaceRescanEtaSeconds(
  capturedTiles: number,
  plannedTiles: number,
  estimatedDurationSeconds: number,
  elapsedSeconds: number,
): number {
  const plannedEstimate = Math.max(estimatedDurationSeconds, 1);
  const tileProgress = plannedTiles > 0
    ? Math.min(1, Math.max(0, capturedTiles / plannedTiles))
    : 0;
  const observedEstimate = tileProgress > 0
    ? elapsedSeconds / tileProgress
    : plannedEstimate;
  return Math.max(0, Math.ceil(
    Math.max(plannedEstimate, observedEstimate) - elapsedSeconds,
  ));
}

export function WorkspacePanel({
  status, setError, openMachine, openJobs,
}: WorkspacePanelProps) {
  const requestDialog = useDialog();
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
  const [clearXyFeedMmPerMin, setClearXyFeedMmPerMin] = useState(23_000);
  const [mapPose, setMapPose] = useState<Pose | null>(status.pose);
  const [placementExecution, setPlacementExecution] = useState<PlacementExecutionRun | null>(null);
  const [selectedScanId, setSelectedScanId] = useState("");
  const [artifactName, setArtifactName] = useState("");
  const [viewport, setViewport] = useState<WorkspaceViewport | null>(null);
  const [discovery, setDiscovery] = useState<WorkspaceBoardDiscoveryReport | null>(null);
  const [analyzing, setAnalyzing] = useState(false);
  const [panning, setPanning] = useState(false);
  const [selection, setSelection] = useState<WorkspaceSelection | null>(null);
  const [selectionAction, setSelectionAction] = useState<
    "moving" | "estimating" | "scanning" | "capturing" | null
  >(null);
  const [selectionMessage, setSelectionMessage] = useState("");
  const [quickCaptureUrl, setQuickCaptureUrl] = useState("");
  const [rescanProgress, setRescanProgress] = useState<WorkspaceRescanProgress | null>(null);
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
  const mapPoseRef = useRef<Pose | null>(status.pose);
  const mapPoseAnimationRef = useRef<number | null>(null);

  const animateMapPose = useCallback((target: Pose | null) => {
    if (mapPoseAnimationRef.current !== null
        && typeof window.cancelAnimationFrame === "function") {
      window.cancelAnimationFrame(mapPoseAnimationRef.current);
    }
    mapPoseAnimationRef.current = null;
    if (!target) {
      mapPoseRef.current = null;
      setMapPose(null);
      return;
    }
    const source = mapPoseRef.current ?? target;
    if (typeof window.requestAnimationFrame !== "function") {
      mapPoseRef.current = target;
      setMapPose(target);
      return;
    }
    const durationMs = workspaceMoveAnimationDuration(
      source, target, clearXyFeedMmPerMin,
    );
    let startedAt: number | null = null;
    const frame = (now: number) => {
      startedAt ??= now;
      const progress = Math.min(1, (now - startedAt) / durationMs);
      const eased = 1 - (1 - progress) ** 3;
      const next = {
        x_mm: source.x_mm + (target.x_mm - source.x_mm) * eased,
        y_mm: source.y_mm + (target.y_mm - source.y_mm) * eased,
        z_mm: source.z_mm + (target.z_mm - source.z_mm) * eased,
        rotation_deg: source.rotation_deg
          + (target.rotation_deg - source.rotation_deg) * eased,
      };
      mapPoseRef.current = next;
      setMapPose(next);
      if (progress < 1) {
        mapPoseAnimationRef.current = window.requestAnimationFrame(frame);
      } else {
        mapPoseAnimationRef.current = null;
      }
    };
    mapPoseAnimationRef.current = window.requestAnimationFrame(frame);
  }, [clearXyFeedMmPerMin]);

  useEffect(() => {
    let cancelled = false;
    void api.machineProfile().then((profile) => {
      if (!cancelled) {
        setClearXyFeedMmPerMin(profile.transit_motion.clear_xy_feed_mm_per_min);
      }
    }).catch(() => undefined);
    return () => { cancelled = true; };
  }, [status.profile_id]);

  useEffect(() => {
    animateMapPose(status.pose);
  }, [
    animateMapPose, status.pose?.rotation_deg, status.pose?.x_mm,
    status.pose?.y_mm, status.pose?.z_mm,
  ]);

  useEffect(() => () => {
    if (mapPoseAnimationRef.current !== null
        && typeof window.cancelAnimationFrame === "function") {
      window.cancelAnimationFrame(mapPoseAnimationRef.current);
    }
  }, []);

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

  const loadDisplayPatches = useCallback(async () => {
    if (!selectedScanId || selectedSummary?.status !== "succeeded") {
      setDisplayPatches([]);
      return [];
    }
    const patches = await api.workspaceDisplayPatches(selectedScanId);
    const accepted = patches.filter((patch) => patch.accepted);
    setDisplayPatches(accepted);
    return accepted;
  }, [selectedScanId, selectedSummary?.status]);

  useEffect(() => {
    let cancelled = false;
    void loadDisplayPatches().catch((reason) => {
      if (!cancelled) setError(reason instanceof Error ? reason.message : String(reason));
    });
    return () => { cancelled = true; };
  }, [loadDisplayPatches, selectedSummary?.updated_at, setError]);

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
  const machineOnMap = Boolean(mapPose && region
    && mapPose.x_mm >= region.x_min_mm && mapPose.x_mm <= region.x_max_mm
    && mapPose.y_mm >= region.y_min_mm && mapPose.y_mm <= region.y_max_mm);
  const activeNozzleId = placementExecution?.report?.nozzle_id ?? null;
  const capturedTiles = selectedSummary?.captured_tiles ?? selectedScan?.captured_tiles ?? 0;
  const plannedTiles = selectedSummary?.planned_tiles ?? selectedScan?.planned_tiles ?? 0;
  const selectedStatus = selectedSummary?.status ?? selectedScan?.status ?? null;
  const selectionActionAllowed = status.state === "ready"
    && status.homed
    && status.pose !== null
    && status.calibration_revision === selectedScan?.calibration_id
    && selectionAction === null;
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
    setSelection(null);
    setSelectionMessage("");
    setQuickCaptureUrl("");
    setRescanProgress(null);
  }, [selectedSummary?.id]);

  useEffect(() => () => {
    if (quickCaptureUrl.startsWith("blob:")
        && typeof URL.revokeObjectURL === "function") {
      URL.revokeObjectURL(quickCaptureUrl);
    }
  }, [quickCaptureUrl]);

  const selectTarget = (target: WorkspaceSelection) => {
    if (selectionAction !== null) return;
    setSelection(target);
    setSelectionMessage("");
    setQuickCaptureUrl("");
    setRescanProgress(null);
  };

  const selectBoard = (
    event: WorkspaceActivateEvent,
    board: MapPolygon,
  ) => {
    event.stopPropagation();
    if (!region) return;
    selectTarget({
      kind: "board",
      id: board.id,
      label: board.label,
      point: polygonCenter(board.points),
      scanRegion: paddedRegion(board.points, region, 6),
      objectId: board.id,
    });
  };

  const selectPlacement = (
    event: WorkspaceActivateEvent,
    placement: PlacementMarker,
  ) => {
    event.stopPropagation();
    if (!region) return;
    selectTarget({
      kind: "placement",
      id: placement.id,
      label: `${placement.reference} on ${placement.boardLabel}`,
      point: placement.point,
      scanRegion: paddedRegion(placement.boardPoints, region, 6),
      objectId: placement.boardId,
    });
  };

  const selectFeeder = (
    event: WorkspaceActivateEvent,
    feeder: StripFeederMapRegistration,
  ) => {
    event.stopPropagation();
    if (!region) return;
    selectTarget({
      kind: "feeder",
      id: feeder.id,
      label: feeder.name,
      point: polygonCenter(feeder.pockets.map((pocket) => pocket.center_machine_mm)),
      scanRegion: feederScanRegion(feeder, region),
      objectId: feeder.id,
    });
  };

  const selectPocket = (
    event: WorkspaceActivateEvent,
    feeder: StripFeederMapRegistration,
    pocket: StripFeederMapRegistration["pockets"][number],
  ) => {
    event.stopPropagation();
    if (!region) return;
    selectTarget({
      kind: "pocket",
      id: `${feeder.id}-${pocket.pocket_index}`,
      label: `${feeder.name} · pocket ${pocket.pocket_index}`,
      point: pocket.center_machine_mm,
      scanRegion: feederScanRegion(feeder, region),
      objectId: feeder.id,
    });
  };

  const activateOnKeyboard = (
    event: ReactKeyboardEvent<SVGElement>,
    activate: () => void,
  ) => {
    if (event.key !== "Enter" && event.key !== " ") return;
    event.preventDefault();
    event.stopPropagation();
    activate();
  };

  const selectArbitraryPoint = (event: ReactMouseEvent<SVGSVGElement>) => {
    if (event.shiftKey || panning) return;
    const svg = event.currentTarget;
    const matrix = svg.getScreenCTM?.() ?? null;
    if (!matrix || typeof svg.createSVGPoint !== "function") return;
    const point = svg.createSVGPoint();
    point.x = event.clientX;
    point.y = event.clientY;
    const mapPoint = point.matrixTransform(matrix.inverse());
    selectTarget({
      kind: "point",
      id: `point-${mapPoint.x.toFixed(3)}-${mapPoint.y.toFixed(3)}`,
      label: "Workspace point",
      point: [mapPoint.x, -mapPoint.y],
      scanRegion: null,
      objectId: null,
    });
  };

  const moveTopCamera = async (target: WorkspaceSelection): Promise<string> => {
    if (!status.pose) throw new Error("The machine pose is not known.");
    const lease = await api.acquireOrRenewLease("VelocityPnP workspace", 3600);
    const targetPose = {
      x_mm: target.point[0],
      y_mm: target.point[1],
      z_mm: status.pose.z_mm,
      rotation_deg: status.pose.rotation_deg,
    };
    animateMapPose(targetPose);
    const operation = await api.jog(
      lease.id, targetPose, clearXyFeedMmPerMin,
    );
    const completed = await waitForOperation(operation);
    if (completed.status !== "succeeded") {
      throw new Error(completed.message ?? "The move did not complete.");
    }
    return lease.id;
  };

  const goToSelection = async () => {
    if (!selection || !status.pose) return;
    setSelectionAction("moving");
    setSelectionMessage("Moving top camera…");
    setError(null);
    try {
      await moveTopCamera(selection);
      setSelectionMessage("Top camera is at the selected position.");
    } catch (reason) {
      const message = reason instanceof Error ? reason.message : String(reason);
      setSelectionMessage("");
      setError(message);
    } finally {
      setSelectionAction(null);
    }
  };

  const rescanSelection = async () => {
    if (!selection?.scanRegion || !selectedScan) return;
    const rescanScope = selection.kind === "placement"
      ? "the full PCB around this placement"
      : selection.kind === "pocket"
        ? "the feeder strip around this pocket"
        : selection.kind === "board" ? "this PCB" : "this feeder strip";
    setSelectionAction("estimating");
    setSelectionMessage("Estimating rescan…");
    setQuickCaptureUrl("");
    setRescanProgress(null);
    setError(null);
    let progressTimer: number | null = null;
    try {
      const estimate = await api.estimateWorkspaceSurvey(
        selectedScan.id,
        selectedScan.calibration_id,
        selection.scanRegion,
      );
      setSelectionAction(null);
      setSelectionMessage("");
      const confirmed = await requestDialog({
        title: `Rescan ${selection.label}`,
        message: `Scan ${rescanScope} and align the result to the current full workspace scan. Estimated completion time is about ${formatWorkspaceDuration(estimate.estimated_duration_seconds)} for ${estimate.planned_tiles} tiles. Confirm that the scan region and travel path are clear, both nozzles are level, and power isolation is reachable.`,
        confirmLabel: "Start rescan",
      });
      if (confirmed === null) return;
      setSelectionAction("scanning");
      setSelectionMessage("Preparing rescan…");
      const lease = await api.acquireOrRenewLease("VelocityPnP workspace", 3600);
      const started = await api.startWorkspaceSurvey(
        selectedScan.id,
        selectedScan.calibration_id,
        selection.scanRegion,
        lease.id,
      );
      const startedAtMs = Date.now();
      setRescanProgress({
        capturedTiles: started.scan.captured_tiles,
        plannedTiles: started.scan.planned_tiles,
        estimatedDurationSeconds: started.scan.estimated_duration_seconds,
        startedAtMs,
        status: started.scan.status,
      });
      const refreshProgress = async () => {
        try {
          const scan = await api.workspaceScan(started.scan.id);
          setRescanProgress({
            capturedTiles: scan.captured_tiles,
            plannedTiles: scan.planned_tiles,
            estimatedDurationSeconds: scan.estimated_duration_seconds,
            startedAtMs,
            status: scan.status,
          });
        } catch {
          // Keep the last known progress while the operation poll remains active.
        }
      };
      progressTimer = window.setInterval(() => void refreshProgress(), 500);
      setSelectionMessage("Scanning selected area…");
      const completed = await waitForOperation(started.operation, (operation) => {
        setSelectionMessage(operation.message ?? "Scanning selected area…");
      });
      if (completed.status !== "succeeded") {
        throw new Error(completed.message ?? "The rescan did not complete.");
      }
      await refreshProgress();
      setRescanProgress((current) => current ? {
        ...current,
        capturedTiles: current.plannedTiles,
        status: "succeeded",
      } : current);
      if (selection.kind === "feeder" || selection.kind === "pocket") {
        setSelectionMessage("Checking feeder alignment…");
        const discoveryReport = await api.discoverWorkspaceObjects(started.scan.id);
        const candidates = discoveryReport.candidates.filter((candidate) => {
          const classification = candidate.classification;
          return classification?.class === "feeder"
            || (classification?.class === "unknown"
              && classification.evidence.some((evidence) =>
                evidence.suggested_class === "feeder" && evidence.confidence >= .5));
        }).sort((left, right) => Math.hypot(
          left.center_x_mm - selection.point[0],
          left.center_y_mm - selection.point[1],
        ) - Math.hypot(
          right.center_x_mm - selection.point[0],
          right.center_y_mm - selection.point[1],
        ));
        if (candidates[0]) {
          await api.analyzeStripFeeder(started.scan.id, candidates[0].candidate_index);
        }
      }
      setSelectionMessage("Checking alignment…");
      const patches = await loadDisplayPatches();
      const accepted = patches.some((patch) =>
        patch.source_scan_id === started.scan.id
          && patch.object_id === selection.objectId);
      setSelectionMessage(accepted
        ? "Rescan complete. The aligned image is now shown."
        : "Rescan complete. Alignment checks did not accept an image update.");
      await loadScanSummaries();
    } catch (reason) {
      const message = reason instanceof Error ? reason.message : String(reason);
      setSelectionMessage("");
      setError(message);
    } finally {
      if (progressTimer !== null) window.clearInterval(progressTimer);
      setSelectionAction(null);
    }
  };

  const quickCapture = async () => {
    if (!selection || !["placement", "pocket", "point"].includes(selection.kind)) return;
    setSelectionAction("capturing");
    setSelectionMessage("Moving top camera…");
    setQuickCaptureUrl("");
    setError(null);
    try {
      const leaseId = await moveTopCamera(selection);
      setSelectionMessage("Capturing top-camera image…");
      let lightsRequested = false;
      try {
        lightsRequested = true;
        await waitForOperation(await api.actuate(leaseId, "lights", true));
        const image = await api.cameraSnapshot("top_camera");
        if (typeof URL.createObjectURL !== "function") {
          throw new Error("This browser cannot display the captured image.");
        }
        setQuickCaptureUrl(URL.createObjectURL(image));
      } finally {
        if (lightsRequested) {
          await waitForOperation(await api.actuate(leaseId, "lights", false));
        }
      }
      setSelectionMessage("Quick capture complete.");
    } catch (reason) {
      const message = reason instanceof Error ? reason.message : String(reason);
      setSelectionMessage("");
      setError(message);
    } finally {
      setSelectionAction(null);
    }
  };
  const rescanPercent = rescanProgress?.plannedTiles
    ? Math.round(rescanProgress.capturedTiles / rescanProgress.plannedTiles * 100)
    : 0;
  const rescanEtaSeconds = rescanProgress
    ? workspaceRescanEtaSeconds(
      rescanProgress.capturedTiles,
      rescanProgress.plannedTiles,
      rescanProgress.estimatedDurationSeconds,
      (Date.now() - rescanProgress.startedAtMs) / 1000,
    )
    : 0;
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
          onClick={selectArbitraryPoint}
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
            <linearGradient id="workspace-gantry-aluminum" x1="0" y1="0" x2="0" y2="1">
              <stop offset="0" stopColor="#b3c0ba" />
              <stop offset=".08" stopColor="#38433f" />
              <stop offset=".38" stopColor="#65716c" />
              <stop offset=".5" stopColor="#111816" />
              <stop offset=".62" stopColor="#65716c" />
              <stop offset=".92" stopColor="#38433f" />
              <stop offset="1" stopColor="#b3c0ba" />
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

          {layers.boards && boardPolygons.map((board) => <g
            className={selection?.kind === "board" && selection.id === board.id
              ? "workspace-selectable is-selected" : "workspace-selectable"}
            key={board.id}
            onClick={(event) => selectBoard(event, board)}
            onKeyDown={(event) => activateOnKeyboard(
              event, () => selectBoard(event, board),
            )}
            role="button"
            tabIndex={0}
          >
            <polygon
              className="workspace-job-board"
              points={board.points.map(([xMm, yMm]) => `${xMm},${-yMm}`).join(" ")}
            />
            <title>{board.label}</title>
          </g>)}
          {layers.placements && placementMarkers.map((placement) => <g
            className={`workspace-selectable workspace-placement-position ${placement.status} ${placement.current ? "current" : ""} ${selection?.kind === "placement" && selection.id === placement.id ? "is-selected" : ""}`}
            key={placement.id}
            onClick={(event) => selectPlacement(event, placement)}
            onKeyDown={(event) => activateOnKeyboard(
              event,
              () => selectPlacement(event, placement),
            )}
            role="button"
            tabIndex={0}
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
            className={`workspace-selectable workspace-feeder-pockets ${selection?.kind === "feeder" && selection.id === feeder.id ? "is-selected" : ""}`}
            key={feeder.id}
            onClick={(event) => selectFeeder(event, feeder)}
            onKeyDown={(event) => activateOnKeyboard(
              event,
              () => selectFeeder(event, feeder),
            )}
            role="button"
            tabIndex={0}
          >
            <polyline
              className="workspace-feeder-hit"
              points={feeder.pockets.map(
                (pocket) => `${pocket.center_machine_mm[0]},${-pocket.center_machine_mm[1]}`,
              ).join(" ")}
            />
            <polyline points={feeder.pockets.map(
              (pocket) => `${pocket.center_machine_mm[0]},${-pocket.center_machine_mm[1]}`,
            ).join(" ")} />
            {feeder.pockets.map((pocket) => <circle
              className="workspace-pocket-hit"
              cx={pocket.center_machine_mm[0]}
              cy={-pocket.center_machine_mm[1]}
              key={`hit-${pocket.pocket_index}`}
              onClick={(event) => selectPocket(event, feeder, pocket)}
              r={Math.max(markerSize * .38, .8)}
            />)}
            {feeder.pockets.map((pocket) => <circle
              key={pocket.pocket_index}
              className={`${pocket.occupancy} status-${pocket.status} ${pocket.selected ? "selected" : ""} ${selection?.kind === "pocket" && selection.id === `${feeder.id}-${pocket.pocket_index}` ? "is-user-selected" : ""}`}
              cx={pocket.center_machine_mm[0]}
              cy={-pocket.center_machine_mm[1]}
              onClick={(event) => selectPocket(event, feeder, pocket)}
              onKeyDown={(event) => activateOnKeyboard(
                event,
                () => selectPocket(event, feeder, pocket),
              )}
              r={selection?.kind === "pocket"
                  && selection.id === `${feeder.id}-${pocket.pocket_index}`
                ? Math.max(markerSize * .22, .55)
                : Math.max(markerSize * .38, .8)}
              role="button"
              tabIndex={0}
            >
              <title>{`${feeder.name} · pocket ${pocket.pocket_index} · ${pocket.occupancy} · ${label(pocket.status)}`}</title>
            </circle>)}
          </g>)}

          {layers.machine && mapPose && machineOnMap && <g
            className={`workspace-machine-overlay ${status.homed ? "is-trusted" : "is-untrusted"}`}
          >
            <g className="workspace-gantry-guide">
              <rect
                className="workspace-gantry-shadow"
                x={region.x_min_mm}
                y={-mapPose.y_mm - 8}
                width={mapWidth}
                height={20}
                rx={1.5}
              />
              <rect
                className="workspace-gantry-extrusion"
                x={region.x_min_mm}
                y={-mapPose.y_mm - 10}
                width={mapWidth}
                height={20}
                rx={1.5}
              />
              <line
                className="workspace-gantry-rail"
                x1={region.x_min_mm}
                x2={region.x_max_mm}
                y1={-mapPose.y_mm - 7}
                y2={-mapPose.y_mm - 7}
              />
              <line
                className="workspace-gantry-rail"
                x1={region.x_min_mm}
                x2={region.x_max_mm}
                y1={-mapPose.y_mm + 7}
                y2={-mapPose.y_mm + 7}
              />
              <rect
                className="workspace-gantry-slot"
                x={region.x_min_mm}
                y={-mapPose.y_mm - 2}
                width={mapWidth}
                height={4}
              />
              <title>{`Approximate 20 mm X-gantry extrusion at Y${mapPose.y_mm.toFixed(3)}`}</title>
            </g>
            {machineGeometry && <rect
              className="workspace-camera-footprint"
              x={mapPose.x_mm
                + machineGeometry.camera_footprint_relative_mm.x_min_mm}
              y={-(mapPose.y_mm
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
              transform={`translate(${mapPose.x_mm} ${-mapPose.y_mm})`}
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
                <title>{`Top camera X${mapPose.x_mm.toFixed(3)} Y${mapPose.y_mm.toFixed(3)}`}</title>
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
          {selection && selection.kind !== "pocket" && <g
            className="workspace-selection-marker"
          >
            {selection.scanRegion && <rect
              x={selection.scanRegion.x_min_mm}
              y={-selection.scanRegion.y_max_mm}
              width={selection.scanRegion.x_max_mm - selection.scanRegion.x_min_mm}
              height={selection.scanRegion.y_max_mm - selection.scanRegion.y_min_mm}
            />}
            <circle
              cx={selection.point[0]}
              cy={-selection.point[1]}
              r={Math.max(markerSize * .7, 1.5)}
            />
            <line
              x1={selection.point[0] - Math.max(markerSize, 2.2)}
              x2={selection.point[0] + Math.max(markerSize, 2.2)}
              y1={-selection.point[1]}
              y2={-selection.point[1]}
            />
            <line
              x1={selection.point[0]}
              x2={selection.point[0]}
              y1={-selection.point[1] - Math.max(markerSize, 2.2)}
              y2={-selection.point[1] + Math.max(markerSize, 2.2)}
            />
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
        {selection && <section className="workspace-selection-card">
          <button
            aria-label="Clear workspace selection"
            className="workspace-selection-close"
            disabled={selectionAction !== null}
            onClick={() => setSelection(null)}
            type="button"
          >×</button>
          <p className="eyebrow">{label(selection.kind)}</p>
          <h3>{selection.label}</h3>
          <p className="workspace-selection-coordinate">
            X {selection.point[0].toFixed(2)} · Y {selection.point[1].toFixed(2)} mm
          </p>
          <div className="workspace-selection-actions">
            <button
              disabled={!selectionActionAllowed}
              onClick={() => void goToSelection()}
              type="button"
            >{selectionAction === "moving" ? "Moving…" : "Go to"}</button>
            {selection.scanRegion && <button
              disabled={!selectionActionAllowed}
              onClick={() => void rescanSelection()}
              type="button"
            >{selectionAction === "estimating"
                ? "Estimating…"
                : selectionAction === "scanning" ? "Scanning…" : "Rescan"}</button>}
            {["placement", "pocket", "point"].includes(selection.kind) && <button
                disabled={!selectionActionAllowed}
                onClick={() => void quickCapture()}
                type="button"
              >{selectionAction === "capturing" ? "Capturing…" : "Quick capture"}</button>}
          </div>
          {rescanProgress && <div className="workspace-selection-progress" role="status">
            <div className="workspace-progress-label">
              <strong>{rescanPercent}%</strong>
              <span>{rescanProgress.status === "succeeded"
                ? `${rescanProgress.plannedTiles}/${rescanProgress.plannedTiles} tiles · complete`
                : rescanProgress.capturedTiles >= rescanProgress.plannedTiles
                  ? `Finalizing scan${rescanEtaSeconds > 0
                    ? ` · about ${formatWorkspaceDuration(rescanEtaSeconds)} left`
                    : ""}`
                  : `${rescanProgress.capturedTiles}/${rescanProgress.plannedTiles} tiles · about ${formatWorkspaceDuration(rescanEtaSeconds)} left`}</span>
            </div>
            <progress
              value={rescanProgress.capturedTiles}
              max={Math.max(rescanProgress.plannedTiles, 1)}
            />
          </div>}
          {selectionMessage && <p className="workspace-selection-message" role="status">
            {selectionMessage}
          </p>}
          {!selectionActionAllowed && selectionAction === null && <p
            className="workspace-selection-message"
          >Machine must be ready and fine-homed.</p>}
          {quickCaptureUrl && <img
            alt={`Top-camera capture at X ${selection.point[0].toFixed(2)}, Y ${selection.point[1].toFixed(2)}`}
            className="workspace-quick-capture"
            onError={() => setSelectionMessage("Quick capture failed.")}
            src={quickCaptureUrl}
          />}
        </section>}
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
        {mapPose && region && !machineOnMap && <div className="workspace-map-note">
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
