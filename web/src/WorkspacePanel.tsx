import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type {
  KeyboardEvent as ReactKeyboardEvent, MouseEvent as ReactMouseEvent,
  PointerEvent as ReactPointerEvent,
} from "react";
import { api, waitForOperation } from "./api";
import { useDialog } from "./Dialog";
import type {
  BoardDesign, BoardInstance, BoardPopulationRefreshReport, Job, JobExecutionRun,
  MachineOverlayGeometry, MachineStatus, PlacementExecutionRun,
  PlacementOutcomeRecord, Pose,
  StripFeederLaneProposal, StripFeederMapRegistration,
  WorkspaceBoardCandidate, WorkspaceBoardDiscoveryReport, WorkspaceDisplayPatch,
  WorkspaceObjectClass,
  WorkspaceFeederLaneSlot, WorkspaceScanRegion, WorkspaceScanReport,
  WorkspaceScanSummary, WorkspaceActivityFeed, WorkspaceActivityActionId,
} from "./types";

type WorkspaceLayer =
  | "image" | "objects" | "boards" | "placements" | "feeders" | "machine";

const WORKSPACE_LAYERS: Array<{ id: WorkspaceLayer; label: string }> = [
  { id: "image", label: "Scan image" },
  { id: "objects", label: "Detected objects" },
  { id: "boards", label: "Job boards" },
  { id: "placements", label: "Placement positions" },
  { id: "feeders", label: "Feeder lanes / pockets" },
  { id: "machine", label: "Machine position" },
];

interface WorkspacePanelProps {
  status: MachineStatus;
  setError: (value: string | null) => void;
  openMachine: () => void;
  openJobs: () => void;
  focusFeederId?: string | null;
  onFocusConsumed?: () => void;
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
  status: "populated" | "unpopulated" | "uncertain";
  current: boolean;
  rotationDeg: number;
}

type WorkspaceSelectionKind =
  | "feeder" | "feeder_slot" | "board" | "pocket" | "placement" | "point";

interface WorkspaceSelection {
  kind: WorkspaceSelectionKind;
  id: string;
  label: string;
  point: [number, number];
  scanRegion: WorkspaceScanRegion | null;
  objectId: string | null;
  pocketIndex?: number;
}

export interface WorkspaceFeederSlotTarget {
  id: string;
  label: string;
  slot: WorkspaceFeederLaneSlot;
}

export interface WorkspaceFeederBank {
  bankIndex: number;
  label: string;
  candidate: WorkspaceBoardCandidate;
}

export interface WorkspaceFeederLaneTarget {
  feeder: StripFeederMapRegistration;
  label: string;
  bankOutline: Array<[number, number]> | null;
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
  const cropHalfCross = halfWidth + 3;
  const cropHalfAlong = Math.max(feeder.tape_pitch_mm * 1.5, 6);
  const radians = feeder.tape_rotation_deg * Math.PI / 180;
  const axis = [Math.cos(radians), Math.sin(radians)];
  const normal = [-axis[1], axis[0]];
  const cropHalfX = Math.abs(normal[0]) * cropHalfCross
    + Math.abs(axis[0]) * cropHalfAlong;
  const cropHalfY = Math.abs(normal[1]) * cropHalfCross
    + Math.abs(axis[1]) * cropHalfAlong;
  // Reserve one more pocket pitch for a newly exposed end pocket. The extra
  // millimeter keeps the inference crop away from the raster boundary.
  const padding = Math.max(cropHalfX, cropHalfY) + feeder.tape_pitch_mm + 1;
  return paddedRegion(
    feeder.pockets.map((pocket) => pocket.center_machine_mm),
    parent,
    padding,
  );
}

export function feederLaneOutline(
  feeder: StripFeederMapRegistration,
  bankOutline: Array<[number, number]> | null = null,
): Array<[number, number]> {
  if (feeder.pockets.length === 0) return [];
  const radians = feeder.tape_rotation_deg * Math.PI / 180;
  const axis: [number, number] = [Math.cos(radians), Math.sin(radians)];
  const normal: [number, number] = [-axis[1], axis[0]];
  const along = feeder.pockets.map((pocket) =>
    pocket.center_machine_mm[0] * axis[0]
      + pocket.center_machine_mm[1] * axis[1]
  );
  const across = feeder.pockets.map((pocket) =>
    pocket.center_machine_mm[0] * normal[0]
      + pocket.center_machine_mm[1] * normal[1]
  );
  const centerAcross = across.reduce((sum, value) => sum + value, 0)
    / across.length;
  const bankAlong = bankOutline?.map((point) =>
    point[0] * axis[0] + point[1] * axis[1]
  ) ?? [];
  const start = bankAlong.length > 0 ? Math.min(...bankAlong) : Math.min(...along) - 2;
  const end = bankAlong.length > 0 ? Math.max(...bankAlong) : Math.max(...along) + 2;
  const halfWidth = feeder.tape_width_mm / 2;
  return [
    [start, centerAcross - halfWidth],
    [end, centerAcross - halfWidth],
    [end, centerAcross + halfWidth],
    [start, centerAcross + halfWidth],
  ].map(([alongValue, acrossValue]) => [
    axis[0] * alongValue + normal[0] * acrossValue,
    axis[1] * alongValue + normal[1] * acrossValue,
  ]);
}

function pointToSegmentDistance(
  point: [number, number],
  segment: [[number, number], [number, number]],
): number {
  const delta = [
    segment[1][0] - segment[0][0],
    segment[1][1] - segment[0][1],
  ];
  const lengthSquared = delta[0] ** 2 + delta[1] ** 2;
  const fraction = lengthSquared > 0
    ? Math.min(1, Math.max(0, (
      (point[0] - segment[0][0]) * delta[0]
        + (point[1] - segment[0][1]) * delta[1]
    ) / lengthSquared))
    : 0;
  return Math.hypot(
    point[0] - (segment[0][0] + fraction * delta[0]),
    point[1] - (segment[0][1] + fraction * delta[1]),
  );
}

export function nearestFeederProposal(
  proposals: StripFeederLaneProposal[],
  point: [number, number],
): StripFeederLaneProposal | null {
  return proposals
    .filter((proposal) => proposal.pick_centerline_machine_mm !== null)
    .sort((left, right) => pointToSegmentDistance(
      point, left.pick_centerline_machine_mm!,
    ) - pointToSegmentDistance(
      point, right.pick_centerline_machine_mm!,
    ))[0] ?? null;
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
  boardPopulation: BoardPopulationRefreshReport[] = [],
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
  const latestPhysicalEvidence = new Map<string, string>();
  for (const outcome of outcomes.filter((outcome) =>
    compatibleJobIds.has(outcome.job_id)
  )) {
    const current = latestPhysicalEvidence.get(outcome.placement_reference);
    if (!current || outcome.updated_at > current) {
      latestPhysicalEvidence.set(outcome.placement_reference, outcome.updated_at);
    }
  }
  for (const step of execution?.steps ?? []) {
    if (step.status === "succeeded") {
      populatedReferences.add(step.placement_reference);
      const current = latestPhysicalEvidence.get(step.placement_reference);
      if (execution && (!current || execution.updated_at > current)) {
        latestPhysicalEvidence.set(step.placement_reference, execution.updated_at);
      }
    }
  }
  const latestPopulation = new Map<string, {
    createdAt: string;
    occupancy: "populated" | "empty" | "uncertain";
  }>();
  for (const report of boardPopulation.filter((report) => report.job_id === job.id)) {
    const instance = job.boards.find((board) =>
      board.id === report.board_instance_id
        && board.board_id === report.board_design_id
        && board.design_to_machine?.matrix.every(
          (value, index) => value === report.design_to_machine.matrix[index],
        ));
    if (!instance) continue;
    for (const observation of report.observations) {
      const key = `${instance.id}:${observation.placement_reference}`;
      const current = latestPopulation.get(key);
      if (!current || report.created_at > current.createdAt) {
        latestPopulation.set(key, {
          createdAt: report.created_at,
          occupancy: observation.occupancy,
        });
      }
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
        const scanEvidence = latestPopulation.get(
          `${board.id}:${placement.reference}`,
        );
        const scanned = scanEvidence
          && scanEvidence.createdAt
            >= (latestPhysicalEvidence.get(placement.reference) ?? "")
          ? scanEvidence
          : undefined;
        return {
          id: `${board.id}-${placement.reference}`,
          reference: placement.reference,
          point: applyTransform(matrix, placement.pose.x_mm, placement.pose.y_mm),
          boardId: board.id,
          boardLabel: board.label,
          boardPoints,
          status: scanned?.occupancy === "populated"
              || (!scanned && populatedReferences.has(placement.reference))
            ? "populated"
            : scanned?.occupancy === "uncertain" ? "uncertain" : "unpopulated",
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

export function activeWorkspaceDisplayPatches(
  patches: WorkspaceDisplayPatch[],
  feederLanes: StripFeederMapRegistration[],
): WorkspaceDisplayPatch[] {
  const activeFeederIds = new Set(feederLanes.map((feeder) => feeder.id));
  return patches.filter((patch) =>
    patch.kind !== "feeder_lane" || activeFeederIds.has(patch.object_id)
  );
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

export function workspaceFeederBanks(
  discovery: WorkspaceBoardDiscoveryReport | null,
): WorkspaceFeederBank[] {
  return (discovery?.candidates ?? [])
    .filter((candidate) => candidateClass(candidate) === "feeder")
    .sort((left, right) => left.center_x_mm - right.center_x_mm
      || left.center_y_mm - right.center_y_mm)
    .map((candidate, bankIndex) => ({
      bankIndex,
      label: `Feeder Bank ${bankIndex + 1}`,
      candidate,
    }));
}

function feederCenter(feeder: StripFeederMapRegistration): [number, number] {
  return feeder.pockets.reduce<[number, number]>(
    (sum, pocket) => [
      sum[0] + pocket.center_machine_mm[0],
      sum[1] + pocket.center_machine_mm[1],
    ],
    [0, 0],
  ).map((value) => value / Math.max(feeder.pockets.length, 1)) as [number, number];
}

function feederIsInBank(
  feeder: StripFeederMapRegistration,
  bank: WorkspaceFeederBank,
): boolean {
  const [xMm, yMm] = feederCenter(feeder);
  return xMm >= bank.candidate.region.x_min_mm
    && xMm <= bank.candidate.region.x_max_mm
    && yMm >= bank.candidate.region.y_min_mm
    && yMm <= bank.candidate.region.y_max_mm;
}

function feederBankIsVertical(bank: WorkspaceFeederBank): boolean {
  return bank.candidate.region.y_max_mm - bank.candidate.region.y_min_mm
    >= bank.candidate.region.x_max_mm - bank.candidate.region.x_min_mm;
}

function feederBankLaneAnchors(
  bank: WorkspaceFeederBank,
  feeders: StripFeederMapRegistration[],
): number[] {
  const coordinateIndex = feederBankIsVertical(bank) ? 0 : 1;
  const coordinates = [
    ...(bank.candidate.feeder_lane_slots ?? []).map(
      (slot) => polygonCenter(slot.outline_machine_mm)[coordinateIndex],
    ),
    ...feeders.filter((feeder) => feederIsInBank(feeder, bank)).map(
      (feeder) => feederCenter(feeder)[coordinateIndex],
    ),
  ].sort((left, right) => left - right);
  const anchors: number[] = [];
  const clustered: number[][] = [];
  for (const coordinate of coordinates) {
    const current = clustered.at(-1);
    const center = current
      ? current.reduce((sum, value) => sum + value, 0) / current.length
      : null;
    if (current && center !== null && Math.abs(coordinate - center) <= 4) {
      current.push(coordinate);
    } else {
      clustered.push([coordinate]);
    }
  }
  for (const cluster of clustered) {
    anchors.push(cluster.reduce((sum, value) => sum + value, 0) / cluster.length);
  }
  return anchors;
}

function nearestFeederLaneIndex(coordinate: number, anchors: number[]): number {
  let nearestIndex = 0;
  let nearestDistance = Number.POSITIVE_INFINITY;
  anchors.forEach((anchor, index) => {
    const distance = Math.abs(anchor - coordinate);
    if (distance < nearestDistance) {
      nearestIndex = index;
      nearestDistance = distance;
    }
  });
  return nearestIndex;
}

export function workspaceFeederLaneTargets(
  feeders: StripFeederMapRegistration[],
  banks: WorkspaceFeederBank[],
): WorkspaceFeederLaneTarget[] {
  const targets = new Map<string, WorkspaceFeederLaneTarget>();
  for (const bank of banks) {
    const coordinateIndex = feederBankIsVertical(bank) ? 0 : 1;
    const anchors = feederBankLaneAnchors(bank, feeders);
    feeders.filter((feeder) => feederIsInBank(feeder, bank)).forEach((feeder) => {
      const laneIndex = nearestFeederLaneIndex(
        feederCenter(feeder)[coordinateIndex], anchors,
      );
      targets.set(feeder.id, {
        feeder,
        label: `${bank.label} - lane ${laneIndex + 1}`,
        bankOutline: bank.candidate.outline_machine_mm,
      });
    });
  }
  return feeders.map((feeder) => targets.get(feeder.id) ?? {
    feeder,
    label: feeder.name,
    bankOutline: null,
  });
}

export function workspaceFeederSlotTargets(
  banks: WorkspaceFeederBank[],
  feeders: StripFeederMapRegistration[],
): WorkspaceFeederSlotTarget[] {
  return banks.flatMap((bank) => {
    const vertical = feederBankIsVertical(bank);
    const coordinateIndex = vertical ? 0 : 1;
    const anchors = feederBankLaneAnchors(bank, feeders);
    return [...(bank.candidate.feeder_lane_slots ?? [])]
      .sort((left, right) => {
        const leftCenter = polygonCenter(left.outline_machine_mm);
        const rightCenter = polygonCenter(right.outline_machine_mm);
        return leftCenter[coordinateIndex] - rightCenter[coordinateIndex];
      })
      .map((slot) => ({
        id: `candidate-${bank.candidate.candidate_index}-slot-${slot.slot_index}`,
        label: `${bank.label} - lane ${
          nearestFeederLaneIndex(
            polygonCenter(slot.outline_machine_mm)[coordinateIndex], anchors,
          ) + 1
        }`,
        slot: normalizeFeederSlotToBank(
          slot, bank.candidate.outline_machine_mm,
        ),
      }));
  });
}

export function normalizeFeederSlotToBank(
  slot: WorkspaceFeederLaneSlot,
  bankOutline: Array<[number, number]>,
): WorkspaceFeederLaneSlot {
  const delta = [
    slot.centerline_machine_mm[1][0] - slot.centerline_machine_mm[0][0],
    slot.centerline_machine_mm[1][1] - slot.centerline_machine_mm[0][1],
  ];
  const length = Math.hypot(delta[0], delta[1]);
  if (length <= 0 || bankOutline.length === 0) return slot;
  const axis: [number, number] = [delta[0] / length, delta[1] / length];
  const normal: [number, number] = [-axis[1], axis[0]];
  const along = bankOutline.map((point) => point[0] * axis[0] + point[1] * axis[1]);
  const across = slot.outline_machine_mm.map(
    (point) => point[0] * normal[0] + point[1] * normal[1],
  );
  const start = Math.min(...along);
  const end = Math.max(...along);
  const crossMin = Math.min(...across);
  const crossMax = Math.max(...across);
  const centerAcross = (crossMin + crossMax) / 2;
  const point = (alongValue: number, acrossValue: number): [number, number] => [
    axis[0] * alongValue + normal[0] * acrossValue,
    axis[1] * alongValue + normal[1] * acrossValue,
  ];
  return {
    ...slot,
    centerline_machine_mm: [point(start, centerAcross), point(end, centerAcross)],
    outline_machine_mm: [
      point(start, crossMin), point(end, crossMin),
      point(end, crossMax), point(start, crossMax),
    ],
    length_mm: end - start,
  };
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
  status, setError, openMachine, openJobs, focusFeederId, onFocusConsumed,
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
  const [boardPopulation, setBoardPopulation] = useState<
    BoardPopulationRefreshReport[]
  >([]);
  const [designs, setDesigns] = useState<BoardDesign[]>([]);
  const [feederRegistrations, setFeederRegistrations] = useState<
    StripFeederMapRegistration[]
  >([]);
  const [machineGeometry, setMachineGeometry] = useState<MachineOverlayGeometry | null>(null);
  const [clearXyFeedMmPerMin, setClearXyFeedMmPerMin] = useState(23_000);
  const [mapPose, setMapPose] = useState<Pose | null>(status.pose);
  const [placementExecution, setPlacementExecution] = useState<PlacementExecutionRun | null>(null);
  const [activity, setActivity] = useState<WorkspaceActivityFeed>({
    active: null,
    recent: [],
  });
  const [activityAction, setActivityAction] = useState<WorkspaceActivityActionId | null>(null);
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
      const [
        jobValues, executionValues, designValues, outcomeValues,
        populationValues,
      ]
        = await Promise.all([
          api.jobs(), api.jobExecutions(), api.boards(), api.placementOutcomes(),
          api.boardPopulationRefreshes(),
      ]);
      setJobs(jobValues);
      setExecutions(executionValues);
      setDesigns(designValues);
      setPlacementOutcomes(outcomeValues);
      setBoardPopulation(populationValues);
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    }
  }, [setError]);

  useEffect(() => {
    void loadOperationalData();
    const timer = window.setInterval(() => void loadOperationalData(), 1000);
    return () => window.clearInterval(timer);
  }, [loadOperationalData]);

  const loadActivity = useCallback(async () => {
    try {
      setActivity(await api.workspaceActivity());
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    }
  }, [setError]);

  useEffect(() => {
    void loadActivity();
    const timer = window.setInterval(() => void loadActivity(), 750);
    return () => window.clearInterval(timer);
  }, [loadActivity]);

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
  }, [selectedScanId, selectedSummary?.status, selectedSummary?.updated_at, setError]);

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
  }, [
    selectedScan?.id, selectedScan?.updated_at,
    selectedScan?.board_discovery?.report_artifact_name, setError,
  ]);

  const activeExecution = useMemo(
    () => selectActiveJobExecution(executions),
    [executions],
  );
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
      boardPopulation,
    ),
    [
      activeExecution, boardPopulation, designs, jobs, placementOutcomes,
      workspaceJob,
    ],
  );
  const feederLanes = useMemo(
    () => latestFeederLanes(
      feederRegistrations, status.profile_id, selectedSummary,
    ),
    [feederRegistrations, selectedSummary, status.profile_id],
  );
  const visibleDisplayPatches = useMemo(
    () => activeWorkspaceDisplayPatches(displayPatches, feederLanes),
    [displayPatches, feederLanes],
  );
  const feederBanks = useMemo(() => workspaceFeederBanks(discovery), [discovery]);
  const feederLaneTargets = useMemo(
    () => workspaceFeederLaneTargets(feederLanes, feederBanks),
    [feederBanks, feederLanes],
  );
  useEffect(() => {
    if (selection?.kind !== "pocket"
        || selection.objectId === null
        || selection.pocketIndex === undefined) return;
    const pocket = feederLaneTargets
      .find((target) => target.feeder.id === selection.objectId)
      ?.feeder.pockets.find(
        (candidate) => candidate.pocket_index === selection.pocketIndex,
      );
    if (!pocket) return;
    setSelection((current) => {
      if (current?.kind !== "pocket"
          || current.objectId !== selection.objectId
          || current.pocketIndex !== selection.pocketIndex
          || current.point[0] === pocket.center_machine_mm[0]
            && current.point[1] === pocket.center_machine_mm[1]) return current;
      return { ...current, point: pocket.center_machine_mm };
    });
  }, [feederLaneTargets, selection]);
  const feederSlots = useMemo(
    () => workspaceFeederSlotTargets(feederBanks, feederLanes),
    [feederBanks, feederLanes],
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

  useEffect(() => {
    if (!focusFeederId || !region || selectionAction !== null) return;
    const lane = feederLaneTargets.find(
      (target) => target.feeder.id === focusFeederId,
    );
    if (lane) {
      setSelection({
        kind: "feeder",
        id: lane.feeder.id,
        label: lane.label,
        point: polygonCenter(
          lane.feeder.pockets.map((pocket) => pocket.center_machine_mm),
        ),
        scanRegion: feederScanRegion(lane.feeder, region),
        objectId: lane.feeder.id,
      });
      onFocusConsumed?.();
      return;
    }
    const slot = feederSlots.find((target) => target.id === focusFeederId);
    if (!slot) return;
    setSelection({
      kind: "feeder_slot",
      id: slot.id,
      label: slot.label,
      point: polygonCenter(slot.slot.outline_machine_mm),
      scanRegion: paddedRegion(slot.slot.outline_machine_mm, region, 6),
      objectId: null,
    });
    onFocusConsumed?.();
  }, [
    feederLaneTargets, feederSlots, focusFeederId, onFocusConsumed, region,
    selectionAction,
  ]);

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
    feederLabel: string,
  ) => {
    event.stopPropagation();
    if (!region) return;
    selectTarget({
      kind: "feeder",
      id: feeder.id,
      label: feederLabel,
      point: polygonCenter(feeder.pockets.map((pocket) => pocket.center_machine_mm)),
      scanRegion: feederScanRegion(feeder, region),
      objectId: feeder.id,
    });
  };

  const selectFeederSlot = (
    event: WorkspaceActivateEvent,
    target: WorkspaceFeederSlotTarget,
  ) => {
    event.stopPropagation();
    if (!region) return;
    selectTarget({
      kind: "feeder_slot",
      id: target.id,
      label: target.label,
      point: polygonCenter(target.slot.outline_machine_mm),
      scanRegion: paddedRegion(target.slot.outline_machine_mm, region, 6),
      objectId: null,
    });
  };

  const selectPocket = (
    event: WorkspaceActivateEvent,
    feeder: StripFeederMapRegistration,
    feederLabel: string,
    pocket: StripFeederMapRegistration["pockets"][number],
  ) => {
    event.stopPropagation();
    if (!region) return;
    selectTarget({
      kind: "pocket",
      id: `${feeder.id}-${pocket.pocket_index}`,
      label: `${feederLabel} · pocket ${pocket.pocket_index}`,
      point: pocket.center_machine_mm,
      scanRegion: feederScanRegion(feeder, region),
      objectId: feeder.id,
      pocketIndex: pocket.pocket_index,
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
        : selection.kind === "board" ? "this PCB" : "this feeder lane";
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
        message: `Scan ${rescanScope} and align the result to the current full workspace scan. Estimated scan time is about ${formatWorkspaceDuration(estimate.estimated_duration_seconds)} for ${estimate.planned_tiles} tiles.${["feeder", "feeder_slot", "pocket"].includes(selection.kind) ? " A short lane-alignment check will run automatically after the scan." : ""} Confirm that the scan region and travel path are clear, both nozzles are level, and power isolation is reachable.`,
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
      let rescanObjectId = selection.objectId;
      let feederResultMessage = "";
      let boardResultMessage = "";
      if (["feeder", "feeder_slot", "pocket"].includes(selection.kind)) {
        setSelectionMessage("Detecting feeder tape and pocket occupancy…");
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
          const vision = await api.analyzeStripFeeder(
            started.scan.id, candidates[0].candidate_index,
          );
          const proposal = nearestFeederProposal(vision.proposals, selection.point);
          if (proposal) {
            if (!rescanObjectId) {
              const existing = feederLanes.find((feeder) => {
                if (feeder.pockets.length < 2) return false;
                const segment: [[number, number], [number, number]] = [
                  feeder.pockets[0].center_machine_mm,
                  feeder.pockets[feeder.pockets.length - 1].center_machine_mm,
                ];
                return pointToSegmentDistance(selection.point, segment)
                  <= feeder.tape_width_mm * .6;
              });
              if (existing) {
                rescanObjectId = existing.id;
              } else {
                const registration = await api.createStripFeederDraft(
                  started.scan.id,
                  candidates[0].candidate_index,
                  proposal.proposal_index,
                  undefined,
                  undefined,
                  selection.label,
                );
                rescanObjectId = registration.id;
                feederResultMessage = "Tape was detected and a feeder draft was created.";
              }
            }
            const occupancy = await api.refreshStripFeederOccupancy(
              started.scan.id,
              candidates[0].candidate_index,
              rescanObjectId,
            );
            const canCheckAlignment = occupancy.observations.some(
              (observation) => observation.occupancy === "occupied"
                && !observation.review_required,
            );
            let alignmentResult = "Lane alignment was not checked because no occupied pocket is ready.";
            if (canCheckAlignment) {
              setSelectionMessage("Checking lane alignment…");
              const alignment = await api.startStripFeederLiveHoldout(
                rescanObjectId, lease.id,
              );
              await waitForOperation(alignment.operation, (operation) => {
                setSelectionMessage(
                  operation.message ?? "Checking lane alignment…",
                );
              });
              alignmentResult = "Lane alignment passed automatically.";
            }
            const modelObservationCount = occupancy.observations.filter(
              (observation) => observation.model_prediction !== null,
            ).length;
            feederResultMessage = [
              feederResultMessage,
              `${occupancy.observations.length} pockets were promoted as one lane revision.`,
              modelObservationCount > 0
                ? `Model evidence was available for ${modelObservationCount}.`
                : occupancy.model_available
                  ? "The model did not produce usable evidence."
                  : "The model was not available.",
              alignmentResult,
            ].filter(Boolean).join(" ");
            await loadFeederData();
          } else {
            feederResultMessage = "No carrier tape was detected in the selected lane.";
          }
        } else {
          feederResultMessage = "No feeder lane was detected in the rescan.";
        }
      } else if (["board", "placement"].includes(selection.kind)) {
        const board = workspaceJob?.boards.find(
          (candidate) => candidate.id === selection.objectId,
        );
        if (!workspaceJob || !board) {
          throw new Error("The selected PCB is not part of the current workspace job.");
        }
        setSelectionMessage("Detecting board population…");
        const population = await api.refreshBoardPopulation(
          started.scan.id,
          workspaceJob.id,
          board.id,
        );
        const populated = population.observations.filter(
          (observation) => observation.occupancy === "populated",
        ).length;
        const empty = population.observations.filter(
          (observation) => observation.occupancy === "empty",
        ).length;
        const review = population.observations.filter(
          (observation) => observation.review_required,
        ).length;
        boardResultMessage = [
          `${population.observations.length} placement states were refreshed.`,
          `${populated} populated, ${empty} empty.`,
          review > 0 ? `${review} require review.` : "",
        ].filter(Boolean).join(" ");
        await loadOperationalData();
      }
      setSelectionMessage("Checking image alignment…");
      const patches = await loadDisplayPatches();
      const accepted = patches.some((patch) =>
        patch.source_scan_id === started.scan.id
          && patch.object_id === rescanObjectId);
      setSelectionMessage([
        "Rescan complete.",
        feederResultMessage,
        boardResultMessage,
        accepted
          ? "The aligned image is now shown."
          : "Alignment checks did not accept an image update.",
      ].filter(Boolean).join(" "));
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
    if (!selection || ![
      "feeder", "feeder_slot", "placement", "pocket", "point",
    ].includes(selection.kind)) return;
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

  const runActivityAction = async (actionId: WorkspaceActivityActionId) => {
    const current = activity.active;
    if (!current?.execution_id) return;
    setActivityAction(actionId);
    setError(null);
    try {
      if (actionId === "pause_job") {
        await api.pauseJobExecution(current.execution_id);
      } else if (actionId === "resume_job") {
        if (await requestDialog({
          title: "Resume job?",
          message: "Confirm that the PCB and feeders are fixed, the travel path is clear, and power isolation is reachable.",
          confirmLabel: "Resume job",
        }) === null) return;
        const lease = await api.acquireOrRenewLease(
          "VelocityPnP workspace job resume", 900,
        );
        await api.resumeJobExecution(current.execution_id, lease.id);
      } else if (actionId === "retry_step" || actionId === "skip_step") {
        if (current.step_index === null || current.step_index === undefined) return;
        const skip = actionId === "skip_step";
        const note = await requestDialog({
          title: skip ? "Skip this placement?" : "Retry this placement?",
          message: skip
            ? "Record why it is safe to continue without this placement."
            : "Add an optional execution note before the retry.",
          confirmLabel: skip ? "Skip placement" : "Retry placement",
          tone: skip ? "danger" : "default",
          input: {
            label: skip ? "Reason" : "Execution note",
            placeholder: skip ? "Reason for the skip" : "Optional note",
            required: skip,
          },
        });
        if (note === null) return;
        await api.resolveJobExecutionStep(
          current.execution_id,
          current.step_index,
          skip ? "skip" : "retry",
          note,
        );
      } else if (actionId === "cancel_job") {
        if (await requestDialog({
          title: "Cancel this job?",
          message: "The execution cannot resume after it is cancelled.",
          confirmLabel: "Cancel job",
          tone: "danger",
        }) === null) return;
        await api.cancelJobExecution(current.execution_id);
      }
      await Promise.all([loadActivity(), loadOperationalData()]);
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setActivityAction(null);
    }
  };

  const selectedFeeder = selection?.objectId
    ? feederLaneTargets.find(
      (target) => target.feeder.id === selection.objectId,
    ) ?? null
    : null;
  const latestActivityResult = activity.recent[0] ?? null;

  return <div className="workspace-layout">
    <section className="workspace-map-card">
      <div className="workspace-map-heading">
        <div>
          <p className="eyebrow">Machine-coordinate view</p>
          <h2>{selectedSummary ? "Whole workspace" : "Workspace"}</h2>
        </div>
        <div className="workspace-map-badges">
          {visibleDisplayPatches.length > 0 && <span className="map-status">
            Full scan + {visibleDisplayPatches.length} aligned updates
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
            {visibleDisplayPatches.map((patch) => <clipPath
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
          {layers.image && visibleDisplayPatches.map((patch) => {
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
              <title>{feederBanks.find(
                (bank) => bank.candidate.candidate_index === candidate.candidate_index,
              )?.label ?? `${label(candidateClass(candidate))} ${candidate.candidate_index + 1}`}</title>
            </polygon>
          )}

          {layers.feeders && feederSlots.map((target) => <g
            aria-label={`${target.label} recognized feeder slot`}
            className={`workspace-selectable workspace-feeder-slot ${selection?.kind === "feeder_slot" && selection.id === target.id ? "is-selected" : ""}`}
            key={target.id}
            onClick={(event) => selectFeederSlot(event, target)}
            onKeyDown={(event) => activateOnKeyboard(
              event, () => selectFeederSlot(event, target),
            )}
            role="button"
            tabIndex={0}
          >
            <title>{`${target.label} · ${target.slot.width_mm.toFixed(1)} mm physical slot${target.slot.review_required ? " · review" : ""}`}</title>
            <polygon
              className="workspace-feeder-slot-hit"
              points={target.slot.outline_machine_mm.map(
                ([xMm, yMm]) => `${xMm},${-yMm}`,
              ).join(" ")}
            />
            <polygon
              className="workspace-feeder-slot-boundary"
              points={target.slot.outline_machine_mm.map(
                ([xMm, yMm]) => `${xMm},${-yMm}`,
              ).join(" ")}
            />
          </g>)}

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
          {activity.active?.target?.region && <rect
            className="workspace-active-target-region"
            x={activity.active.target.region.x_min_mm}
            y={-activity.active.target.region.y_max_mm}
            width={activity.active.target.region.x_max_mm
              - activity.active.target.region.x_min_mm}
            height={activity.active.target.region.y_max_mm
              - activity.active.target.region.y_min_mm}
          >
            <title>{activity.active.target.label}</title>
          </rect>}
          {layers.placements && placementMarkers.map((placement) => <g
            className={`workspace-selectable workspace-placement-position ${placement.status} ${placement.current ? "current" : ""} ${placement.current && activeExecution?.status === "needs_attention" ? "has-failure" : ""} ${selection?.kind === "placement" && selection.id === placement.id ? "is-selected" : ""}`}
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
            {placement.current && activeExecution?.status === "needs_attention"
              && <circle className="workspace-failure-marker" r={placementMarkerRadius * 2.2} />}
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

          {layers.feeders && feederLaneTargets.map((target) => <g
            aria-label={`${target.label} feeder lane`}
            className={`workspace-selectable workspace-feeder-pockets ${selection?.kind === "feeder" && selection.id === target.feeder.id ? "is-selected" : ""}`}
            key={target.feeder.id}
            onClick={(event) => selectFeeder(event, target.feeder, target.label)}
            onKeyDown={(event) => activateOnKeyboard(
              event,
              () => selectFeeder(event, target.feeder, target.label),
            )}
            role="button"
            tabIndex={0}
          >
            <title>{`${target.label} recognized feeder lane`}</title>
            <polygon
              className="workspace-feeder-lane-hit"
              points={feederLaneOutline(target.feeder, target.bankOutline).map(
                ([xMm, yMm]) => `${xMm},${-yMm}`,
              ).join(" ")}
            />
            <polygon
              className="workspace-feeder-lane-boundary"
              points={feederLaneOutline(target.feeder, target.bankOutline).map(
                ([xMm, yMm]) => `${xMm},${-yMm}`,
              ).join(" ")}
            />
            <polyline
              className="workspace-feeder-hit"
              points={target.feeder.pockets.map(
                (pocket) => `${pocket.center_machine_mm[0]},${-pocket.center_machine_mm[1]}`,
              ).join(" ")}
            />
            <polyline points={target.feeder.pockets.map(
              (pocket) => `${pocket.center_machine_mm[0]},${-pocket.center_machine_mm[1]}`,
            ).join(" ")} />
            {target.feeder.pockets.map((pocket) => <circle
              className="workspace-pocket-hit"
              cx={pocket.center_machine_mm[0]}
              cy={-pocket.center_machine_mm[1]}
              key={`hit-${pocket.pocket_index}`}
              onClick={(event) => selectPocket(
                event, target.feeder, target.label, pocket,
              )}
              r={Math.max(markerSize * .38, .8)}
            />)}
            {target.feeder.pockets.map((pocket) => <circle
              key={pocket.pocket_index}
              className={`${pocket.occupancy} status-${pocket.status} ${pocket.operator_review_label === "unusable" || pocket.operator_review_label === "component_on_edge" ? "reviewed-unavailable" : ""} ${pocket.selected ? "selected" : ""} ${selection?.kind === "pocket" && selection.id === `${target.feeder.id}-${pocket.pocket_index}` ? "is-user-selected" : ""}`}
              cx={pocket.center_machine_mm[0]}
              cy={-pocket.center_machine_mm[1]}
              onClick={(event) => selectPocket(
                event, target.feeder, target.label, pocket,
              )}
              onKeyDown={(event) => activateOnKeyboard(
                event,
                () => selectPocket(event, target.feeder, target.label, pocket),
              )}
              r={selection?.kind === "pocket"
                  && selection.id === `${target.feeder.id}-${pocket.pocket_index}`
                ? placementMarkerRadius
                : Math.max(markerSize * .38, .8)}
              role="button"
              tabIndex={0}
            >
              <title>{`${target.label} · pocket ${pocket.pocket_index} · ${pocket.occupancy}${pocket.occupancy_source === "workspace_rescan" ? " from workspace rescan" : ""}${pocket.occupancy_review_required ? " · review" : ""}${pocket.operator_review_label ? ` · operator: ${label(pocket.operator_review_label)}` : ""} · ${label(pocket.status)}`}</title>
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
          {selection && <g className="workspace-selection-marker">
            {selection.scanRegion && selection.kind !== "pocket" && <rect
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
          <p className="eyebrow">
            {["feeder", "feeder_slot"].includes(selection.kind)
              ? "Feeder lane" : label(selection.kind)}
          </p>
          <h3>{selection.label}</h3>
          <p className="workspace-selection-coordinate">
            X {selection.point[0].toFixed(2)} · Y {selection.point[1].toFixed(2)} mm
          </p>
          {selectedFeeder && <dl className="workspace-facts compact workspace-feeder-facts">
            <div><dt>State</dt><dd>{selectionAction === "scanning"
              ? "Scanning"
              : selectedFeeder.feeder.status === "ready"
                ? "Ready"
                : selectedFeeder.feeder.vision_review_required
                  || selectedFeeder.feeder.status === "vision_draft"
                  ? "Needs review"
                  : "Tape detected"}</dd></div>
            <div><dt>Part</dt><dd>{selectedFeeder.feeder.part_key ?? "Not assigned"}</dd></div>
            <div><dt>Tape</dt><dd>{selectedFeeder.feeder.tape_width_mm.toFixed(0)} mm / {selectedFeeder.feeder.tape_pitch_mm.toFixed(1)} mm pitch</dd></div>
            <div><dt>Occupancy</dt><dd>{selectedFeeder.feeder.pockets.filter(
              (pocket) => pocket.occupancy === "occupied",
            ).length} occupied · {selectedFeeder.feeder.pockets.filter(
              (pocket) => pocket.occupancy === "empty",
            ).length} empty</dd></div>
            <div><dt>Revision</dt><dd>{selectedFeeder.feeder.active_revision
              ? new Date(selectedFeeder.feeder.active_revision.created_at).toLocaleString()
              : "Initial survey"}</dd></div>
            <div><dt>Qualification</dt><dd>{label(selectedFeeder.feeder.status)}</dd></div>
          </dl>}
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
            {["feeder", "feeder_slot", "placement", "pocket", "point"].includes(selection.kind) && <button
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
          {quickCaptureUrl && <div className="workspace-quick-capture-frame">
            <img
              alt={`Top-camera capture at X ${selection.point[0].toFixed(2)}, Y ${selection.point[1].toFixed(2)}`}
              className="workspace-quick-capture"
              onError={() => setSelectionMessage("Quick capture failed.")}
              src={quickCaptureUrl}
            />
            <span aria-hidden="true" className="workspace-quick-capture-crosshair" />
          </div>}
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
          <span className="legend-feeder">Feeder lane</span>
          <span className="legend-board">Board</span>
          <span className="legend-fiducial">Machine fiducial</span>
          <span className="legend-unknown">Unknown</span>
        </div>
        <div className="workspace-legend-group">
          <strong>Position state</strong>
          <span className="legend-populated">Occupied / populated</span>
          <span className="legend-empty">Empty / pending</span>
          <span className="legend-uncertain">Uncertain / review</span>
          <span className="legend-unavailable">Reviewed unavailable</span>
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

      <section className={`card workspace-job-card ${activity.active ? "has-active-job" : ""}`}>
        <div className="card-heading">
          <div><p className="eyebrow">Activity</p><h2>{activity.active
            ? activity.active.kind === "operation"
              ? label(activity.active.title)
              : activity.active.title
            : "Machine is idle"}</h2></div>
          {activity.active && <span className={`badge state-${activity.active.status}`}>
            {label(activity.active.status)}
          </span>}
        </div>
        {activity.active ? <>
          <div className="workspace-current-step">
            <small>{activity.active.target?.kind ?? "Current phase"}</small>
            <strong>{activity.active.target?.label ?? label(activity.active.phase)}</strong>
            <span>{label(activity.active.phase)}</span>
          </div>
          {activity.active.progress && <>
          <div className="workspace-progress-label">
            <strong>{activity.active.progress.completed}/{activity.active.progress.total}</strong>
            <span>{activity.active.progress.unit}{activity.active.progress.eta_seconds !== null
              ? ` · about ${formatWorkspaceDuration(activity.active.progress.eta_seconds)} left`
              : ""}</span>
          </div>
          <progress
            value={activity.active.progress.completed}
            max={Math.max(activity.active.progress.total, 1)}
          />
          </>}
          {activity.active.message && <p>{activity.active.message}</p>}
          {activity.active.allowed_actions.length > 0 && <div
            className="workspace-activity-actions"
          >
            {activity.active.allowed_actions.map((action) => <button
              className={action.destructive ? "danger" : undefined}
              disabled={activityAction !== null}
              key={action.id}
              onClick={() => void runActivityAction(action.id)}
              type="button"
            >{activityAction === action.id ? "Working…" : action.label}</button>)}
          </div>}
          {activity.active.evidence.length > 0 && <details
            className="workspace-evidence-details"
          >
            <summary>Evidence ({activity.active.evidence.length})</summary>
            {activity.active.evidence.map((item) => <a
              href={item.url}
              key={item.url}
              rel="noreferrer"
              target="_blank"
            >{item.label}</a>)}
          </details>}
        </> : <p>Start a job or select a map target.</p>}
        {(selectionMessage || latestActivityResult) && <div
          className={`workspace-latest-result state-${latestActivityResult?.status ?? "succeeded"}`}
        >
          <small>Latest result</small>
          <strong>{selectionMessage || (latestActivityResult?.kind === "operation"
            ? label(latestActivityResult.title)
            : latestActivityResult?.title)}</strong>
          {!selectionMessage && latestActivityResult?.message
            && <span>{latestActivityResult.message}</span>}
          {!selectionMessage && latestActivityResult
            && latestActivityResult.evidence.length > 0 && <details
              className="workspace-evidence-details"
            >
              <summary>Evidence ({latestActivityResult.evidence.length})</summary>
              {latestActivityResult.evidence.map((item) => <a
                href={item.url}
                key={item.url}
                rel="noreferrer"
                target="_blank"
              >{item.label}</a>)}
            </details>}
        </div>}
        {(!activity.active || activity.active.kind === "job") && <button
          className="secondary"
          onClick={openJobs}
        >{activity.active?.kind === "job" ? "Open job setup" : "Open jobs"}</button>}
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
            <div><dt>Aligned updates</dt><dd>{visibleDisplayPatches.length}</dd></div>
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
