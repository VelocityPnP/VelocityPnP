// @vitest-environment jsdom

import { cleanup, render, screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { afterEach, describe, expect, it, vi } from "vitest";
import { api } from "./api";
import { DialogProvider } from "./Dialog";
import type {
  BoardDesign, Job, JobExecutionRun, MachineStatus, PlacementExecutionRun,
  PlacementOutcomeRecord, StripFeederMapRegistration, WorkspaceDisplayPatch,
  WorkspaceScanEstimate, WorkspaceScanReport, WorkspaceScanSummary,
} from "./types";
import {
  feederScanRegion, formatWorkspaceDuration, jobPlacementMarkers,
  latestFeederLanes, panWorkspaceViewport, selectActiveJobExecution,
  selectWorkspaceJob, workspaceArtifactOptions, workspaceComposition,
  workspaceMoveAnimationDuration, workspaceRescanEtaSeconds, WorkspacePanel,
  zoomWorkspaceViewport,
} from "./WorkspacePanel";

const scan: WorkspaceScanReport = {
  id: "scan-id",
  operation_id: "scan-operation",
  calibration_id: "calibration-id",
  profile_id: "lumenpnp-v3.0.2",
  parent_scan_id: null,
  created_at: "2026-10-03T18:00:00Z",
  updated_at: "2026-10-03T18:01:00Z",
  status: "succeeded",
  mode: "reconnaissance",
  artifact_retention: "mosaic_only",
  region: { x_min_mm: 0, x_max_mm: 100, y_min_mm: 0, y_max_mm: 80 },
  raster_transform: {
    top_left_x_mm: 0, top_left_y_mm: 80,
    x_mm_per_pixel: 1, y_mm_per_pixel: -1,
  },
  rows: 1,
  columns: 2,
  planned_tiles: 2,
  captured_tiles: 2,
  mosaic_width_px: 100,
  mosaic_height_px: 80,
  covered_fraction: .95,
  estimated_duration_seconds: 90,
  estimated_retained_bytes: 1024,
  retained_artifact_bytes: 900,
  tiles: [
    {
      index: 0, row: 0, column: 0, capture_order: 0,
      nominal_pose: { x_mm: 25, y_mm: 40, z_mm: 30, rotation_deg: 0 },
      placement_correction_x_mm: 0,
      placement_correction_y_mm: 0,
      artifact_name: "tile-0.png",
    },
    {
      index: 1, row: 0, column: 1, capture_order: 1,
      nominal_pose: { x_mm: 75, y_mm: 40, z_mm: 30, rotation_deg: 0 },
      placement_correction_x_mm: 0,
      placement_correction_y_mm: 0,
      artifact_name: "tile-1.png",
    },
  ],
  mosaic_artifact_name: "mosaic.png",
  mosaic_layers: [],
  height_map: null,
  seam_health: null,
  refinement_composite: null,
  board_discovery: null,
  strip_feeder_inventory: null,
  change_set: null,
  warnings: [],
  failure: null,
};

const scanSummary: WorkspaceScanSummary = {
  id: scan.id,
  profile_id: scan.profile_id,
  created_at: scan.created_at,
  updated_at: scan.updated_at,
  status: scan.status,
  mode: scan.mode,
  parent_scan_id: scan.parent_scan_id,
  region: scan.region,
  raster_transform: scan.raster_transform,
  planned_tiles: scan.planned_tiles,
  captured_tiles: scan.captured_tiles,
  mosaic_width_px: scan.mosaic_width_px,
  mosaic_height_px: scan.mosaic_height_px,
  display_artifact_name: scan.mosaic_artifact_name,
  height_map_available: scan.height_map !== null,
  refinement_composite: null,
  board_discovery: scan.board_discovery,
  strip_feeder_inventory: scan.strip_feeder_inventory,
  change_set: scan.change_set,
  failure: scan.failure,
};

const surveyEstimate: WorkspaceScanEstimate = {
  mode: "survey",
  artifact_retention: "transient_sources",
  surface_model: "top_camera_working_plane",
  surface_models: ["top_camera_working_plane", "installed_plate_planes"],
  region: scan.region,
  raster_transform: scan.raster_transform!,
  overlap_fraction: .2,
  output_mm_per_pixel: .05,
  feed_mm_per_min: 23_000,
  capture_settle_frames: 3,
  rows: 4,
  columns: 6,
  planned_tiles: 24,
  planned_registrations: 38,
  mosaic_width_px: 2000,
  mosaic_height_px: 1600,
  estimated_duration_seconds: 75,
  estimated_retained_bytes: 1024,
  artifact_quota_bytes: 1024 * 1024,
  persistent_capture: true,
  warnings: [],
};

const displayPatch: WorkspaceDisplayPatch = {
  id: "display-patch",
  algorithm_version: "workspace_display_patch.v1",
  base_scan_id: scan.id,
  source_scan_id: "local-update",
  profile_id: scan.profile_id,
  calibration_id: scan.calibration_id,
  kind: "board",
  object_id: "board-instance",
  object_label: "Main PCB",
  artifact_name: "mosaic.png",
  source_raster_transform: {
    top_left_x_mm: 20,
    top_left_y_mm: 60,
    x_mm_per_pixel: 1,
    y_mm_per_pixel: -1,
  },
  source_width_px: 20,
  source_height_px: 40,
  source_to_base: { matrix: [1, 0, .5, 0, 1, -.25, 0, 0, 1] },
  clip_polygon_machine_mm: [[20, 20], [40, 20], [40, 40], [20, 40]],
  anchor_count: 3,
  alignment_residuals: {
    sample_count: 3, rms_mm: .02, p95_mm: .03, max_mm: .03,
  },
  accepted: true,
  rejection_reason: null,
  findings: ["Matched 3 registered board fiducials"],
  source_created_at: "2026-10-03T18:02:00Z",
  evaluated_at: "2026-10-03T18:03:00Z",
};

const feederRegistration: StripFeederMapRegistration = {
  id: "feeder-registration",
  profile_id: scan.profile_id,
  updated_at: "2026-10-03T18:02:00Z",
  status: "ready",
  name: "Feeder 1",
  tape_width_mm: 8,
  tape_rotation_deg: 90,
  pockets: [
    {
      pocket_index: 0,
      center_machine_mm: [30, 30],
      occupancy: "occupied",
      occupancy_confidence: .98,
      status: "available",
      selected: true,
    },
    {
      pocket_index: 1,
      center_machine_mm: [30, 34],
      occupancy: "empty",
      occupancy_confidence: .95,
      status: "empty",
      selected: false,
    },
  ],
};

const job: Job = {
  id: "job-id",
  name: "Demo assembly",
  boards: [{
    id: "board-instance",
    board_id: "board-design",
    label: "Main PCB",
    side: "top",
    design_to_machine: { matrix: [1, 0, 20, 0, 1, 20, 0, 0, 1] },
    surface_z_mm: 1,
    placement_surface_offset_mm: 0,
  }],
  placement_references: [],
  placement_seating_offset_mm: null,
  placement_release: null,
  status: "ready",
};

const design: BoardDesign = {
  id: "board-design",
  name: "Main PCB",
  source_version: 20240108,
  width_mm: 40,
  height_mm: 20,
  outline_bounds: { min_x_mm: 0, max_x_mm: 40, min_y_mm: 0, max_y_mm: 20 },
  placements: [{
    reference: "R1",
    value: "10k",
    package: "R_0603",
    side: "top",
    pose: { x_mm: 10, y_mm: 5, z_mm: 0, rotation_deg: 0 },
    enabled: true,
    fiducial: false,
    properties: {},
  }],
  warnings: [],
};

const execution: JobExecutionRun = {
  id: "execution-id",
  operation_id: "job-operation",
  job_id: job.id,
  calibration_id: "calibration-id",
  simulated: true,
  status: "running",
  created_at: "2026-10-03T18:02:00Z",
  updated_at: "2026-10-03T18:03:00Z",
  steps: [{
    step_index: 0,
    placement_reference: "R1",
    status: "running",
    execution_id: null,
    attempt_execution_ids: [],
    outcome_disposition: null,
    board_contacted: false,
    message: null,
  }],
  current_step_index: 0,
  pause_requested: false,
  cancel_requested: false,
  message: null,
};

const placementOutcome: PlacementOutcomeRecord = {
  id: "placement-outcome",
  execution_id: "placement-execution",
  job_id: job.id,
  placement_reference: "R1",
  part_key: "resistor-10k",
  package: "R_0603",
  nozzle_id: "N1",
  placement_surface_offset_mm: 0,
  placement_seating_offset_mm: 0,
  automatic_algorithm_version: "placement_outcome.v2",
  automatic_disposition: "good",
  automatic_confidence: .99,
  operator_disposition: null,
  operator_note: null,
  measured_offset_x_mm: null,
  measured_offset_y_mm: null,
  measured_rotation_error_deg: null,
  created_at: "2026-10-03T18:02:00Z",
  updated_at: "2026-10-03T18:03:00Z",
};

const status: MachineStatus = {
  state: "running",
  simulated: true,
  profile_id: "lumenpnp-v3.0.2",
  pose: { x_mm: 45, y_mm: 35, z_mm: 30, rotation_deg: 0 },
  homed: true,
  fault: null,
  control_lease: null,
  calibration_revision: "calibration-id",
  controller: null,
};

afterEach(() => {
  cleanup();
  vi.restoreAllMocks();
  vi.unstubAllGlobals();
});

describe("workspace operational map", () => {
  it("selects the newest unfinished job", () => {
    expect(selectActiveJobExecution([
      { ...execution, status: "succeeded", updated_at: "2026-10-03T19:00:00Z" },
      execution,
    ])?.id).toBe(execution.id);
  });

  it("uses placement history for the current board when no job is active", () => {
    const finishedExecution = { ...execution, status: "succeeded" as const };

    expect(selectWorkspaceJob([job], [finishedExecution], [placementOutcome]))
      .toBe(job);
    expect(jobPlacementMarkers(
      job, null, [design], [job], [placementOutcome],
    )[0].status).toBe("populated");
  });

  it("maps intended placement rotation into machine coordinates", () => {
    const rotatedDesign = {
      ...design,
      placements: design.placements.map((placement) => ({
        ...placement,
        pose: { ...placement.pose, rotation_deg: 45 },
      })),
    };

    expect(jobPlacementMarkers(
      job, execution, [rotatedDesign], [job], [placementOutcome],
    )[0].rotationDeg).toBe(45);
  });

  it("orders useful image layers and removes duplicate artifacts", () => {
    const refined: WorkspaceScanReport = {
      ...scan,
      height_map: {
        corrected_mosaic_artifact_name: "corrected.png",
        preview_artifact_name: "height.png",
        class_preview_artifact_name: "classes.png",
        measured_cells: 10,
        inferred_cells: 2,
        unknown_cells: 0,
      },
      refinement_composite: {
        accepted: true,
        mosaic_artifact_name: "corrected.png",
        quality_preview_artifact_name: "quality.png",
      },
    };

    expect(workspaceArtifactOptions(refined).map((artifact) => artifact.name))
      .toEqual(["corrected.png", "mosaic.png", "classes.png"]);
    expect(workspaceArtifactOptions(refined)[0].label)
      .toBe("Full scan + refinements");
  });

  it("zooms around the pointer and stays inside the workspace", () => {
    const full = { x: 0, y: -80, width: 100, height: 80 };

    expect(zoomWorkspaceViewport(full, full, [50, -40], .5)).toEqual({
      x: 25, y: -60, width: 50, height: 40,
    });
    expect(zoomWorkspaceViewport(
      { x: 75, y: -40, width: 25, height: 20 }, full, [100, -20], 10,
    )).toEqual(full);
  });

  it("pans inside the workspace limits", () => {
    const full = { x: 0, y: -80, width: 100, height: 80 };

    expect(panWorkspaceViewport(
      { x: 25, y: -60, width: 50, height: 40 }, full, -30, 30,
    )).toEqual({ x: 0, y: -40, width: 50, height: 40 });
  });

  it("animates clear travel at the configured machine feed", () => {
    const from = { x_mm: 0, y_mm: 0, z_mm: 31.5, rotation_deg: 0 };

    expect(workspaceMoveAnimationDuration(
      from, { ...from, x_mm: 191.667 }, 23_000,
    )).toBeCloseTo(500, 0);
    expect(workspaceMoveAnimationDuration(from, from, 23_000)).toBe(100);
    expect(workspaceMoveAnimationDuration(
      from, { ...from, x_mm: 1000 }, 300,
    )).toBe(1200);
  });

  it("formats the scan estimate and updates its ETA from tile progress", () => {
    expect(formatWorkspaceDuration(167)).toBe("2 min 47 sec");
    expect(workspaceRescanEtaSeconds(0, 20, 100, 10)).toBe(90);
    expect(workspaceRescanEtaSeconds(5, 20, 100, 50)).toBe(150);
    expect(workspaceRescanEtaSeconds(20, 20, 100, 90)).toBe(10);
  });

  it("keeps only the newest registration for each physical feeder lane", () => {
    const olderSameLane = {
      ...feederRegistration,
      id: "older-same-lane",
      updated_at: "2026-10-03T18:01:30Z",
    };
    const adjacentLane = {
      ...feederRegistration,
      id: "adjacent-lane",
      updated_at: "2026-10-03T18:01:45Z",
      pockets: feederRegistration.pockets.map((pocket) => ({
        ...pocket,
        center_machine_mm: [42, pocket.center_machine_mm[1]] as [number, number],
      })),
    };

    const baseUpdatedAfterFeeders = {
      ...scanSummary,
      updated_at: "2026-10-04T18:00:00Z",
    };
    expect(latestFeederLanes(
      [olderSameLane, adjacentLane, feederRegistration],
      scan.profile_id,
      baseUpdatedAfterFeeders,
    ).map((registration) => registration.id)).toEqual([
      feederRegistration.id, adjacentLane.id,
    ]);
  });

  it("uses the full feeder lane for feeder and pocket rescans", () => {
    expect(feederScanRegion(feederRegistration, scan.region)).toEqual({
      x_min_mm: 20,
      x_max_mm: 40,
      y_min_mm: 20,
      y_max_mm: 44,
    });
  });

  it("uses a full scan as the base and layers newer local scans", () => {
    const base = {
      ...scanSummary,
      id: "full-workspace",
      updated_at: "2026-10-03T18:01:00Z",
      region: { x_min_mm: 0, x_max_mm: 400, y_min_mm: 0, y_max_mm: 250 },
    };
    const feeder = {
      ...scanSummary,
      id: "feeder-update",
      mode: "survey" as const,
      parent_scan_id: base.id,
      height_map_available: true,
      updated_at: "2026-10-03T18:02:00Z",
      region: { x_min_mm: 140, x_max_mm: 175, y_min_mm: 80, y_max_mm: 210 },
    };
    const board = {
      ...scanSummary,
      id: "board-update",
      mode: "survey" as const,
      parent_scan_id: base.id,
      refinement_composite: { accepted: true },
      updated_at: "2026-10-03T18:03:00Z",
      region: { x_min_mm: 290, x_max_mm: 380, y_min_mm: 215, y_max_mm: 305 },
    };
    const newestFeeder = {
      ...feeder,
      id: "newest-feeder-update",
      mode: "reconnaissance" as const,
      updated_at: "2026-10-03T18:04:00Z",
    };
    const uncorrectedSurvey = {
      ...feeder,
      id: "uncorrected-survey",
      height_map_available: false,
      updated_at: "2026-10-03T18:05:00Z",
    };
    const unlinkedCorrectedSurvey = {
      ...feeder,
      id: "unlinked-corrected-survey",
      parent_scan_id: null,
      updated_at: "2026-10-03T18:06:00Z",
    };

    const composition = workspaceComposition(
      [
        unlinkedCorrectedSurvey, uncorrectedSurvey,
        newestFeeder, board, feeder, base,
      ],
      base.profile_id,
    );

    expect(composition.bases.map((value) => value.id)).toEqual([base.id]);
    expect(composition.overlays.map((value) => value.id)).toEqual([
      feeder.id, board.id,
    ]);
  });

  it("renders machine, scan, board, and current-placement overlays", async () => {
    const user = userEvent.setup();
    const currentExecution = {
      ...execution,
      steps: execution.steps.map((step) => ({
        ...step, execution_id: "placement-execution",
      })),
    };
    const localUpdate: WorkspaceScanSummary = {
      ...scanSummary,
      id: "local-update",
      parent_scan_id: scan.id,
      updated_at: "2026-10-03T18:02:00Z",
      region: { x_min_mm: 20, x_max_mm: 40, y_min_mm: 20, y_max_mm: 60 },
      raster_transform: null,
      mosaic_width_px: null,
      mosaic_height_px: null,
      mode: "survey",
      height_map_available: true,
    };
    vi.spyOn(api, "workspaceScanSummaries")
      .mockResolvedValue([localUpdate, scanSummary]);
    vi.spyOn(api, "workspaceScan").mockResolvedValue({
      ...scan,
      warnings: [
        "top-camera coverage reserves 0.057 mm X and 0.055 mm Y for calibrated controller corrections",
      ],
    });
    vi.spyOn(api, "workspaceDisplayPatches").mockResolvedValue([
      displayPatch,
      { ...displayPatch, id: "rejected-patch", accepted: false },
    ]);
    vi.spyOn(api, "machineOverlayGeometry").mockResolvedValue({
      profile_id: scan.profile_id,
      calibration_id: scan.calibration_id,
      geometry_source: "calibrated",
      camera_footprint_relative_mm: {
        x_min_mm: -12, x_max_mm: 12, y_min_mm: -8, y_max_mm: 8,
      },
      nozzles: [
        { nozzle_id: "N1", offset_machine_mm: [-24, -68] },
        { nozzle_id: "N2", offset_machine_mm: [24, -68] },
      ],
    });
    vi.spyOn(api, "machineProfile").mockResolvedValue({
      id: scan.profile_id,
      transit_motion: { clear_xy_feed_mm_per_min: 23_000 },
    });
    vi.spyOn(api, "acquireOrRenewLease").mockResolvedValue({
      id: "workspace-lease",
      client_name: "test",
      acquired_at: "2026-10-03T18:00:00Z",
      expires_at: "2026-10-03T19:00:00Z",
    });
    const jog = vi.spyOn(api, "jog").mockResolvedValue({
      id: "jog-operation",
      kind: "machine.jog",
      status: "succeeded",
      message: null,
    });
    const actuate = vi.spyOn(api, "actuate").mockImplementation(
      async (_leaseId, _actuatorId, enabled) => ({
        id: `lights-${enabled ? "on" : "off"}`,
        kind: "machine.actuate",
        status: "succeeded",
        message: null,
      }),
    );
    const cameraSnapshot = vi.spyOn(api, "cameraSnapshot")
      .mockResolvedValue(new Blob(["capture"], { type: "image/png" }));
    vi.spyOn(api, "estimateWorkspaceSurvey").mockResolvedValue(surveyEstimate);
    const createObjectURL = vi.fn(() => "blob:quick-capture");
    vi.stubGlobal("URL", {
      createObjectURL,
      revokeObjectURL: vi.fn(),
    });
    vi.spyOn(api, "jobs").mockResolvedValue([job]);
    vi.spyOn(api, "jobExecutions").mockResolvedValue([currentExecution]);
    vi.spyOn(api, "placementExecution").mockResolvedValue({
      report: { nozzle_id: "N1" },
    } as unknown as PlacementExecutionRun);
    vi.spyOn(api, "placementOutcomes").mockResolvedValue([placementOutcome]);
    vi.spyOn(api, "boards").mockResolvedValue([design]);
    vi.spyOn(api, "stripFeederPocketMap").mockResolvedValue([feederRegistration]);

    const { container } = render(<DialogProvider><WorkspacePanel
      status={{ ...status, state: "ready" }}
      setError={vi.fn()}
      openMachine={vi.fn()}
      openJobs={vi.fn()}
    /></DialogProvider>);

    expect(await screen.findByRole("img", { name: "Live machine workspace map" }))
      .toBeTruthy();
    expect(await screen.findByText("Demo assembly")).toBeTruthy();
    expect(await screen.findByText("Workspace scan")).toBeTruthy();
    expect(screen.queryByText(/top-camera coverage reserves/)).toBeNull();
    expect(screen.queryByText("Estimate")).toBeNull();
    expect(screen.queryByText("Artifacts")).toBeNull();
    await waitFor(() => expect(
      container.querySelectorAll(".workspace-map-update"),
    ).toHaveLength(1));
    expect(container.querySelector(".workspace-map-update image")
      ?.getAttribute("transform")).toBe("matrix(1 0 0 1 0.5 0.25)");
    expect(container.querySelectorAll(".workspace-map defs polygon")).toHaveLength(1);
    expect(container.querySelector(".workspace-map")?.getAttribute("viewBox"))
      .toBe("0 -80 100 80");
    expect(container.querySelector(".workspace-machine-overlay")).toBeTruthy();
    expect(container.querySelector(".workspace-gantry-guide")).toBeTruthy();
    expect(container.querySelector(".workspace-gantry-extrusion")
      ?.getAttribute("height")).toBe("20");
    expect(container.querySelectorAll(".workspace-gantry-rail"))
      .toHaveLength(2);
    await waitFor(() => expect(
      container.querySelectorAll(".workspace-nozzle"),
    ).toHaveLength(2));
    expect(container.querySelector(".workspace-camera-footprint")).toBeTruthy();
    expect(container.querySelector(".workspace-top-camera")).toBeTruthy();
    expect(container.querySelector(".workspace-toolhead-chassis")).toBeTruthy();
    expect(container.querySelectorAll(".workspace-toolhead-motor"))
      .toHaveLength(2);
    expect(container.querySelector(".workspace-toolhead-link")).toBeNull();
    await waitFor(() => expect(
      container.querySelector(".workspace-nozzle.is-active title")?.textContent,
    ).toContain("N1 calibrated axis center · active"));
    expect(container.querySelector(".workspace-job-board")).toBeTruthy();
    expect(container.querySelector(".workspace-placement-position.current")).toBeTruthy();
    expect(container.querySelector(".workspace-placement-position circle")
      ?.getAttribute("r")).toBe("0.65");
    expect(container.querySelector(".workspace-placement-position.current")
      ?.getAttribute("transform")).toContain("rotate(0)");
    expect(container.querySelector(".workspace-placement-position.current line")
      ?.getAttribute("x1")).toBe("-0.65");
    expect(container.querySelectorAll(
      ".workspace-feeder-pockets circle:not(.workspace-pocket-hit)",
    ))
      .toHaveLength(2);
    expect(container.querySelectorAll(".workspace-pocket-hit")).toHaveLength(2);
    await user.click(container.querySelector(
      ".workspace-feeder-pockets > polyline:not(.workspace-feeder-hit)",
    )!);
    expect(screen.getByText("Feeder 1", { selector: ".workspace-selection-card h3" }))
      .toBeTruthy();
    await user.click(container.querySelector(
      ".workspace-feeder-pockets circle.occupied",
    )!);
    expect(screen.getByText(
      "Feeder 1 · pocket 0", { selector: ".workspace-selection-card h3" },
    )).toBeTruthy();
    const selectedPocket = container.querySelector(
      ".workspace-feeder-pockets circle.is-user-selected",
    );
    expect(selectedPocket).toBeTruthy();
    expect(selectedPocket?.getAttribute("r")).toBe("0.65");
    expect(container.querySelector(".workspace-feeder-pockets.is-selected"))
      .toBeNull();
    expect(container.querySelector(".workspace-selection-marker circle"))
      .toBeTruthy();
    expect(container.querySelectorAll(".workspace-selection-marker line"))
      .toHaveLength(2);
    expect(container.querySelector(".workspace-selection-marker rect"))
      .toBeNull();
    expect(screen.getByRole("button", { name: "Quick capture" })).toBeTruthy();
    await user.click(screen.getByRole("button", { name: "Quick capture" }));
    await waitFor(() => expect(cameraSnapshot).toHaveBeenCalledWith("top_camera"));
    expect(actuate.mock.calls.map(([, actuatorId, enabled]) => [actuatorId, enabled]))
      .toEqual([["lights", true], ["lights", false]]);
    expect(createObjectURL).toHaveBeenCalledOnce();
    expect(screen.getByRole("img", { name: /Top-camera capture/ })
      .getAttribute("src")).toBe("blob:quick-capture");
    await user.click(container.querySelector(".workspace-placement-position")!);
    expect(screen.getByRole("button", { name: "Rescan" })).toBeTruthy();
    expect(screen.getByRole("button", { name: "Quick capture" })).toBeTruthy();
    await user.click(screen.getByRole("button", { name: "Rescan" }));
    expect(await screen.findByText(
      /Estimated completion time is about 1 min 15 sec for 24 tiles/,
    )).toBeTruthy();
    await user.click(screen.getByRole("button", { name: "Cancel" }));
    await user.click(container.querySelector(".workspace-job-board")!);
    expect(screen.getByText("Main PCB", { selector: ".workspace-selection-card h3" }))
      .toBeTruthy();
    expect(screen.getByRole("button", { name: "Go to" }).hasAttribute("disabled"))
      .toBe(false);
    expect(screen.getByRole("button", { name: "Rescan" }).hasAttribute("disabled"))
      .toBe(false);
    await user.click(screen.getByRole("button", { name: "Go to" }));
    expect(screen.queryByRole("dialog")).toBeNull();
    await waitFor(() => expect(jog).toHaveBeenCalledWith(
      "workspace-lease",
      { x_mm: 40, y_mm: 30, z_mm: 30, rotation_deg: 0 },
      23_000,
    ));
    expect(screen.queryByRole("button", { name: "Tiles" })).toBeNull();
    const map = container.querySelector(".workspace-map");
    const plainWheel = new WheelEvent("wheel", {
      deltaY: -100, cancelable: true,
    });
    map?.dispatchEvent(plainWheel);
    expect(plainWheel.defaultPrevented).toBe(false);
    expect(map?.getAttribute("viewBox")).toBe("0 -80 100 80");
    const shiftedWheel = new WheelEvent("wheel", {
      deltaY: -100, cancelable: true, shiftKey: true,
    });
    map?.dispatchEvent(shiftedWheel);
    expect(shiftedWheel.defaultPrevented).toBe(true);
    await waitFor(() => expect(
      container.querySelector(".workspace-map")?.getAttribute("viewBox"),
    ).not.toBe("0 -80 100 80"));
    await user.click(screen.getByRole("button", { name: "Placement positions" }));
    await user.click(screen.getByRole("button", { name: "Feeder pockets" }));
    await user.click(screen.getByRole("button", { name: "Machine position" }));
    expect(container.querySelector(".workspace-placement-position")).toBeNull();
    expect(container.querySelector(".workspace-feeder-pockets")).toBeNull();
    expect(container.querySelector(".workspace-machine-overlay")).toBeNull();
  });
});
