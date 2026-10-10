// @vitest-environment jsdom

import { cleanup, render, screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { afterEach, describe, expect, it, vi } from "vitest";
import { api } from "./api";
import { DialogProvider } from "./Dialog";
import { FeedersPanel } from "./FeedersPanel";
import type {
  MachineStatus, StripFeederMapRegistration, StripFeederOccupancyReviewQueue,
  StripFeederRegistration, WorkspaceBoardDiscoveryReport, WorkspaceScanSummary,
} from "./types";

afterEach(() => {
  cleanup();
  vi.restoreAllMocks();
});

const status: MachineStatus = {
  state: "ready",
  simulated: true,
  profile_id: "lumenpnp-v3.0.2",
  pose: { x_mm: 40, y_mm: 40, z_mm: 30, rotation_deg: 0 },
  homed: true,
  fault: null,
  control_lease: null,
  calibration_revision: "calibration-id",
  controller: null,
};

const scan = {
  id: "scan-id",
  profile_id: status.profile_id,
  created_at: "2026-10-07T18:00:00Z",
  updated_at: "2026-10-07T18:01:00Z",
  status: "succeeded",
  mode: "reconnaissance",
  parent_scan_id: null,
  region: { x_min_mm: 0, x_max_mm: 100, y_min_mm: 0, y_max_mm: 80 },
  raster_transform: null,
  planned_tiles: 10,
  captured_tiles: 10,
  mosaic_width_px: 100,
  mosaic_height_px: 80,
  display_artifact_name: "mosaic.png",
  height_map_available: true,
  refinement_composite: null,
  board_discovery: { report_artifact_name: "objects.json" },
  strip_feeder_inventory: null,
  change_set: null,
  failure: null,
} as WorkspaceScanSummary;

const discovery = {
  candidates: [{
    candidate_index: 2,
    region: { x_min_mm: 20, x_max_mm: 55, y_min_mm: 10, y_max_mm: 70 },
    outline_machine_mm: [[20, 10], [55, 10], [55, 70], [20, 70]],
    center_x_mm: 37.5,
    center_y_mm: 40,
    classification: { class: "feeder" },
    feeder_lane_slots: [30, 40, 50].map((xMm, slotIndex) => ({
      slot_index: slotIndex,
      centerline_machine_mm: [[xMm, 10], [xMm, 70]],
      outline_machine_mm: [
        [xMm - 4, 10], [xMm + 4, 10],
        [xMm + 4, 70], [xMm - 4, 70],
      ],
      width_mm: 8,
      length_mm: 60,
      rotation_deg: 90,
      confidence: .9,
      review_required: false,
    })),
  }],
} as unknown as WorkspaceBoardDiscoveryReport;

const mapFeeder: StripFeederMapRegistration = {
  id: "lane-three",
  profile_id: status.profile_id,
  updated_at: "2026-10-07T18:02:00Z",
  status: "ready",
  name: "Inventory feeder 7",
  part_key: "R10K",
  part_rotation_deg: 0,
  tape_width_mm: 8,
  tape_pitch_mm: 4,
  tape_rotation_deg: 90,
  pickup_z_mm: 12.3,
  pickup_sequence_passed: true,
  latest_live_holdout_id: "holdout-id",
  latest_pickup_qualification_id: "qualification-id",
  latest_placement_validation_id: null,
  vision_review_required: false,
  active_revision: {
    id: "revision-id",
    source_scan_id: "rescan-id",
    created_at: "2026-10-07T18:02:00Z",
    mosaic_artifact_name: "mosaic.png",
    vision_confidence: .98,
  },
  revision_count: 1,
  pockets: [0, 1, 2].map((pocketIndex) => ({
    pocket_index: pocketIndex,
    center_machine_mm: [50.5, 20 + pocketIndex * 4],
    occupancy: "occupied",
    occupancy_confidence: .98,
    status: "available",
    selected: pocketIndex === 0,
  })),
};

const registration = {
  ...mapFeeder,
  calibration_id: "calibration-id",
  source_scan_id: "scan-id",
  source_candidate_index: 2,
  source_proposal_index: 0,
  created_at: "2026-10-07T18:00:00Z",
  pick_centerline_machine_mm: [[50.5, 10], [50.5, 70]],
  pocket_centers_machine_mm: mapFeeder.pockets.map((pocket) => pocket.center_machine_mm),
  pockets: mapFeeder.pockets.map((pocket) => ({
    ...pocket,
    nominal_center_machine_mm: pocket.center_machine_mm,
    measured_center_machine_mm: pocket.center_machine_mm,
    attempt_count: 0,
    consecutive_failure_count: 0,
    reservation_id: null,
    last_qualification_id: null,
    last_outcome: null,
    updated_at: "2026-10-07T18:02:00Z",
  })),
  latest_pickup_sequence_qualification_ids: [],
  findings: [],
} as unknown as StripFeederRegistration;

describe("feeder setup journey", () => {
  it("reviews an uncertain pocket and persists the operator label", async () => {
    const reviewFeeder: StripFeederMapRegistration = {
      ...mapFeeder,
      vision_review_required: true,
      pockets: mapFeeder.pockets.map((pocket) => pocket.pocket_index === 1 ? {
        ...pocket,
        occupancy: "uncertain",
        occupancy_review_required: true,
        status: "needs_review",
      } : pocket),
    };
    const reviewRegistration = {
      ...registration,
      vision_review_required: true,
      pockets: registration.pockets.map((pocket) => pocket.pocket_index === 1 ? {
        ...pocket,
        occupancy: "uncertain",
        status: "needs_review",
      } : pocket),
    } as StripFeederRegistration;
    const queue: StripFeederOccupancyReviewQueue = {
      registration_id: reviewFeeder.id,
      lane_revision_id: "revision-id",
      source_scan_id: "rescan-id",
      created_at: "2026-10-07T18:02:00Z",
      unresolved_count: 1,
      mosaic: {
        artifact_name: "mosaic.png",
        sha256: "a".repeat(64),
        width_px: 200,
        height_px: 800,
        raster_transform: {
          top_left_x_mm: 45,
          top_left_y_mm: 75,
          x_mm_per_pixel: .05,
          y_mm_per_pixel: -.05,
        },
      },
      pockets: reviewFeeder.pockets.map((pocket) => ({
        pocket_index: pocket.pocket_index,
        center_machine_mm: pocket.center_machine_mm,
        original_occupancy: pocket.occupancy,
        original_confidence: pocket.occupancy_confidence,
        model_prediction: null,
        automatic_review_required: pocket.pocket_index === 1,
        unresolved: pocket.pocket_index === 1,
        findings: pocket.pocket_index === 1
          ? ["audit-only deterministic occupancy agreed with the model"]
          : [],
        crop_artifact_name: null,
        crop_sha256: null,
        effective_occupancy: pocket.occupancy,
        effective_status: pocket.status,
        review: null,
      })),
    };
    const reviewedQueue = {
      ...queue,
      unresolved_count: 0,
      pockets: queue.pockets.map((pocket) => pocket.pocket_index === 1 ? {
        ...pocket,
        unresolved: false,
        effective_occupancy: "empty" as const,
        effective_status: "empty" as const,
        review: {
          id: "review-id",
          registration_id: reviewFeeder.id,
          lane_revision_id: queue.lane_revision_id,
          source_scan_id: queue.source_scan_id,
          pocket_index: 1,
          label: "empty" as const,
          note: null,
          reviewer: "web_ui",
          created_at: "2026-10-07T18:03:00Z",
          supersedes_review_id: null,
          source_mosaic_artifact_name: "mosaic.png",
          source_mosaic_sha256: "a".repeat(64),
          crop_artifact_name: null,
          crop_sha256: null,
          original_occupancy: "uncertain" as const,
          original_review_required: true,
          original_model_prediction: null,
          corrected_outline_px: null,
        },
      } : pocket),
    };
    const reviewedFeeder = {
      ...reviewFeeder,
      vision_review_required: false,
      pockets: reviewFeeder.pockets.map((pocket) => pocket.pocket_index === 1 ? {
        ...pocket,
        occupancy: "empty" as const,
        occupancy_review_required: false,
        operator_review_label: "empty" as const,
        status: "empty" as const,
      } : pocket),
    };
    vi.spyOn(api, "workspaceScanSummaries").mockResolvedValue([scan]);
    vi.spyOn(api, "workspaceScanArtifact").mockResolvedValue(discovery);
    vi.spyOn(api, "stripFeederPocketMap")
      .mockResolvedValueOnce([reviewFeeder])
      .mockResolvedValue([reviewedFeeder]);
    vi.spyOn(api, "stripFeederRegistrations").mockResolvedValue([reviewRegistration]);
    vi.spyOn(api, "stripFeederRevisions").mockResolvedValue([]);
    vi.spyOn(api, "parts").mockResolvedValue([]);
    vi.spyOn(api, "stripFeederOccupancyReview").mockResolvedValue(queue);
    const save = vi.spyOn(api, "reviewStripFeederPocketOccupancy")
      .mockResolvedValue({
        review: reviewedQueue.pockets[1].review!,
        registration: reviewRegistration,
      });
    const user = userEvent.setup();

    render(<DialogProvider><FeedersPanel
      status={status}
      setError={vi.fn()}
      openWorkspace={vi.fn()}
    /></DialogProvider>);

    await user.click(await screen.findByRole("button", { name: "Needs review (1)" }));
    expect(await screen.findByText("Pocket occupancy review")).toBeTruthy();
    expect(screen.getByText("1 unresolved")).toBeTruthy();
    expect(screen.getByText(
      "Audit-only deterministic occupancy agreed with the model",
    )).toBeTruthy();
    expect(screen.getByRole("button", { name: "0" }).className)
      .toContain("is-occupied");
    expect(screen.getByRole("button", { name: "1" }).className)
      .toContain("is-unresolved");
    expect(screen.getByRole("button", { name: "Save and Close" })).toBeTruthy();
    expect(screen.getByRole("button", { name: "Discard Changes" })).toBeTruthy();
    await user.click(screen.getByRole("button", { name: "Empty", pressed: false }));
    expect(screen.getByText("0 unresolved")).toBeTruthy();
    expect(screen.getByRole("button", { name: "1" }).className)
      .toContain("is-empty");
    expect(save).not.toHaveBeenCalled();
    await user.click(screen.getByRole("button", { name: "Discard Changes" }));
    expect(screen.queryByText("Pocket occupancy review")).toBeNull();
    expect(save).not.toHaveBeenCalled();

    await user.click(screen.getByRole("button", { name: "Needs review (1)" }));
    const pocketZero = screen.getByRole("button", { name: "0" });
    await user.click(pocketZero);
    expect(screen.getByRole("button", { name: "Occupied", pressed: true })).toBeTruthy();
    await user.keyboard("{ArrowRight}");
    expect(document.activeElement).toBe(screen.getByRole("button", { name: "1" }));
    expect(screen.getByText("Pocket 1")).toBeTruthy();
    await user.click(screen.getByRole("button", { name: "Empty", pressed: false }));
    expect(save).not.toHaveBeenCalled();
    await user.click(screen.getByRole("button", { name: "Save and Close" }));
    await waitFor(() => expect(save).toHaveBeenCalledWith(
      reviewFeeder.id, 1, "revision-id", "empty", "",
    ));
    expect(screen.queryByText("Pocket occupancy review")).toBeNull();
    expect(await screen.findByRole("button", { name: "Review pockets" })).toBeTruthy();
  });

  it("restores active lane scan progress after the pane opens", async () => {
    const activeScan: WorkspaceScanSummary = {
      ...scan,
      id: "active-lane-scan",
      created_at: "2026-10-07T18:03:00Z",
      updated_at: "2026-10-07T18:04:00Z",
      status: "running",
      mode: "survey",
      parent_scan_id: scan.id,
      planned_tiles: 153,
      captured_tiles: 36,
      display_artifact_name: null,
      height_map_available: false,
    };
    vi.spyOn(api, "workspaceScanSummaries").mockResolvedValue([activeScan, scan]);
    vi.spyOn(api, "workspaceScanArtifact").mockResolvedValue(discovery);
    vi.spyOn(api, "stripFeederPocketMap").mockResolvedValue([mapFeeder]);
    vi.spyOn(api, "stripFeederRegistrations").mockResolvedValue([registration]);
    vi.spyOn(api, "stripFeederRevisions").mockResolvedValue([]);
    vi.spyOn(api, "parts").mockResolvedValue([]);

    const { container } = render(<DialogProvider><FeedersPanel
      status={status}
      setError={vi.fn()}
      openWorkspace={vi.fn()}
    /></DialogProvider>);

    expect(await screen.findByText("24%")).toBeTruthy();
    expect(screen.getByText("36 of 153 tiles")).toBeTruthy();
    expect(container.querySelector(
      ".workspace-selection-progress .workspace-progress-label",
    )).toBeTruthy();
  });

  it("shows stable lanes, changes assignment, and opens an empty lane", async () => {
    vi.spyOn(api, "workspaceScanSummaries").mockResolvedValue([scan]);
    vi.spyOn(api, "workspaceScanArtifact").mockResolvedValue(discovery);
    vi.spyOn(api, "stripFeederPocketMap").mockResolvedValue([mapFeeder]);
    vi.spyOn(api, "stripFeederRegistrations").mockResolvedValue([registration]);
    vi.spyOn(api, "stripFeederRevisions").mockResolvedValue([{
      source_scan_id: "rescan-id",
      created_at: "2026-10-07T18:02:00Z",
      model_available: true,
      pocket_count: 3,
      occupied_count: 3,
      empty_count: 0,
      review_count: 0,
      lane_revision: {
        id: "revision-id",
        mosaic_artifact_name: "mosaic.png",
        vision_confidence: .98,
      },
      warnings: [],
    }]);
    vi.spyOn(api, "parts").mockResolvedValue([
      { id: "part-r", key: "R10K", manufacturer_part_number: null, package: "0603" },
      { id: "part-c", key: "C100N", manufacturer_part_number: null, package: "0603" },
    ]);
    const update = vi.spyOn(api, "updateStripFeederAssignment")
      .mockResolvedValue(registration);
    const openWorkspace = vi.fn();
    const user = userEvent.setup();

    render(<DialogProvider><FeedersPanel
      status={status}
      setError={vi.fn()}
      openWorkspace={openWorkspace}
    /></DialogProvider>);

    expect(await screen.findByRole("heading", {
      name: "Feeder Bank 1 - lane 3",
    })).toBeTruthy();
    expect(screen.queryByText("Inventory feeder 7")).toBeNull();
    expect(screen.getByRole("button", { name: "Rescan lane" })).toBeTruthy();
    await user.selectOptions(screen.getByLabelText("Part"), "C100N");
    await user.clear(screen.getByLabelText("Part rotation"));
    await user.type(screen.getByLabelText("Part rotation"), "90");
    await user.click(screen.getByRole("button", { name: "Save assignment" }));
    await waitFor(() => expect(update).toHaveBeenCalledWith(
      "lane-three", "C100N", 90,
    ));

    await user.click(screen.getByRole("button", {
      name: /Feeder Bank 1 - lane 1/,
    }));
    await user.click(screen.getByRole("button", {
      name: "Load tape and rescan in Workspace",
    }));
    expect(openWorkspace).toHaveBeenCalledWith("candidate-2-slot-0");
  });

  it("checks lane alignment automatically before qualification", async () => {
    const unalignedFeeder = {
      ...mapFeeder,
      latest_live_holdout_id: null,
      status: "live_validation_required" as const,
    };
    const unalignedRegistration = {
      ...registration,
      latest_live_holdout_id: null,
      status: "live_validation_required" as const,
    };
    vi.spyOn(api, "workspaceScanSummaries").mockResolvedValue([scan]);
    vi.spyOn(api, "workspaceScanArtifact").mockResolvedValue(discovery);
    vi.spyOn(api, "stripFeederPocketMap").mockResolvedValue([unalignedFeeder]);
    vi.spyOn(api, "stripFeederRegistrations")
      .mockResolvedValue([unalignedRegistration]);
    vi.spyOn(api, "stripFeederRevisions").mockResolvedValue([]);
    vi.spyOn(api, "parts").mockResolvedValue([]);
    vi.spyOn(api, "acquireOrRenewLease").mockResolvedValue({
      id: "qualification-lease",
      client_name: "test",
      acquired_at: "2026-10-07T18:00:00Z",
      expires_at: "2026-10-07T19:00:00Z",
    });
    const align = vi.spyOn(api, "startStripFeederLiveHoldout")
      .mockResolvedValue({
        holdout: { id: "alignment-id" },
        operation: {
          id: "alignment-operation",
          kind: "vision.strip_feeder_live_holdout",
          status: "succeeded",
          message: "Lane alignment passed.",
        },
      } as never);
    const qualify = vi.spyOn(api, "startStripFeederPickupQualification")
      .mockImplementation(async (_registrationId, _leaseId, _nozzleId,
        _fallback, pocketIndex) => ({
        qualification: { id: `qualification-${pocketIndex}` },
        operation: {
          id: `qualification-operation-${pocketIndex}`,
          kind: "vision.strip_feeder_pickup_qualification",
          status: "succeeded",
          message: "Qualification passed.",
        },
      } as never));
    vi.spyOn(api, "stripFeederPickupQualification").mockResolvedValue({
      report: { passed: true },
    } as never);
    const user = userEvent.setup();

    render(<DialogProvider><FeedersPanel
      status={status}
      setError={vi.fn()}
      openWorkspace={vi.fn()}
    /></DialogProvider>);

    await user.click(await screen.findByRole(
      "button", { name: "Qualify three pockets" },
    ));
    expect(await screen.findByText(
      /The machine will first check lane alignment/,
    )).toBeTruthy();
    await user.click(screen.getByRole("button", { name: "Start qualification" }));
    await waitFor(() => expect(qualify).toHaveBeenCalledTimes(3));
    expect(align).toHaveBeenCalledWith("lane-three", "qualification-lease");
  });

  it("rescans a registered lane and promotes model occupancy", async () => {
    vi.spyOn(api, "workspaceScanSummaries").mockResolvedValue([scan]);
    vi.spyOn(api, "workspaceScanArtifact").mockResolvedValue(discovery);
    vi.spyOn(api, "stripFeederPocketMap").mockResolvedValue([mapFeeder]);
    vi.spyOn(api, "stripFeederRegistrations").mockResolvedValue([registration]);
    vi.spyOn(api, "stripFeederRevisions").mockResolvedValue([]);
    vi.spyOn(api, "parts").mockResolvedValue([]);
    vi.spyOn(api, "workspaceScan").mockResolvedValue({
      ...scan,
      operation_id: "base-operation",
      calibration_id: "calibration-id",
      artifact_retention: "full_sources",
      rows: 1,
      columns: 1,
      estimated_duration_seconds: 10,
      estimated_retained_bytes: 100,
      retained_artifact_bytes: 100,
      tiles: [],
      mosaic_artifact_name: "mosaic.png",
      mosaic_layers: [],
      height_map: null,
      seam_health: null,
      warnings: [],
    } as never);
    vi.spyOn(api, "estimateWorkspaceSurvey").mockResolvedValue({
      planned_tiles: 4,
      planned_registrations: 4,
      mosaic_width_px: 100,
      mosaic_height_px: 400,
      estimated_duration_seconds: 20,
      estimated_retained_bytes: 1000,
      artifact_quota_bytes: 10_000,
      persistent_capture: true,
      warnings: [],
    } as never);
    vi.spyOn(api, "acquireOrRenewLease").mockResolvedValue({
      id: "rescan-lease",
      client_name: "test",
      acquired_at: "2026-10-07T18:00:00Z",
      expires_at: "2026-10-07T19:00:00Z",
    });
    vi.spyOn(api, "startWorkspaceSurvey").mockResolvedValue({
      scan: {
        id: "lane-rescan",
        captured_tiles: 4,
        planned_tiles: 4,
        estimated_duration_seconds: 20,
      },
      operation: {
        id: "rescan-operation",
        kind: "workspace.scan",
        status: "succeeded",
        message: null,
      },
    } as never);
    vi.spyOn(api, "discoverWorkspaceObjects").mockResolvedValue(discovery);
    vi.spyOn(api, "analyzeStripFeeder").mockResolvedValue({
      proposals: [{
        proposal_index: 0,
        pick_centerline_machine_mm: [[50.5, 10], [50.5, 70]],
      }],
    } as never);
    const refresh = vi.spyOn(api, "refreshStripFeederOccupancy")
      .mockResolvedValue({
        model_available: true,
        observations: [0, 1, 2].map((pocketIndex) => ({
          pocket_index: pocketIndex,
          occupancy: "occupied",
          review_required: false,
          model_prediction: {},
        })),
      } as never);
    const align = vi.spyOn(api, "startStripFeederLiveHoldout")
      .mockResolvedValue({
        holdout: { id: "alignment-id" },
        operation: {
          id: "alignment-operation",
          kind: "vision.strip_feeder_live_holdout",
          status: "succeeded",
          message: "Lane alignment passed.",
        },
      } as never);
    const user = userEvent.setup();
    const setError = vi.fn();

    render(<DialogProvider><FeedersPanel
      status={status}
      setError={setError}
      openWorkspace={vi.fn()}
    /></DialogProvider>);

    await user.click(await screen.findByRole("button", { name: "Rescan lane" }));
    expect(await screen.findByText(
      /Estimated scan time is about 20 sec for 4 tiles/,
    )).toBeTruthy();
    await user.click(screen.getByRole("button", { name: "Start rescan" }));
    await waitFor(() => expect(refresh).toHaveBeenCalledWith(
      "lane-rescan", 2, "lane-three",
    ), { onTimeout: (error) => new Error(
      `${error.message}\nErrors: ${JSON.stringify(setError.mock.calls)}`,
    ) });
    expect(align).toHaveBeenCalledWith("lane-three", "rescan-lease");
    expect(await screen.findByText(
      /Rescan complete. 3 pocket states were refreshed.*Lane alignment passed automatically/,
    )).toBeTruthy();
  });
});
