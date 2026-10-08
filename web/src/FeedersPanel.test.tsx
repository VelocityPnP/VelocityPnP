// @vitest-environment jsdom

import { cleanup, render, screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { afterEach, describe, expect, it, vi } from "vitest";
import { api } from "./api";
import { DialogProvider } from "./Dialog";
import { FeedersPanel } from "./FeedersPanel";
import type {
  MachineStatus, StripFeederMapRegistration, StripFeederRegistration,
  WorkspaceBoardDiscoveryReport, WorkspaceScanSummary,
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
});
