// @vitest-environment jsdom

import { cleanup, render, screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { afterEach, describe, expect, it, vi } from "vitest";
import App, {
  attachedMachineDevices, calibrationRunSupersededBySnapshot,
  currentCalibrationCheck, latestCalibrationChecks,
} from "./App";
import { api } from "./api";
import type {
  AutomatedCalibrationRun, CalibrationArtifact, CalibrationSnapshot,
  ControlLease, Device, MachineStatus, Operation,
} from "./types";

const controlLease: ControlLease = {
  id: "lease-id",
  client_name: "VelocityPnP web UI",
  acquired_at: "2026-09-07T18:00:00Z",
  expires_at: "2026-09-07T19:00:00Z",
};

function machineStatus(overrides: Partial<MachineStatus>): MachineStatus {
  return {
    state: "disconnected",
    simulated: true,
    profile_id: "lumenpnp-v3.0.2",
    pose: null,
    homed: false,
    fault: null,
    control_lease: null,
    calibration_revision: null,
    controller: null,
    ...overrides,
  };
}

function calibrationSnapshot(
  id: string,
  parentId: string | null = null,
  completedSteps: string[] = [],
  values: Record<string, unknown> = {},
): CalibrationSnapshot {
  return {
    id,
    machine_serial: "simulator",
    profile_id: "lumenpnp-v3.0.2",
    created_at: "2026-09-07T18:00:00Z",
    completed_steps: completedSteps,
    residuals: {},
    values,
    artifacts: {},
    parent_id: parentId,
    production_ready: false,
  };
}

function operation(
  kind: string,
  status: Operation["status"] = "succeeded",
): Operation {
  return {
    id: `${kind}-operation`,
    kind,
    status,
    message: null,
  };
}

function calibrationArtifact(): CalibrationArtifact {
  return {
    kind: "captured",
    recorded_at: "2026-09-07T18:00:00Z",
    source_run_id: null,
    algorithm_version: "test",
    input_fingerprints: {},
    dependency_fingerprints: {},
    fingerprint: "test-fingerprint",
    value: {},
  };
}

function calibrationRun(
  calibrationId: string,
  status: AutomatedCalibrationRun["status"] = "rejected",
): AutomatedCalibrationRun {
  return {
    id: "calibration-run",
    calibration_id: calibrationId,
    operation_id: "calibration-operation",
    profile_id: "lumenpnp-v3.0.2",
    created_at: "2026-09-07T18:00:00Z",
    updated_at: "2026-09-07T18:01:00Z",
    status,
    phase: "top_camera_reproducibility",
    config: {
      camera_sample_count: 30,
      tooling_sample_count: 5,
      visual_homing_sample_count: 5,
      excursion_x_mm: 5,
      excursion_y_mm: 5,
      excursion_feed_mm_per_min: 600,
      visual_homing_p95_limit_mm: 0.05,
    },
    checks: [],
    camera_reports: {},
    visual_homing_samples: [],
    visual_homing_report: null,
    head_geometry_report: null,
    installed_geometry_report: null,
    failure: status === "rejected"
      ? "top-camera reproducibility did not meet its profile thresholds"
      : null,
    simulated: true,
  };
}

function mockBrowserServices() {
  vi.stubGlobal("EventSource", class {
    addEventListener() {}
    close() {}
  });
}

function mockCalibrationApi(
  snapshots: CalibrationSnapshot[],
  runs: AutomatedCalibrationRun[] = [],
) {
  vi.spyOn(api, "workflow").mockResolvedValue([]);
  vi.spyOn(api, "calibrations").mockResolvedValue(snapshots);
  vi.spyOn(api, "calibrationRuns").mockResolvedValue(runs);
  vi.spyOn(api, "calibrationPlan").mockResolvedValue({
    targets: [], items: [], requires_motion: false, estimated_duration_seconds: 0,
  });
}

async function enabledButton(name: string): Promise<HTMLButtonElement> {
  const button = await screen.findByRole("button", { name });
  await waitFor(() => expect((button as HTMLButtonElement).disabled).toBe(false));
  return button as HTMLButtonElement;
}

afterEach(() => {
  cleanup();
  vi.restoreAllMocks();
  vi.unstubAllGlobals();
});

describe("machine bootstrap action states", () => {
  it.each([
    ["a disconnected simulator", machineStatus({}), "Connect simulator"],
    ["preflight", machineStatus({ state: "preflight" }), "Confirm workspace clear"],
    ["an unhomed machine without a lease", machineStatus({ state: "unhomed" }), "Acquire 15 min lease"],
    ["an unhomed leased machine", machineStatus({ state: "unhomed", control_lease: controlLease }), "Home machine"],
    ["a coarse-homed machine", machineStatus({ state: "ready", homed: true, control_lease: controlLease }), "Set up precision calibration"],
  ])("offers the next action for %s", async (_name, status, expectedAction) => {
    mockBrowserServices();
    vi.spyOn(api, "status").mockResolvedValue(status);
    vi.spyOn(api, "devices").mockResolvedValue([]);
    vi.spyOn(api, "calibrations").mockResolvedValue([]);

    render(<App />);

    await enabledButton(expectedAction);
  });
});

describe("machine bootstrap journey", () => {
  it("moves from disconnected state to initial calibration", async () => {
    mockBrowserServices();
    const user = userEvent.setup();
    let status = machineStatus({});
    const snapshot = calibrationSnapshot("initial-calibration");
    const run = calibrationRun(snapshot.id, "pending");
    vi.spyOn(api, "status").mockImplementation(async () => ({ ...status }));
    vi.spyOn(api, "devices").mockResolvedValue([]);
    mockCalibrationApi([]);
    vi.spyOn(api, "connect").mockImplementation(async () => {
      status = { ...status, state: "preflight" };
      return operation("machine.connect");
    });
    vi.spyOn(api, "preflight").mockImplementation(async () => {
      status = { ...status, state: "unhomed" };
      return operation("machine.preflight");
    });
    vi.spyOn(api, "lease").mockImplementation(async () => {
      status = { ...status, control_lease: controlLease };
      return controlLease;
    });
    vi.spyOn(api, "home").mockImplementation(async () => {
      status = {
        ...status,
        state: "ready",
        homed: true,
        pose: { x_mm: 0, y_mm: 0, z_mm: 31.5, rotation_deg: 0 },
      };
      return operation("machine.home");
    });
    vi.spyOn(api, "createCalibration").mockResolvedValue(snapshot);
    vi.spyOn(api, "acquireOrRenewLease").mockResolvedValue(controlLease);
    vi.spyOn(api, "calibrationRun").mockResolvedValue(run);
    const startCalibration = vi.spyOn(api, "startCalibration")
      .mockResolvedValue({ run, operation: operation("calibration.full_machine") });

    render(<App />);
    await user.click(await enabledButton("Connect simulator"));
    await user.click(await enabledButton("Confirm workspace clear"));
    await user.click(await enabledButton("Acquire 15 min lease"));
    await user.click(await enabledButton("Home machine"));
    await user.click(await screen.findByRole("button", { name: "Start homing" }));
    await user.click(await enabledButton("Set up precision calibration"));
    await user.click(await enabledButton("Create calibration"));
    await user.click(await enabledButton("Run full calibration"));
    await user.click(await screen.findByRole("button", { name: "Start full calibration" }));

    await waitFor(() => expect(startCalibration).toHaveBeenCalledWith(
      snapshot.id, controlLease.id,
    ));
  });

  it("keeps invalid parent reuse out of the bootstrap path", async () => {
    mockBrowserServices();
    const user = userEvent.setup();
    const status = machineStatus({
      state: "ready", homed: true, control_lease: controlLease,
    });
    const parent = calibrationSnapshot("empty-parent");
    const child = calibrationSnapshot(
      "partial-child", parent.id, ["coarse_homing"],
    );
    vi.spyOn(api, "status").mockResolvedValue(status);
    vi.spyOn(api, "devices").mockResolvedValue([]);
    mockCalibrationApi([child, parent], [calibrationRun(child.id)]);

    render(<App />);
    await user.click(await enabledButton("Set up precision calibration"));

    await enabledButton("Run full calibration again");
    expect((await screen.findByRole("button", {
      name: "Recalibrate head geometry",
    }) as HTMLButtonElement).disabled).toBe(true);
    expect((screen.getByRole("button", {
      name: "New revision",
    }) as HTMLButtonElement).disabled).toBe(true);
    expect(screen.queryByText(/This revision cannot reuse its parent/)).toBeNull();
    expect(screen.getByText(
      "Top-camera reproducibility did not meet its profile thresholds.",
    )).toBeTruthy();
  });

  it("allows head geometry recalibration only from a qualified parent", async () => {
    mockBrowserServices();
    const user = userEvent.setup();
    const status = machineStatus({
      state: "ready", homed: true, control_lease: controlLease,
    });
    const parent = calibrationSnapshot(
      "qualified-parent",
      null,
      ["coarse_homing", "camera_reproducibility", "visual_homing"],
      { visual_homing: { report: { passed: true } } },
    );
    const child = calibrationSnapshot("new-revision", parent.id);
    const run = calibrationRun(child.id, "pending");
    vi.spyOn(api, "status").mockResolvedValue(status);
    vi.spyOn(api, "devices").mockResolvedValue([]);
    mockCalibrationApi([child, parent]);
    vi.spyOn(api, "acquireOrRenewLease").mockResolvedValue(controlLease);
    vi.spyOn(api, "calibrationRun").mockResolvedValue(run);
    const startHeadGeometry = vi.spyOn(api, "startHeadGeometryCalibration")
      .mockResolvedValue({ run, operation: operation("calibration.head_geometry") });

    render(<App />);
    await user.click(await enabledButton("Set up precision calibration"));
    await user.click(await enabledButton("Recalibrate head geometry"));
    await user.click(await screen.findByRole("button", { name: "Start recalibration" }));

    await waitFor(() => expect(startHeadGeometry).toHaveBeenCalledWith(
      child.id, controlLease.id,
    ));
  });

  it("fine-homes directly from a compatible stored calibration", async () => {
    mockBrowserServices();
    const user = userEvent.setup();
    const status = machineStatus({
      state: "ready", homed: true, control_lease: controlLease,
    });
    const snapshot = calibrationSnapshot(
      "precision-ready",
      null,
      ["local_xy_direction_response", "nozzle_height_model"],
      { vacuum_probe: { N2: { passed: true } } },
    );
    snapshot.artifacts = {
      local_xy_direction_response: calibrationArtifact(),
      runtime_tip_transform_n2: calibrationArtifact(),
      vacuum_contact_n2: calibrationArtifact(),
    };
    vi.spyOn(api, "status").mockResolvedValue(status);
    vi.spyOn(api, "devices").mockResolvedValue([]);
    vi.spyOn(api, "calibrations").mockResolvedValue([snapshot]);
    vi.spyOn(api, "acquireOrRenewLease").mockResolvedValue(controlLease);
    const fineHome = vi.spyOn(api, "fineHome")
      .mockResolvedValue(operation("machine.fine_home"));

    render(<App />);
    await user.click(await enabledButton("Fine-home XYZ"));
    await user.click(await screen.findByRole("button", {
      name: "Start fine homing",
    }));

    await waitFor(() => expect(fineHome).toHaveBeenCalledWith(
      snapshot.id, controlLease.id,
    ));
  });
});

describe("machine device display", () => {
  it("keeps one capture node per configured machine camera", () => {
    const device = (overrides: Partial<Device>): Device => ({
      kind: "camera",
      path: "/dev/video0",
      stable_path: null,
      vendor_id: null,
      product_id: null,
      serial: null,
      product: "Camera",
      capture_capable: true,
      suggested_role: null,
      ...overrides,
    });
    const visible = attachedMachineDevices([
      device({ path: "/dev/video0", product: "Laptop webcam" }),
      device({ path: "/dev/video2", suggested_role: "bottom_camera" }),
      device({ path: "/dev/video3", capture_capable: false, suggested_role: "bottom_camera" }),
      device({ path: "/dev/video4", suggested_role: "top_camera" }),
      device({ path: "/dev/video5", capture_capable: false, suggested_role: "top_camera" }),
      device({ kind: "serial_controller", path: "/dev/ttyACM0" }),
    ]);

    expect(visible.map((value) => value.path)).toEqual([
      "/dev/video2", "/dev/video4", "/dev/ttyACM0",
    ]);
  });
});

describe("calibration progress", () => {
  it("selects the active check instead of a waiting check", () => {
    const run = calibrationRun("calibration", "running");
    run.checks = [
      {
        id: "coarse_homing", title: "Coarse home", source: "controller",
        status: "passed", message: "Complete", evidence: null,
      },
      {
        id: "top_camera", title: "Top camera", source: "vision",
        status: "pending", message: "Measured 12 of 30 frames", evidence: null,
      },
      {
        id: "tooling_N1", title: "N1 tooling", source: "vision",
        status: "pending", message: "Waiting for measurement", evidence: null,
      },
    ];

    expect(currentCalibrationCheck(run)?.id).toBe("top_camera");
  });

  it("shows live state for a standalone vacuum characterization", async () => {
    mockBrowserServices();
    const user = userEvent.setup();
    const status = machineStatus({
      state: "ready", homed: true, control_lease: controlLease,
      pose: { x_mm: 220.131, y_mm: 190.271, z_mm: 31.5, rotation_deg: 0 },
    });
    const snapshot = calibrationSnapshot(
      "calibration",
      null,
      ["nozzle_height_model"],
      { vacuum_probe: { N1: { passed: true } } },
    );
    vi.spyOn(api, "status").mockResolvedValue(status);
    vi.spyOn(api, "devices").mockResolvedValue([]);
    mockCalibrationApi([snapshot]);
    vi.spyOn(api, "acquireOrRenewLease").mockResolvedValue(controlLease);
    vi.spyOn(api, "characterizeVacuum").mockResolvedValue(
      operation("calibration.vacuum_characterization", "running"),
    );
    vi.stubGlobal("fetch", vi.fn(() => new Promise<Response>(() => {})));

    render(<App />);
    await user.click(await enabledButton("Set up precision calibration"));
    await user.click(await enabledButton("Characterize N1 vacuum"));
    await user.click(await screen.findByRole("button", {
      name: "Start characterization",
    }));

    expect(await screen.findByText("N1 vacuum sensor characterization"))
      .toBeTruthy();
    expect(screen.getByText(/Elapsed: 0 s\. Position: X220\.131/)).toBeTruthy();

    await user.click(screen.getByText("Machine"));
    await user.click(screen.getByText("Calibration"));
    expect(await screen.findByText("N1 vacuum sensor characterization"))
      .toBeTruthy();
    expect((await screen.findByRole("button", {
      name: "Probe N2 vacuum",
    }) as HTMLButtonElement).disabled).toBe(true);
  });

  it("replaces a superseded run failure with current snapshot state", async () => {
    mockBrowserServices();
    const user = userEvent.setup();
    const status = machineStatus({
      state: "ready", homed: true, control_lease: controlLease,
    });
    const snapshot = calibrationSnapshot(
      "calibration",
      null,
      ["vacuum_sensor_characterization"],
      { vacuum_sensor_characterization: {
        N1: {
          tip_id: "N045", passed: true, median_signal_delta_counts: 9,
          robust_distribution_gap_counts: 9, acquire_threshold_raw_value: 231,
          release_threshold_raw_value: 234, observed_at: "2026-09-08T18:00:00Z",
        },
        N2: {
          tip_id: "N24", passed: true, median_signal_delta_counts: 11,
          robust_distribution_gap_counts: 11, acquire_threshold_raw_value: 245,
          release_threshold_raw_value: 248, observed_at: "2026-09-08T18:01:00Z",
        },
      } },
    );
    snapshot.artifacts.vacuum_characterization_n1 = calibrationArtifact();
    snapshot.artifacts.vacuum_characterization_n2 = calibrationArtifact();
    const run = calibrationRun(snapshot.id, "rejected");
    run.phase = "vacuum_sensor_characterization";
    run.failure = "vacuum characterization did not reproduce contact at the stored safe Z";
    run.checks = [{
      id: "vacuum_characterization_N1",
      title: "N1 vacuum sensor characterization",
      source: "controller",
      status: "failed",
      message: run.failure,
      evidence: null,
    }, {
      id: "vacuum_characterization_N2",
      title: "N2 vacuum sensor characterization",
      source: "controller",
      status: "pending",
      message: "Waiting for pump-off, open, sealed, and release samples",
      evidence: null,
    }];
    vi.spyOn(api, "status").mockResolvedValue(status);
    vi.spyOn(api, "devices").mockResolvedValue([]);
    mockCalibrationApi([snapshot], [run]);

    expect(calibrationRunSupersededBySnapshot(run, snapshot)).toBe(true);
    expect(latestCalibrationChecks(run, snapshot).map((check) => check.status))
      .toEqual(["passed", "passed"]);
    render(<App />);
    await user.click(await enabledButton("Set up precision calibration"));

    expect(await screen.findByText("Recovered")).toBeTruthy();
    expect(screen.getByText(/Later targeted measurements resolved/)).toBeTruthy();
    expect(screen.getByText(/N1 N045 characterized a 9.0-count reference seal/))
      .toBeTruthy();
    expect(screen.getByText(/N2 N24 characterized an? 11.0-count reference seal/))
      .toBeTruthy();
    expect(screen.queryByText(/Did not reproduce contact/)).toBeNull();
  });
});
