// @vitest-environment jsdom

import { cleanup, render, screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { afterEach, describe, expect, it, vi } from "vitest";
import App from "./App";
import { api } from "./api";
import type {
  AutomatedCalibrationRun, CalibrationSnapshot, ControlLease, MachineStatus,
  Operation,
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

function operation(kind: string): Operation {
  return {
    id: `${kind}-operation`,
    kind,
    status: "succeeded",
    message: null,
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
    ["a coarse-homed machine", machineStatus({ state: "ready", homed: true, control_lease: controlLease }), "Continue to fine-home XYZ"],
  ])("offers the next action for %s", async (_name, status, expectedAction) => {
    mockBrowserServices();
    vi.spyOn(api, "status").mockResolvedValue(status);
    vi.spyOn(api, "devices").mockResolvedValue([]);

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
      .mockResolvedValue({ run, operation: operation("calibration.initial_machine") });

    render(<App />);
    await user.click(await enabledButton("Connect simulator"));
    await user.click(await enabledButton("Confirm workspace clear"));
    await user.click(await enabledButton("Acquire 15 min lease"));
    await user.click(await enabledButton("Home machine"));
    await user.click(await screen.findByRole("button", { name: "Start homing" }));
    await user.click(await enabledButton("Continue to fine-home XYZ"));
    await user.click(await enabledButton("Create calibration"));
    await user.click(await enabledButton("Run initial calibration"));
    await user.click(await screen.findByRole("button", { name: "Start calibration" }));

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
    await user.click(await enabledButton("Continue to fine-home XYZ"));

    await enabledButton("Run full calibration again");
    expect((await screen.findByRole("button", {
      name: "Recalibrate head geometry",
    }) as HTMLButtonElement).disabled).toBe(true);
    expect((screen.getByRole("button", {
      name: "New revision",
    }) as HTMLButtonElement).disabled).toBe(true);
    expect(await screen.findByText(/This revision cannot reuse its parent/))
      .toBeTruthy();
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
    await user.click(await enabledButton("Continue to fine-home XYZ"));
    await user.click(await enabledButton("Recalibrate head geometry"));
    await user.click(await screen.findByRole("button", { name: "Start recalibration" }));

    await waitFor(() => expect(startHeadGeometry).toHaveBeenCalledWith(
      child.id, controlLease.id,
    ));
  });
});
