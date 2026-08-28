import type {
  AutomatedCalibrationRun, BoardDesign, BoardRegistrationRun, CalibrationArtifactId,
  CalibrationArtifactPlan, CalibrationSnapshot, CalibrationStep, CameraControl, CameraProfile,
  CameraReproducibilityReport, CameraRole, ControlLease, Device, JogTarget, MachineStatus,
  Operation, PlacementExecutionRun, SensorReading, StripFeederLiveHoldoutRun,
  StripFeederPickupQualificationRun, StripFeederRegistration,
  StripFeederInventoryReport, StripFeederVisionReport, VisualHomingResult,
  WorkspaceBoardDiscoveryReport,
  WorkspaceChangeSetReport, WorkspaceInventoryBaseline, WorkspaceScanReport,
} from "./types";

const BASE = "/api/v1";

export class ApiError extends Error {
  constructor(public status: number, message: string) {
    super(message);
  }
}

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await fetch(`${BASE}${path}`, init);
  const data = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new ApiError(response.status, data?.error?.message ?? `Request failed (${response.status})`);
  }
  return data as T;
}

function jsonPost<T>(path: string, body?: unknown): Promise<T> {
  return request(path, {
    method: "POST",
    headers: body === undefined ? undefined : { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
}

function jsonPut<T>(path: string, body: unknown): Promise<T> {
  return request(path, {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
}

export async function waitForOperation(operation: Operation): Promise<Operation> {
  let current = operation;
  while (current.status === "pending" || current.status === "running") {
    await new Promise((resolve) => window.setTimeout(resolve, 120));
    current = await request<Operation>(`/operations/${operation.id}`);
  }
  if (current.status === "failed") throw new Error(current.message ?? "Operation failed");
  return current;
}

export const api = {
  status: async () => (await request<{ machine: MachineStatus }>("/machine/status")).machine,
  devices: async () => (await request<{ devices: Device[] }>("/system/devices")).devices,
  cameraControls: async (role: CameraRole) =>
    (await request<{ controls: CameraControl[] }>(`/cameras/${role}/controls`)).controls,
  setCameraControls: async (
    role: CameraRole, controls: Array<{ id: number; value: number | boolean }>,
  ) => (await jsonPut<{ controls: CameraControl[] }>(`/cameras/${role}/controls`, { controls })).controls,
  cameraProfile: (role: CameraRole) => request<CameraProfile>(`/cameras/${role}/profile`),
  setCameraProfile: (role: CameraRole, profile: CameraProfile) =>
    jsonPut<CameraProfile>(`/cameras/${role}/profile`, profile),
  cameraReproducibility: async (role: CameraRole, leaseId: string, sampleCount = 30) =>
    (await jsonPost<{ report: CameraReproducibilityReport }>(
      `/cameras/${role}/reproducibility`,
      { lease_id: leaseId, sample_count: sampleCount },
    )).report,
  connect: () => jsonPost<Operation>("/machine/connect"),
  disconnect: () => jsonPost<Operation>("/machine/disconnect"),
  preflight: () => jsonPost<Operation>("/machine/preflight", { confirmed: true }),
  lease: async (clientName = "VelocityPnP web UI", durationSeconds = 900) =>
    (await jsonPost<{ lease: ControlLease }>("/control/lease", { client_name: clientName, duration_seconds: durationSeconds })).lease,
  renewLease: async (leaseId: string, durationSeconds = 3600) =>
    (await jsonPost<{ lease: ControlLease }>("/control/lease/renew", { lease_id: leaseId, duration_seconds: durationSeconds })).lease,
  home: (leaseId: string) => jsonPost<Operation>("/machine/home", { lease_id: leaseId, confirmed: true }),
  jog: (leaseId: string, target: JogTarget, feed = 300) =>
    jsonPost<Operation>("/machine/jog", { lease_id: leaseId, target, feed_mm_per_min: feed }),
  actuate: (leaseId: string, actuatorId: string, enabled: boolean) =>
    jsonPost<Operation>(`/machine/actuators/${actuatorId}`, { lease_id: leaseId, enabled }),
  readSensor: (leaseId: string, sensorId: string) =>
    jsonPost<SensorReading>(`/machine/sensors/${sensorId}/read`, { lease_id: leaseId }),
  visualHome: (leaseId: string, applyOffset = false, moveToSeed = true) =>
    jsonPost<VisualHomingResult>("/machine/visual-home", {
      lease_id: leaseId,
      apply_offset: applyOffset,
      move_to_seed: moveToSeed,
    }),
  emergencyStop: () => jsonPost<Operation>("/machine/emergency-stop"),
  workflow: async () => (await request<{ steps: CalibrationStep[] }>("/calibration/workflow")).steps,
  calibrations: async () =>
    (await request<{ calibrations: CalibrationSnapshot[] }>("/calibrations")).calibrations,
  createCalibration: (machineSerial: string) =>
    jsonPost<CalibrationSnapshot>("/calibrations", { machine_serial: machineSerial }),
  calibration: (snapshotId: string) =>
    request<CalibrationSnapshot>(`/calibrations/${snapshotId}`),
  calibrationPlan: (snapshotId: string, targets: CalibrationArtifactId[] = []) =>
    jsonPost<CalibrationArtifactPlan>(`/calibrations/${snapshotId}/artifact-plan`, { targets }),
  recomputeDerivedCalibration: (
    snapshotId: string, targets: CalibrationArtifactId[] = [],
  ) => jsonPost<{ recomputed: CalibrationArtifactId[]; snapshot: CalibrationSnapshot }>(
    `/calibrations/${snapshotId}/derived-artifacts/recompute`, { targets },
  ),
  calibrationRuns: async (snapshotId: string) =>
    (await request<{ runs: AutomatedCalibrationRun[] }>(`/calibrations/${snapshotId}/runs`)).runs,
  calibrationRun: (runId: string) =>
    request<AutomatedCalibrationRun>(`/calibration-runs/${runId}`),
  startCalibration: (
    snapshotId: string, leaseId: string, visualHomingSampleCount = 5,
  ) => jsonPost<{ run: AutomatedCalibrationRun; operation: Operation }>(
    `/calibrations/${snapshotId}/runs`,
    {
      lease_id: leaseId,
      confirmed_external_safety_checks: true,
      visual_homing_sample_count: visualHomingSampleCount,
    },
  ),
  startHeadGeometryCalibration: (snapshotId: string, leaseId: string) =>
    jsonPost<{ run: AutomatedCalibrationRun; operation: Operation }>(
      `/calibrations/${snapshotId}/head-geometry-runs`,
      {
        lease_id: leaseId,
        confirmed_external_safety_checks: true,
      },
    ),
  startInstalledGeometryCalibration: (snapshotId: string, leaseId: string) =>
    jsonPost<{ run: AutomatedCalibrationRun; operation: Operation }>(
      `/calibrations/${snapshotId}/installed-geometry-runs`,
      {
        lease_id: leaseId,
        confirmed_external_safety_checks: true,
      },
    ),
  startArtifactCalibration: (
    snapshotId: string, leaseId: string, targets: CalibrationArtifactId[],
  ) => jsonPost<{ run: AutomatedCalibrationRun; operation: Operation }>(
    `/calibrations/${snapshotId}/artifact-runs`,
    {
      lease_id: leaseId,
      confirmed_external_safety_checks: true,
      targets,
    },
  ),
  vacuumProbe: (snapshotId: string, leaseId: string, nozzleId: "N1" | "N2") =>
    jsonPost<Operation>(`/calibrations/${snapshotId}/vacuum-probe`, {
      lease_id: leaseId,
      nozzle_id: nozzleId,
      confirmed_external_safety_checks: true,
    }),
  fineHome: (snapshotId: string, leaseId: string, nozzleId: "N1" | "N2" = "N2") =>
    jsonPost<Operation>(`/calibrations/${snapshotId}/fine-home`, {
      lease_id: leaseId,
      nozzle_id: nozzleId,
      confirmed_external_safety_checks: true,
    }),
  characterizeVacuum: (
    snapshotId: string, leaseId: string, nozzleId: "N1" | "N2",
  ) => jsonPost<Operation>(`/calibrations/${snapshotId}/vacuum-characterization`, {
    lease_id: leaseId,
    nozzle_id: nozzleId,
    confirmed_external_safety_checks: true,
  }),
  calibrateXyDirectionResponse: (snapshotId: string, leaseId: string) =>
    jsonPost<Operation>(`/calibrations/${snapshotId}/xy-direction-response`, {
      lease_id: leaseId,
      confirmed_external_safety_checks: true,
    }),
  observeNozzleHeight: (
    snapshotId: string, nozzleId: "N1" | "N2", nozzleZMm: number,
    errorXMm: number, errorYMm: number, uncertaintyMm: number,
    postCorrectionResidual = false,
  ) => jsonPost<CalibrationSnapshot>(
    `/calibrations/${snapshotId}/nozzle-height-observation`,
    {
      nozzle_id: nozzleId,
      nozzle_z_mm: nozzleZMm,
      observed_error_x_mm: errorXMm,
      observed_error_y_mm: errorYMm,
      measurement_uncertainty_mm: uncertaintyMm,
      post_correction_residual: postCorrectionResidual,
    },
  ),
  invalidateCalibration: (snapshotId: string, stepId: string) =>
    jsonPost<CalibrationSnapshot>(`/calibrations/${snapshotId}/steps/${stepId}/invalidate`),
  boards: async () => (await request<{ boards: BoardDesign[] }>("/boards")).boards,
  importBoard: async (pcb: File, bom: File | null) => {
    const form = new FormData();
    form.append("pcb", pcb);
    form.append("name", pcb.name.replace(/\.kicad_pcb$/i, ""));
    if (bom) form.append("bom", bom);
    return request<BoardDesign>("/boards/import", { method: "POST", body: form });
  },
  workspaceScans: async () =>
    (await request<{ scans: WorkspaceScanReport[] }>("/workspace-scans")).scans,
  discoverWorkspaceBoards: (scanId: string) =>
    jsonPost<WorkspaceBoardDiscoveryReport>(
      `/workspace-scans/${scanId}/board-discovery`,
    ),
  discoverWorkspaceObjects: (scanId: string) =>
    jsonPost<WorkspaceBoardDiscoveryReport>(
      `/workspace-scans/${scanId}/object-proposals`,
    ),
  analyzeStripFeeder: (scanId: string, candidateIndex: number) =>
    jsonPost<StripFeederVisionReport>(
      `/workspace-scans/${scanId}/strip-feeder-vision`,
      { candidate_index: candidateIndex },
    ),
  buildStripFeederInventory: (
    scanId: string,
    evidenceSources: Array<{ scan_id: string; candidate_index: number }>,
  ) => jsonPost<StripFeederInventoryReport>(
    `/workspace-scans/${scanId}/strip-feeder-inventory`,
    { evidence_sources: evidenceSources },
  ),
  createStripFeederDraft: (
    scanId: string, candidateIndex: number, proposalIndex: number,
    partKey?: string, partRotationDeg?: number,
  ) => jsonPost<StripFeederRegistration>(
    "/strip-feeder-registrations",
    {
      scan_id: scanId,
      candidate_index: candidateIndex,
      proposal_index: proposalIndex,
      part_key: partKey || null,
      part_rotation_deg: partRotationDeg ?? null,
    },
  ),
  startStripFeederLiveHoldout: (
    registrationId: string, leaseId: string,
  ) => jsonPost<{ holdout: StripFeederLiveHoldoutRun; operation: Operation }>(
    `/strip-feeder-registrations/${registrationId}/live-holdouts`,
    {
      lease_id: leaseId,
      pocket_index: null,
      sample_count: 3,
      confirmed_external_safety_checks: true,
    },
  ),
  stripFeederLiveHoldout: (holdoutId: string) =>
    request<StripFeederLiveHoldoutRun>(
      `/strip-feeder-live-holdouts/${holdoutId}`,
    ),
  startStripFeederPickupQualification: (
    registrationId: string, leaseId: string,
    nozzleId = "N1", fallbackDropSurfaceMachineMm?: [number, number],
    pocketIndex?: number,
  ) => jsonPost<{
    qualification: StripFeederPickupQualificationRun;
    operation: Operation;
  }>(
    `/strip-feeder-registrations/${registrationId}/pickup-qualifications`,
    {
      lease_id: leaseId,
      nozzle_id: nozzleId,
      pocket_index: pocketIndex ?? null,
      fallback_drop_surface_machine_mm: fallbackDropSurfaceMachineMm ?? null,
      confirmed_external_safety_checks: true,
    },
  ),
  stripFeederPickupQualification: (qualificationId: string) =>
    request<StripFeederPickupQualificationRun>(
      `/strip-feeder-pickup-qualifications/${qualificationId}`,
    ),
  startGuardedSinglePlacement: (
    jobId: string, leaseId: string, calibrationId: string, stepIndex = 0,
  ) => jsonPost<{ execution: PlacementExecutionRun; operation: Operation }>(
    `/jobs/${jobId}/placement-executions`,
    {
      lease_id: leaseId,
      calibration_id: calibrationId,
      step_index: stepIndex,
      confirmed_external_safety_checks: true,
    },
  ),
  placementExecution: (executionId: string) =>
    request<PlacementExecutionRun>(`/placement-executions/${executionId}`),
  reanalyzeStripFeederReturn: (qualificationId: string) =>
    jsonPost<StripFeederPickupQualificationRun>(
      `/strip-feeder-pickup-qualifications/${qualificationId}/reanalyze-return`,
    ),
  stripFeederDraft: (registrationId: string) =>
    request<StripFeederRegistration>(
      `/strip-feeder-registrations/${registrationId}`,
    ),
  startBoardRegistration: (
    scanId: string, candidateIndex: number, boardId: string, leaseId: string,
    label?: string,
  ) => jsonPost<{ registration: BoardRegistrationRun; operation: Operation }>(
    `/workspace-scans/${scanId}/board-registrations`,
    {
      lease_id: leaseId,
      candidate_index: candidateIndex,
      board_id: boardId,
      label,
      sample_count: 3,
      confirmed_external_safety_checks: true,
    },
  ),
  boardRegistrations: async () =>
    (await request<{ registrations: BoardRegistrationRun[] }>(
      "/board-registrations",
    )).registrations,
  boardRegistration: (registrationId: string) =>
    request<BoardRegistrationRun>(`/board-registrations/${registrationId}`),
  setWorkspaceInventoryBaseline: (scanId: string) =>
    jsonPost<WorkspaceInventoryBaseline>(
      `/workspace-scans/${scanId}/inventory-baseline`,
    ),
  compareWorkspaceObservation: (scanId: string, baselineId?: string) =>
    jsonPost<WorkspaceChangeSetReport>(
      `/workspace-scans/${scanId}/change-set`,
      baselineId ? { baseline_id: baselineId } : {},
    ),
};
