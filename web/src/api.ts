import type {
  AutomatedCalibrationRun, BoardDesign, BoardPopulationRefreshReport, BoardRegistrationRun,
  CalibrationArtifactId,
  CalibrationArtifactPlan, CalibrationSnapshot, CalibrationStep, CameraControl, CameraProfile,
  CameraReproducibilityReport, CameraRole, ControlLease, Device, JogTarget, MachineStatus,
  Job, JobExecutionRun, MachineMotionProfile, MachineOverlayGeometry, Operation,
  Part, PlacementExecutionRun,
  PlacementOutcomeRecord, SensorReading,
  StripFeederLiveHoldoutRun,
  StripFeederOccupancyRefreshReport, StripFeederOccupancyReviewQueue,
  StripFeederPocketOccupancyReview, StripFeederPocketReviewLabel,
  StripFeederPickupQualificationRun, StripFeederRegistration,
  StripFeederInventoryReport, StripFeederVisionReport, VisualHomingResult,
  StripFeederMapRegistration, StripFeederRevisionSummary,
  WorkspaceActivityFeed,
  WorkspaceBoardDiscoveryReport,
  WorkspaceChangeSetReport, WorkspaceDisplayPatch, WorkspaceInventoryBaseline,
  WorkspaceScanEstimate, WorkspaceScanRegion, WorkspaceScanReport, WorkspaceScanSummary,
} from "./types";

const BASE = "/api/v1";

export function formatErrorMessage(message: string): string {
  const trimmed = message.trim();
  if (trimmed.length === 0) return "An unexpected error occurred.";
  const capitalized = /^[a-z]/.test(trimmed)
    ? `${trimmed[0].toUpperCase()}${trimmed.slice(1)}`
    : trimmed;
  return /[.!?]$/.test(capitalized) ? capitalized : `${capitalized}.`;
}

export class ApiError extends Error {
  constructor(public status: number, message: string) {
    super(formatErrorMessage(message));
  }
}

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await fetch(`${BASE}${path}`, { ...init, cache: "no-store" });
  let data: unknown;
  try {
    data = await response.json();
  } catch {
    const method = init?.method ?? "GET";
    const contentType = response.headers.get("content-type") ?? "unknown content type";
    throw new ApiError(
      response.ok ? 502 : response.status,
      `The daemon returned ${contentType} for ${method} ${BASE}${path}. `
        + "The web UI and daemon may be different versions",
    );
  }
  if (!response.ok) {
    const message = typeof data === "object" && data !== null
      && "error" in data && typeof data.error === "object" && data.error !== null
      && "message" in data.error && typeof data.error.message === "string"
      ? data.error.message
      : `Request failed (${response.status})`;
    throw new ApiError(response.status, message);
  }
  return data as T;
}

async function requestBlob(path: string): Promise<Blob> {
  const response = await fetch(`${BASE}${path}`);
  if (!response.ok) {
    const data = await response.json().catch(() => ({}));
    throw new ApiError(
      response.status,
      data?.error?.message ?? `Request failed (${response.status})`,
    );
  }
  return response.blob();
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

function workspaceSurveySpecification(
  parentScanId: string,
  calibrationId: string,
  region: WorkspaceScanRegion,
) {
  return {
    calibration_id: calibrationId,
    mode: "survey",
    artifact_retention: "transient_sources",
    region,
    region_preset: "installed_plates",
    parent_scan_id: parentScanId,
    output_mm_per_pixel: .05,
  };
}

async function acquireLease(
  clientName = "VelocityPnP web UI", durationSeconds = 900,
): Promise<ControlLease> {
  return (await jsonPost<{ lease: ControlLease }>("/control/lease", {
    client_name: clientName,
    duration_seconds: durationSeconds,
  })).lease;
}

async function renewLease(
  leaseId: string, durationSeconds = 3600,
): Promise<ControlLease> {
  return (await jsonPost<{ lease: ControlLease }>("/control/lease/renew", {
    lease_id: leaseId,
    duration_seconds: durationSeconds,
  })).lease;
}

async function acquireOrRenewLease(
  clientName = "VelocityPnP web UI", durationSeconds = 900,
): Promise<ControlLease> {
  const machine = (await request<{ machine: MachineStatus }>("/machine/status")).machine;
  if (!machine.control_lease) return acquireLease(clientName, durationSeconds);
  try {
    return await renewLease(machine.control_lease.id, durationSeconds);
  } catch (reason) {
    const staleLease = reason instanceof ApiError
      && reason.status === 400
      && reason.message === "Control lease does not match.";
    if (!staleLease) throw reason;
    return acquireLease(clientName, durationSeconds);
  }
}

export async function waitForOperation(
  operation: Operation,
  onUpdate?: (operation: Operation) => void,
): Promise<Operation> {
  let current = operation;
  onUpdate?.(current);
  while (current.status === "pending" || current.status === "running") {
    await new Promise((resolve) => globalThis.setTimeout(resolve, 120));
    current = await request<Operation>(`/operations/${operation.id}`);
    onUpdate?.(current);
  }
  if (current.status === "failed") throw new Error(current.message ?? "Operation failed");
  return current;
}

export const api = {
  status: async () => (await request<{ machine: MachineStatus }>("/machine/status")).machine,
  machineProfile: () => request<MachineMotionProfile>("/machine/profile"),
  machineOverlayGeometry: (calibrationId: string) =>
    request<MachineOverlayGeometry>(`/machine/overlay-geometry/${calibrationId}`),
  devices: async () => (await request<{ devices: Device[] }>("/system/devices")).devices,
  cameraControls: async (role: CameraRole) =>
    (await request<{ controls: CameraControl[] }>(`/cameras/${role}/controls`)).controls,
  setCameraControls: async (
    role: CameraRole, controls: Array<{ id: number; value: number | boolean }>,
  ) => (await jsonPut<{ controls: CameraControl[] }>(`/cameras/${role}/controls`, { controls })).controls,
  cameraProfile: (role: CameraRole) => request<CameraProfile>(`/cameras/${role}/profile`),
  cameraSnapshot: (role: CameraRole) => requestBlob(`/cameras/${role}/snapshot`),
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
  lease: acquireLease,
  renewLease,
  acquireOrRenewLease,
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
  startPlacementTransferCalibration: (
    snapshotId: string, leaseId: string, jobId: string,
    nozzleId: "N1" | "N2", placementReferences: string[] = [],
  ) => jsonPost<{ run: AutomatedCalibrationRun; operation: Operation }>(
    `/calibrations/${snapshotId}/placement-transfer-runs`,
    {
      lease_id: leaseId,
      job_id: jobId,
      nozzle_id: nozzleId,
      placement_references: placementReferences,
      confirmed_external_safety_checks: true,
      confirmed_targets_empty: true,
    },
  ),
  startPlacementTransferValidation: (
    snapshotId: string, leaseId: string, jobId: string,
    nozzleId: "N1" | "N2", placementReferences: string[] = [],
  ) => jsonPost<{ run: AutomatedCalibrationRun; operation: Operation }>(
    `/calibrations/${snapshotId}/placement-transfer-validation-runs`,
    {
      lease_id: leaseId,
      job_id: jobId,
      nozzle_id: nozzleId,
      placement_references: placementReferences,
      confirmed_external_safety_checks: true,
      confirmed_targets_empty: true,
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
  workspaceScanSummaries: async () =>
    (await request<{ scans: WorkspaceScanSummary[] }>(
      "/workspace-scans/summaries",
    )).scans,
  workspaceScan: (scanId: string) =>
    request<WorkspaceScanReport>(`/workspace-scans/${scanId}`),
  estimateWorkspaceSurvey: (
    parentScanId: string,
    calibrationId: string,
    region: WorkspaceScanRegion,
  ) => jsonPost<WorkspaceScanEstimate>(
    "/workspace-scans/estimate",
    workspaceSurveySpecification(parentScanId, calibrationId, region),
  ),
  startWorkspaceSurvey: (
    parentScanId: string,
    calibrationId: string,
    region: WorkspaceScanRegion,
    leaseId: string,
  ) => jsonPost<{ scan: WorkspaceScanReport; operation: Operation }>(
    "/workspace-scans",
    {
      lease_id: leaseId,
      confirmed_external_safety_checks: true,
      ...workspaceSurveySpecification(parentScanId, calibrationId, region),
    },
  ),
  workspaceDisplayPatches: (scanId: string) =>
    jsonPost<{ patches: WorkspaceDisplayPatch[] }>(
      `/workspace-scans/${scanId}/display-patches`,
    ).then((response) => response.patches),
  workspaceActivity: () => request<WorkspaceActivityFeed>("/workspace/activity"),
  workspaceScanArtifact: <T>(scanId: string, artifactName: string) =>
    request<T>(
      `/workspace-scans/${scanId}/artifacts/${encodeURIComponent(artifactName)}`,
    ),
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
  refreshStripFeederOccupancy: (
    scanId: string, candidateIndex: number, registrationId: string,
  ) => jsonPost<StripFeederOccupancyRefreshReport>(
    `/workspace-scans/${scanId}/strip-feeder-occupancy`,
    { candidate_index: candidateIndex, registration_id: registrationId },
  ),
  refreshBoardPopulation: (
    scanId: string, jobId: string, boardInstanceId: string,
  ) => jsonPost<BoardPopulationRefreshReport>(
    `/workspace-scans/${scanId}/board-population`,
    { job_id: jobId, board_instance_id: boardInstanceId },
  ),
  boardPopulationRefreshes: async () => (
    await request<{ refreshes: BoardPopulationRefreshReport[] }>(
      "/board-population-refreshes",
    )
  ).refreshes,
  buildStripFeederInventory: (
    scanId: string,
    evidenceSources: Array<{ scan_id: string; candidate_index: number }>,
  ) => jsonPost<StripFeederInventoryReport>(
    `/workspace-scans/${scanId}/strip-feeder-inventory`,
    { evidence_sources: evidenceSources },
  ),
  createStripFeederDraft: (
    scanId: string, candidateIndex: number, proposalIndex: number,
    partKey?: string, partRotationDeg?: number, name?: string,
  ) => jsonPost<StripFeederRegistration>(
    "/strip-feeder-registrations",
    {
      scan_id: scanId,
      candidate_index: candidateIndex,
      proposal_index: proposalIndex,
      name: name || null,
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
    pocketIndex?: number, rotationCaptureOffsetsDeg: number[] = [],
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
      rotation_capture_offsets_deg: rotationCaptureOffsetsDeg,
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
  jobs: async () => (await request<{ jobs: Job[] }>("/jobs")).jobs,
  jobExecutions: async () =>
    (await request<{ executions: JobExecutionRun[] }>("/job-executions")).executions,
  placementOutcomes: async () =>
    (await request<{ outcomes: PlacementOutcomeRecord[] }>("/placement-outcomes"))
      .outcomes,
  jobExecution: (executionId: string) =>
    request<JobExecutionRun>(`/job-executions/${executionId}`),
  startJobExecution: (
    jobId: string, leaseId: string, calibrationId: string,
  ) => jsonPost<{ execution: JobExecutionRun; operation: Operation }>(
    `/jobs/${jobId}/executions`,
    {
      lease_id: leaseId,
      calibration_id: calibrationId,
      confirmed_external_safety_checks: true,
    },
  ),
  pauseJobExecution: (executionId: string) =>
    jsonPost<JobExecutionRun>(`/job-executions/${executionId}/pause`),
  cancelJobExecution: (executionId: string) =>
    jsonPost<JobExecutionRun>(`/job-executions/${executionId}/cancel`),
  resumeJobExecution: (executionId: string, leaseId: string) =>
    jsonPost<{ execution: JobExecutionRun; operation: Operation }>(
      `/job-executions/${executionId}/resume`,
      { lease_id: leaseId, confirmed_external_safety_checks: true },
    ),
  resolveJobExecutionStep: (
    executionId: string, stepIndex: number, action: "retry" | "skip", note?: string,
  ) => jsonPost<JobExecutionRun>(
    `/job-executions/${executionId}/steps/${stepIndex}/resolve`,
    { action, note: note ?? null },
  ),
  reanalyzeStripFeederReturn: (qualificationId: string) =>
    jsonPost<StripFeederPickupQualificationRun>(
      `/strip-feeder-pickup-qualifications/${qualificationId}/reanalyze-return`,
    ),
  stripFeederDraft: (registrationId: string) =>
    request<StripFeederRegistration>(
      `/strip-feeder-registrations/${registrationId}`,
    ),
  stripFeederRegistrations: async () =>
    (await request<{ registrations: StripFeederRegistration[] }>(
      "/strip-feeder-registrations",
    )).registrations,
  stripFeederPocketMap: async () =>
    (await request<{ registrations: StripFeederMapRegistration[] }>(
      "/strip-feeder-registrations/pocket-map",
    )).registrations,
  stripFeederRevisions: async (registrationId: string) =>
    (await request<{ revisions: StripFeederRevisionSummary[] }>(
      `/strip-feeder-registrations/${registrationId}/revisions`,
    )).revisions,
  stripFeederOccupancyReview: (registrationId: string) =>
    request<StripFeederOccupancyReviewQueue>(
      `/strip-feeder-registrations/${registrationId}/occupancy-review`,
    ),
  reviewStripFeederPocketOccupancy: (
    registrationId: string, pocketIndex: number, laneRevisionId: string,
    label: StripFeederPocketReviewLabel, note?: string,
  ) => jsonPut<{
    review: StripFeederPocketOccupancyReview;
    registration: StripFeederRegistration;
  }>(
    `/strip-feeder-registrations/${registrationId}/occupancy-review/${pocketIndex}`,
    { lane_revision_id: laneRevisionId, label, note: note || null },
  ),
  updateStripFeederAssignment: (
    registrationId: string, partKey: string | null, partRotationDeg: number | null,
  ) => jsonPut<StripFeederRegistration>(
    `/strip-feeder-registrations/${registrationId}/assignment`,
    { part_key: partKey, part_rotation_deg: partRotationDeg },
  ),
  parts: async () => (await request<{ parts: Part[] }>("/parts")).parts,
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
