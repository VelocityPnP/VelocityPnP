import type {
  BoardDesign, CalibrationSnapshot, CalibrationStep, CameraControl, CameraProfile,
  CameraReproducibilityReport, CameraRole, ControlLease, Device, JogTarget, MachineStatus,
  Operation, Residuals, SensorReading, VisualHomingResult,
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
  lease: async (clientName = "VelocityPnP web UI") =>
    (await jsonPost<{ lease: ControlLease }>("/control/lease", { client_name: clientName, duration_seconds: 900 })).lease,
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
  completeCalibration: (
    snapshotId: string, stepId: string, residuals: Residuals | null, values: Record<string, unknown> = {},
  ) => jsonPost<CalibrationSnapshot>(`/calibrations/${snapshotId}/steps/${stepId}/complete`, {
    confirmed: true, residuals, values,
  }),
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
};
