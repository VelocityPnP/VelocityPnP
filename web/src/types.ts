export type MachineState =
  | "disconnected" | "connected" | "preflight" | "unhomed" | "homing"
  | "ready" | "running" | "paused" | "fault" | "e_stopped";

export interface Pose {
  x_mm: number;
  y_mm: number;
  z_mm: number;
  rotation_deg: number;
}

export interface JogTarget {
  x_mm: number;
  y_mm: number;
  z_mm: number;
  rotation_deg?: number;
}

export interface ControlLease {
  id: string;
  client_name: string;
  acquired_at: string;
  expires_at: string;
}

export interface MachineStatus {
  state: MachineState;
  simulated: boolean;
  profile_id: string;
  pose: Pose | null;
  homed: boolean;
  fault: { code: string; message: string; recoverable: boolean } | null;
  control_lease: ControlLease | null;
  calibration_revision: string | null;
}

export interface Device {
  kind: "serial_controller" | "camera";
  path: string;
  stable_path: string | null;
  vendor_id: string | null;
  product_id: string | null;
  serial: string | null;
  product: string | null;
  capture_capable: boolean | null;
  suggested_role: string | null;
}

export interface CameraControl {
  id: number;
  name: string;
  kind: string;
  minimum: number;
  maximum: number;
  step: number;
  default: number;
  flags: string;
  menu_items: Array<[number, string]>;
  current: number | boolean | string | null;
}

export interface Operation {
  id: string;
  kind: string;
  status: "pending" | "running" | "succeeded" | "failed" | "cancelled";
  message: string | null;
}

export interface CalibrationStep {
  id: string;
  title: string;
  description: string;
  dependencies: string[];
  requires_motion: boolean;
  requires_fixture: boolean;
}

export interface CalibrationSnapshot {
  id: string;
  machine_serial: string;
  profile_id: string;
  created_at: string;
  completed_steps: string[];
  residuals: Record<string, Residuals>;
  values: Record<string, unknown>;
  production_ready: boolean;
}

export interface Residuals {
  sample_count: number;
  rms_mm: number;
  p95_mm: number;
  max_mm: number;
}

export interface BoardDesign {
  id: string;
  name: string;
  source_version: number | null;
  width_mm: number | null;
  height_mm: number | null;
  placements: Array<{ reference: string; side: "top" | "bottom"; enabled: boolean; fiducial: boolean }>;
  warnings: string[];
}

export interface MachineEvent {
  sequence: number;
  at: string;
  kind: string;
  payload: unknown;
}
