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

export type CameraRole = "top_camera" | "bottom_camera";

export interface CameraProfile {
  machine_profile_id: string;
  role: CameraRole;
  width: number;
  height: number;
  frames_per_second: number;
  settle_frames: number;
  controls: Array<{ id: number; value: number | boolean }>;
  lighting_enabled: boolean;
  thresholds: {
    max_relative_luma_range: number;
    max_relative_focus_range: number;
    min_target_detection_rate: number;
    max_center_jitter_px: number;
  };
}

export interface CameraReproducibilityReport {
  requested_samples: number;
  captured_samples: number;
  capture_failures: number;
  mean_luma: number;
  relative_luma_range: number;
  mean_laplacian_variance: number;
  relative_focus_range: number;
  target_detection_rate: number;
  mean_target_x_px: number | null;
  mean_target_y_px: number | null;
  center_jitter_p95_px: number | null;
  passed: boolean;
}

export interface SensorReading {
  sensor_id: string;
  raw_value: number;
  unit: string;
  observed_at: string;
}

export interface VisualHomingPlan {
  target: { x_px: number; y_px: number; diameter_px: number; confidence: number };
  error_x_px: number;
  error_y_px: number;
  error_radius_px: number;
  correction_x_mm: number;
  correction_y_mm: number;
  correction_distance_mm: number;
  within_capture_bound: boolean;
  within_correction_bound: boolean;
  accepted: boolean;
}

export interface VisualHomingResult {
  applied: boolean;
  seed_pose: Pose;
  initial: VisualHomingPlan;
  corrected_pose?: Pose;
  verification?: VisualHomingPlan;
  final_pose?: Pose;
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
  automation: "automated" | "planned";
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

export interface CalibrationCheck {
  id: string;
  title: string;
  source: "controller" | "vision" | "operator";
  status: "pending" | "passed" | "failed";
  message: string;
  evidence: unknown;
}

export interface VisualHomingValidationSample {
  cycle: number;
  observed_at: string;
  initial_correction_mm: number;
  correction_count: number;
  verification_residual_mm: number;
  verification_residual_px: number;
  return_residual_mm: number;
  return_residual_px: number;
  return_error_x_px: number;
  return_error_y_px: number;
}

export interface VisualHomingValidationReport {
  visual_verification: Residuals;
  independent_return: Residuals;
  initial_correction_min_mm: number;
  initial_correction_mean_mm: number;
  initial_correction_max_mm: number;
  passed: boolean;
}

export interface AutomatedCalibrationRun {
  id: string;
  calibration_id: string;
  operation_id: string;
  profile_id: string;
  created_at: string;
  updated_at: string;
  status: "pending" | "running" | "succeeded" | "rejected" | "failed";
  phase: string;
  config: {
    camera_sample_count: number;
    tooling_sample_count: number;
    visual_homing_sample_count: number;
    excursion_x_mm: number;
    excursion_y_mm: number;
    excursion_feed_mm_per_min: number;
    visual_homing_p95_limit_mm: number;
  };
  checks: CalibrationCheck[];
  camera_reports: Record<string, unknown>;
  visual_homing_samples: VisualHomingValidationSample[];
  visual_homing_report: VisualHomingValidationReport | null;
  failure: string | null;
  simulated: boolean;
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
