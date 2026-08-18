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
  values: Record<string, unknown> & {
    vacuum_probe?: Partial<Record<"N1" | "N2", VacuumProbeReport>>;
    nozzle_height_model?: {
      camera_ray_transfer?: CameraRayTransferReport | null;
      models: NozzleHeightModel[];
    };
  };
  parent_id: string | null;
  production_ready: boolean;
}

export interface VacuumProbeReport {
  nozzle_id: "N1" | "N2";
  tip_id: string;
  sensor_id: string;
  reference_surface: string;
  nominal_target_surface_x_mm?: number;
  nominal_target_surface_y_mm?: number;
  target_surface_x_mm: number;
  target_surface_y_mm: number;
  target_centering_correction_x_mm?: number;
  target_centering_correction_y_mm?: number;
  target_centering_residual_mm?: number;
  target_observed_x_px?: number;
  target_observed_y_px?: number;
  target_observed_diameter_px?: number;
  head_controller_x_mm: number;
  head_controller_y_mm: number;
  controller_angle_deg: number;
  runout_correction_x_mm: number;
  runout_correction_y_mm: number;
  measured_tip_offset_x_mm?: number;
  measured_tip_offset_y_mm?: number;
  tip_offset_model_error_mm?: number;
  alignment_nozzle_z_mm?: number;
  height_correction_x_mm?: number;
  height_correction_y_mm?: number;
  height_model_applied?: boolean;
  open_nozzle_median_raw_value: number;
  minimum_signal_delta_counts: number;
  contact_nozzle_z_mm: number | null;
  contact_controller_z_mm: number | null;
  samples: Array<{
    nozzle_z_mm: number;
    controller_z_mm: number;
    median_raw_value: number;
    signal_delta_counts: number;
  }>;
  passed: boolean;
  observed_at: string;
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
  return_refinement_count?: number;
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

export interface BottomCameraMappingReport {
  pixel_error_to_machine_mm: [[number, number], [number, number]];
  observed_pixel_per_machine_mm: [[number, number], [number, number]];
  probe_distance_mm: number;
  returned_center_error_px: number;
  returned_center_error_mm: number;
}

export interface HeadFeatureCenterReport {
  feature_id: string;
  controller_x_mm: number;
  controller_y_mm: number;
  target_x_px: number;
  target_y_px: number;
  target_diameter_px: number;
  correction_count: number;
  verification_residual_mm: number;
}

export interface NozzleRunoutReport {
  nozzle_id: string;
  tip_id: string;
  rotation_axis: string;
  reference_controller_angle_deg: number;
  axis_controller_x_mm: number;
  axis_controller_y_mm: number;
  head_offset_x_mm: number;
  head_offset_y_mm: number;
  correction_cos_x_mm: number;
  correction_sin_x_mm: number;
  correction_cos_y_mm: number;
  correction_sin_y_mm: number;
  runout_radius_mm: number;
  runout_phase_deg: number;
  fit_residuals: Residuals;
  passed: boolean;
}

export interface NozzleHeightSample {
  nozzle_z_mm: number;
  controller_z_mm: number;
  target_x_px: number;
  target_y_px: number;
  target_diameter_px: number;
  local_mapping?: BottomCameraMappingReport;
  lateral_shift_x_mm: number;
  lateral_shift_y_mm: number;
}

export interface NozzleHeightModel {
  nozzle_id: string;
  reference_nozzle_z_mm: number;
  measured_min_nozzle_z_mm: number;
  measured_max_nozzle_z_mm: number;
  max_extrapolation_mm: number;
  axis_slope_x_mm_per_mm: number;
  axis_slope_y_mm_per_mm: number;
  reference_transfer_correction_x_mm: number;
  reference_transfer_correction_y_mm: number;
  nozzle_working_plane_correction_x_mm: number;
  nozzle_working_plane_correction_y_mm: number;
  transfer_observation_source_nozzle_id: string | null;
  transfer_observation_nozzle_z_mm: number | null;
  transfer_observation_error_x_mm: number | null;
  transfer_observation_error_y_mm: number | null;
  transfer_observation_uncertainty_mm: number | null;
  nozzle_residual_observation_nozzle_z_mm: number | null;
  nozzle_residual_observation_error_x_mm: number | null;
  nozzle_residual_observation_error_y_mm: number | null;
  nozzle_residual_observation_uncertainty_mm: number | null;
  fit_residuals: Residuals;
  samples: NozzleHeightSample[];
  passed: boolean;
}

export interface HeadGeometryReport {
  bottom_camera_mapping: BottomCameraMappingReport;
  top_camera_working_plane_mapping: BottomCameraMappingReport | null;
  top_camera_reciprocal_mapping: BottomCameraMappingReport;
  top_camera_center: HeadFeatureCenterReport;
  bottom_camera_reciprocal_observation: HeadFeatureCenterReport;
  camera_ray_transfer: CameraRayTransferReport | null;
  nozzles: NozzleRunoutReport[];
  nozzle_height_models: NozzleHeightModel[];
  n1_to_n2_x_mm: number;
  n1_to_n2_y_mm: number;
  combined_runout_fit_residuals: Residuals;
  passed: boolean;
}

export interface CameraRayTransferReport {
  top_camera_lens_controller_x_mm: number;
  top_camera_lens_controller_y_mm: number;
  bottom_camera_lens_controller_x_mm: number;
  bottom_camera_lens_controller_y_mm: number;
  top_working_plane_pixel_per_mm: number;
  bottom_camera_reference_diameter_mm: number;
  bottom_camera_reference_diameter_px: number;
  bottom_camera_reference_pixel_per_mm: number;
  working_plane_distance_ratio: number;
  relative_camera_displacement_x_mm: number;
  relative_camera_displacement_y_mm: number;
  transfer_correction_x_mm: number;
  transfer_correction_y_mm: number;
  transfer_correction_mm: number;
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
  head_geometry_report: HeadGeometryReport | null;
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
