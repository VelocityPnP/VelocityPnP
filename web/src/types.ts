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

export type CalibrationArtifactId =
  | "top_camera_reproducibility"
  | "bottom_camera_reproducibility"
  | "tooling_n1"
  | "tooling_n2"
  | "visual_homing_qualification"
  | "top_camera_working_plane_map"
  | "local_xy_direction_response"
  | "bottom_camera_local_map"
  | "reciprocal_camera_geometry"
  | "camera_ray_transfer"
  | "nozzle_axis_runout_n1"
  | "nozzle_axis_runout_n2"
  | "head_nozzle_offsets"
  | "nozzle_height_n1"
  | "nozzle_height_n2"
  | "runtime_tip_transform_n1"
  | "runtime_tip_transform_n2"
  | "vacuum_contact_n1"
  | "vacuum_contact_n2"
  | "vacuum_characterization_n1"
  | "vacuum_characterization_n2"
  | "top_camera_planar_model"
  | "bottom_camera_planar_model"
  | "installed_plate_geometry";

export interface CalibrationArtifact {
  kind: "captured" | "qualification" | "derived";
  recorded_at: string;
  source_run_id: string | null;
  algorithm_version: string;
  input_fingerprints: Record<string, string>;
  dependency_fingerprints: Partial<Record<CalibrationArtifactId, string>>;
  fingerprint: string;
  value: unknown;
}

export interface CalibrationArtifactPlan {
  targets: CalibrationArtifactId[];
  items: Array<{
    artifact: CalibrationArtifactId;
    action: "reuse" | "recompute" | "recapture" | "blocked";
    reason: string;
    requires_motion: boolean;
    estimated_duration_seconds: number;
  }>;
  requires_motion: boolean;
  estimated_duration_seconds: number;
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
    vacuum_sensor_characterization?: Partial<
      Record<"N1" | "N2", VacuumSensorCharacterizationReport>
    >;
    local_xy_direction_response?: LocalXyDirectionResponseReport;
    nozzle_height_model?: {
      camera_ray_transfer?: CameraRayTransferReport | null;
      models: NozzleHeightModel[];
    };
  };
  artifacts: Partial<Record<CalibrationArtifactId, CalibrationArtifact>>;
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

export interface VacuumStateStatistics {
  sample_count: number;
  minimum_raw_value: number;
  maximum_raw_value: number;
  mean_raw_value: number;
  median_raw_value: number;
  standard_deviation_counts: number;
  p05_raw_value: number;
  p95_raw_value: number;
}

export interface VacuumSensorCharacterizationReport {
  nozzle_id: "N1" | "N2";
  tip_id: string;
  sensor_id: string;
  contact_nozzle_z_mm: number;
  pump_off: VacuumStateStatistics;
  pump_on_open: VacuumStateStatistics;
  sealed_reference: VacuumStateStatistics;
  pump_off_release: VacuumStateStatistics;
  signal_polarity: "lower_is_more_vacuum" | "higher_is_more_vacuum";
  median_signal_delta_counts: number;
  robust_distribution_gap_counts: number;
  acquire_threshold_raw_value: number;
  release_threshold_raw_value: number;
  reference_seal_validated: boolean;
  known_part_validated: boolean;
  passed: boolean;
  observed_at: string;
}

export interface LocalXyAxisEstimate {
  axis: "x" | "y";
  sample_pairs: number;
  negative_mean_correction_x_mm: number;
  negative_mean_correction_y_mm: number;
  positive_mean_correction_x_mm: number;
  positive_mean_correction_y_mm: number;
  direction_gap_x_mm: number;
  direction_gap_y_mm: number;
  primary_axis_gap_mm: number;
  cross_axis_gap_mm: number;
  primary_axis_confidence_interval_95_mm: number;
  effect_resolved: boolean;
}

export interface LocalXyAxisDirectionReport {
  estimate: LocalXyAxisEstimate;
  stopping_reason: string;
  holdout: {
    negative_sample: LocalXyDirectionResponseSample;
    positive_sample: LocalXyDirectionResponseSample;
    negative_residual_x_mm: number;
    negative_residual_y_mm: number;
    positive_residual_x_mm: number;
    positive_residual_y_mm: number;
    direction_gap_mm: number;
    improved: boolean;
  };
  compensation_enabled: boolean;
  applied_negative_compensation_x_mm: number;
  applied_negative_compensation_y_mm: number;
  applied_positive_compensation_x_mm: number;
  applied_positive_compensation_y_mm: number;
}

export interface LocalXyDirectionResponseSample {
  axis: "x" | "y";
  approach_side: "negative" | "positive";
  pair: number;
  approach_distance_mm: number;
  feed_mm_per_min: number;
  commanded_x_mm: number;
  commanded_y_mm: number;
  observed_x_px: number;
  observed_y_px: number;
  residual_correction_x_mm: number;
  residual_correction_y_mm: number;
  observed_at: string;
}

export interface LocalXyDirectionResponseReport {
  reference_controller_x_mm: number;
  reference_controller_y_mm: number;
  reference_observed_x_px: number;
  reference_observed_y_px: number;
  pixel_error_to_machine_mm: [[number, number], [number, number]];
  axes: LocalXyAxisDirectionReport[];
  samples: LocalXyDirectionResponseSample[];
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

export interface PlanarCameraCalibrationReport {
  model: { kind: "affine" | "distortion_polynomial" };
  attempted_positions: number;
  detected_positions: number;
  rejected_positions: number;
  fit_residuals: Residuals;
  holdout_residuals: Residuals;
  affine_holdout_residuals: Residuals;
  selected_model_improved_holdout: boolean;
  passed: boolean;
}

export interface InstalledPlateGeometryReport {
  registrations: Array<{
    plate_id: string;
    role: "unspecified" | "staging" | "build";
    workspace_position: "unspecified" | "bottom" | "top";
    seeded_from_plate_id: string | null;
    attempted_positions: number;
    detected_positions: number;
    offset_x_mm: number | null;
    offset_y_mm: number | null;
    pixel_error_to_machine_mm: [[number, number], [number, number]] | null;
    passed: boolean;
  }>;
  attempted_targets: number;
  detected_targets: number;
  rejected_targets: number;
  qualified_plate_count: number;
  limitations: string[];
  passed: boolean;
}

export interface InstalledGeometryCalibrationReport {
  top_camera: PlanarCameraCalibrationReport;
  bottom_camera: PlanarCameraCalibrationReport;
  plate_geometry: InstalledPlateGeometryReport;
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
  installed_geometry_report: InstalledGeometryCalibrationReport | null;
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

export interface WorkspaceBoardDesignMatch {
  board_id: string;
  board_name: string;
  rotation_deg: number;
  dimension_error_mm: number;
  dimension_error_fraction: number;
  confidence: number;
}

export type WorkspaceObjectClass =
  | "board"
  | "feeder"
  | "tooling_station"
  | "calibration_fixture"
  | "fixed_machine_feature"
  | "unknown";

export interface WorkspaceObjectClassification {
  class: WorkspaceObjectClass;
  confidence: number;
  classifier_id: string;
  review_required: boolean;
  evidence: Array<{
    source: string;
    suggested_class: WorkspaceObjectClass;
    confidence: number;
    explanation: string;
  }>;
}

export interface WorkspaceBoardCandidate {
  candidate_index: number;
  region: {
    x_min_mm: number;
    x_max_mm: number;
    y_min_mm: number;
    y_max_mm: number;
  };
  outline_machine_mm: Array<[number, number]>;
  plate_id: string | null;
  plate_role: "staging" | "build" | "unspecified" | null;
  center_x_mm: number;
  center_y_mm: number;
  width_mm: number;
  height_mm: number;
  rotation_deg: number;
  evidence_cell_count: number;
  measured_cell_fraction: number;
  mean_surface_fraction: number;
  mean_height_confidence: number;
  fill_fraction: number;
  texture_energy_luma: number;
  board_likelihood: number;
  classification: WorkspaceObjectClassification | null;
  known_fixture_id: string | null;
  evidence_artifact_name: string | null;
  design_matches: WorkspaceBoardDesignMatch[];
}

export interface WorkspaceBoardDiscoveryReport {
  algorithm_version: string;
  source_mosaic_artifact_name: string;
  height_sources: Array<{ scan_id: string; artifact_name: string }>;
  installed_plate_fraction: number;
  raised_surface_max_fraction: number;
  board_design_count: number;
  candidates: WorkspaceBoardCandidate[];
  warnings: string[];
}

export interface WorkspaceScanReport {
  id: string;
  updated_at: string;
  status: "pending" | "running" | "succeeded" | "failed";
  mode: "precision" | "survey" | "reconnaissance" | "refinement";
  height_map: object | null;
  refinement_composite: { accepted: boolean } | null;
  board_discovery: {
    algorithm_version: string;
    candidate_count: number;
    matched_candidate_count: number;
  } | null;
}

export interface MachineEvent {
  sequence: number;
  at: string;
  kind: string;
  payload: unknown;
}
