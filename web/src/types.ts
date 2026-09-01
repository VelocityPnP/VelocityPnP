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
  outline_bounds: {
    min_x_mm: number;
    max_x_mm: number;
    min_y_mm: number;
    max_y_mm: number;
  } | null;
  placements: Array<{ reference: string; side: "top" | "bottom"; enabled: boolean; fiducial: boolean }>;
  warnings: string[];
}

export interface BoardInstance {
  id: string;
  board_id: string;
  label: string;
  side: "top" | "bottom";
  design_to_machine: { matrix: number[] } | null;
  surface_z_mm: number | null;
  placement_surface_offset_mm: number;
}

export interface PlacementReleaseSchedule {
  start_seating_offset_mm: number;
  minimum_seating_offset_mm: number;
  step_mm: number;
}

export interface Job {
  id: string;
  name: string;
  boards: BoardInstance[];
  placement_references: string[];
  placement_seating_offset_mm: number | null;
  placement_release: PlacementReleaseSchedule | null;
  status: string;
}

export type PlacementOutcomeDisposition = "good" | "offset" | "rotated"
  | "tombstoned" | "stacked" | "missing" | "stuck_on_nozzle" | "uncertain";

export interface JobExecutionStep {
  step_index: number;
  placement_reference: string;
  status: "pending" | "running" | "succeeded" | "needs_attention" | "skipped";
  execution_id: string | null;
  attempt_execution_ids: string[];
  outcome_disposition: PlacementOutcomeDisposition | null;
  board_contacted: boolean;
  message: string | null;
}

export interface JobExecutionRun {
  id: string;
  operation_id: string;
  job_id: string;
  calibration_id: string;
  simulated: boolean;
  status: "pending" | "running" | "paused" | "needs_attention"
    | "succeeded" | "completed_with_skips" | "cancelled";
  created_at: string;
  updated_at: string;
  steps: JobExecutionStep[];
  current_step_index: number | null;
  pause_requested: boolean;
  cancel_requested: boolean;
  message: string | null;
}

export interface BoardRegistrationFiducialObservation {
  reference: string;
  design_x_mm: number;
  design_y_mm: number;
  predicted_x_mm: number;
  predicted_y_mm: number;
  observed_x_px: number;
  observed_y_px: number;
  measured_x_mm: number;
  measured_y_mm: number;
  diameter_px: number;
  confidence: number;
  artifact_name: string;
}

export interface BoardRegistrationRun {
  id: string;
  operation_id: string;
  profile_id: string;
  calibration_id: string;
  source_scan_id: string;
  source_candidate_index: number;
  board_id: string;
  label: string;
  side: "top" | "bottom";
  status: Operation["status"];
  created_at: string;
  updated_at: string;
  sample_count: number;
  seed_center_x_mm: number;
  seed_center_y_mm: number;
  seed_rotation_deg: number;
  hypotheses: Array<{
    orientation_index: number;
    seed_rotation_deg: number;
    observations: BoardRegistrationFiducialObservation[];
    rejection_reason: string | null;
  }>;
  selected_rotation_deg: number | null;
  fit_residuals: {
    sample_count: number;
    rms_mm: number;
    p95_mm: number;
    max_mm: number;
  } | null;
  design_to_machine: { matrix: number[] } | null;
  board_instance: BoardInstance | null;
  passed: boolean;
  message: string | null;
}

export interface WorkspaceBoardDesignMatch {
  board_id: string;
  board_name: string;
  registration_method: "candidate_dimensions" | "visual_outline";
  rotation_deg: number;
  dimension_error_mm: number;
  dimension_error_fraction: number;
  confidence: number;
  center_x_mm: number | null;
  center_y_mm: number | null;
  outline_machine_mm: Array<[number, number]>;
  edge_support_fraction: number | null;
  orientation_ambiguous: boolean;
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

export interface StripFeederLaneProposal {
  proposal_index: number;
  tape_rotation_deg: number;
  sprocket_pitch_mm: number;
  sprocket_pitch_residual_p95_mm: number;
  sprocket_center_count: number;
  sprocket_centers_machine_mm: Array<[number, number]>;
  tape_width_mm: number | null;
  component_pitch_mm: number | null;
  pick_centerline_machine_mm: [[number, number], [number, number]] | null;
  pocket_centers_machine_mm: Array<[number, number]>;
  component_size_mm: [number, number] | null;
  pocket_observations: StripFeederPocketObservation[];
  present_count: number;
  absent_count: number;
  uncertain_count: number;
  confidence: number;
  review_required: boolean;
  findings: string[];
}

export type StripPocketOccupancy = "occupied" | "empty" | "uncertain";

export interface StripFeederPocketObservation {
  pocket_index: number;
  center_machine_mm: [number, number];
  occupancy: StripPocketOccupancy;
  confidence: number;
  component_outline_machine_mm: Array<[number, number]>;
  component_size_mm: [number, number] | null;
}

export interface StripFeederVisionReport {
  algorithm_version: string;
  source_scan_id: string;
  source_candidate_index: number;
  source_artifact_name: string;
  report_artifact_name: string;
  preview_artifact_name: string;
  created_at: string;
  proposals: StripFeederLaneProposal[];
  rejected_lattices: StripFeederRejectedLattice[];
  warnings: string[];
}

export interface StripFeederRejectedLattice {
  centerline_machine_mm: [[number, number], [number, number]];
  paired_sprocket_centerline_machine_mm: [[number, number], [number, number]];
  offset_mm: number;
  center_count: number;
  confidence: number;
  reason: string;
}

export interface StripFeederInventoryReport {
  algorithm_version: string;
  workspace_scan_id: string;
  source_mosaic_artifact_name: string;
  report_artifact_name: string;
  preview_artifact_name: string;
  created_at: string;
  source_scan_ids: string[];
  evidence_mosaics: Array<{
    scan_id: string;
    artifact_name: string;
    raster_transform: {
      top_left_x_mm: number;
      top_left_y_mm: number;
      x_mm_per_pixel: number;
      y_mm_per_pixel: number;
    };
  }>;
  preview_raster_transform: object | null;
  lane_count: number;
  populated_lane_count: number;
  present_count: number;
  absent_count: number;
  uncertain_count: number;
  coverage_complete: boolean;
  recommended_rescan_region: {
    x_min_mm: number;
    x_max_mm: number;
    y_min_mm: number;
    y_max_mm: number;
  } | null;
  lanes: Array<{
    inventory_index: number;
    source_scan_id: string;
    source_scan_ids: string[];
    source_candidate_index: number;
    source_proposal_index: number;
    strip_outline_machine_mm: Array<[number, number]>;
    negative_end_truncated: boolean;
    positive_end_truncated: boolean;
    proposal: StripFeederLaneProposal;
  }>;
  warnings: string[];
}

export interface StripFeederRegistration {
  id: string;
  profile_id: string;
  calibration_id: string;
  source_scan_id: string;
  source_candidate_index: number;
  source_proposal_index: number;
  created_at: string;
  updated_at: string;
  status:
    | "vision_draft"
    | "live_validation_required"
    | "placement_validation_required"
    | "ready";
  name: string;
  part_key: string | null;
  part_rotation_deg: number | null;
  tape_width_mm: number;
  tape_pitch_mm: number;
  tape_rotation_deg: number;
  pick_centerline_machine_mm: [[number, number], [number, number]];
  pocket_centers_machine_mm: Array<[number, number]>;
  pockets: Array<{
    pocket_index: number;
    nominal_center_machine_mm: [number, number];
    measured_center_machine_mm: [number, number] | null;
    occupancy: "occupied" | "empty" | "uncertain";
    occupancy_confidence: number;
    status: "available" | "empty" | "reserved" | "consumed" | "needs_review";
    attempt_count: number;
    consecutive_failure_count: number;
    reservation_id: string | null;
    last_qualification_id: string | null;
    last_outcome: "returned" | "consumed" | "retryable_failure" | "release_uncertain" | "interrupted" | null;
    updated_at: string;
  }>;
  selected_pocket_machine_mm: [number, number] | null;
  selected_pocket_index: number | null;
  pickup_z_mm: number | null;
  surveyed_part_size_mm: [number, number] | null;
  measured_part_size_mm: [number, number] | null;
  measured_part_rotation_deg: number | null;
  measured_part_confidence: number | null;
  active_feeder_id: string | null;
  latest_live_holdout_id: string | null;
  latest_pickup_qualification_id: string | null;
  pickup_sequence_passed: boolean;
  latest_pickup_sequence_qualification_ids: string[];
  latest_placement_validation_id: string | null;
  vision_confidence: number;
  vision_review_required: boolean;
  findings: string[];
}

export interface StripFeederLiveHoldoutReport {
  algorithm_version: string;
  predicted_pocket_machine_mm: [number, number];
  measured_pocket_machine_mm: [number, number] | null;
  correction_x_mm: number;
  correction_y_mm: number;
  correction_magnitude_mm: number;
  rotation_correction_deg: number;
  sprocket_center_count: number;
  sprocket_pitch_residual_p95_mm: number;
  occupancy: "occupied" | "empty" | "uncertain";
  occupancy_confidence: number;
  confidence: number;
  passed: boolean;
  artifact_name: string;
  findings: string[];
}

export interface StripFeederLiveHoldoutRun {
  id: string;
  operation_id: string;
  registration_id: string;
  profile_id: string;
  calibration_id: string;
  status: Operation["status"];
  created_at: string;
  updated_at: string;
  sample_count: number;
  pocket_index: number;
  predicted_pocket_machine_mm: [number, number];
  report: StripFeederLiveHoldoutReport | null;
  message: string | null;
}

export type PartOrientation =
  | { mode: "axis_symmetric" }
  | { mode: "feeder_referenced" }
  | {
      mode: "brighter_end" | "darker_end";
      minimum_contrast_luma: number;
      rotation_offset_deg: number;
    };

export interface HeldPartInspectionReport {
  algorithm_version: string;
  detected: boolean;
  center_x_px: number | null;
  center_y_px: number | null;
  centering_correction_x_mm: number | null;
  centering_correction_y_mm: number | null;
  centering_correction_mm: number | null;
  width_mm: number | null;
  length_mm: number | null;
  rotation_deg: number | null;
  orientation_rotation_deg: number | null;
  orientation_contrast_luma: number | null;
  orientation_confidence: number;
  changed_pixel_count: number;
  mean_difference_luma: number;
  confidence: number;
  passed: boolean;
  artifact_name: string;
  findings: string[];
}

export interface PlacedPartInspectionReport {
  algorithm_version: string;
  detected: boolean;
  nominal_center_x_px: number;
  nominal_center_y_px: number;
  target_registered: boolean;
  target_correction_x_mm: number | null;
  target_correction_y_mm: number | null;
  target_correction_mm: number | null;
  target_registration_confidence: number;
  expected_center_x_px: number;
  expected_center_y_px: number;
  expected_width_mm: number;
  expected_length_mm: number;
  expected_rotation_deg: number;
  center_x_px: number | null;
  center_y_px: number | null;
  offset_x_mm: number | null;
  offset_y_mm: number | null;
  offset_mm: number | null;
  width_mm: number | null;
  length_mm: number | null;
  rotation_deg: number | null;
  rotation_error_deg: number | null;
  orientation: PartOrientation;
  orientation_rotation_deg: number | null;
  orientation_contrast_luma: number | null;
  orientation_confidence: number;
  orientation_matched: boolean | null;
  changed_pixel_count: number;
  mean_difference_luma: number;
  body_change_fraction: number;
  change_containment: number;
  confidence: number;
  passed: boolean;
  artifact_name: string;
  findings: string[];
}

export interface HeldPartAlignmentReport {
  algorithm_version: string;
  orientation: PartOrientation;
  source_rotation_deg: number;
  target_rotation_deg: number;
  commanded_x_mm: number;
  commanded_y_mm: number;
  commanded_rotation_deg: number;
  final_centering_residual_mm: number | null;
  final_rotation_residual_deg: number | null;
  final_resolved_rotation_deg: number | null;
  passed: boolean;
  observations: Array<{
    iteration: number;
    head_machine_mm: [number, number];
    nozzle_rotation_offset_deg: number;
    inspection: HeldPartInspectionReport;
  }>;
  artifact_names: string[];
  findings: string[];
}

export interface StripFeederPickupQualificationReport {
  algorithm_version: string;
  nozzle_id: string;
  tip_id: string;
  sensor_id: string;
  pocket_machine_mm: [number, number];
  pickup_nozzle_z_mm: number | null;
  pickup_controller_z_mm: number | null;
  open_nozzle_raw_values: number[];
  pickup_samples: VacuumProbeSample[];
  held_raw_values: number[];
  held_median_raw_value: number | null;
  vacuum_acquired: boolean;
  vacuum_held_after_transfer: boolean;
  inspection: HeldPartInspectionReport | null;
  alignment: HeldPartAlignmentReport | null;
  disposition: "returned_to_pocket" | "dropped_at_fallback"
    | "placed_on_board" | "not_released";
  release_surface_machine_mm: [number, number] | null;
  release_raw_values: number[];
  released: boolean;
  nozzle_release_verification: {
    algorithm_version: string;
    attempt_count: number;
    retry_performed: boolean;
    nozzle_clear: boolean;
    passed: boolean;
    observations: HeldPartInspectionReport[];
    artifact_names: string[];
    findings: string[];
  } | null;
  return_verification: {
    algorithm_version: string;
    nominal_center_machine_mm: [number, number] | null;
    before_camera_pose: Pose | null;
    before_occupancy: "occupied" | "empty" | "uncertain";
    before_confidence: number;
    before_measured_center_machine_mm: [number, number] | null;
    before_center_offset_mm: number | null;
    after_camera_pose: Pose | null;
    after_occupancy: "occupied" | "empty" | "uncertain";
    after_confidence: number;
    after_measured_center_machine_mm: [number, number] | null;
    after_center_offset_mm: number | null;
    return_center_shift_mm: number | null;
    appearance_similarity: number | null;
    passed: boolean;
    artifact_name: string;
    findings: string[];
  } | null;
  passed: boolean;
  artifact_names: string[];
  findings: string[];
}

export interface VacuumProbeSample {
  nozzle_z_mm: number;
  controller_z_mm: number;
  raw_values: number[];
  median_raw_value: number;
  signal_delta_counts: number;
}

export interface StripFeederPickupQualificationRun {
  id: string;
  operation_id: string;
  registration_id: string;
  profile_id: string;
  calibration_id: string;
  status: Operation["status"];
  created_at: string;
  updated_at: string;
  nozzle_id: string;
  pocket_index: number;
  fallback_drop_surface_machine_mm: [number, number] | null;
  report: StripFeederPickupQualificationReport | null;
  message: string | null;
}

export interface StripFeederSourcePoseReport {
  algorithm_version: string;
  reference_qualification_id: string;
  reference_nominal_center_machine_mm: [number, number];
  reference_camera_pose: Pose;
  reference_occupancy: "occupied" | "empty" | "uncertain";
  reference_confidence: number;
  reference_measured_center_machine_mm: [number, number] | null;
  candidate_nominal_center_machine_mm: [number, number];
  candidate_camera_pose: Pose;
  candidate_occupancy: "occupied" | "empty" | "uncertain";
  candidate_confidence: number;
  candidate_measured_center_machine_mm: [number, number] | null;
  relative_center_shift_mm: number | null;
  appearance_similarity: number | null;
  minimum_appearance_similarity: number;
  passed: boolean;
  artifact_name: string;
  findings: string[];
}

export interface PlacementExecutionReport {
  algorithm_version: string;
  simulated: boolean;
  board_instance_id: string | null;
  placement_reference: string;
  part_key: string;
  feeder_id: string;
  nozzle_id: string;
  part_orientation: PartOrientation;
  feeder_part_rotation_deg: number;
  feeder_pick: Pose;
  board_place: Pose;
  part_height_mm: number;
  placement_surface_offset_mm: number;
  placement_seating_offset_mm: number;
  placement_release_seating_offsets_mm: number[];
  successful_placement_seating_offset_mm: number | null;
  placement_release_attempts: Array<{
    attempt_index: number;
    seating_offset_mm: number;
    nozzle_z_mm: number;
    controller_z_mm: number;
    release_raw_values: number[];
    sensor_released: boolean;
    retained_part_detected: boolean;
    nozzle_clear: boolean;
    vacuum_reacquired: boolean | null;
    hold_raw_values: number[];
    realignment: HeldPartAlignmentReport | null;
    artifact_names: string[];
  }>;
  pocket_index: number | null;
  pickup_open_nozzle_raw_values: number[];
  pickup_search_samples: VacuumProbeSample[];
  pickup_attempts: Array<{
    attempt_index: number;
    center_correction_mm: [number, number];
    head_machine_mm: [number, number];
    open_nozzle_raw_values: number[];
    samples: VacuumProbeSample[];
    contact: VacuumProbeSample | null;
  }>;
  source_pose_inspection: StripFeederSourcePoseReport | null;
  pickup_report: StripFeederPickupQualificationReport | null;
  board_target_registration: {
    algorithm_version: string;
    registered: boolean;
    nominal_center_x_px: number;
    nominal_center_y_px: number;
    center_x_px: number | null;
    center_y_px: number | null;
    correction_x_mm: number | null;
    correction_y_mm: number | null;
    correction_mm: number | null;
    confidence: number;
    passed: boolean;
    artifact_name: string;
    findings: string[];
  } | null;
  top_inspection: PlacedPartInspectionReport | null;
  artifact_names: string[];
  corrected_move_count: number;
  passed: boolean;
  stages: Array<{
    stage: "preflight" | "board_target_registration" | "pick_approach" | "pickup" | "bottom_inspection"
      | "place_approach" | "placement" | "release" | "top_inspection"
      | "complete";
    passed: boolean;
    message: string;
  }>;
  findings: string[];
}

export interface PlacementExecutionRun {
  id: string;
  operation_id: string;
  job_id: string;
  step_index: number;
  calibration_id: string;
  status: Operation["status"];
  created_at: string;
  updated_at: string;
  report: PlacementExecutionReport | null;
  message: string | null;
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

export interface WorkspaceInventoryBaseline {
  id: string;
  scan_id: string;
  profile_id: string;
  calibration_id: string;
  proposal_algorithm_version: string;
  activated_at: string;
  supersedes_baseline_id: string | null;
}

export interface WorkspaceChangeSetReport {
  algorithm_version: string;
  detection_mode: "semantic_proposals" | "baseline_guided_raster";
  baseline_id: string;
  baseline_scan_id: string;
  observation_scan_id: string;
  added_count: number;
  removed_count: number;
  modified_count: number;
  unchanged_count: number;
  reused_baseline_count: number;
  target_regions: Array<{
    x_min_mm: number;
    x_max_mm: number;
    y_min_mm: number;
    y_max_mm: number;
  }>;
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
  strip_feeder_inventory: {
    algorithm_version: string;
    report_artifact_name: string;
    preview_artifact_name: string;
    lane_count: number;
    populated_lane_count: number;
    present_count: number;
    absent_count: number;
    uncertain_count: number;
    coverage_complete: boolean;
    recommended_rescan_region: {
      x_min_mm: number;
      x_max_mm: number;
      y_min_mm: number;
      y_max_mm: number;
    } | null;
  } | null;
  change_set: {
    baseline_id: string;
    detection_mode: "semantic_proposals" | "baseline_guided_raster";
    added_count: number;
    removed_count: number;
    modified_count: number;
    unchanged_count: number;
    reused_baseline_count: number;
    target_region_count: number;
  } | null;
}

export interface MachineEvent {
  sequence: number;
  at: string;
  kind: string;
  payload: unknown;
}
