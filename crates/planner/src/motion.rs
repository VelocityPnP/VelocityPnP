use serde::{Deserialize, Serialize};
use thiserror::Error;
use uuid::Uuid;
use velocitypnp_domain::{
    ApproachSide, Bounds, LinearAxis, LocalXyAxisDirectionReport, LocalXyDirectionResponseReport,
    PlanarXyDirectionResponseNode, PlanarXyDirectionResponseSurface, Pose,
};

const DIRECTION_EPSILON_MM: f64 = 1e-9;
const FEED_EPSILON_MM_PER_MIN: f64 = 1e-6;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotionMode {
    /// Interpret the target directly in controller coordinates.
    Raw,
    /// Preserve logical coordinates and apply validated calibration.
    Calibrated,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MotionCalibration {
    identity: MotionCalibrationIdentity,
    x: Vec<AxisDirectionCompensation>,
    y: Vec<AxisDirectionCompensation>,
    datum_x_approach: ApproachSide,
    datum_y_approach: ApproachSide,
    datum_approach_feed_mm_per_min: f64,
    datum_compensation_mm: [f64; 2],
    planar_surfaces: Vec<PlanarXyMotionSurface>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MotionCalibrationIdentity {
    pub snapshot_id: Uuid,
    pub artifact_fingerprint: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
struct AxisDirectionCompensation {
    feed_mm_per_min: f64,
    negative_approach_mm: [f64; 2],
    positive_approach_mm: [f64; 2],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlanarXyMotionSurface {
    feed_mm_per_min: f64,
    x_distances_mm: Vec<f64>,
    y_distances_mm: Vec<f64>,
    nodes: Vec<PlanarXyDirectionResponseNode>,
}

impl PlanarXyMotionSurface {
    pub fn from_report(report: &PlanarXyDirectionResponseSurface) -> Result<Self, MotionPlanError> {
        if !report.passed
            || !report.feed_mm_per_min.is_finite()
            || report.feed_mm_per_min <= 0.0
            || !report.maximum_validation_residual_mm.is_finite()
            || !report.independent_validation_mean_residual_mm.is_finite()
            || !report
                .independent_validation_maximum_residual_mm
                .is_finite()
            || report.independent_validation_mean_residual_mm < 0.0
            || report.independent_validation_maximum_residual_mm < 0.0
            || report.maximum_validation_residual_mm < 0.0
        {
            return Err(MotionPlanError::InvalidPlanarSurface);
        }
        validate_distance_axis(&report.x_distances_mm)?;
        validate_distance_axis(&report.y_distances_mm)?;
        let expected_nodes = report.x_distances_mm.len() * report.y_distances_mm.len() * 4;
        if report.nodes.len() != expected_nodes {
            return Err(MotionPlanError::IncompletePlanarSurface);
        }
        for x_approach in [ApproachSide::Negative, ApproachSide::Positive] {
            for y_approach in [ApproachSide::Negative, ApproachSide::Positive] {
                for x_distance_mm in &report.x_distances_mm {
                    for y_distance_mm in &report.y_distances_mm {
                        let matches = report
                            .nodes
                            .iter()
                            .filter(|node| {
                                node.x_approach == x_approach
                                    && node.y_approach == y_approach
                                    && (node.x_distance_mm - x_distance_mm).abs()
                                        < DIRECTION_EPSILON_MM
                                    && (node.y_distance_mm - y_distance_mm).abs()
                                        < DIRECTION_EPSILON_MM
                            })
                            .count();
                        if matches != 1 {
                            return Err(MotionPlanError::IncompletePlanarSurface);
                        }
                    }
                }
            }
        }
        if report.nodes.iter().any(|node| {
            !node.qualified
                || [
                    node.x_distance_mm,
                    node.y_distance_mm,
                    node.correction_x_mm,
                    node.correction_y_mm,
                    node.validation_residual_x_mm,
                    node.validation_residual_y_mm,
                    node.validation_residual_mm,
                ]
                .iter()
                .any(|value| !value.is_finite())
                || node.x_distance_mm <= 0.0
                || node.y_distance_mm <= 0.0
                || node.validation_residual_mm < 0.0
                || (node
                    .validation_residual_x_mm
                    .hypot(node.validation_residual_y_mm)
                    - node.validation_residual_mm)
                    .abs()
                    > DIRECTION_EPSILON_MM
        }) {
            return Err(MotionPlanError::InvalidPlanarSurface);
        }
        let observed_maximum = report
            .nodes
            .iter()
            .map(|node| node.validation_residual_mm)
            .fold(0.0_f64, f64::max);
        if report.independent_validation_sample_count == 0
            || report.independent_validation_mean_residual_mm
                > report.independent_validation_maximum_residual_mm
            || (observed_maximum - report.maximum_validation_residual_mm).abs()
                > DIRECTION_EPSILON_MM
        {
            return Err(MotionPlanError::InvalidPlanarSurface);
        }
        Ok(Self {
            feed_mm_per_min: report.feed_mm_per_min,
            x_distances_mm: report.x_distances_mm.clone(),
            y_distances_mm: report.y_distances_mm.clone(),
            nodes: report.nodes.clone(),
        })
    }

    pub fn feed_mm_per_min(&self) -> f64 {
        self.feed_mm_per_min
    }

    pub fn supports(&self, x_distance_mm: f64, y_distance_mm: f64) -> bool {
        distance_bracket(&self.x_distances_mm, x_distance_mm).is_some()
            && distance_bracket(&self.y_distances_mm, y_distance_mm).is_some()
    }

    fn exceeds_maximum(&self, x_distance_mm: f64, y_distance_mm: f64) -> bool {
        x_distance_mm > *self.x_distances_mm.last().unwrap() + DIRECTION_EPSILON_MM
            || y_distance_mm > *self.y_distances_mm.last().unwrap() + DIRECTION_EPSILON_MM
    }

    pub fn correction(
        &self,
        x_approach: ApproachSide,
        y_approach: ApproachSide,
        x_distance_mm: f64,
        y_distance_mm: f64,
    ) -> Result<[f64; 2], MotionPlanError> {
        let (x_low, x_high, x_weight) = distance_bracket(&self.x_distances_mm, x_distance_mm)
            .ok_or(MotionPlanError::UnqualifiedPlanarDistance)?;
        let (y_low, y_high, y_weight) = distance_bracket(&self.y_distances_mm, y_distance_mm)
            .ok_or(MotionPlanError::UnqualifiedPlanarDistance)?;
        let node = |x_index: usize, y_index: usize| {
            self.nodes
                .iter()
                .find(|node| {
                    node.x_approach == x_approach
                        && node.y_approach == y_approach
                        && (node.x_distance_mm - self.x_distances_mm[x_index]).abs()
                            < DIRECTION_EPSILON_MM
                        && (node.y_distance_mm - self.y_distances_mm[y_index]).abs()
                            < DIRECTION_EPSILON_MM
                })
                .ok_or(MotionPlanError::IncompletePlanarSurface)
        };
        let low_low = node(x_low, y_low)?;
        let high_low = node(x_high, y_low)?;
        let low_high = node(x_low, y_high)?;
        let high_high = node(x_high, y_high)?;
        let interpolate = |component: fn(&PlanarXyDirectionResponseNode) -> f64| {
            let low = linear_interpolate(component(low_low), component(high_low), x_weight);
            let high = linear_interpolate(component(low_high), component(high_high), x_weight);
            linear_interpolate(low, high, y_weight)
        };
        Ok([
            interpolate(|node| node.correction_x_mm),
            interpolate(|node| node.correction_y_mm),
        ])
    }
}

fn validate_distance_axis(distances_mm: &[f64]) -> Result<(), MotionPlanError> {
    if distances_mm.len() < 2
        || distances_mm
            .iter()
            .any(|distance| !distance.is_finite() || *distance <= 0.0)
        || distances_mm.windows(2).any(|pair| pair[0] >= pair[1])
    {
        return Err(MotionPlanError::InvalidPlanarSurface);
    }
    Ok(())
}

fn distance_bracket(distances_mm: &[f64], distance_mm: f64) -> Option<(usize, usize, f64)> {
    if !distance_mm.is_finite()
        || distance_mm < *distances_mm.first()?
        || distance_mm > *distances_mm.last()?
    {
        return None;
    }
    let high = distances_mm.partition_point(|candidate| *candidate < distance_mm);
    if high == 0 {
        return Some((0, 0, 0.0));
    }
    if high == distances_mm.len() {
        let last = high - 1;
        return Some((last, last, 0.0));
    }
    if (distances_mm[high] - distance_mm).abs() < DIRECTION_EPSILON_MM {
        return Some((high, high, 0.0));
    }
    let low = high - 1;
    let weight = (distance_mm - distances_mm[low]) / (distances_mm[high] - distances_mm[low]);
    Some((low, high, weight))
}

fn linear_interpolate(low: f64, high: f64, weight: f64) -> f64 {
    low + (high - low) * weight
}

impl MotionCalibration {
    pub fn from_local_direction_response(
        snapshot_id: Uuid,
        artifact_fingerprint: impl Into<String>,
        report: &LocalXyDirectionResponseReport,
    ) -> Result<Self, MotionPlanError> {
        let artifact_fingerprint = artifact_fingerprint.into();
        if artifact_fingerprint.len() != 64
            || !artifact_fingerprint
                .bytes()
                .all(|byte| byte.is_ascii_hexdigit())
        {
            return Err(MotionPlanError::InvalidCalibrationFingerprint);
        }
        if !report.passed {
            return Err(MotionPlanError::CalibrationNotPassed);
        }
        let x = axis_compensation(&report.axes, LinearAxis::X)?;
        let y = axis_compensation(&report.axes, LinearAxis::Y)?;
        if report.axes.len() != 2 {
            return Err(MotionPlanError::DuplicateOrUnknownAxis);
        }
        if !report.datum_approach_feed_mm_per_min.is_finite()
            || report.datum_approach_feed_mm_per_min <= 0.0
            || report.datum_approach_distance_mm <= 0.0
        {
            return Err(MotionPlanError::InvalidDatumAnchor);
        }
        let component = |items: &[AxisDirectionCompensation], side, axis| {
            let compensation = items
                .iter()
                .find(|item| {
                    (item.feed_mm_per_min - report.datum_approach_feed_mm_per_min).abs()
                        < FEED_EPSILON_MM_PER_MIN
                })
                .ok_or(MotionPlanError::UnqualifiedFeed {
                    axis,
                    feed_mm_per_min: report.datum_approach_feed_mm_per_min,
                })?;
            Ok::<_, MotionPlanError>(match side {
                ApproachSide::Negative => compensation.negative_approach_mm,
                ApproachSide::Positive => compensation.positive_approach_mm,
            })
        };
        let datum_x = component(&x, report.datum_x_approach, LinearAxis::X)?;
        let datum_y = component(&y, report.datum_y_approach, LinearAxis::Y)?;
        let planar_surfaces = report
            .planar_surfaces
            .iter()
            .map(PlanarXyMotionSurface::from_report)
            .collect::<Result<Vec<_>, _>>()?;
        if planar_surfaces.iter().enumerate().any(|(index, surface)| {
            planar_surfaces[..index].iter().any(|prior| {
                (prior.feed_mm_per_min - surface.feed_mm_per_min).abs() < FEED_EPSILON_MM_PER_MIN
            })
        }) {
            return Err(MotionPlanError::DuplicatePlanarFeed);
        }
        Ok(Self {
            identity: MotionCalibrationIdentity {
                snapshot_id,
                artifact_fingerprint,
            },
            x,
            y,
            datum_x_approach: report.datum_x_approach,
            datum_y_approach: report.datum_y_approach,
            datum_approach_feed_mm_per_min: report.datum_approach_feed_mm_per_min,
            datum_compensation_mm: [datum_x[0] + datum_y[0], datum_x[1] + datum_y[1]],
            planar_surfaces,
        })
    }

    fn correction(
        &self,
        axis: LinearAxis,
        side: ApproachSide,
        feed_mm_per_min: f64,
    ) -> Result<[f64; 2], MotionPlanError> {
        let compensations = match axis {
            LinearAxis::X => &self.x,
            LinearAxis::Y => &self.y,
        };
        let compensation = compensations
            .iter()
            .find(|item| (item.feed_mm_per_min - feed_mm_per_min).abs() < FEED_EPSILON_MM_PER_MIN)
            .ok_or(MotionPlanError::UnqualifiedFeed {
                axis,
                feed_mm_per_min,
            })?;
        Ok(match side {
            ApproachSide::Negative => compensation.negative_approach_mm,
            ApproachSide::Positive => compensation.positive_approach_mm,
        })
    }

    fn maximum_controller_margin_mm(&self) -> [f64; 2] {
        let mut maximum = [0.0_f64; 2];
        for x in &self.x {
            for x_correction in [x.negative_approach_mm, x.positive_approach_mm] {
                for y in &self.y {
                    for y_correction in [y.negative_approach_mm, y.positive_approach_mm] {
                        for axis in 0..2 {
                            let correction = x_correction[axis] + y_correction[axis]
                                - self.datum_compensation_mm[axis];
                            maximum[axis] = maximum[axis].max(correction.abs());
                        }
                    }
                }
            }
        }
        for surface in &self.planar_surfaces {
            for node in &surface.nodes {
                maximum[0] = maximum[0].max(node.correction_x_mm.abs());
                maximum[1] = maximum[1].max(node.correction_y_mm.abs());
            }
        }
        maximum
    }

    fn planar_correction(
        &self,
        x_approach: ApproachSide,
        y_approach: ApproachSide,
        x_distance_mm: f64,
        y_distance_mm: f64,
        feed_mm_per_min: f64,
    ) -> Result<Option<[f64; 2]>, MotionPlanError> {
        let Some(surface) = self.planar_surfaces.iter().find(|surface| {
            (surface.feed_mm_per_min - feed_mm_per_min).abs() < FEED_EPSILON_MM_PER_MIN
        }) else {
            return Ok(None);
        };
        if surface.exceeds_maximum(x_distance_mm, y_distance_mm) {
            return Err(MotionPlanError::UnqualifiedPlanarDistance);
        }
        if !surface.supports(x_distance_mm, y_distance_mm) {
            return Ok(None);
        }
        surface
            .correction(x_approach, y_approach, x_distance_mm, y_distance_mm)
            .map(Some)
    }
}

fn axis_compensation(
    axes: &[LocalXyAxisDirectionReport],
    wanted: LinearAxis,
) -> Result<Vec<AxisDirectionCompensation>, MotionPlanError> {
    let mut matches = axes.iter().filter(|axis| axis.axis == wanted);
    let axis = matches.next().ok_or(MotionPlanError::MissingAxis(wanted))?;
    if matches.next().is_some() {
        return Err(MotionPlanError::DuplicateOrUnknownAxis);
    }
    if axis.feeds.is_empty() {
        return Err(MotionPlanError::MissingFeedCompensations(wanted));
    }
    let mut compensations = Vec::with_capacity(axis.feeds.len());
    for report in &axis.feeds {
        let negative = [
            report.applied_negative_compensation_x_mm,
            report.applied_negative_compensation_y_mm,
        ];
        let positive = [
            report.applied_positive_compensation_x_mm,
            report.applied_positive_compensation_y_mm,
        ];
        if !report.feed_mm_per_min.is_finite()
            || report.feed_mm_per_min <= 0.0
            || negative
                .into_iter()
                .chain(positive)
                .any(|value| !value.is_finite())
        {
            return Err(MotionPlanError::NonFiniteCalibration);
        }
        if !report.qualified || report.estimate.axis != wanted {
            return Err(MotionPlanError::UnvalidatedCompensation {
                axis: wanted,
                feed_mm_per_min: report.feed_mm_per_min,
            });
        }
        if !report.compensation_enabled
            && negative
                .into_iter()
                .chain(positive)
                .any(|value| value != 0.0)
        {
            return Err(MotionPlanError::DisabledCompensationIsNonzero(wanted));
        }
        if compensations
            .iter()
            .any(|prior: &AxisDirectionCompensation| {
                (prior.feed_mm_per_min - report.feed_mm_per_min).abs() < FEED_EPSILON_MM_PER_MIN
            })
        {
            return Err(MotionPlanError::DuplicateFeed(wanted));
        }
        compensations.push(AxisDirectionCompensation {
            feed_mm_per_min: report.feed_mm_per_min,
            negative_approach_mm: negative,
            positive_approach_mm: positive,
        });
    }
    Ok(compensations)
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MotionState {
    pub logical_pose: Pose,
    pub controller_pose: Pose,
    pub calibration: Option<MotionCalibrationIdentity>,
    pub x_approach: Option<ApproachSide>,
    pub y_approach: Option<ApproachSide>,
    pub x_approach_feed_mm_per_min: Option<f64>,
    pub y_approach_feed_mm_per_min: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MotionCorrection {
    pub local_direction_x_mm: f64,
    pub local_direction_y_mm: f64,
}

impl MotionCorrection {
    pub fn applied(self) -> bool {
        self.local_direction_x_mm != 0.0 || self.local_direction_y_mm != 0.0
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlannedMotion {
    pub mode: MotionMode,
    pub calibration: Option<MotionCalibrationIdentity>,
    pub logical_start: Pose,
    pub logical_target: Pose,
    pub controller_start: Pose,
    pub controller_target: Pose,
    pub feed_mm_per_min: f64,
    pub correction: MotionCorrection,
    pub x_approach: Option<ApproachSide>,
    pub y_approach: Option<ApproachSide>,
    pub x_approach_feed_mm_per_min: Option<f64>,
    pub y_approach_feed_mm_per_min: Option<f64>,
}

struct CalibratedTarget {
    controller_target: Pose,
    correction: MotionCorrection,
    x_approach: Option<ApproachSide>,
    y_approach: Option<ApproachSide>,
    x_approach_feed_mm_per_min: Option<f64>,
    y_approach_feed_mm_per_min: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct MotionPlanner {
    bounds: Bounds,
    calibration: Option<MotionCalibration>,
}

impl MotionPlanner {
    pub fn new(bounds: Bounds, calibration: Option<MotionCalibration>) -> Self {
        Self {
            bounds,
            calibration,
        }
    }

    pub fn initial_state(&self, controller_pose: Pose) -> Result<MotionState, MotionPlanError> {
        if !controller_pose.finite() {
            return Err(MotionPlanError::NonFiniteControllerPose);
        }
        if !self.bounds.contains(controller_pose) {
            return Err(MotionPlanError::ControllerOutsideBounds);
        }
        let (x_approach, y_approach, x_feed, y_feed) =
            self.calibration
                .as_ref()
                .map_or((None, None, None, None), |calibration| {
                    (
                        Some(calibration.datum_x_approach),
                        Some(calibration.datum_y_approach),
                        Some(calibration.datum_approach_feed_mm_per_min),
                        Some(calibration.datum_approach_feed_mm_per_min),
                    )
                });
        Ok(MotionState {
            logical_pose: controller_pose,
            controller_pose,
            calibration: self.calibration.as_ref().map(|item| item.identity.clone()),
            x_approach,
            y_approach,
            x_approach_feed_mm_per_min: x_feed,
            y_approach_feed_mm_per_min: y_feed,
        })
    }

    pub fn plan(
        &self,
        state: &MotionState,
        logical_target: Pose,
        feed_mm_per_min: f64,
        mode: MotionMode,
    ) -> Result<PlannedMotion, MotionPlanError> {
        if state.calibration != self.calibration.as_ref().map(|item| item.identity.clone()) {
            return Err(MotionPlanError::CalibrationChanged);
        }
        if !logical_target.finite() {
            return Err(MotionPlanError::NonFiniteLogicalPose);
        }
        if !feed_mm_per_min.is_finite() || feed_mm_per_min <= 0.0 {
            return Err(MotionPlanError::InvalidFeed);
        }
        if !self.bounds.contains(logical_target) {
            return Err(MotionPlanError::LogicalOutsideBounds);
        }

        let (
            controller_target,
            correction,
            x_approach,
            y_approach,
            x_approach_feed_mm_per_min,
            y_approach_feed_mm_per_min,
        ) = match mode {
            MotionMode::Raw => (
                logical_target,
                MotionCorrection {
                    local_direction_x_mm: 0.0,
                    local_direction_y_mm: 0.0,
                },
                None,
                None,
                None,
                None,
            ),
            MotionMode::Calibrated => {
                let target = self.calibrated_target(state, logical_target, feed_mm_per_min)?;
                (
                    target.controller_target,
                    target.correction,
                    target.x_approach,
                    target.y_approach,
                    target.x_approach_feed_mm_per_min,
                    target.y_approach_feed_mm_per_min,
                )
            }
        };
        if !self.bounds.contains(controller_target) {
            return Err(MotionPlanError::ControllerOutsideBounds);
        }
        Ok(PlannedMotion {
            mode,
            calibration: state.calibration.clone(),
            logical_start: state.logical_pose,
            logical_target,
            controller_start: state.controller_pose,
            controller_target,
            feed_mm_per_min,
            correction,
            x_approach,
            y_approach,
            x_approach_feed_mm_per_min,
            y_approach_feed_mm_per_min,
        })
    }

    pub fn commit(
        &self,
        state: &mut MotionState,
        planned: &PlannedMotion,
        actual_controller_pose: Pose,
    ) -> Result<(), MotionPlanError> {
        if state.calibration != self.calibration.as_ref().map(|item| item.identity.clone()) {
            return Err(MotionPlanError::CalibrationChanged);
        }
        if state.logical_pose != planned.logical_start
            || state.controller_pose != planned.controller_start
            || state.calibration != planned.calibration
        {
            return Err(MotionPlanError::StateChanged);
        }
        if !actual_controller_pose.finite() {
            return Err(MotionPlanError::NonFiniteControllerPose);
        }
        if !self.bounds.contains(actual_controller_pose) {
            return Err(MotionPlanError::ControllerOutsideBounds);
        }
        state.logical_pose = planned.logical_target;
        state.controller_pose = actual_controller_pose;
        state.x_approach = planned.x_approach;
        state.y_approach = planned.y_approach;
        state.x_approach_feed_mm_per_min = planned.x_approach_feed_mm_per_min;
        state.y_approach_feed_mm_per_min = planned.y_approach_feed_mm_per_min;
        Ok(())
    }

    fn calibrated_target(
        &self,
        state: &MotionState,
        logical_target: Pose,
        feed_mm_per_min: f64,
    ) -> Result<CalibratedTarget, MotionPlanError> {
        let x_move = approach_side(logical_target.x_mm - state.logical_pose.x_mm);
        let y_move = approach_side(logical_target.y_mm - state.logical_pose.y_mm);
        let x_approach = x_move.or(state.x_approach);
        let y_approach = y_move.or(state.y_approach);
        let x_feed = x_move
            .map(|_| feed_mm_per_min)
            .or(state.x_approach_feed_mm_per_min);
        let y_feed = y_move
            .map(|_| feed_mm_per_min)
            .or(state.y_approach_feed_mm_per_min);
        let mut correction = [0.0, 0.0];
        if let Some(calibration) = &self.calibration {
            let planar = match (x_move, y_move) {
                (Some(x_side), Some(y_side)) => calibration.planar_correction(
                    x_side,
                    y_side,
                    (logical_target.x_mm - state.logical_pose.x_mm).abs(),
                    (logical_target.y_mm - state.logical_pose.y_mm).abs(),
                    feed_mm_per_min,
                )?,
                _ => None,
            };
            if let Some(planar) = planar {
                correction = planar;
            } else {
                for (axis, approach) in [
                    (LinearAxis::X, x_approach.zip(x_feed)),
                    (LinearAxis::Y, y_approach.zip(y_feed)),
                ] {
                    if let Some((side, feed)) = approach {
                        let component = calibration.correction(axis, side, feed)?;
                        correction[0] += component[0];
                        correction[1] += component[1];
                    }
                }
                // Fine home assigns the nominal datum after this exact approach.
                // Remove that observed travel response from all later targets.
                correction[0] -= calibration.datum_compensation_mm[0];
                correction[1] -= calibration.datum_compensation_mm[1];
            }
        }
        Ok(CalibratedTarget {
            controller_target: Pose {
                x_mm: logical_target.x_mm + correction[0],
                y_mm: logical_target.y_mm + correction[1],
                ..logical_target
            },
            correction: MotionCorrection {
                local_direction_x_mm: correction[0],
                local_direction_y_mm: correction[1],
            },
            x_approach,
            y_approach,
            x_approach_feed_mm_per_min: x_feed,
            y_approach_feed_mm_per_min: y_feed,
        })
    }

    pub fn bounds(&self) -> Bounds {
        self.bounds
    }

    /// Maximum X/Y space needed for any calibrated controller correction.
    pub fn maximum_controller_margin_mm(&self) -> [f64; 2] {
        self.calibration
            .as_ref()
            .map_or([0.0, 0.0], MotionCalibration::maximum_controller_margin_mm)
    }
}

fn approach_side(delta_mm: f64) -> Option<ApproachSide> {
    if delta_mm > DIRECTION_EPSILON_MM {
        // A positive move arrives from the negative side of the target.
        Some(ApproachSide::Negative)
    } else if delta_mm < -DIRECTION_EPSILON_MM {
        Some(ApproachSide::Positive)
    } else {
        None
    }
}

#[derive(Debug, Error, PartialEq)]
pub enum MotionPlanError {
    #[error("local direction-response calibration did not pass")]
    CalibrationNotPassed,
    #[error("local direction-response artifact fingerprint is invalid")]
    InvalidCalibrationFingerprint,
    #[error("local direction-response datum anchor is invalid")]
    InvalidDatumAnchor,
    #[error("local direction-response calibration has no {0:?} axis")]
    MissingAxis(LinearAxis),
    #[error("local direction-response calibration has no {0:?} feed data")]
    MissingFeedCompensations(LinearAxis),
    #[error("local direction-response calibration has duplicate or unknown axes")]
    DuplicateOrUnknownAxis,
    #[error("local direction-response calibration repeats a feed for {0:?}")]
    DuplicateFeed(LinearAxis),
    #[error("local direction-response calibration contains a non-finite value")]
    NonFiniteCalibration,
    #[error("{axis:?} compensation at {feed_mm_per_min:.0} mm/min lacks holdout evidence")]
    UnvalidatedCompensation {
        axis: LinearAxis,
        feed_mm_per_min: f64,
    },
    #[error("{axis:?} has no qualified compensation at {feed_mm_per_min:.0} mm/min")]
    UnqualifiedFeed {
        axis: LinearAxis,
        feed_mm_per_min: f64,
    },
    #[error("disabled {0:?} compensation is nonzero")]
    DisabledCompensationIsNonzero(LinearAxis),
    #[error("planar XY response surface is invalid")]
    InvalidPlanarSurface,
    #[error("planar XY response surface does not contain each grid node once")]
    IncompletePlanarSurface,
    #[error("planar XY response repeats a feed")]
    DuplicatePlanarFeed,
    #[error("XY move distance is outside the planar response surface")]
    UnqualifiedPlanarDistance,
    #[error("logical target contains a non-finite coordinate")]
    NonFiniteLogicalPose,
    #[error("controller pose contains a non-finite coordinate")]
    NonFiniteControllerPose,
    #[error("motion feed must be finite and greater than zero")]
    InvalidFeed,
    #[error("logical target is outside configured machine bounds")]
    LogicalOutsideBounds,
    #[error("corrected controller target is outside configured machine bounds")]
    ControllerOutsideBounds,
    #[error("motion calibration changed while a sequence was active")]
    CalibrationChanged,
    #[error("motion state changed before the planned move was committed")]
    StateChanged,
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;
    use velocitypnp_domain::{
        LocalXyAxisEstimate, LocalXyCompensationHoldout, LocalXyDirectionResponseSample,
        LocalXyFeedDirectionReport,
    };

    fn bounds() -> Bounds {
        Bounds {
            x_min_mm: 0.0,
            x_max_mm: 100.0,
            y_min_mm: 0.0,
            y_max_mm: 100.0,
            z_min_mm: 0.0,
            z_max_mm: 50.0,
        }
    }

    fn sample(axis: LinearAxis, side: ApproachSide) -> LocalXyDirectionResponseSample {
        LocalXyDirectionResponseSample {
            axis,
            approach_side: side,
            pair: 1,
            approach_distance_mm: 1.0,
            feed_mm_per_min: 300.0,
            commanded_x_mm: 50.0,
            commanded_y_mm: 50.0,
            observed_x_px: 320.0,
            observed_y_px: 240.0,
            residual_correction_x_mm: 0.0,
            residual_correction_y_mm: 0.0,
            observed_at: Utc::now(),
        }
    }

    fn axis_report(
        axis: LinearAxis,
        negative: [f64; 2],
        positive: [f64; 2],
    ) -> LocalXyAxisDirectionReport {
        let estimate = LocalXyAxisEstimate {
            axis,
            sample_pairs: 3,
            negative_mean_correction_x_mm: negative[0],
            negative_mean_correction_y_mm: negative[1],
            positive_mean_correction_x_mm: positive[0],
            positive_mean_correction_y_mm: positive[1],
            direction_gap_x_mm: 0.0,
            direction_gap_y_mm: 0.0,
            primary_axis_gap_mm: 0.05,
            cross_axis_gap_mm: 0.01,
            primary_axis_confidence_interval_95_mm: 0.01,
            effect_resolved: true,
        };
        LocalXyAxisDirectionReport {
            axis,
            feeds: vec![LocalXyFeedDirectionReport {
                feed_mm_per_min: 300.0,
                estimate,
                stopping_reason: "test".into(),
                holdout: LocalXyCompensationHoldout {
                    negative_residual_x_mm: 0.0,
                    negative_residual_y_mm: 0.0,
                    positive_residual_x_mm: 0.0,
                    positive_residual_y_mm: 0.0,
                    direction_gap_mm: 0.01,
                    improved: true,
                    negative_sample: sample(axis, ApproachSide::Negative),
                    positive_sample: sample(axis, ApproachSide::Positive),
                },
                compensation_enabled: true,
                qualified: true,
                applied_negative_compensation_x_mm: negative[0],
                applied_negative_compensation_y_mm: negative[1],
                applied_positive_compensation_x_mm: positive[0],
                applied_positive_compensation_y_mm: positive[1],
            }],
        }
    }

    fn planar_surface_report() -> PlanarXyDirectionResponseSurface {
        let mut nodes = Vec::new();
        for x_approach in [ApproachSide::Negative, ApproachSide::Positive] {
            for y_approach in [ApproachSide::Negative, ApproachSide::Positive] {
                for x_distance_mm in [5.0, 25.0] {
                    for y_distance_mm in [5.0, 25.0] {
                        let quadrant_x = if x_approach == ApproachSide::Negative {
                            0.01
                        } else {
                            -0.01
                        };
                        let quadrant_y = if y_approach == ApproachSide::Negative {
                            0.02
                        } else {
                            -0.02
                        };
                        nodes.push(PlanarXyDirectionResponseNode {
                            x_approach,
                            y_approach,
                            x_distance_mm,
                            y_distance_mm,
                            correction_x_mm: quadrant_x + x_distance_mm * 0.001,
                            correction_y_mm: quadrant_y + y_distance_mm * 0.002,
                            validation_residual_x_mm: 0.001,
                            validation_residual_y_mm: -0.001,
                            validation_residual_mm: 2.0_f64.sqrt() * 0.001,
                            qualified: true,
                        });
                    }
                }
            }
        }
        PlanarXyDirectionResponseSurface {
            feed_mm_per_min: 300.0,
            x_distances_mm: vec![5.0, 25.0],
            y_distances_mm: vec![5.0, 25.0],
            nodes,
            maximum_validation_residual_mm: 2.0_f64.sqrt() * 0.001,
            independent_validation_sample_count: 20,
            independent_validation_mean_residual_mm: 0.005,
            independent_validation_maximum_residual_mm: 0.01,
            passed: true,
        }
    }

    fn calibration() -> MotionCalibration {
        let report = LocalXyDirectionResponseReport {
            datum_x_approach: ApproachSide::Negative,
            datum_y_approach: ApproachSide::Negative,
            datum_approach_distance_mm: 5.0,
            datum_approach_feed_mm_per_min: 300.0,
            reference_controller_x_mm: 50.0,
            reference_controller_y_mm: 50.0,
            reference_observed_x_px: 320.0,
            reference_observed_y_px: 240.0,
            pixel_error_to_machine_mm: [[0.01, 0.0], [0.0, 0.01]],
            axes: vec![
                axis_report(LinearAxis::X, [0.03, -0.005], [-0.03, 0.005]),
                axis_report(LinearAxis::Y, [0.002, 0.02], [-0.002, -0.02]),
            ],
            planar_surfaces: vec![],
            samples: vec![],
            passed: true,
            observed_at: Utc::now(),
        };
        MotionCalibration::from_local_direction_response(Uuid::nil(), "0".repeat(64), &report)
            .unwrap()
    }

    fn pose(x_mm: f64, y_mm: f64, z_mm: f64) -> Pose {
        Pose {
            x_mm,
            y_mm,
            z_mm,
            rotation_deg: 0.0,
        }
    }

    #[test]
    fn planar_surface_bilinearly_interpolates_a_quadrant() {
        let surface = PlanarXyMotionSurface::from_report(&planar_surface_report()).unwrap();

        let correction = surface
            .correction(ApproachSide::Negative, ApproachSide::Positive, 15.0, 20.0)
            .unwrap();

        assert!((correction[0] - 0.025).abs() < 1e-12);
        assert!((correction[1] - 0.020).abs() < 1e-12);
    }

    #[test]
    fn planar_surface_rejects_an_understated_node_residual() {
        let mut report = planar_surface_report();
        report.maximum_validation_residual_mm = 0.0;

        let error = PlanarXyMotionSurface::from_report(&report).unwrap_err();

        assert_eq!(error, MotionPlanError::InvalidPlanarSurface);
    }

    #[test]
    fn physical_planar_surface_is_complete_and_interpolates() {
        let report: PlanarXyDirectionResponseSurface = serde_json::from_str(include_str!(
            "../../../datasets/motion-model-qualification-2026-09-06/surface-23000.json"
        ))
        .unwrap();
        let surface = PlanarXyMotionSurface::from_report(&report).unwrap();

        let correction = surface
            .correction(ApproachSide::Positive, ApproachSide::Positive, 175.0, 125.0)
            .unwrap();

        assert_eq!(surface.feed_mm_per_min(), 23_000.0);
        assert!(surface.supports(5.0, 5.0));
        assert!(surface.supports(200.0, 180.0));
        assert!(!surface.supports(201.0, 180.0));
        assert!((correction[0] - -0.047_514_864_359_342_25).abs() < 1e-12);
        assert!((correction[1] - -0.090_762_570_971_710_13).abs() < 1e-12);
    }

    #[test]
    fn calibrated_diagonal_motion_uses_the_planar_surface() {
        let mut calibration = calibration();
        calibration.planar_surfaces =
            vec![PlanarXyMotionSurface::from_report(&planar_surface_report()).unwrap()];
        let planner = MotionPlanner::new(bounds(), Some(calibration));
        let state = planner.initial_state(pose(50.0, 50.0, 10.0)).unwrap();

        let planned = planner
            .plan(
                &state,
                pose(65.0, 65.0, 10.0),
                300.0,
                MotionMode::Calibrated,
            )
            .unwrap();

        assert!((planned.correction.local_direction_x_mm - 0.025).abs() < 1e-12);
        assert!((planned.correction.local_direction_y_mm - 0.050).abs() < 1e-12);
        assert!((planned.controller_target.x_mm - 65.025).abs() < 1e-12);
        assert!((planned.controller_target.y_mm - 65.050).abs() < 1e-12);
    }

    #[test]
    fn calibrated_diagonal_motion_rejects_distance_beyond_the_surface() {
        let mut calibration = calibration();
        calibration.planar_surfaces =
            vec![PlanarXyMotionSurface::from_report(&planar_surface_report()).unwrap()];
        let planner = MotionPlanner::new(bounds(), Some(calibration));
        let state = planner.initial_state(pose(50.0, 50.0, 10.0)).unwrap();

        let error = planner
            .plan(
                &state,
                pose(80.0, 60.0, 10.0),
                300.0,
                MotionMode::Calibrated,
            )
            .unwrap_err();

        assert_eq!(error, MotionPlanError::UnqualifiedPlanarDistance);
    }

    #[test]
    fn calibrated_move_selects_the_side_that_it_approaches_from() {
        let planner = MotionPlanner::new(bounds(), Some(calibration()));
        let state = planner.initial_state(pose(50.0, 50.0, 30.0)).unwrap();

        let positive = planner
            .plan(
                &state,
                pose(60.0, 50.0, 30.0),
                300.0,
                MotionMode::Calibrated,
            )
            .unwrap();
        assert_eq!(positive.x_approach, Some(ApproachSide::Negative));
        assert_eq!(positive.controller_target, pose(60.0, 50.0, 30.0));

        let negative = planner
            .plan(
                &state,
                pose(40.0, 50.0, 30.0),
                300.0,
                MotionMode::Calibrated,
            )
            .unwrap();
        assert_eq!(negative.x_approach, Some(ApproachSide::Positive));
        assert_eq!(negative.controller_target, pose(39.94, 50.01, 30.0));
    }

    #[test]
    fn controller_margin_covers_all_direction_combinations() {
        let planner = MotionPlanner::new(bounds(), Some(calibration()));

        assert_eq!(planner.maximum_controller_margin_mm(), [0.064, 0.04]);
        assert_eq!(
            MotionPlanner::new(bounds(), None).maximum_controller_margin_mm(),
            [0.0, 0.0]
        );
    }

    #[test]
    fn diagonal_move_adds_both_axis_components() {
        let planner = MotionPlanner::new(bounds(), Some(calibration()));
        let state = planner.initial_state(pose(50.0, 50.0, 30.0)).unwrap();

        let planned = planner
            .plan(
                &state,
                pose(60.0, 60.0, 30.0),
                300.0,
                MotionMode::Calibrated,
            )
            .unwrap();

        assert_eq!(planned.controller_target, pose(60.0, 60.0, 30.0));
    }

    #[test]
    fn opposite_arrival_subtracts_the_fine_home_direction_value() {
        let planner = MotionPlanner::new(bounds(), Some(calibration()));
        let state = planner.initial_state(pose(50.0, 50.0, 30.0)).unwrap();

        let planned = planner
            .plan(
                &state,
                pose(40.0, 40.0, 30.0),
                300.0,
                MotionMode::Calibrated,
            )
            .unwrap();

        assert_eq!(planned.controller_target, pose(39.936, 39.97, 30.0));
    }

    #[test]
    fn vertical_move_keeps_the_active_xy_compensation() {
        let planner = MotionPlanner::new(bounds(), Some(calibration()));
        let mut state = planner.initial_state(pose(50.0, 50.0, 30.0)).unwrap();
        let lateral = planner
            .plan(
                &state,
                pose(60.0, 60.0, 30.0),
                300.0,
                MotionMode::Calibrated,
            )
            .unwrap();
        planner
            .commit(&mut state, &lateral, lateral.controller_target)
            .unwrap();

        let vertical = planner
            .plan(
                &state,
                pose(60.0, 60.0, 10.0),
                120.0,
                MotionMode::Calibrated,
            )
            .unwrap();

        assert_eq!(vertical.controller_target, pose(60.0, 60.0, 10.0));
        assert_eq!(vertical.logical_target, pose(60.0, 60.0, 10.0));
    }

    #[test]
    fn raw_motion_bypasses_and_clears_direction_compensation() {
        let planner = MotionPlanner::new(bounds(), Some(calibration()));
        let mut state = planner.initial_state(pose(50.0, 50.0, 30.0)).unwrap();
        let calibrated = planner
            .plan(
                &state,
                pose(55.0, 55.0, 30.0),
                300.0,
                MotionMode::Calibrated,
            )
            .unwrap();
        planner
            .commit(&mut state, &calibrated, calibrated.controller_target)
            .unwrap();
        assert!(state.x_approach.is_some());
        assert!(state.y_approach.is_some());

        let target = pose(60.0, 60.0, 30.0);

        let planned = planner
            .plan(&state, target, 300.0, MotionMode::Raw)
            .unwrap();
        planner
            .commit(&mut state, &planned, planned.controller_target)
            .unwrap();

        assert_eq!(planned.controller_target, target);
        assert!(!planned.correction.applied());
        assert_eq!(state.x_approach, None);
        assert_eq!(state.y_approach, None);
    }

    #[test]
    fn controller_bounds_are_checked_after_correction() {
        let planner = MotionPlanner::new(bounds(), Some(calibration()));
        let state = planner.initial_state(pose(50.0, 50.0, 30.0)).unwrap();

        let error = planner
            .plan(&state, pose(0.0, 50.0, 30.0), 300.0, MotionMode::Calibrated)
            .unwrap_err();

        assert_eq!(error, MotionPlanError::ControllerOutsideBounds);
    }

    #[test]
    fn artifact_change_invalidates_an_active_motion_state() {
        let first = MotionPlanner::new(bounds(), Some(calibration()));
        let state = first.initial_state(pose(50.0, 50.0, 30.0)).unwrap();
        let mut changed_calibration = calibration();
        changed_calibration.identity.artifact_fingerprint = "1".repeat(64);
        let changed = MotionPlanner::new(bounds(), Some(changed_calibration));

        let error = changed
            .plan(
                &state,
                pose(60.0, 60.0, 30.0),
                300.0,
                MotionMode::Calibrated,
            )
            .unwrap_err();

        assert_eq!(error, MotionPlanError::CalibrationChanged);
    }

    #[test]
    fn calibrated_motion_rejects_a_feed_without_holdout_evidence() {
        let planner = MotionPlanner::new(bounds(), Some(calibration()));
        let state = planner.initial_state(pose(50.0, 50.0, 30.0)).unwrap();
        let error = planner
            .plan(
                &state,
                pose(60.0, 60.0, 30.0),
                6_000.0,
                MotionMode::Calibrated,
            )
            .unwrap_err();
        assert_eq!(
            error,
            MotionPlanError::UnqualifiedFeed {
                axis: LinearAxis::X,
                feed_mm_per_min: 6_000.0,
            }
        );
    }
}
