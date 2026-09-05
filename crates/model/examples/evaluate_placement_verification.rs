use anyhow::{ensure, Context, Result};
use serde::Deserialize;
use serde_json::json;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use velocitypnp_domain::PlacementVerificationClass;
use velocitypnp_model::{
    PlacementVerificationModel, PLACEMENT_VERIFICATION_CONTRACT, PLACEMENT_VERIFICATION_RUNTIME,
    PLACEMENT_VERIFICATION_SPAN_MM,
};
use velocitypnp_vision::decode_gray_png;

#[derive(Deserialize)]
struct GroundTruthManifest {
    samples: Vec<GroundTruthSample>,
}

#[derive(Deserialize)]
struct GroundTruthSample {
    id: String,
    split: String,
    image_path: PathBuf,
    included: bool,
    class_name: Option<String>,
    outline_px: Option<[[f64; 2]; 4]>,
}

fn main() -> Result<()> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    ensure!(
        matches!(arguments.len(), 2 | 3),
        "usage: evaluate_placement_verification MODEL_PACKAGE DATASET \
         [train|eval|all]"
    );
    let split = arguments.get(2).map(String::as_str).unwrap_or("eval");
    ensure!(
        matches!(split, "train" | "eval" | "all"),
        "split must be train, eval, or all"
    );
    let model = PlacementVerificationModel::load(Path::new(&arguments[0]))?;
    let dataset = Path::new(&arguments[1]).canonicalize()?;
    let manifest_path = dataset.join("manifest.ground-truth.json");
    let manifest: GroundTruthManifest = serde_json::from_slice(
        &fs::read(&manifest_path)
            .with_context(|| format!("cannot read {}", manifest_path.display()))?,
    )?;

    let mut sample_count = 0_u32;
    let mut class_mismatches = Vec::new();
    let mut gated_samples = 0_u32;
    let mut center_errors_mm = Vec::new();
    let mut angle_errors_deg = Vec::new();
    for sample in manifest
        .samples
        .iter()
        .filter(|sample| sample.included && (split == "all" || sample.split == split))
    {
        let expected = parse_class(sample.class_name.as_deref())?;
        let image_path = dataset.join(&sample.image_path);
        let frame = decode_gray_png(
            &fs::read(&image_path)
                .with_context(|| format!("cannot read {}", image_path.display()))?,
        )?;
        let prediction = model.predict_gray(frame.width, frame.height, &frame.pixels)?;
        sample_count += 1;
        gated_samples += u32::from(prediction.confidence_gate_passed);
        if prediction.class != expected {
            class_mismatches.push(json!({
                "id": sample.id,
                "expected": expected,
                "actual": prediction.class,
                "probability": prediction.class_confidence,
                "margin": prediction.class_margin,
                "gate_passed": prediction.confidence_gate_passed,
            }));
        }
        if expected == PlacementVerificationClass::ComponentFlat {
            if let Some(outline) = sample.outline_px {
                let expected_center = outline.iter().fold([0.0, 0.0], |sum, point| {
                    [sum[0] + point[0] * 0.25, sum[1] + point[1] * 0.25]
                });
                let center_error_px = (expected_center[0] - prediction.pose.center_px[0])
                    .hypot(expected_center[1] - prediction.pose.center_px[1]);
                center_errors_mm.push(
                    center_error_px * PLACEMENT_VERIFICATION_SPAN_MM / f64::from(frame.width),
                );
                let expected_angle = rectangle_long_axis_deg(outline);
                angle_errors_deg.push(
                    axis_angle_error_deg(expected_angle, prediction.pose.rotation_deg_mod_180)
                        .abs(),
                );
            }
        }
    }
    ensure!(
        sample_count > 0,
        "dataset has no included samples for {split}"
    );

    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "contract_version": PLACEMENT_VERIFICATION_CONTRACT,
            "runtime": PLACEMENT_VERIFICATION_RUNTIME,
            "split": split,
            "sample_count": sample_count,
            "class_mismatch_count": class_mismatches.len(),
            "class_mismatches": class_mismatches,
            "confidence_gate_pass_count": gated_samples,
            "pose_sample_count": center_errors_mm.len(),
            "mean_center_error_mm": mean(&center_errors_mm),
            "maximum_center_error_mm": maximum(&center_errors_mm),
            "mean_angle_error_degrees_mod_180": mean(&angle_errors_deg),
        }))?
    );
    Ok(())
}

fn parse_class(value: Option<&str>) -> Result<PlacementVerificationClass> {
    match value {
        None | Some("empty") => Ok(PlacementVerificationClass::Empty),
        Some("component_flat") => Ok(PlacementVerificationClass::ComponentFlat),
        Some("component_on_edge") => Ok(PlacementVerificationClass::ComponentOnEdge),
        Some("multiple_components") => Ok(PlacementVerificationClass::MultipleComponents),
        Some(other) => anyhow::bail!("unsupported ground-truth class {other}"),
    }
}

fn rectangle_long_axis_deg(points: [[f64; 2]; 4]) -> f64 {
    let mut longest = [0.0, 0.0];
    let mut longest_length = -1.0;
    for index in 0..4 {
        let next = (index + 1) % 4;
        let edge = [
            points[next][0] - points[index][0],
            points[next][1] - points[index][1],
        ];
        let length = edge[0].hypot(edge[1]);
        if length > longest_length {
            longest = edge;
            longest_length = length;
        }
    }
    longest[1].atan2(longest[0]).to_degrees().rem_euclid(180.0)
}

fn axis_angle_error_deg(first: f64, second: f64) -> f64 {
    (first - second + 90.0).rem_euclid(180.0) - 90.0
}

fn mean(values: &[f64]) -> Option<f64> {
    (!values.is_empty()).then(|| values.iter().sum::<f64>() / values.len() as f64)
}

fn maximum(values: &[f64]) -> Option<f64> {
    values.iter().copied().reduce(f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn long_axis_uses_the_long_rectangle_edge() {
        let angle = rectangle_long_axis_deg([[0.0, 0.0], [1.0, 1.0], [0.0, 2.0], [-1.0, 1.0]]);
        assert!((angle - 45.0).abs() < 1e-9);
    }

    #[test]
    fn axis_error_wraps_at_one_hundred_eighty_degrees() {
        assert!((axis_angle_error_deg(178.0, 2.0) + 4.0).abs() < 1e-9);
    }
}
