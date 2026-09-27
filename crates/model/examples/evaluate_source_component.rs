use anyhow::{ensure, Context, Result};
use serde::Deserialize;
use serde_json::json;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use velocitypnp_domain::SourceComponentClass;
use velocitypnp_model::SourceComponentModel;
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
        "usage: evaluate_source_component MODEL_PACKAGE DATASET [train|eval|all]"
    );
    let split = arguments.get(2).map(String::as_str).unwrap_or("eval");
    ensure!(
        matches!(split, "train" | "eval" | "all"),
        "split must be train, eval, or all"
    );
    let model = SourceComponentModel::load(Path::new(&arguments[0]))?;
    let dataset = Path::new(&arguments[1]).canonicalize()?;
    let manifest_path = dataset.join("manifest.ground-truth.json");
    let manifest: GroundTruthManifest = serde_json::from_slice(
        &fs::read(&manifest_path)
            .with_context(|| format!("cannot read {}", manifest_path.display()))?,
    )?;
    let mut sample_count = 0_u32;
    let mut class_mismatches = Vec::new();
    let mut center_errors = Vec::new();
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
        let prediction = model
            .predict_gray(frame.width, frame.height, &frame.pixels)
            .with_context(|| format!("cannot evaluate source-component sample {}", sample.id))?;
        sample_count += 1;
        if prediction.class != expected {
            class_mismatches.push(json!({
                "id": sample.id,
                "expected": expected,
                "actual": prediction.class,
                "probability": prediction.class_confidence,
            }));
        }
        if let Some(outline) = sample.outline_px {
            let expected_center = outline.iter().fold([0.0, 0.0], |sum, point| {
                [sum[0] + point[0] * 0.25, sum[1] + point[1] * 0.25]
            });
            center_errors.push(
                (expected_center[0] - prediction.pose.center_px[0])
                    .hypot(expected_center[1] - prediction.pose.center_px[1]),
            );
        }
    }
    ensure!(
        sample_count > 0,
        "dataset has no included samples for {split}"
    );
    let mean_center_error_px = if center_errors.is_empty() {
        None
    } else {
        Some(center_errors.iter().sum::<f64>() / center_errors.len() as f64)
    };
    let passed = class_mismatches.is_empty();
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "contract_version": velocitypnp_model::SOURCE_COMPONENT_POSE_CONTRACT,
            "runtime": velocitypnp_model::SOURCE_COMPONENT_RUNTIME,
            "split": split,
            "sample_count": sample_count,
            "class_mismatch_count": class_mismatches.len(),
            "class_mismatches": class_mismatches,
            "mean_center_error_px": mean_center_error_px,
            "passed": passed,
        }))?
    );
    ensure!(passed, "source-component runtime changed a {split} class");
    Ok(())
}

fn parse_class(value: Option<&str>) -> Result<SourceComponentClass> {
    match value {
        None | Some("empty") => Ok(SourceComponentClass::Empty),
        Some("component_flat") => Ok(SourceComponentClass::ComponentFlat),
        Some("component_on_edge") => Ok(SourceComponentClass::ComponentOnEdge),
        Some(other) => anyhow::bail!("unsupported ground-truth class {other}"),
    }
}
