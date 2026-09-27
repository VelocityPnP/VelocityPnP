//! Versioned learned-model loading and inference.

mod held_part;
mod placement_verification;

pub use held_part::*;
pub use placement_verification::*;

use anyhow::{ensure, Context, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::fs;
use std::path::{Component, Path, PathBuf};
use tract_onnx::prelude::*;
use velocitypnp_domain::{
    SourceComponentClass, SourceComponentClassProbabilities, SourceComponentModelPrediction,
    SourceComponentPosePrediction,
};

pub const SOURCE_COMPONENT_POSE_CONTRACT: &str = "velocitypnp.source-component-pose.v2";
pub const SOURCE_COMPONENT_RUNTIME: &str = "tract-onnx-0.22";
pub const SOURCE_COMPONENT_CLASS_PROBABILITY_GATE: f64 = 0.75;
pub const SOURCE_COMPONENT_CLASS_MARGIN_GATE: f64 = 0.25;

const EXPECTED_IMAGE_SIZE: usize = 192;
const EXPECTED_CLASSES: [&str; 3] = ["empty", "component_flat", "component_on_edge"];
const EXPECTED_RESIZE: &str = "velocitypnp.half-pixel-bilinear-u8.v1";

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum LearnedModelState {
    Disabled,
    Ready,
    Invalid,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct SourceComponentModelStatus {
    pub state: LearnedModelState,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub contract_version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub artifact_sha256: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub runtime: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

impl SourceComponentModelStatus {
    pub fn disabled() -> Self {
        Self {
            state: LearnedModelState::Disabled,
            contract_version: None,
            artifact_sha256: None,
            runtime: None,
            reason: Some("no source-component model package was configured".into()),
        }
    }

    pub fn invalid(reason: impl Into<String>) -> Self {
        Self {
            state: LearnedModelState::Invalid,
            contract_version: None,
            artifact_sha256: None,
            runtime: None,
            reason: Some(reason.into()),
        }
    }
}

#[derive(Debug, Deserialize)]
struct PackageManifest {
    contract_version: String,
    model: ManifestModel,
    license: ManifestLicense,
    artifacts: ManifestArtifacts,
}

#[derive(Debug, Deserialize)]
struct ManifestModel {
    config: ManifestModelConfig,
    classes: Vec<String>,
    input: ManifestInput,
    angle_period_degrees: u32,
    polarity_supported: bool,
}

#[derive(Debug, Deserialize)]
struct ManifestModelConfig {
    image_size: usize,
    input_channels: usize,
    class_count: usize,
}

#[derive(Debug, Deserialize)]
struct ManifestInput {
    name: String,
    shape: Vec<serde_json::Value>,
    #[serde(rename = "type")]
    datum_type: String,
    range: [f64; 2],
    resize: String,
    letterbox_value: u8,
    #[serde(default)]
    normalization_span_mm: Option<f64>,
    #[serde(default)]
    target_center_normalized: Option<[f64; 2]>,
    #[serde(default)]
    target_long_axis: Option<String>,
}

#[derive(Debug, Deserialize)]
struct ManifestLicense {
    artifact: String,
}

#[derive(Debug, Deserialize)]
struct ManifestArtifacts {
    onnx: ManifestArtifact,
}

#[derive(Debug, Deserialize)]
struct ManifestArtifact {
    path: String,
    sha256: String,
    size_bytes: u64,
    opset: u32,
}

struct Letterbox {
    source_width: usize,
    source_height: usize,
    resized_width: usize,
    resized_height: usize,
    left: usize,
    top: usize,
}

pub struct SourceComponentModel {
    plan: TypedRunnableModel<TypedModel>,
    image_size: usize,
    artifact_sha256: String,
    package_dir: PathBuf,
}

impl std::fmt::Debug for SourceComponentModel {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("SourceComponentModel")
            .field("image_size", &self.image_size)
            .field("artifact_sha256", &self.artifact_sha256)
            .field("package_dir", &self.package_dir)
            .finish_non_exhaustive()
    }
}

impl SourceComponentModel {
    pub fn load(package_dir: impl AsRef<Path>) -> Result<Self> {
        let package_dir = package_dir.as_ref().canonicalize().with_context(|| {
            format!(
                "cannot resolve source-component model package {}",
                package_dir.as_ref().display()
            )
        })?;
        ensure!(
            package_dir.is_dir(),
            "model package path is not a directory"
        );
        let manifest_path = package_dir.join("model-manifest.json");
        let manifest_bytes = fs::read(&manifest_path)
            .with_context(|| format!("cannot read {}", manifest_path.display()))?;
        let manifest: PackageManifest = serde_json::from_slice(&manifest_bytes)
            .with_context(|| format!("cannot decode {}", manifest_path.display()))?;
        validate_manifest(&manifest)?;

        let relative_path = Path::new(&manifest.artifacts.onnx.path);
        ensure!(
            relative_path
                .components()
                .all(|component| matches!(component, Component::Normal(_))),
            "ONNX artifact path must stay inside the model package"
        );
        let model_path = package_dir
            .join(relative_path)
            .canonicalize()
            .with_context(|| format!("cannot resolve ONNX artifact {}", relative_path.display()))?;
        ensure!(
            model_path.starts_with(&package_dir),
            "ONNX artifact must stay inside the model package"
        );
        let metadata = fs::metadata(&model_path)
            .with_context(|| format!("cannot inspect {}", model_path.display()))?;
        ensure!(
            metadata.len() == manifest.artifacts.onnx.size_bytes,
            "ONNX artifact size does not match the manifest"
        );
        let artifact_sha256 = sha256_file(&model_path)?;
        ensure!(
            artifact_sha256 == manifest.artifacts.onnx.sha256,
            "ONNX artifact SHA-256 does not match the manifest"
        );

        let image_size = manifest.model.config.image_size;
        let plan = tract_onnx::onnx()
            .model_for_path(&model_path)
            .with_context(|| format!("cannot load {}", model_path.display()))?
            .with_input_fact(
                0,
                f32::fact([1, 3, image_size as i64, image_size as i64]).into(),
            )?
            .into_optimized()
            .context("cannot optimize source-component model")?
            .into_runnable()
            .context("cannot prepare source-component model")?;
        let model = Self {
            plan,
            image_size,
            artifact_sha256,
            package_dir,
        };
        model
            .predict_gray(
                image_size as u32,
                image_size as u32,
                &vec![127; image_size * image_size],
            )
            .context("source-component model failed its startup inference")?;
        Ok(model)
    }

    pub fn status(&self) -> SourceComponentModelStatus {
        SourceComponentModelStatus {
            state: LearnedModelState::Ready,
            contract_version: Some(SOURCE_COMPONENT_POSE_CONTRACT.into()),
            artifact_sha256: Some(self.artifact_sha256.clone()),
            runtime: Some(SOURCE_COMPONENT_RUNTIME.into()),
            reason: None,
        }
    }

    pub fn package_dir(&self) -> &Path {
        &self.package_dir
    }

    pub fn predict_gray(
        &self,
        width: u32,
        height: u32,
        pixels: &[u8],
    ) -> Result<SourceComponentModelPrediction> {
        let (input, letterbox) = letterbox_gray(width, height, pixels, self.image_size)?;
        let input =
            tract_ndarray::Array4::from_shape_vec((1, 3, self.image_size, self.image_size), input)?
                .into_tensor();
        let outputs = self
            .plan
            .run(tvec!(input.into()))
            .context("source-component inference failed")?;
        ensure!(
            outputs.len() == 2,
            "source-component model must have two outputs"
        );
        let class_values = outputs[0]
            .to_array_view::<f32>()
            .context("source-component class output is not float32")?
            .iter()
            .copied()
            .collect::<Vec<_>>();
        let pose_values = outputs[1]
            .to_array_view::<f32>()
            .context("source-component pose output is not float32")?
            .iter()
            .copied()
            .collect::<Vec<_>>();
        ensure!(
            class_values.len() == 3,
            "class output must have three values"
        );
        ensure!(pose_values.len() == 6, "pose output must have six values");
        ensure!(
            class_values_are_usable(&class_values),
            "source-component model returned invalid class values: {class_values:?}"
        );
        ensure!(
            pose_values.iter().all(|value| value.is_finite()),
            "source-component model returned non-finite pose values: {pose_values:?}"
        );

        let probabilities = softmax_three(class_values.try_into().unwrap());
        let mut order = [0_usize, 1, 2];
        order.sort_by(|first, second| probabilities[*second].total_cmp(&probabilities[*first]));
        let class = match order[0] {
            0 => SourceComponentClass::Empty,
            1 => SourceComponentClass::ComponentFlat,
            2 => SourceComponentClass::ComponentOnEdge,
            _ => unreachable!(),
        };
        let class_confidence = f64::from(probabilities[order[0]]);
        let class_margin = class_confidence - f64::from(probabilities[order[1]]);
        let pose = decode_pose(pose_values.try_into().unwrap(), &letterbox)?;
        let confidence_gate_passed = class_confidence >= SOURCE_COMPONENT_CLASS_PROBABILITY_GATE
            && class_margin >= SOURCE_COMPONENT_CLASS_MARGIN_GATE
            && (!class.occupied() || pose.valid);
        let mut findings = Vec::new();
        if class_confidence < SOURCE_COMPONENT_CLASS_PROBABILITY_GATE {
            findings.push(format!(
                "class probability {class_confidence:.3} is below the shadow gate \
                 {SOURCE_COMPONENT_CLASS_PROBABILITY_GATE:.3}"
            ));
        }
        if class_margin < SOURCE_COMPONENT_CLASS_MARGIN_GATE {
            findings.push(format!(
                "class margin {class_margin:.3} is below the shadow gate \
                 {SOURCE_COMPONENT_CLASS_MARGIN_GATE:.3}"
            ));
        }
        if class.occupied() && !pose.valid {
            findings.push("the occupied pose is outside the source crop".into());
        }
        Ok(SourceComponentModelPrediction {
            contract_version: SOURCE_COMPONENT_POSE_CONTRACT.into(),
            model_artifact_sha256: self.artifact_sha256.clone(),
            runtime: SOURCE_COMPONENT_RUNTIME.into(),
            class,
            probabilities: SourceComponentClassProbabilities {
                empty: f64::from(probabilities[0]),
                component_flat: f64::from(probabilities[1]),
                component_on_edge: f64::from(probabilities[2]),
            },
            class_confidence,
            class_margin,
            confidence_gate_passed,
            pose,
            findings,
        })
    }
}

fn validate_manifest(manifest: &PackageManifest) -> Result<()> {
    ensure!(
        manifest.contract_version == SOURCE_COMPONENT_POSE_CONTRACT,
        "unsupported source-component model contract {}",
        manifest.contract_version
    );
    ensure!(
        manifest.license.artifact == "Apache-2.0",
        "source-component model artifact must use Apache-2.0 terms"
    );
    ensure!(
        manifest.model.config.image_size == EXPECTED_IMAGE_SIZE,
        "source-component model input must be {EXPECTED_IMAGE_SIZE} pixels"
    );
    ensure!(
        manifest.model.config.input_channels == 3,
        "source-component model must have three input channels"
    );
    ensure!(
        manifest.model.config.class_count == EXPECTED_CLASSES.len(),
        "source-component model must have three classes"
    );
    ensure!(
        manifest.model.classes == EXPECTED_CLASSES,
        "source-component model classes or class order changed"
    );
    ensure!(
        manifest.model.input.name == "images",
        "model input name changed"
    );
    ensure!(
        manifest.model.input.datum_type == "float32",
        "model input must use float32"
    );
    ensure!(
        manifest.model.input.range == [0.0, 1.0],
        "model input range changed"
    );
    ensure!(
        manifest.model.input.resize == EXPECTED_RESIZE,
        "model input resize changed"
    );
    ensure!(
        manifest.model.input.letterbox_value == 127,
        "model letterbox value changed"
    );
    ensure!(
        manifest.model.input.shape
            == [
                serde_json::Value::String("batch".into()),
                serde_json::Value::from(3),
                serde_json::Value::from(EXPECTED_IMAGE_SIZE),
                serde_json::Value::from(EXPECTED_IMAGE_SIZE),
            ],
        "model input shape changed"
    );
    ensure!(
        manifest.model.angle_period_degrees == 180,
        "source-component angle period changed"
    );
    ensure!(
        !manifest.model.polarity_supported,
        "v1 source-component model cannot claim polarity support"
    );
    ensure!(manifest.artifacts.onnx.opset == 17, "ONNX opset must be 17");
    ensure!(
        manifest.artifacts.onnx.sha256.len() == 64
            && manifest
                .artifacts
                .onnx
                .sha256
                .bytes()
                .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase()),
        "ONNX artifact SHA-256 is invalid"
    );
    Ok(())
}

fn sha256_file(path: &Path) -> Result<String> {
    let data = fs::read(path).with_context(|| format!("cannot read {}", path.display()))?;
    Ok(Sha256::digest(data)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect())
}

fn letterbox_gray(
    width: u32,
    height: u32,
    pixels: &[u8],
    image_size: usize,
) -> Result<(Vec<f32>, Letterbox)> {
    let width = usize::try_from(width)?;
    let height = usize::try_from(height)?;
    ensure!(width > 0 && height > 0, "source crop is empty");
    ensure!(
        width.checked_mul(height) == Some(pixels.len()),
        "source crop dimensions do not match its pixels"
    );
    let scale = (image_size as f64 / width as f64).min(image_size as f64 / height as f64);
    let resized_width = (width as f64 * scale).round().max(1.0) as usize;
    let resized_height = (height as f64 * scale).round().max(1.0) as usize;
    let left = (image_size - resized_width) / 2;
    let top = (image_size - resized_height) / 2;
    let mut plane = vec![127.0_f32 / 255.0; image_size * image_size];
    for target_y in 0..resized_height {
        let source_y = ((target_y as f64 + 0.5) * height as f64 / resized_height as f64 - 0.5)
            .clamp(0.0, (height - 1) as f64);
        let y0 = source_y.floor() as usize;
        let y1 = (y0 + 1).min(height - 1);
        let y_fraction = source_y - y0 as f64;
        for target_x in 0..resized_width {
            let source_x = ((target_x as f64 + 0.5) * width as f64 / resized_width as f64 - 0.5)
                .clamp(0.0, (width - 1) as f64);
            let x0 = source_x.floor() as usize;
            let x1 = (x0 + 1).min(width - 1);
            let x_fraction = source_x - x0 as f64;
            let top_value = f64::from(pixels[y0 * width + x0]) * (1.0 - x_fraction)
                + f64::from(pixels[y0 * width + x1]) * x_fraction;
            let bottom_value = f64::from(pixels[y1 * width + x0]) * (1.0 - x_fraction)
                + f64::from(pixels[y1 * width + x1]) * x_fraction;
            let value = (top_value * (1.0 - y_fraction) + bottom_value * y_fraction)
                .round()
                .clamp(0.0, 255.0) as f32
                / 255.0;
            plane[(top + target_y) * image_size + left + target_x] = value;
        }
    }
    let mut input = Vec::with_capacity(3 * plane.len());
    for _ in 0..3 {
        input.extend_from_slice(&plane);
    }
    Ok((
        input,
        Letterbox {
            source_width: width,
            source_height: height,
            resized_width,
            resized_height,
            left,
            top,
        },
    ))
}

fn softmax_three(values: [f32; 3]) -> [f32; 3] {
    let maximum = values.into_iter().reduce(f32::max).unwrap();
    let exponentials = values.map(|value| (value - maximum).exp());
    let sum = exponentials.iter().sum::<f32>();
    exponentials.map(|value| value / sum)
}

fn class_values_are_usable(values: &[f32]) -> bool {
    // Tract can represent a saturated log-sigmoid as negative infinity. This
    // is a valid log-probability of zero, provided another class is finite.
    values.iter().any(|value| value.is_finite())
        && values
            .iter()
            .all(|value| value.is_finite() || *value == f32::NEG_INFINITY)
}

fn decode_pose(values: [f32; 6], letterbox: &Letterbox) -> Result<SourceComponentPosePrediction> {
    let angle_norm = values[4].hypot(values[5]);
    ensure!(angle_norm > 1e-6, "model angle has no direction");
    let rotation = 0.5 * f64::from(values[5]).atan2(f64::from(values[4]));
    let center_model = [
        f64::from(values[0]) * EXPECTED_IMAGE_SIZE as f64,
        f64::from(values[1]) * EXPECTED_IMAGE_SIZE as f64,
    ];
    let size_model = [
        f64::from(values[2]) * EXPECTED_IMAGE_SIZE as f64,
        f64::from(values[3]) * EXPECTED_IMAGE_SIZE as f64,
    ];
    let cosine = rotation.cos();
    let sine = rotation.sin();
    let local = [
        [-0.5 * size_model[0], -0.5 * size_model[1]],
        [0.5 * size_model[0], -0.5 * size_model[1]],
        [0.5 * size_model[0], 0.5 * size_model[1]],
        [-0.5 * size_model[0], 0.5 * size_model[1]],
    ];
    let model_outline = local.map(|point| {
        [
            center_model[0] + cosine * point[0] - sine * point[1],
            center_model[1] + sine * point[0] + cosine * point[1],
        ]
    });
    let scale_x = letterbox.resized_width as f64 / letterbox.source_width as f64;
    let scale_y = letterbox.resized_height as f64 / letterbox.source_height as f64;
    let to_source = |point: [f64; 2]| {
        [
            (point[0] - letterbox.left as f64) / scale_x,
            (point[1] - letterbox.top as f64) / scale_y,
        ]
    };
    let center_px = to_source(center_model);
    let outline_px = model_outline.map(to_source);
    let size_px = [size_model[0] / scale_x, size_model[1] / scale_y];
    let valid = center_px[0] >= 0.0
        && center_px[0] < letterbox.source_width as f64
        && center_px[1] >= 0.0
        && center_px[1] < letterbox.source_height as f64
        && size_px.iter().all(|value| *value >= 1.0)
        && outline_px.iter().all(|point| {
            point[0] >= 0.0
                && point[0] < letterbox.source_width as f64
                && point[1] >= 0.0
                && point[1] < letterbox.source_height as f64
        });
    Ok(SourceComponentPosePrediction {
        center_px,
        size_px,
        rotation_deg_mod_180: rotation.to_degrees(),
        outline_px,
        valid,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn manifest(value: serde_json::Value) -> PackageManifest {
        serde_json::from_value(value).unwrap()
    }

    fn valid_manifest_value() -> serde_json::Value {
        serde_json::json!({
            "contract_version": SOURCE_COMPONENT_POSE_CONTRACT,
            "model": {
                "config": {
                    "image_size": 192,
                    "input_channels": 3,
                    "class_count": 3
                },
                "classes": ["empty", "component_flat", "component_on_edge"],
                "input": {
                    "name": "images",
                    "shape": ["batch", 3, 192, 192],
                    "type": "float32",
                    "range": [0.0, 1.0],
                    "resize": EXPECTED_RESIZE,
                    "letterbox_value": 127
                },
                "angle_period_degrees": 180,
                "polarity_supported": false
            },
            "license": {"artifact": "Apache-2.0"},
            "artifacts": {
                "onnx": {
                    "path": "best.onnx",
                    "sha256": "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
                    "size_bytes": 100,
                    "opset": 17
                }
            }
        })
    }

    #[test]
    fn letterbox_has_three_equal_channels_and_centered_padding() {
        let pixels = vec![255_u8; 350 * 300];
        let (input, box_) = letterbox_gray(350, 300, &pixels, 192).unwrap();
        assert_eq!(box_.resized_width, 192);
        assert_eq!(box_.resized_height, 165);
        assert_eq!(box_.top, 13);
        assert_eq!(input.len(), 3 * 192 * 192);
        assert_eq!(input[0], 127.0 / 255.0);
        assert_eq!(input[13 * 192], 1.0);
        assert_eq!(&input[..192 * 192], &input[192 * 192..2 * 192 * 192]);
    }

    #[test]
    fn letterbox_matches_the_cross_runtime_contract_fixture() {
        let pixels = (0_u8..35).map(|value| value * 7).collect::<Vec<_>>();
        let (input, box_) = letterbox_gray(7, 5, &pixels, 8).unwrap();
        let bytes = input
            .iter()
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();
        assert_eq!(box_.resized_width, 8);
        assert_eq!(box_.resized_height, 6);
        assert_eq!(
            format!("{:x}", Sha256::digest(bytes)),
            "2102e1393effcfabe9254138412eee6a1dd39a10764f25fbe66454d9d6e4a909"
        );
    }

    #[test]
    fn decoded_pose_maps_out_of_the_letterbox() {
        let box_ = Letterbox {
            source_width: 350,
            source_height: 300,
            resized_width: 192,
            resized_height: 165,
            left: 0,
            top: 13,
        };
        let pose = decode_pose([0.5, 0.5, 0.1, 0.05, 1.0, 0.0], &box_).unwrap();
        assert!((pose.center_px[0] - 175.0).abs() < 1e-6);
        assert!((pose.center_px[1] - 150.909).abs() < 0.001);
        assert!(pose.valid);
        assert_eq!(pose.rotation_deg_mod_180, 0.0);
    }

    #[test]
    fn softmax_is_normalized() {
        let probabilities = softmax_three([-2.0, 3.0, 1.0]);
        assert!((probabilities.iter().sum::<f32>() - 1.0).abs() < 1e-6);
        assert!(probabilities[1] > probabilities[2]);
    }

    #[test]
    fn negative_infinite_class_log_probability_is_usable() {
        let values = [f32::NEG_INFINITY, -3.0e-6, -12.0];
        assert!(class_values_are_usable(&values));
        let probabilities = softmax_three(values);
        assert_eq!(probabilities[0], 0.0);
        assert!(probabilities[1] > probabilities[2]);
    }

    #[test]
    fn class_values_reject_invalid_non_finite_combinations() {
        assert!(!class_values_are_usable(&[f32::NEG_INFINITY; 3]));
        assert!(!class_values_are_usable(&[0.0, f32::INFINITY, -1.0]));
        assert!(!class_values_are_usable(&[0.0, f32::NAN, -1.0]));
    }

    #[test]
    fn manifest_contract_accepts_the_versioned_permissive_package() {
        validate_manifest(&manifest(valid_manifest_value())).unwrap();
    }

    #[test]
    fn manifest_contract_rejects_non_permissive_artifact_terms() {
        let mut value = valid_manifest_value();
        value["license"]["artifact"] = serde_json::json!("AGPL-3.0-only");
        let error = validate_manifest(&manifest(value)).unwrap_err();
        assert!(error.to_string().contains("Apache-2.0"));
    }

    #[test]
    fn manifest_contract_rejects_changed_class_order() {
        let mut value = valid_manifest_value();
        value["model"]["classes"] =
            serde_json::json!(["component_flat", "empty", "component_on_edge"]);
        let error = validate_manifest(&manifest(value)).unwrap_err();
        assert!(error.to_string().contains("class order"));
    }
}
