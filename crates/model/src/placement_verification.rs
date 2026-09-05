use super::{
    decode_pose, letterbox_gray, sha256_file, LearnedModelState, PackageManifest,
    EXPECTED_IMAGE_SIZE, EXPECTED_RESIZE,
};
use anyhow::{ensure, Context, Result};
use serde::Serialize;
use std::fs;
use std::path::{Component, Path, PathBuf};
use tract_onnx::prelude::*;
use velocitypnp_domain::{
    PlacementVerificationClass, PlacementVerificationClassProbabilities,
    PlacementVerificationModelPrediction, PlacementVerificationPosePrediction,
};

pub const PLACEMENT_VERIFICATION_CONTRACT: &str = "velocitypnp.placement-verification.v1";
pub const PLACEMENT_VERIFICATION_RUNTIME: &str = "tract-onnx-0.22";
pub const PLACEMENT_VERIFICATION_CLASS_PROBABILITY_GATE: f64 = 0.60;
pub const PLACEMENT_VERIFICATION_CLASS_MARGIN_GATE: f64 = 0.25;
pub const PLACEMENT_VERIFICATION_SPAN_MM: f64 = 4.0;

const EXPECTED_CLASSES: [&str; 4] = [
    "empty",
    "component_flat",
    "component_on_edge",
    "multiple_components",
];

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct PlacementVerificationModelStatus {
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

impl PlacementVerificationModelStatus {
    pub fn disabled() -> Self {
        Self {
            state: LearnedModelState::Disabled,
            contract_version: None,
            artifact_sha256: None,
            runtime: None,
            reason: Some("no placement-verification model package was configured".into()),
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

pub struct PlacementVerificationModel {
    plan: TypedRunnableModel<TypedModel>,
    image_size: usize,
    artifact_sha256: String,
    package_dir: PathBuf,
}

impl std::fmt::Debug for PlacementVerificationModel {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("PlacementVerificationModel")
            .field("image_size", &self.image_size)
            .field("artifact_sha256", &self.artifact_sha256)
            .field("package_dir", &self.package_dir)
            .finish_non_exhaustive()
    }
}

impl PlacementVerificationModel {
    pub fn load(package_dir: impl AsRef<Path>) -> Result<Self> {
        let package_dir = package_dir.as_ref().canonicalize().with_context(|| {
            format!(
                "cannot resolve placement-verification model package {}",
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
            .context("cannot optimize placement-verification model")?
            .into_runnable()
            .context("cannot prepare placement-verification model")?;
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
            .context("placement-verification model failed its startup inference")?;
        Ok(model)
    }

    pub fn status(&self) -> PlacementVerificationModelStatus {
        PlacementVerificationModelStatus {
            state: LearnedModelState::Ready,
            contract_version: Some(PLACEMENT_VERIFICATION_CONTRACT.into()),
            artifact_sha256: Some(self.artifact_sha256.clone()),
            runtime: Some(PLACEMENT_VERIFICATION_RUNTIME.into()),
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
    ) -> Result<PlacementVerificationModelPrediction> {
        let (input, letterbox) = letterbox_gray(width, height, pixels, self.image_size)?;
        let input =
            tract_ndarray::Array4::from_shape_vec((1, 3, self.image_size, self.image_size), input)?
                .into_tensor();
        let outputs = self
            .plan
            .run(tvec!(input.into()))
            .context("placement-verification inference failed")?;
        ensure!(
            outputs.len() == 2,
            "placement-verification model must have two outputs"
        );
        let class_values = outputs[0]
            .to_array_view::<f32>()
            .context("placement-verification class output is not float32")?
            .iter()
            .copied()
            .collect::<Vec<_>>();
        let pose_values = outputs[1]
            .to_array_view::<f32>()
            .context("placement-verification pose output is not float32")?
            .iter()
            .copied()
            .collect::<Vec<_>>();
        ensure!(
            class_values.len() == 4,
            "class output must have four values"
        );
        ensure!(pose_values.len() == 6, "pose output must have six values");
        ensure!(
            class_values
                .iter()
                .chain(&pose_values)
                .all(|value| value.is_finite()),
            "placement-verification model returned a non-finite value"
        );

        let probabilities = softmax_four(class_values.try_into().unwrap());
        let mut order = [0_usize, 1, 2, 3];
        order.sort_by(|first, second| probabilities[*second].total_cmp(&probabilities[*first]));
        let class = match order[0] {
            0 => PlacementVerificationClass::Empty,
            1 => PlacementVerificationClass::ComponentFlat,
            2 => PlacementVerificationClass::ComponentOnEdge,
            3 => PlacementVerificationClass::MultipleComponents,
            _ => unreachable!(),
        };
        let class_confidence = f64::from(probabilities[order[0]]);
        let class_margin = class_confidence - f64::from(probabilities[order[1]]);
        let source_pose = decode_pose(pose_values.try_into().unwrap(), &letterbox)?;
        let pose = PlacementVerificationPosePrediction {
            center_px: source_pose.center_px,
            size_px: source_pose.size_px,
            rotation_deg_mod_180: source_pose.rotation_deg_mod_180,
            outline_px: source_pose.outline_px,
            valid: source_pose.valid,
        };
        let confidence_gate_passed = class_confidence
            >= PLACEMENT_VERIFICATION_CLASS_PROBABILITY_GATE
            && class_margin >= PLACEMENT_VERIFICATION_CLASS_MARGIN_GATE
            && (!class.occupied() || pose.valid);
        let mut findings = Vec::new();
        if class_confidence < PLACEMENT_VERIFICATION_CLASS_PROBABILITY_GATE {
            findings.push(format!(
                "class probability {class_confidence:.3} is below the promotion gate \
                 {PLACEMENT_VERIFICATION_CLASS_PROBABILITY_GATE:.3}"
            ));
        }
        if class_margin < PLACEMENT_VERIFICATION_CLASS_MARGIN_GATE {
            findings.push(format!(
                "class margin {class_margin:.3} is below the promotion gate \
                 {PLACEMENT_VERIFICATION_CLASS_MARGIN_GATE:.3}"
            ));
        }
        if class.occupied() && !pose.valid {
            findings.push("the occupied pose is outside the placement crop".into());
        }
        Ok(PlacementVerificationModelPrediction {
            contract_version: PLACEMENT_VERIFICATION_CONTRACT.into(),
            model_artifact_sha256: self.artifact_sha256.clone(),
            runtime: PLACEMENT_VERIFICATION_RUNTIME.into(),
            class,
            probabilities: PlacementVerificationClassProbabilities {
                empty: f64::from(probabilities[0]),
                component_flat: f64::from(probabilities[1]),
                component_on_edge: f64::from(probabilities[2]),
                multiple_components: f64::from(probabilities[3]),
            },
            class_confidence,
            class_margin,
            confidence_gate_passed,
            pose,
            findings,
        })
    }
}

fn softmax_four(values: [f32; 4]) -> [f32; 4] {
    let maximum = values.into_iter().reduce(f32::max).unwrap();
    let exponentials = values.map(|value| (value - maximum).exp());
    let sum = exponentials.iter().sum::<f32>();
    exponentials.map(|value| value / sum)
}

fn validate_manifest(manifest: &PackageManifest) -> Result<()> {
    ensure!(
        manifest.contract_version == PLACEMENT_VERIFICATION_CONTRACT,
        "unsupported placement-verification model contract {}",
        manifest.contract_version
    );
    ensure!(
        manifest.license.artifact == "Apache-2.0",
        "placement-verification model artifact must use Apache-2.0 terms"
    );
    ensure!(
        manifest.model.config.image_size == EXPECTED_IMAGE_SIZE,
        "placement-verification model input must be {EXPECTED_IMAGE_SIZE} pixels"
    );
    ensure!(
        manifest.model.config.input_channels == 3,
        "placement-verification model must have three input channels"
    );
    ensure!(
        manifest.model.config.class_count == EXPECTED_CLASSES.len(),
        "placement-verification model must have four classes"
    );
    ensure!(
        manifest.model.classes == EXPECTED_CLASSES,
        "placement-verification model classes or class order changed"
    );
    ensure!(
        manifest.model.input.name == "images",
        "placement-verification model input name changed"
    );
    ensure!(
        manifest.model.input.datum_type == "float32",
        "placement-verification model input must be float32"
    );
    ensure!(
        manifest.model.input.range == [0.0, 1.0],
        "placement-verification model input range changed"
    );
    ensure!(
        manifest.model.input.resize == EXPECTED_RESIZE,
        "placement-verification model input resize changed"
    );
    ensure!(
        manifest.model.input.letterbox_value == 127,
        "placement-verification model letterbox value changed"
    );
    ensure!(
        manifest.model.input.normalization_span_mm == Some(PLACEMENT_VERIFICATION_SPAN_MM),
        "placement-verification crop span changed"
    );
    ensure!(
        manifest.model.input.target_center_normalized == Some([0.5, 0.5]),
        "placement-verification target center changed"
    );
    ensure!(
        manifest.model.input.target_long_axis.as_deref() == Some("positive_x"),
        "placement-verification target axis changed"
    );
    ensure!(
        manifest.model.input.shape
            == [
                serde_json::Value::String("batch".into()),
                serde_json::Value::from(3),
                serde_json::Value::from(EXPECTED_IMAGE_SIZE),
                serde_json::Value::from(EXPECTED_IMAGE_SIZE),
            ],
        "placement-verification model input shape changed"
    );
    ensure!(
        manifest.model.angle_period_degrees == 180,
        "placement-verification angle period changed"
    );
    ensure!(
        !manifest.model.polarity_supported,
        "placement-verification v1 cannot claim polarity support"
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

#[cfg(test)]
mod tests {
    use super::*;

    fn manifest(value: serde_json::Value) -> PackageManifest {
        serde_json::from_value(value).unwrap()
    }

    fn valid_manifest_value() -> serde_json::Value {
        serde_json::json!({
            "contract_version": PLACEMENT_VERIFICATION_CONTRACT,
            "model": {
                "config": {
                    "image_size": 192,
                    "input_channels": 3,
                    "class_count": 4
                },
                "classes": EXPECTED_CLASSES,
                "input": {
                    "name": "images",
                    "shape": ["batch", 3, 192, 192],
                    "type": "float32",
                    "range": [0.0, 1.0],
                    "resize": EXPECTED_RESIZE,
                    "letterbox_value": 127,
                    "normalization_span_mm": 4.0,
                    "target_center_normalized": [0.5, 0.5],
                    "target_long_axis": "positive_x"
                },
                "angle_period_degrees": 180,
                "polarity_supported": false
            },
            "license": {"artifact": "Apache-2.0"},
            "artifacts": {
                "onnx": {
                    "path": "best.onnx",
                    "sha256":
                        "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
                    "size_bytes": 100,
                    "opset": 17
                }
            }
        })
    }

    #[test]
    fn accepts_the_versioned_permissive_contract() {
        validate_manifest(&manifest(valid_manifest_value())).unwrap();
    }

    #[test]
    fn rejects_a_changed_normalization_span() {
        let mut value = valid_manifest_value();
        value["model"]["input"]["normalization_span_mm"] = serde_json::json!(5.0);
        let error = validate_manifest(&manifest(value)).unwrap_err();
        assert!(error.to_string().contains("crop span"));
    }

    #[test]
    fn rejects_untrained_class_order_changes() {
        let mut value = valid_manifest_value();
        value["model"]["classes"][2] = serde_json::json!("multiple_components");
        let error = validate_manifest(&manifest(value)).unwrap_err();
        assert!(error.to_string().contains("class order"));
    }
}
