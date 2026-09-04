use super::{
    decode_pose, letterbox_gray, sha256_file, softmax_three, LearnedModelState, PackageManifest,
    EXPECTED_IMAGE_SIZE, EXPECTED_RESIZE,
};
use anyhow::{ensure, Context, Result};
use serde::Serialize;
use std::fs;
use std::path::{Component, Path, PathBuf};
use tract_onnx::prelude::*;
use velocitypnp_domain::{
    HeldPartClass, HeldPartClassProbabilities, HeldPartModelPrediction, HeldPartPosePrediction,
};

pub const HELD_PART_POSE_CONTRACT: &str = "velocitypnp.held-part-pose.v1";
pub const HELD_PART_RUNTIME: &str = "tract-onnx-0.22";
pub const HELD_PART_CLASS_PROBABILITY_GATE: f64 = 0.75;
pub const HELD_PART_CLASS_MARGIN_GATE: f64 = 0.25;

const EXPECTED_HELD_PART_CLASSES: [&str; 3] =
    ["bare_nozzle", "component_flat", "component_on_edge"];

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct HeldPartModelStatus {
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

impl HeldPartModelStatus {
    pub fn disabled() -> Self {
        Self {
            state: LearnedModelState::Disabled,
            contract_version: None,
            artifact_sha256: None,
            runtime: None,
            reason: Some("no held-part model package was configured".into()),
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

pub struct HeldPartModel {
    plan: TypedRunnableModel<TypedModel>,
    image_size: usize,
    artifact_sha256: String,
    package_dir: PathBuf,
}

impl std::fmt::Debug for HeldPartModel {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("HeldPartModel")
            .field("image_size", &self.image_size)
            .field("artifact_sha256", &self.artifact_sha256)
            .field("package_dir", &self.package_dir)
            .finish_non_exhaustive()
    }
}

impl HeldPartModel {
    pub fn load(package_dir: impl AsRef<Path>) -> Result<Self> {
        let package_dir = package_dir.as_ref().canonicalize().with_context(|| {
            format!(
                "cannot resolve held-part model package {}",
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
        validate_held_part_manifest(&manifest)?;

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
                f32::fact([1, 6, image_size as i64, image_size as i64]).into(),
            )?
            .into_optimized()
            .context("cannot optimize held-part model")?
            .into_runnable()
            .context("cannot prepare held-part model")?;
        let model = Self {
            plan,
            image_size,
            artifact_sha256,
            package_dir,
        };
        let startup = vec![127; image_size * image_size];
        model
            .predict_gray_pair(image_size as u32, image_size as u32, &startup, &startup)
            .context("held-part model failed its startup inference")?;
        Ok(model)
    }

    pub fn status(&self) -> HeldPartModelStatus {
        HeldPartModelStatus {
            state: LearnedModelState::Ready,
            contract_version: Some(HELD_PART_POSE_CONTRACT.into()),
            artifact_sha256: Some(self.artifact_sha256.clone()),
            runtime: Some(HELD_PART_RUNTIME.into()),
            reason: None,
        }
    }

    pub fn package_dir(&self) -> &Path {
        &self.package_dir
    }

    pub fn predict_gray_pair(
        &self,
        width: u32,
        height: u32,
        bare_pixels: &[u8],
        observed_pixels: &[u8],
    ) -> Result<HeldPartModelPrediction> {
        let (mut input, bare_box) = letterbox_gray(width, height, bare_pixels, self.image_size)?;
        let (observed, observed_box) =
            letterbox_gray(width, height, observed_pixels, self.image_size)?;
        ensure!(
            bare_box.source_width == observed_box.source_width
                && bare_box.source_height == observed_box.source_height
                && bare_box.resized_width == observed_box.resized_width
                && bare_box.resized_height == observed_box.resized_height
                && bare_box.left == observed_box.left
                && bare_box.top == observed_box.top,
            "held-part pair preprocessing differs"
        );
        input.extend(observed);
        let input =
            tract_ndarray::Array4::from_shape_vec((1, 6, self.image_size, self.image_size), input)?
                .into_tensor();
        let outputs = self
            .plan
            .run(tvec!(input.into()))
            .context("held-part inference failed")?;
        ensure!(outputs.len() == 2, "held-part model must have two outputs");
        let class_values = outputs[0]
            .to_array_view::<f32>()
            .context("held-part class output is not float32")?
            .iter()
            .copied()
            .collect::<Vec<_>>();
        let pose_values = outputs[1]
            .to_array_view::<f32>()
            .context("held-part pose output is not float32")?
            .iter()
            .copied()
            .collect::<Vec<_>>();
        ensure!(
            class_values.len() == 3,
            "class output must have three values"
        );
        ensure!(pose_values.len() == 6, "pose output must have six values");
        ensure!(
            class_values
                .iter()
                .chain(&pose_values)
                .all(|value| value.is_finite()),
            "held-part model returned a non-finite value"
        );

        let probabilities = softmax_three(class_values.try_into().unwrap());
        let mut order = [0_usize, 1, 2];
        order.sort_by(|first, second| probabilities[*second].total_cmp(&probabilities[*first]));
        let class = match order[0] {
            0 => HeldPartClass::BareNozzle,
            1 => HeldPartClass::ComponentFlat,
            2 => HeldPartClass::ComponentOnEdge,
            _ => unreachable!(),
        };
        let class_confidence = f64::from(probabilities[order[0]]);
        let class_margin = class_confidence - f64::from(probabilities[order[1]]);
        let source_pose = decode_pose(pose_values.try_into().unwrap(), &bare_box)?;
        let pose = HeldPartPosePrediction {
            center_px: source_pose.center_px,
            size_px: source_pose.size_px,
            rotation_deg_mod_180: source_pose.rotation_deg_mod_180,
            outline_px: source_pose.outline_px,
            valid: source_pose.valid,
        };
        let confidence_gate_passed = class_confidence >= HELD_PART_CLASS_PROBABILITY_GATE
            && class_margin >= HELD_PART_CLASS_MARGIN_GATE
            && (!class.occupied() || pose.valid);
        let mut findings = Vec::new();
        if class_confidence < HELD_PART_CLASS_PROBABILITY_GATE {
            findings.push(format!(
                "class probability {class_confidence:.3} is below the shadow gate \
                 {HELD_PART_CLASS_PROBABILITY_GATE:.3}"
            ));
        }
        if class_margin < HELD_PART_CLASS_MARGIN_GATE {
            findings.push(format!(
                "class margin {class_margin:.3} is below the shadow gate \
                 {HELD_PART_CLASS_MARGIN_GATE:.3}"
            ));
        }
        if class.occupied() && !pose.valid {
            findings.push("the occupied pose is outside the held-part crop".into());
        }
        Ok(HeldPartModelPrediction {
            contract_version: HELD_PART_POSE_CONTRACT.into(),
            model_artifact_sha256: self.artifact_sha256.clone(),
            runtime: HELD_PART_RUNTIME.into(),
            class,
            probabilities: HeldPartClassProbabilities {
                bare_nozzle: f64::from(probabilities[0]),
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

fn validate_held_part_manifest(manifest: &PackageManifest) -> Result<()> {
    ensure!(
        manifest.contract_version == HELD_PART_POSE_CONTRACT,
        "unsupported held-part model contract {}",
        manifest.contract_version
    );
    ensure!(
        manifest.license.artifact == "Apache-2.0",
        "held-part model artifact must use Apache-2.0 terms"
    );
    ensure!(
        manifest.model.config.image_size == EXPECTED_IMAGE_SIZE,
        "held-part model input must be {EXPECTED_IMAGE_SIZE} pixels"
    );
    ensure!(
        manifest.model.config.input_channels == 6,
        "held-part model must have six input channels"
    );
    ensure!(
        manifest.model.config.class_count == EXPECTED_HELD_PART_CLASSES.len(),
        "held-part model must have three classes"
    );
    ensure!(
        manifest.model.classes == EXPECTED_HELD_PART_CLASSES,
        "held-part model classes or class order changed"
    );
    ensure!(
        manifest.model.input.name == "image_pair",
        "held-part model input name changed"
    );
    ensure!(
        manifest.model.input.datum_type == "float32",
        "held-part model input must use float32"
    );
    ensure!(
        manifest.model.input.range == [0.0, 1.0],
        "held-part model input range changed"
    );
    ensure!(
        manifest.model.input.resize == EXPECTED_RESIZE,
        "held-part model input resize changed"
    );
    ensure!(
        manifest.model.input.letterbox_value == 127,
        "held-part model letterbox value changed"
    );
    ensure!(
        manifest.model.input.shape
            == [
                serde_json::Value::String("batch".into()),
                serde_json::Value::from(6),
                serde_json::Value::from(EXPECTED_IMAGE_SIZE),
                serde_json::Value::from(EXPECTED_IMAGE_SIZE),
            ],
        "held-part model input shape changed"
    );
    ensure!(
        manifest.model.angle_period_degrees == 180,
        "held-part angle period changed"
    );
    ensure!(
        !manifest.model.polarity_supported,
        "held-part v1 cannot claim polarity support"
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
            "contract_version": HELD_PART_POSE_CONTRACT,
            "model": {
                "config": {
                    "image_size": 192,
                    "input_channels": 6,
                    "class_count": 3
                },
                "classes": [
                    "bare_nozzle",
                    "component_flat",
                    "component_on_edge"
                ],
                "input": {
                    "name": "image_pair",
                    "shape": ["batch", 6, 192, 192],
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
        validate_held_part_manifest(&manifest(valid_manifest_value())).unwrap();
    }

    #[test]
    fn rejects_a_directed_axis_claim() {
        let mut value = valid_manifest_value();
        value["model"]["polarity_supported"] = serde_json::json!(true);
        let error = validate_held_part_manifest(&manifest(value)).unwrap_err();
        assert!(error.to_string().contains("polarity"));
    }

    #[test]
    fn rejects_changed_input_channels() {
        let mut value = valid_manifest_value();
        value["model"]["config"]["input_channels"] = serde_json::json!(3);
        let error = validate_held_part_manifest(&manifest(value)).unwrap_err();
        assert!(error.to_string().contains("six input channels"));
    }
}
