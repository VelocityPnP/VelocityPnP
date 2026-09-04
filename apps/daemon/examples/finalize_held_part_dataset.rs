use anyhow::{bail, ensure, Context, Result};
use chrono::Utc;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use uuid::Uuid;
use velocitypnp_domain::{HeldPartInspectionReport, PartOrientation};

#[derive(Debug, Deserialize)]
struct DraftManifest {
    name: String,
    generated_at: String,
    detector_algorithm: String,
    source_database: String,
    machine_profile: String,
    crop_size_px: u32,
    classes: Vec<String>,
    evaluation_registration_ids: Vec<Uuid>,
    split_policy: String,
    notes: Vec<String>,
    samples: Vec<DraftSample>,
}

#[derive(Debug, Deserialize)]
struct DraftSample {
    id: String,
    split: String,
    group_id: Uuid,
    source_kind: String,
    source_id: Uuid,
    source_artifact: String,
    observation_kind: String,
    bare_source_sha256: String,
    observed_source_sha256: String,
    bare_image_path: String,
    observed_image_path: String,
    bare_image_sha256: String,
    observed_image_sha256: String,
    width_px: u32,
    height_px: u32,
    nozzle_id: String,
    part_key: Option<String>,
    orientation: PartOrientation,
    evidence: Vec<String>,
    draft_state: String,
    deterministic: HeldPartInspectionReport,
}

#[derive(Debug, Deserialize)]
struct ReviewFile {
    schema_version: u32,
    dataset: String,
    source_manifest_generated_at: String,
    decisions: BTreeMap<String, ReviewDecision>,
}

#[derive(Debug, Deserialize)]
struct ReviewDecision {
    decision: String,
    #[serde(default)]
    note: String,
    class_name: Option<String>,
    outline_px: Option<[[f64; 2]; 4]>,
    marker_status: Option<String>,
    marker_end_px: Option<[f64; 2]>,
}

#[derive(Debug)]
struct ResolvedLabel {
    included: bool,
    class_name: Option<String>,
    outline_px: Option<[[f64; 2]; 4]>,
    marker_status: String,
    marker_end_px: Option<[f64; 2]>,
}

#[derive(Debug, Serialize)]
struct GroundTruthManifest {
    schema_version: u32,
    name: String,
    generated_at: String,
    source_draft_manifest: String,
    source_draft_manifest_sha256: String,
    source_review: String,
    source_review_sha256: String,
    detector_algorithm: String,
    source_database: String,
    machine_profile: String,
    crop_size_px: u32,
    classes: Vec<String>,
    evaluation_registration_ids: Vec<Uuid>,
    split_policy: String,
    notes: Vec<String>,
    samples: Vec<GroundTruthSample>,
}

#[derive(Debug, Serialize)]
struct GroundTruthSample {
    id: String,
    split: String,
    group_id: Uuid,
    source_kind: String,
    source_id: Uuid,
    source_artifact: String,
    observation_kind: String,
    bare_source_sha256: String,
    observed_source_sha256: String,
    bare_image_path: String,
    observed_image_path: String,
    bare_image_sha256: String,
    observed_image_sha256: String,
    width_px: u32,
    height_px: u32,
    nozzle_id: String,
    part_key: Option<String>,
    orientation: PartOrientation,
    evidence: Vec<String>,
    draft_state: String,
    deterministic: HeldPartInspectionReport,
    included: bool,
    class_name: Option<String>,
    outline_px: Option<[[f64; 2]; 4]>,
    marker_status: String,
    marker_end_px: Option<[f64; 2]>,
    label_source: String,
    review_note: String,
}

fn main() -> Result<()> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    ensure!(
        arguments.len() == 2,
        "usage: finalize_held_part_dataset DATASET_DIR REVIEW_FILE"
    );
    let root = PathBuf::from(&arguments[0]);
    let review_input = resolve_review_path(&root, Path::new(&arguments[1]));
    let manifest_path = root.join("manifest.draft.json");
    let output_path = root.join("manifest.ground-truth.json");
    ensure!(
        !output_path.exists(),
        "ground-truth manifest already exists"
    );
    let manifest_bytes = fs::read(&manifest_path)
        .with_context(|| format!("cannot read {}", manifest_path.display()))?;
    let review_bytes = fs::read(&review_input)
        .with_context(|| format!("cannot read {}", review_input.display()))?;
    let manifest: DraftManifest = serde_json::from_slice(&manifest_bytes)?;
    let review: ReviewFile = serde_json::from_slice(&review_bytes)?;
    validate_review(&manifest, &review)?;

    let mut samples = Vec::with_capacity(manifest.samples.len());
    for sample in manifest.samples {
        validate_sample_images(&root, &sample)?;
        let decision = &review.decisions[&sample.id];
        let resolved = resolve_decision(decision)?;
        validate_label(&manifest.classes, &sample, &resolved)
            .with_context(|| format!("sample {} has an invalid label", sample.id))?;
        samples.push(GroundTruthSample {
            id: sample.id,
            split: sample.split,
            group_id: sample.group_id,
            source_kind: sample.source_kind,
            source_id: sample.source_id,
            source_artifact: sample.source_artifact,
            observation_kind: sample.observation_kind,
            bare_source_sha256: sample.bare_source_sha256,
            observed_source_sha256: sample.observed_source_sha256,
            bare_image_path: sample.bare_image_path,
            observed_image_path: sample.observed_image_path,
            bare_image_sha256: sample.bare_image_sha256,
            observed_image_sha256: sample.observed_image_sha256,
            width_px: sample.width_px,
            height_px: sample.height_px,
            nozzle_id: sample.nozzle_id,
            part_key: sample.part_key,
            orientation: sample.orientation,
            evidence: sample.evidence,
            draft_state: sample.draft_state,
            deterministic: sample.deterministic,
            included: resolved.included,
            class_name: resolved.class_name,
            outline_px: resolved.outline_px,
            marker_status: resolved.marker_status,
            marker_end_px: resolved.marker_end_px,
            label_source: "operator_review".into(),
            review_note: decision.note.clone(),
        });
    }
    for split in ["train", "eval"] {
        ensure!(
            samples
                .iter()
                .any(|sample| sample.included && sample.split == split),
            "{split} split is empty"
        );
    }

    let review_name = "review-decisions.json";
    fs::write(root.join(review_name), &review_bytes)?;
    let mut notes = manifest.notes;
    notes.extend([
        "every included label passed explicit operator review".into(),
        "rejected image pairs remain as excluded provenance records".into(),
        "directed-end labels exist only where the marker was visible".into(),
    ]);
    let output = GroundTruthManifest {
        schema_version: 1,
        name: "velocitypnp-held-parts-v1-ground-truth".into(),
        generated_at: Utc::now().to_rfc3339(),
        source_draft_manifest: "manifest.draft.json".into(),
        source_draft_manifest_sha256: hex_sha256(&manifest_bytes),
        source_review: review_name.into(),
        source_review_sha256: hex_sha256(&review_bytes),
        detector_algorithm: manifest.detector_algorithm,
        source_database: manifest.source_database,
        machine_profile: manifest.machine_profile,
        crop_size_px: manifest.crop_size_px,
        classes: manifest.classes,
        evaluation_registration_ids: manifest.evaluation_registration_ids,
        split_policy: manifest.split_policy,
        notes,
        samples,
    };
    fs::write(&output_path, serde_json::to_vec_pretty(&output)?)?;

    let included = output
        .samples
        .iter()
        .filter(|sample| sample.included)
        .count();
    let bare = output
        .samples
        .iter()
        .filter(|sample| sample.included && sample.class_name.is_none())
        .count();
    let edge = output
        .samples
        .iter()
        .filter(|sample| sample.class_name.as_deref() == Some("component_on_edge"))
        .count();
    let multiple = output
        .samples
        .iter()
        .filter(|sample| sample.class_name.as_deref() == Some("multiple_components"))
        .count();
    let directed = output
        .samples
        .iter()
        .filter(|sample| sample.marker_status == "visible")
        .count();
    println!(
        "finalized {included} image pairs: {bare} bare, {edge} on-edge, \
         {multiple} multiple, {directed} with visible directed ends"
    );
    println!("dataset {}", output_path.display());
    Ok(())
}

fn resolve_review_path(root: &Path, review: &Path) -> PathBuf {
    if review.is_absolute() {
        review.to_path_buf()
    } else {
        root.join(review)
    }
}

fn validate_review(manifest: &DraftManifest, review: &ReviewFile) -> Result<()> {
    ensure!(review.schema_version == 1, "unsupported review schema");
    ensure!(
        review.dataset == manifest.name,
        "review is for another dataset"
    );
    ensure!(
        review.source_manifest_generated_at == manifest.generated_at,
        "review is for another export of this dataset"
    );
    let expected = manifest
        .samples
        .iter()
        .map(|sample| sample.id.as_str())
        .collect::<BTreeSet<_>>();
    let actual = review
        .decisions
        .keys()
        .map(String::as_str)
        .collect::<BTreeSet<_>>();
    ensure!(
        expected.len() == manifest.samples.len(),
        "draft manifest has duplicate sample IDs"
    );
    ensure!(
        actual == expected,
        "review does not contain every draft sample"
    );
    Ok(())
}

fn validate_sample_images(root: &Path, sample: &DraftSample) -> Result<()> {
    ensure!(
        matches!(sample.split.as_str(), "train" | "eval"),
        "sample {} has invalid split {}",
        sample.id,
        sample.split
    );
    for (path, digest) in [
        (&sample.bare_image_path, &sample.bare_image_sha256),
        (&sample.observed_image_path, &sample.observed_image_sha256),
    ] {
        let relative = Path::new(path);
        ensure!(
            !relative.is_absolute()
                && relative
                    .components()
                    .all(|part| matches!(part, std::path::Component::Normal(_))),
            "sample {} has an unsafe image path",
            sample.id
        );
        let bytes = fs::read(root.join(relative))
            .with_context(|| format!("cannot read sample image {path}"))?;
        ensure!(
            hex_sha256(&bytes) == *digest,
            "sample {} image changed after export",
            sample.id
        );
    }
    Ok(())
}

fn resolve_decision(decision: &ReviewDecision) -> Result<ResolvedLabel> {
    match decision.decision.as_str() {
        "accept" => Ok(ResolvedLabel {
            included: true,
            class_name: Some(
                decision
                    .class_name
                    .clone()
                    .context("accepted sample has no class")?,
            ),
            outline_px: Some(decision.outline_px.context("accepted sample has no OBB")?),
            marker_status: decision
                .marker_status
                .clone()
                .unwrap_or_else(|| "not_applicable".into()),
            marker_end_px: decision.marker_end_px,
        }),
        "empty" => Ok(ResolvedLabel {
            included: true,
            class_name: None,
            outline_px: None,
            marker_status: "not_applicable".into(),
            marker_end_px: None,
        }),
        "reject" => Ok(ResolvedLabel {
            included: false,
            class_name: None,
            outline_px: None,
            marker_status: "not_applicable".into(),
            marker_end_px: None,
        }),
        value => bail!("unsupported review decision {value}"),
    }
}

fn validate_label(classes: &[String], sample: &DraftSample, label: &ResolvedLabel) -> Result<()> {
    if !label.included || label.class_name.is_none() {
        ensure!(
            label.outline_px.is_none()
                && label.marker_end_px.is_none()
                && label.marker_status == "not_applicable",
            "bare or rejected labels cannot contain component geometry"
        );
        return Ok(());
    }
    let class_name = label.class_name.as_deref().unwrap();
    ensure!(
        classes.iter().any(|value| value == class_name),
        "unknown class {class_name}"
    );
    validate_outline(label.outline_px.unwrap(), sample.width_px, sample.height_px)?;
    ensure!(
        matches!(
            label.marker_status.as_str(),
            "not_applicable" | "visible" | "not_visible" | "uncertain"
        ),
        "invalid marker state {}",
        label.marker_status
    );
    if label.marker_status == "visible" {
        ensure!(
            sample.orientation.is_directed(),
            "axis-symmetric part has a directed end"
        );
        let marker = label
            .marker_end_px
            .context("visible marker has no endpoint")?;
        ensure!(
            marker.into_iter().all(f64::is_finite)
                && (0.0..=f64::from(sample.width_px)).contains(&marker[0])
                && (0.0..=f64::from(sample.height_px)).contains(&marker[1]),
            "directed endpoint is outside the image"
        );
    } else {
        ensure!(
            label.marker_end_px.is_none(),
            "hidden marker state has an endpoint"
        );
    }
    Ok(())
}

fn validate_outline(outline: [[f64; 2]; 4], width: u32, height: u32) -> Result<()> {
    for point in outline {
        ensure!(
            point.into_iter().all(f64::is_finite)
                && (0.0..=f64::from(width)).contains(&point[0])
                && (0.0..=f64::from(height)).contains(&point[1]),
            "OBB leaves the image"
        );
    }
    let edges = std::array::from_fn::<_, 4, _>(|index| {
        let next = (index + 1) % 4;
        [
            outline[next][0] - outline[index][0],
            outline[next][1] - outline[index][1],
        ]
    });
    let lengths = edges.map(|edge| edge[0].hypot(edge[1]));
    ensure!(
        lengths.iter().all(|length| *length >= 2.0),
        "OBB edge is too short"
    );
    for index in 0..4 {
        let next = (index + 1) % 4;
        let cosine = (edges[index][0] * edges[next][0] + edges[index][1] * edges[next][1])
            / (lengths[index] * lengths[next]);
        ensure!(cosine.abs() <= 0.02, "OBB corners are not square");
    }
    for (first, second) in [(0, 2), (1, 3)] {
        let scale = lengths[first].max(lengths[second]);
        ensure!(
            (lengths[first] - lengths[second]).abs() <= 0.02 * scale,
            "OBB opposite edges differ"
        );
    }
    Ok(())
}

fn hex_sha256(data: &[u8]) -> String {
    Sha256::digest(data)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn relative_review_path_uses_the_dataset_directory() {
        let root = Path::new("dataset");
        assert_eq!(
            resolve_review_path(root, Path::new("review.json")),
            root.join("review.json")
        );
        assert_eq!(
            resolve_review_path(root, Path::new("/tmp/review.json")),
            PathBuf::from("/tmp/review.json")
        );
    }

    #[test]
    fn empty_decision_drops_stale_editor_geometry() {
        let decision = ReviewDecision {
            decision: "empty".into(),
            note: String::new(),
            class_name: Some("component_flat".into()),
            outline_px: Some([[0.0; 2]; 4]),
            marker_status: Some("visible".into()),
            marker_end_px: Some([1.0, 1.0]),
        };
        let resolved = resolve_decision(&decision).unwrap();
        assert!(resolved.included);
        assert!(resolved.class_name.is_none());
        assert!(resolved.outline_px.is_none());
        assert!(resolved.marker_end_px.is_none());
    }

    #[test]
    fn accepts_a_rotated_rectangular_obb() {
        let angle = 0.37_f64;
        let u = [angle.cos(), angle.sin()];
        let v = [-u[1], u[0]];
        let outline = [[-20.0, -8.0], [20.0, -8.0], [20.0, 8.0], [-20.0, 8.0]].map(|offset| {
            [
                50.0 + u[0] * offset[0] + v[0] * offset[1],
                40.0 + u[1] * offset[0] + v[1] * offset[1],
            ]
        });
        validate_outline(outline, 100, 80).unwrap();
    }
}
