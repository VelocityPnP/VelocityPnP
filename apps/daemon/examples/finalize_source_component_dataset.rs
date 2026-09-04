use anyhow::{bail, ensure, Context, Result};
use chrono::Utc;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Debug, Deserialize)]
struct RefinedManifest {
    name: String,
    classes: Vec<String>,
    samples: Vec<RefinedSample>,
}

#[derive(Debug, Deserialize)]
struct RefinedSample {
    id: String,
    split: String,
    image_path: String,
    width_px: u32,
    height_px: u32,
    part_key: Option<String>,
    first_decision: String,
    class_name: Option<String>,
    outline_px: Option<[[f64; 2]; 4]>,
    requires_review: bool,
}

#[derive(Debug, Deserialize)]
struct ReviewFile {
    schema_version: u32,
    dataset: String,
    decisions: BTreeMap<String, FinalDecision>,
}

#[derive(Debug, Deserialize)]
struct FinalDecision {
    decision: String,
    #[serde(default)]
    note: String,
    class_name: Option<String>,
    outline_px: Option<[[f64; 2]; 4]>,
}

#[derive(Debug)]
struct ResolvedLabel {
    included: bool,
    class_name: Option<String>,
    outline_px: Option<[[f64; 2]; 4]>,
    source: String,
    note: String,
}

#[derive(Debug, Serialize)]
struct GroundTruthManifest {
    schema_version: u32,
    name: String,
    generated_at: String,
    source_refined_manifest: String,
    source_refined_manifest_sha256: String,
    final_review: String,
    final_review_sha256: String,
    carried_ground_truth_manifest: Option<String>,
    carried_ground_truth_manifest_sha256: Option<String>,
    classes: Vec<String>,
    notes: Vec<String>,
    samples: Vec<GroundTruthSample>,
}

#[derive(Debug, Serialize)]
struct GroundTruthSample {
    id: String,
    split: String,
    image_path: String,
    part_key: Option<String>,
    included: bool,
    class_name: Option<String>,
    outline_px: Option<[[f64; 2]; 4]>,
    label_source: String,
    review_note: String,
}

#[derive(Debug, Deserialize)]
struct CarriedGroundTruthManifest {
    classes: Vec<String>,
    samples: Vec<CarriedGroundTruthSample>,
}

#[derive(Debug, Deserialize)]
struct CarriedGroundTruthSample {
    id: String,
    split: String,
    image_path: String,
    part_key: Option<String>,
    included: bool,
    class_name: Option<String>,
    outline_px: Option<[[f64; 2]; 4]>,
    label_source: String,
    review_note: String,
    width_px: u32,
    height_px: u32,
    image_sha256: String,
}

fn main() -> Result<()> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    ensure!(
        arguments.len() == 2,
        "usage: finalize_source_component_dataset DATASET_DIR REVIEW_FILE"
    );
    let root = PathBuf::from(&arguments[0]);
    let review_input = resolve_review_path(&root, Path::new(&arguments[1]));
    let manifest_path = root.join("manifest.refined.json");
    let final_manifest_path = root.join("manifest.ground-truth.json");
    ensure!(
        !final_manifest_path.exists(),
        "ground-truth manifest already exists"
    );
    let manifest_bytes = fs::read(&manifest_path)
        .with_context(|| format!("cannot read {}", manifest_path.display()))?;
    let review_bytes = fs::read(&review_input)
        .with_context(|| format!("cannot read {}", review_input.display()))?;
    let manifest: RefinedManifest = serde_json::from_slice(&manifest_bytes)?;
    let review: ReviewFile = serde_json::from_slice(&review_bytes)?;
    let carried_path = root.join("manifest.carried-ground-truth.json");
    let carried_bytes = carried_path
        .is_file()
        .then(|| fs::read(&carried_path))
        .transpose()?;
    let carried: Option<CarriedGroundTruthManifest> = carried_bytes
        .as_deref()
        .map(serde_json::from_slice)
        .transpose()?;
    validate_review(&manifest, &review)?;
    validate_samples(&root, &manifest, &review)?;
    if let Some(carried) = &carried {
        ensure!(
            carried.classes == manifest.classes,
            "carried ground truth uses different classes"
        );
        validate_carried_samples(&root, &manifest, carried)?;
    }

    let carried_count = carried.as_ref().map_or(0, |carried| carried.samples.len());
    let mut samples = Vec::with_capacity(manifest.samples.len() + carried_count);
    for sample in manifest.samples {
        let resolved = if sample.requires_review {
            resolve_final_decision(&review.decisions[&sample.id])?
        } else {
            resolve_first_decision(&sample)?
        };
        if let Some(outline) = resolved.outline_px {
            validate_outline(outline, sample.width_px, sample.height_px)
                .with_context(|| format!("sample {} has an invalid OBB", sample.id))?;
        }
        validate_class_and_outline(
            &manifest.classes,
            resolved.class_name.as_deref(),
            resolved.outline_px,
        )?;
        samples.push(GroundTruthSample {
            id: sample.id,
            split: sample.split,
            image_path: sample.image_path,
            part_key: sample.part_key,
            included: resolved.included,
            class_name: resolved.class_name,
            outline_px: resolved.outline_px,
            label_source: resolved.source,
            review_note: resolved.note,
        });
    }
    if let Some(carried) = carried {
        for sample in carried.samples {
            let image = fs::read(root.join(&sample.image_path))
                .with_context(|| format!("cannot read carried image {}", sample.image_path))?;
            ensure!(
                hex_sha256(&image) == sample.image_sha256,
                "carried image {} changed after review",
                sample.id
            );
            if let Some(outline) = sample.outline_px {
                validate_outline(outline, sample.width_px, sample.height_px)
                    .with_context(|| format!("carried sample {} has an invalid OBB", sample.id))?;
            }
            validate_class_and_outline(
                &manifest.classes,
                sample.class_name.as_deref(),
                sample.outline_px,
            )?;
            samples.push(GroundTruthSample {
                id: sample.id,
                split: sample.split,
                image_path: sample.image_path,
                part_key: sample.part_key,
                included: sample.included,
                class_name: sample.class_name,
                outline_px: sample.outline_px,
                label_source: sample.label_source,
                review_note: sample.review_note,
            });
        }
    }

    for split in ["train", "eval"] {
        ensure!(
            samples
                .iter()
                .any(|sample| sample.included && sample.split == split),
            "{split} split is empty"
        );
    }

    let review_name = "review-refined-decisions.json";
    fs::write(root.join(review_name), &review_bytes)?;
    let output = GroundTruthManifest {
        schema_version: 1,
        name: format!("{}-ground-truth", manifest.name),
        generated_at: Utc::now().to_rfc3339(),
        source_refined_manifest: "manifest.refined.json".into(),
        source_refined_manifest_sha256: hex_sha256(&manifest_bytes),
        final_review: review_name.into(),
        final_review_sha256: hex_sha256(&review_bytes),
        carried_ground_truth_manifest: carried_bytes
            .as_ref()
            .map(|_| "manifest.carried-ground-truth.json".into()),
        carried_ground_truth_manifest_sha256: carried_bytes.as_deref().map(hex_sha256),
        classes: manifest.classes,
        notes: vec![
            "included labels passed one or two explicit operator reviews".into(),
            "empty images remain in the manifest with no outline".into(),
            "rejected images remain in the manifest as excluded samples".into(),
            "the evaluation split does not yet cover every class or negatives".into(),
            "OBB axes do not encode the 180-degree component direction".into(),
        ],
        samples,
    };
    fs::write(&final_manifest_path, serde_json::to_vec_pretty(&output)?)?;

    let included = output
        .samples
        .iter()
        .filter(|sample| sample.included)
        .count();
    let positive = output
        .samples
        .iter()
        .filter(|sample| sample.included && sample.class_name.is_some())
        .count();
    let on_edge = output
        .samples
        .iter()
        .filter(|sample| sample.class_name.as_deref() == Some("component_on_edge"))
        .count();
    let empty = included - positive;
    println!(
        "finalized {included} images: {positive} positive, \
         {on_edge} on-edge, {empty} empty"
    );
    println!("dataset {}", final_manifest_path.display());
    Ok(())
}

fn resolve_review_path(root: &Path, review: &Path) -> PathBuf {
    if review.is_absolute() {
        review.to_path_buf()
    } else {
        root.join(review)
    }
}

fn validate_review(manifest: &RefinedManifest, review: &ReviewFile) -> Result<()> {
    ensure!(review.schema_version == 1, "unsupported review schema");
    ensure!(
        review.dataset == manifest.name,
        "review is for another dataset"
    );
    let expected = manifest
        .samples
        .iter()
        .filter(|sample| sample.requires_review)
        .map(|sample| sample.id.as_str())
        .collect::<BTreeSet<_>>();
    let actual = review
        .decisions
        .keys()
        .map(String::as_str)
        .collect::<BTreeSet<_>>();
    ensure!(
        actual == expected,
        "final review does not match the review set"
    );
    Ok(())
}

fn validate_samples(root: &Path, manifest: &RefinedManifest, review: &ReviewFile) -> Result<()> {
    let ids = manifest
        .samples
        .iter()
        .map(|sample| sample.id.as_str())
        .collect::<BTreeSet<_>>();
    ensure!(
        ids.len() == manifest.samples.len(),
        "refined manifest has duplicate sample IDs"
    );
    for sample in &manifest.samples {
        ensure!(
            matches!(sample.split.as_str(), "train" | "eval"),
            "sample {} has invalid split {}",
            sample.id,
            sample.split
        );
        ensure!(
            root.join(&sample.image_path).is_file(),
            "sample {} image is absent",
            sample.id
        );
        let resolved = if sample.requires_review {
            resolve_final_decision(&review.decisions[&sample.id])?
        } else {
            resolve_first_decision(sample)?
        };
        if let Some(outline) = resolved.outline_px {
            validate_outline(outline, sample.width_px, sample.height_px)
                .with_context(|| format!("sample {} has an invalid OBB", sample.id))?;
        }
        validate_class_and_outline(
            &manifest.classes,
            resolved.class_name.as_deref(),
            resolved.outline_px,
        )?;
    }
    Ok(())
}

fn validate_carried_samples(
    root: &Path,
    manifest: &RefinedManifest,
    carried: &CarriedGroundTruthManifest,
) -> Result<()> {
    let new_ids = manifest
        .samples
        .iter()
        .map(|sample| sample.id.as_str())
        .collect::<BTreeSet<_>>();
    let carried_ids = carried
        .samples
        .iter()
        .map(|sample| sample.id.as_str())
        .collect::<BTreeSet<_>>();
    ensure!(
        carried_ids.len() == carried.samples.len(),
        "carried ground truth has duplicate sample IDs"
    );
    ensure!(
        carried_ids.is_disjoint(&new_ids),
        "carried ground truth overlaps the new review set"
    );
    for sample in &carried.samples {
        ensure!(
            matches!(sample.split.as_str(), "train" | "eval"),
            "carried sample {} has invalid split {}",
            sample.id,
            sample.split
        );
        let image_path = Path::new(&sample.image_path);
        ensure!(
            !image_path.is_absolute()
                && image_path
                    .components()
                    .all(|component| matches!(component, std::path::Component::Normal(_))),
            "carried sample {} has an unsafe image path",
            sample.id
        );
        let image = fs::read(root.join(image_path))
            .with_context(|| format!("cannot read carried image {}", sample.image_path))?;
        ensure!(
            hex_sha256(&image) == sample.image_sha256,
            "carried image {} changed after review",
            sample.id
        );
        if let Some(outline) = sample.outline_px {
            validate_outline(outline, sample.width_px, sample.height_px)
                .with_context(|| format!("carried sample {} has an invalid OBB", sample.id))?;
        }
        validate_class_and_outline(
            &manifest.classes,
            sample.class_name.as_deref(),
            sample.outline_px,
        )?;
    }
    Ok(())
}

fn resolve_first_decision(sample: &RefinedSample) -> Result<ResolvedLabel> {
    match sample.first_decision.as_str() {
        "accept" => Ok(ResolvedLabel {
            included: true,
            class_name: Some(
                sample
                    .class_name
                    .clone()
                    .context("accepted first-pass box has no class")?,
            ),
            outline_px: Some(
                sample
                    .outline_px
                    .context("accepted first-pass box has no OBB")?,
            ),
            source: "first_review".into(),
            note: String::new(),
        }),
        "empty" => Ok(ResolvedLabel {
            included: true,
            class_name: None,
            outline_px: None,
            source: "first_review".into(),
            note: String::new(),
        }),
        "reject" => Ok(ResolvedLabel {
            included: false,
            class_name: None,
            outline_px: None,
            source: "first_review".into(),
            note: String::new(),
        }),
        value => bail!("unresolved first-pass decision {value}"),
    }
}

fn resolve_final_decision(decision: &FinalDecision) -> Result<ResolvedLabel> {
    match decision.decision.as_str() {
        "accept" => Ok(ResolvedLabel {
            included: true,
            class_name: Some(
                decision
                    .class_name
                    .clone()
                    .context("accepted box has no class")?,
            ),
            outline_px: Some(decision.outline_px.context("accepted box has no OBB")?),
            source: "second_review".into(),
            note: decision.note.clone(),
        }),
        "empty" => Ok(ResolvedLabel {
            included: true,
            class_name: None,
            outline_px: None,
            source: "second_review".into(),
            note: decision.note.clone(),
        }),
        "reject" => Ok(ResolvedLabel {
            included: false,
            class_name: None,
            outline_px: None,
            source: "second_review".into(),
            note: decision.note.clone(),
        }),
        value => bail!("unsupported final decision {value}"),
    }
}

fn validate_outline(outline: [[f64; 2]; 4], width: u32, height: u32) -> Result<()> {
    for point in outline {
        ensure!(
            point[0].is_finite() && point[1].is_finite(),
            "OBB has a non-finite point"
        );
        ensure!(
            (0.0..=f64::from(width)).contains(&point[0])
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
    let winding = edges
        .iter()
        .zip(edges.iter().cycle().skip(1))
        .take(4)
        .map(|(first, second)| first[0] * second[1] - first[1] * second[0])
        .collect::<Vec<_>>();
    ensure!(
        winding.iter().all(|value| *value > 0.0) || winding.iter().all(|value| *value < 0.0),
        "OBB point order is not convex"
    );
    Ok(())
}

fn validate_class_and_outline(
    classes: &[String],
    class_name: Option<&str>,
    outline: Option<[[f64; 2]; 4]>,
) -> Result<()> {
    ensure!(
        class_name.is_some() == outline.is_some(),
        "class and outline must occur together"
    );
    if let Some(class_name) = class_name {
        ensure!(
            classes.iter().any(|candidate| candidate == class_name),
            "unknown class {class_name}"
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
    fn accepts_a_rotated_rectangular_obb() {
        let angle = 0.37_f64;
        let u = [angle.cos(), angle.sin()];
        let v = [-u[1], u[0]];
        let points = [[-20.0, -8.0], [20.0, -8.0], [20.0, 8.0], [-20.0, 8.0]].map(|offset| {
            [
                50.0 + u[0] * offset[0] + v[0] * offset[1],
                40.0 + u[1] * offset[0] + v[1] * offset[1],
            ]
        });
        validate_outline(points, 100, 80).unwrap();
    }

    #[test]
    fn rejects_a_non_rectangular_box() {
        let points = [[10.0, 10.0], [50.0, 10.0], [45.0, 30.0], [10.0, 30.0]];
        assert!(validate_outline(points, 100, 80).is_err());
    }

    #[test]
    fn empty_decision_becomes_an_included_negative() {
        let decision = FinalDecision {
            decision: "empty".into(),
            note: String::new(),
            class_name: Some("component_flat".into()),
            outline_px: Some([[0.0; 2]; 4]),
        };
        let resolved = resolve_final_decision(&decision).unwrap();
        assert!(resolved.included);
        assert!(resolved.class_name.is_none());
        assert!(resolved.outline_px.is_none());
    }
}
