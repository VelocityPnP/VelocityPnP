use anyhow::{ensure, Context, Result};
use chrono::Utc;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use uuid::Uuid;
use velocitypnp_domain::{
    BottomCameraMappingReport, CalibrationArtifactId, HeldPartInspectionReport, MachineProfile,
    Part, PartOrientation, StripFeederPickupQualificationReport, StripFeederRegistration,
};
use velocitypnp_store::Store;
use velocitypnp_vision::{
    decode_gray_png, encode_gray_png, inspect_held_part_difference_with_orientation, GrayFrame,
    HELD_PART_INSPECTION_ALGORITHM,
};

const BARE_NOZZLE_ARTIFACT: &str = "bare-nozzle.png";
const CROP_SIZE_PX: u32 = 512;

#[derive(Debug, Serialize)]
struct DatasetManifest {
    schema_version: u32,
    name: String,
    generated_at: String,
    detector_algorithm: String,
    source_database: String,
    machine_profile: String,
    crop_size_px: u32,
    classes: Vec<String>,
    evaluation_registration_ids: Vec<Uuid>,
    split_policy: String,
    carried_ground_truth_manifest: Option<String>,
    carried_sample_count: usize,
    notes: Vec<String>,
    samples: Vec<DatasetSample>,
}

#[derive(Debug, Serialize)]
struct DatasetSample {
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
    #[serde(skip_serializing_if = "Option::is_none")]
    requested_nozzle_rotation_offset_deg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    actual_nozzle_rotation_offset_deg: Option<f64>,
    evidence: Vec<String>,
    draft_state: String,
    draft_outline_px: Option<[[f64; 2]; 4]>,
    seed_outline_px: [[f64; 2]; 4],
    draft_marker_status: String,
    draft_marker_end_px: Option<[f64; 2]>,
    deterministic: HeldPartInspectionReport,
}

#[derive(Debug, Deserialize)]
struct ReviewedGroundTruthManifest {
    classes: Vec<String>,
    samples: Vec<ReviewedGroundTruthSample>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
struct ReviewedGroundTruthSample {
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    requested_nozzle_rotation_offset_deg: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    actual_nozzle_rotation_offset_deg: Option<f64>,
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

struct ReviewedDataset {
    root: PathBuf,
    draft_manifest_sha256: String,
    ground_truth_manifest_sha256: String,
    samples_by_id: BTreeMap<String, ReviewedGroundTruthSample>,
}

#[derive(Debug, Serialize, Deserialize)]
struct CarriedGroundTruthManifest {
    schema_version: u32,
    source_dataset: String,
    source_draft_manifest_sha256: String,
    source_ground_truth_manifest_sha256: String,
    classes: Vec<String>,
    samples: Vec<ReviewedGroundTruthSample>,
}

struct ExportContext<'a> {
    output: &'a Path,
    profile: &'a MachineProfile,
    evaluation_groups: &'a BTreeSet<Uuid>,
    parts: &'a BTreeMap<String, Part>,
    reviewed: Option<&'a ReviewedDataset>,
    mappings: BTreeMap<Uuid, [[f64; 2]; 2]>,
    samples: Vec<DatasetSample>,
    carried_samples: Vec<ReviewedGroundTruthSample>,
    omitted_incompatible_pairs: usize,
}

struct Observation<'a> {
    source_kind: &'static str,
    source_id: Uuid,
    group_id: Uuid,
    calibration_id: Uuid,
    artifact_name: &'a str,
    observation_kind: &'static str,
    nozzle_id: &'a str,
    part_key: Option<&'a str>,
    requested_nozzle_rotation_offset_deg: Option<f64>,
    actual_nozzle_rotation_offset_deg: Option<f64>,
    learned_outline_px: Option<[[f64; 2]; 4]>,
    evidence: Vec<String>,
}

fn main() -> Result<()> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    let mut positional = Vec::new();
    let mut reviewed_root = None;
    let mut index = 0;
    while index < arguments.len() {
        if arguments[index] == "--carry-reviewed" {
            index += 1;
            let path = arguments
                .get(index)
                .context("--carry-reviewed requires a dataset directory")?;
            reviewed_root = Some(PathBuf::from(path));
        } else {
            positional.push(arguments[index].clone());
        }
        index += 1;
    }
    ensure!(
        positional.len() >= 4,
        "usage: export_held_part_dataset DATABASE PROFILE OUTPUT_DIR \
         EVAL_REGISTRATION_UUID [EVAL_REGISTRATION_UUID ...] \
         [--carry-reviewed DATASET_DIR]"
    );
    let database = PathBuf::from(&positional[0]);
    let profile_path = PathBuf::from(&positional[1]);
    let output = PathBuf::from(&positional[2]);
    let evaluation_groups = positional[3..]
        .iter()
        .map(|value| {
            value
                .parse::<Uuid>()
                .with_context(|| format!("invalid evaluation registration {value}"))
        })
        .collect::<Result<BTreeSet<_>>>()?;
    let reviewed = reviewed_root
        .as_deref()
        .map(load_reviewed_dataset)
        .transpose()?;
    ensure!(database.is_file(), "database does not exist");
    ensure!(profile_path.is_file(), "machine profile does not exist");
    ensure!(
        !output.join("manifest.draft.json").exists(),
        "output already contains a draft manifest"
    );

    let profile: MachineProfile = serde_json::from_slice(&fs::read(&profile_path)?)?;
    ensure!(
        profile.bottom_vision.is_some() && profile.strip_feeder_pickup.is_some(),
        "machine profile has no bottom-camera pickup policy"
    );
    profile
        .bottom_vision
        .as_ref()
        .unwrap()
        .validate()
        .map_err(anyhow::Error::msg)?;
    profile
        .strip_feeder_pickup
        .as_ref()
        .unwrap()
        .validate()
        .map_err(anyhow::Error::msg)?;
    let store = Store::open(&database)?;
    let registrations = store.list_strip_feeder_registrations()?;
    let registrations_by_id = registrations
        .iter()
        .map(|registration| (registration.id, registration.clone()))
        .collect::<BTreeMap<_, _>>();
    let registrations_by_feeder = registrations
        .iter()
        .filter_map(|registration| {
            registration
                .active_feeder_id
                .map(|feeder_id| (feeder_id, registration.clone()))
        })
        .collect::<BTreeMap<_, _>>();
    for group in &evaluation_groups {
        ensure!(
            registrations_by_id.contains_key(group),
            "evaluation registration {group} does not exist"
        );
    }
    let parts = store
        .list_parts()?
        .into_iter()
        .map(|part| (part.key.clone(), part))
        .collect::<BTreeMap<_, _>>();

    fs::create_dir_all(&output)?;
    for split in ["train", "eval"] {
        fs::create_dir_all(output.join("images").join(split))?;
    }
    let mut context = ExportContext {
        output: &output,
        profile: &profile,
        evaluation_groups: &evaluation_groups,
        parts: &parts,
        reviewed: reviewed.as_ref(),
        mappings: BTreeMap::new(),
        samples: Vec::new(),
        carried_samples: Vec::new(),
        omitted_incompatible_pairs: 0,
    };

    export_pickup_qualifications(&store, &registrations_by_id, &mut context)?;
    export_placement_executions(&store, &registrations_by_feeder, &mut context)?;
    ensure!(
        !context.samples.is_empty(),
        "no paired bottom-camera images were found"
    );
    ensure!(
        context.samples.iter().any(|sample| sample.split == "train")
            || context
                .carried_samples
                .iter()
                .any(|sample| sample.split == "train"),
        "the train split is empty"
    );
    ensure!(
        context.samples.iter().any(|sample| sample.split == "eval")
            || context
                .carried_samples
                .iter()
                .any(|sample| sample.split == "eval"),
        "the evaluation split is empty"
    );
    context.samples.sort_by_key(|sample| {
        (
            draft_state_priority(&sample.draft_state),
            sample.split != "eval",
            sample.group_id,
            sample.source_id,
            sample.source_artifact.clone(),
        )
    });
    context
        .carried_samples
        .sort_by(|first, second| first.id.cmp(&second.id));

    let carried_manifest_name = reviewed
        .as_ref()
        .map(|_| "manifest.carried-ground-truth.json".to_string());
    if let Some(reviewed) = &reviewed {
        let carried = CarriedGroundTruthManifest {
            schema_version: 1,
            source_dataset: reviewed.root.display().to_string(),
            source_draft_manifest_sha256: reviewed.draft_manifest_sha256.clone(),
            source_ground_truth_manifest_sha256: reviewed.ground_truth_manifest_sha256.clone(),
            classes: vec![
                "component_flat".into(),
                "component_on_edge".into(),
                "multiple_components".into(),
            ],
            samples: context.carried_samples.clone(),
        };
        fs::write(
            output.join(
                carried_manifest_name
                    .as_deref()
                    .expect("a reviewed dataset has a carried manifest"),
            ),
            serde_json::to_vec_pretty(&carried)?,
        )?;
    }
    let carried_count = context.carried_samples.len();

    let manifest = DatasetManifest {
        schema_version: 1,
        name: "velocitypnp-held-parts-v1-draft".into(),
        generated_at: Utc::now().to_rfc3339(),
        detector_algorithm: HELD_PART_INSPECTION_ALGORITHM.into(),
        source_database: database.display().to_string(),
        machine_profile: profile_path.display().to_string(),
        crop_size_px: CROP_SIZE_PX,
        classes: vec![
            "component_flat".into(),
            "component_on_edge".into(),
            "multiple_components".into(),
        ],
        evaluation_registration_ids: evaluation_groups.iter().copied().collect(),
        split_policy:
            "all images from one feeder registration or retained feeder identity stay in one split"
                .into(),
        carried_ground_truth_manifest: carried_manifest_name,
        carried_sample_count: carried_count,
        notes: vec![
            "draft labels are weak supervision, not ground truth".into(),
            "reviewed labels are carried only when both source and crop hashes match".into(),
            "each sample keeps aligned-size bare and observed camera crops".into(),
            "empty is an included negative state with no component outline".into(),
            "the multiple-components class uses one enclosing oriented box".into(),
            "a visible polarity marker is stored as a directed endpoint".into(),
            format!(
                "{} incompatible or simulator-sized pairs were omitted",
                context.omitted_incompatible_pairs
            ),
            "the first evaluation split is provisional and uses one machine".into(),
            "model output must remain shadow-only until independent gates pass".into(),
        ],
        samples: context.samples,
    };
    let manifest_json = serde_json::to_string_pretty(&manifest)?;
    fs::write(output.join("manifest.draft.json"), &manifest_json)?;
    fs::write(
        output.join("review.html"),
        review_html(&manifest, &manifest_json),
    )?;
    fs::write(
        output.join("README.txt"),
        format!(
            "Open review.html. Review only the new image pairs.\n\
             {carried_count} prior reviewed labels were carried without another review.\n\
             Correct the state and green oriented box. Mark a polarity end only when\n\
             it is visible. Download review-decisions.json when all samples are done.\n"
        ),
    )?;

    let train = manifest
        .samples
        .iter()
        .filter(|sample| sample.split == "train")
        .count();
    let evaluation = manifest.samples.len() - train;
    let held = manifest
        .samples
        .iter()
        .filter(|sample| sample.observation_kind.starts_with("held"))
        .count();
    let release = manifest.samples.len() - held;
    let conflicts = manifest
        .samples
        .iter()
        .filter(|sample| sample.draft_state == "evidence_conflict")
        .count();
    println!(
        "exported {} new paired samples and carried {carried_count} reviewed samples: \
         {train} new train, {evaluation} new eval, {held} held, {release} post-release, \
         {conflicts} evidence conflicts",
        manifest.samples.len(),
    );
    println!("review {}", output.join("review.html").display());
    Ok(())
}

fn export_pickup_qualifications(
    store: &Store,
    registrations: &BTreeMap<Uuid, StripFeederRegistration>,
    context: &mut ExportContext<'_>,
) -> Result<()> {
    for run in store.list_strip_feeder_pickup_qualifications()? {
        let Some(report) = run.report else {
            continue;
        };
        let registration = registrations
            .get(&run.registration_id)
            .with_context(|| format!("registration {} is absent", run.registration_id))?;
        let Some((bare_type, bare_bytes)) =
            store.get_strip_feeder_pickup_artifact(run.id, BARE_NOZZLE_ARTIFACT)?
        else {
            continue;
        };
        ensure!(
            bare_type == "image/png",
            "bare-nozzle artifact is not a PNG"
        );
        for (name, media_type) in store.list_strip_feeder_pickup_artifacts(run.id)? {
            let Some(observation_kind) = observation_kind(&name) else {
                continue;
            };
            ensure!(media_type == "image/png", "observed artifact is not a PNG");
            let rotation_capture = report
                .rotation_captures
                .iter()
                .find(|capture| capture.held_artifact_name == name);
            let paired_bare = if let Some(capture) = rotation_capture {
                let (media_type, data) = store
                    .get_strip_feeder_pickup_artifact(run.id, &capture.bare_artifact_name)?
                    .with_context(|| {
                        format!(
                            "pickup {} is missing artifact {}",
                            run.id, capture.bare_artifact_name
                        )
                    })?;
                ensure!(
                    media_type == "image/png",
                    "rotation bare artifact is not a PNG"
                );
                Some(data)
            } else {
                None
            };
            let (_, observed_bytes) = store
                .get_strip_feeder_pickup_artifact(run.id, &name)?
                .with_context(|| format!("pickup {} is missing artifact {name}", run.id))?;
            let evidence = pickup_evidence(&report, observation_kind, &name);
            export_observation(
                store,
                context,
                paired_bare.as_deref().unwrap_or(&bare_bytes),
                &observed_bytes,
                Observation {
                    source_kind: "pickup_qualification",
                    source_id: run.id,
                    group_id: registration.id,
                    calibration_id: run.calibration_id,
                    artifact_name: &name,
                    observation_kind,
                    nozzle_id: &report.nozzle_id,
                    part_key: registration.part_key.as_deref(),
                    requested_nozzle_rotation_offset_deg: rotation_capture
                        .map(|capture| capture.requested_rotation_offset_deg),
                    actual_nozzle_rotation_offset_deg: rotation_capture
                        .map(|capture| capture.held_rotation_offset_deg),
                    learned_outline_px: rotation_capture.and_then(|capture| {
                        let prediction = &capture.shadow.as_ref()?.prediction;
                        (prediction.class.occupied() && prediction.pose.valid)
                            .then_some(prediction.pose.outline_px)
                    }),
                    evidence,
                },
            )?;
        }
    }
    Ok(())
}

fn export_placement_executions(
    store: &Store,
    registrations: &BTreeMap<Uuid, StripFeederRegistration>,
    context: &mut ExportContext<'_>,
) -> Result<()> {
    for run in store.list_placement_executions()? {
        let Some(report) = run.report else {
            continue;
        };
        let group_id = registrations
            .get(&report.feeder_id)
            .map_or(report.feeder_id, |registration| registration.id);
        let Some((bare_type, bare_bytes)) =
            store.get_placement_execution_artifact(run.id, BARE_NOZZLE_ARTIFACT)?
        else {
            continue;
        };
        ensure!(
            bare_type == "image/png",
            "bare-nozzle artifact is not a PNG"
        );
        for (name, media_type) in store.list_placement_execution_artifacts(run.id)? {
            let Some(observation_kind) = observation_kind(&name) else {
                continue;
            };
            ensure!(media_type == "image/png", "observed artifact is not a PNG");
            let (_, observed_bytes) = store
                .get_placement_execution_artifact(run.id, &name)?
                .with_context(|| format!("placement {} is missing artifact {name}", run.id))?;
            let evidence = placement_evidence(&report, observation_kind, &name);
            export_observation(
                store,
                context,
                &bare_bytes,
                &observed_bytes,
                Observation {
                    source_kind: "placement_execution",
                    source_id: run.id,
                    group_id,
                    calibration_id: run.calibration_id,
                    artifact_name: &name,
                    observation_kind,
                    nozzle_id: &report.nozzle_id,
                    part_key: Some(&report.part_key),
                    requested_nozzle_rotation_offset_deg: None,
                    actual_nozzle_rotation_offset_deg: None,
                    learned_outline_px: None,
                    evidence,
                },
            )?;
        }
    }
    Ok(())
}

fn export_observation(
    store: &Store,
    context: &mut ExportContext<'_>,
    bare_bytes: &[u8],
    observed_bytes: &[u8],
    observation: Observation<'_>,
) -> Result<()> {
    let mapping = if let Some(mapping) = context.mappings.get(&observation.calibration_id) {
        *mapping
    } else {
        let mapping = load_bottom_camera_mapping(store, observation.calibration_id)?;
        context.mappings.insert(observation.calibration_id, mapping);
        mapping
    };
    let bare = decode_gray_png(bare_bytes)?;
    let observed = decode_gray_png(observed_bytes)?;
    ensure!(
        bare.width == observed.width && bare.height == observed.height,
        "paired bottom-camera images have different dimensions"
    );
    if bare.width < CROP_SIZE_PX || bare.height < CROP_SIZE_PX {
        context.omitted_incompatible_pairs += 1;
        return Ok(());
    }
    let bottom = context.profile.bottom_vision.as_ref().unwrap();
    let pickup = context.profile.strip_feeder_pickup.as_ref().unwrap();
    let orientation = observation
        .part_key
        .and_then(|key| context.parts.get(key))
        .and_then(|part| part.orientation)
        .unwrap_or_default();
    let artifacts = inspect_held_part_difference_with_orientation(
        &bare,
        &observed,
        [bottom.frame_center_x_px, bottom.frame_center_y_px],
        mapping,
        pickup.maximum_part_offset_mm,
        pickup.minimum_part_size_mm,
        pickup.maximum_part_size_mm,
        orientation,
        "dataset-draft.png",
    )?;
    let (bare_crop, origin) = centered_crop(
        &bare,
        [bottom.frame_center_x_px, bottom.frame_center_y_px],
        CROP_SIZE_PX,
    )?;
    let (observed_crop, observed_origin) = centered_crop(
        &observed,
        [bottom.frame_center_x_px, bottom.frame_center_y_px],
        CROP_SIZE_PX,
    )?;
    ensure!(origin == observed_origin, "paired crop origins differ");
    let draft_outline = artifacts
        .outline_px
        .map(|outline| shift_outline(outline, origin));
    let part = observation.part_key.and_then(|key| context.parts.get(key));
    let seed_outline = observation
        .learned_outline_px
        .or(draft_outline)
        .unwrap_or_else(|| seed_outline(part, CROP_SIZE_PX));
    let evidence_expected = expected_presence(&observation.evidence);
    let detected = artifacts.report.detected;
    let draft_state = match evidence_expected {
        Some(expected) if expected != detected => "evidence_conflict",
        Some(_) => "evidence_agreement",
        None if detected => "detected_unverified",
        None => "bare_unverified",
    };
    let split = if context.evaluation_groups.contains(&observation.group_id) {
        "eval"
    } else {
        "train"
    };
    let artifact_stem = observation
        .artifact_name
        .strip_suffix(".png")
        .unwrap_or(observation.artifact_name);
    let id = format!(
        "{}-{}-{artifact_stem}",
        observation.source_kind, observation.source_id
    );
    ensure!(
        !context.samples.iter().any(|sample| sample.id == id)
            && !context.carried_samples.iter().any(|sample| sample.id == id),
        "duplicate dataset sample {id}"
    );
    let encoded_bare = encode_gray_png(&bare_crop)?;
    let encoded_observed = encode_gray_png(&observed_crop)?;
    let bare_source_sha256 = hex_sha256(bare_bytes);
    let observed_source_sha256 = hex_sha256(observed_bytes);
    if let Some(reviewed) = context.reviewed {
        if let Some(carried) = carry_reviewed_sample(
            reviewed,
            &id,
            &bare_source_sha256,
            &observed_source_sha256,
            &encoded_bare,
            &encoded_observed,
        )? {
            write_carried_image(context.output, &carried.bare_image_path, &encoded_bare, &id)?;
            write_carried_image(
                context.output,
                &carried.observed_image_path,
                &encoded_observed,
                &id,
            )?;
            context.carried_samples.push(carried);
            return Ok(());
        }
    }
    let bare_path = format!("images/{split}/{id}-bare.png");
    let observed_path = format!("images/{split}/{id}-observed.png");
    fs::write(context.output.join(&bare_path), &encoded_bare)?;
    fs::write(context.output.join(&observed_path), &encoded_observed)?;
    let (draft_marker_status, draft_marker_end_px) = draft_marker(
        orientation,
        &artifacts.report,
        mapping,
        origin,
        seed_outline,
    );
    context.samples.push(DatasetSample {
        id,
        split: split.into(),
        group_id: observation.group_id,
        source_kind: observation.source_kind.into(),
        source_id: observation.source_id,
        source_artifact: observation.artifact_name.into(),
        observation_kind: observation.observation_kind.into(),
        bare_source_sha256,
        observed_source_sha256,
        bare_image_path: bare_path,
        observed_image_path: observed_path,
        bare_image_sha256: hex_sha256(&encoded_bare),
        observed_image_sha256: hex_sha256(&encoded_observed),
        width_px: bare_crop.width,
        height_px: bare_crop.height,
        nozzle_id: observation.nozzle_id.into(),
        part_key: observation.part_key.map(str::to_string),
        orientation,
        requested_nozzle_rotation_offset_deg: observation.requested_nozzle_rotation_offset_deg,
        actual_nozzle_rotation_offset_deg: observation.actual_nozzle_rotation_offset_deg,
        evidence: if observation.learned_outline_px.is_some() {
            [
                observation.evidence,
                vec!["learned_model_box_proposal".into()],
            ]
            .concat()
        } else {
            observation.evidence
        },
        draft_state: draft_state.into(),
        draft_outline_px: draft_outline,
        seed_outline_px: seed_outline,
        draft_marker_status,
        draft_marker_end_px,
        deterministic: artifacts.report,
    });
    Ok(())
}

fn load_reviewed_dataset(root: &Path) -> Result<ReviewedDataset> {
    let draft_path = root.join("manifest.draft.json");
    let ground_truth_path = root.join("manifest.ground-truth.json");
    let draft_bytes =
        fs::read(&draft_path).with_context(|| format!("cannot read {}", draft_path.display()))?;
    let ground_truth_bytes = fs::read(&ground_truth_path)
        .with_context(|| format!("cannot read {}", ground_truth_path.display()))?;
    let ground_truth: ReviewedGroundTruthManifest = serde_json::from_slice(&ground_truth_bytes)?;
    ensure!(
        ground_truth.classes == ["component_flat", "component_on_edge", "multiple_components"],
        "reviewed dataset uses different classes"
    );
    let sample_count = ground_truth.samples.len();
    let samples_by_id = ground_truth
        .samples
        .into_iter()
        .map(|sample| (sample.id.clone(), sample))
        .collect::<BTreeMap<_, _>>();
    ensure!(
        samples_by_id.len() == sample_count,
        "reviewed ground truth has duplicate sample IDs"
    );
    Ok(ReviewedDataset {
        root: root.to_path_buf(),
        draft_manifest_sha256: hex_sha256(&draft_bytes),
        ground_truth_manifest_sha256: hex_sha256(&ground_truth_bytes),
        samples_by_id,
    })
}

#[allow(clippy::too_many_arguments)]
fn carry_reviewed_sample(
    reviewed: &ReviewedDataset,
    id: &str,
    bare_source_sha256: &str,
    observed_source_sha256: &str,
    encoded_bare: &[u8],
    encoded_observed: &[u8],
) -> Result<Option<ReviewedGroundTruthSample>> {
    let Some(sample) = reviewed.samples_by_id.get(id) else {
        return Ok(None);
    };
    if sample.bare_source_sha256 != bare_source_sha256
        || sample.observed_source_sha256 != observed_source_sha256
        || sample.bare_image_sha256 != hex_sha256(encoded_bare)
        || sample.observed_image_sha256 != hex_sha256(encoded_observed)
    {
        return Ok(None);
    }
    for (path, expected) in [
        (&sample.bare_image_path, encoded_bare),
        (&sample.observed_image_path, encoded_observed),
    ] {
        let relative = safe_image_path(path, id)?;
        let prior = fs::read(reviewed.root.join(relative)).with_context(|| {
            format!(
                "cannot read reviewed image {}",
                reviewed.root.join(relative).display()
            )
        })?;
        if prior != expected {
            return Ok(None);
        }
    }
    Ok(Some(sample.clone()))
}

fn write_carried_image(output: &Path, path: &str, data: &[u8], id: &str) -> Result<()> {
    let relative = safe_image_path(path, id)?;
    let destination = output.join(relative);
    let parent = destination
        .parent()
        .context("carried image has no parent")?;
    fs::create_dir_all(parent)?;
    fs::write(destination, data)?;
    Ok(())
}

fn safe_image_path<'a>(path: &'a str, id: &str) -> Result<&'a Path> {
    let relative = Path::new(path);
    ensure!(
        !relative.is_absolute()
            && relative
                .components()
                .all(|component| matches!(component, std::path::Component::Normal(_))),
        "reviewed sample {id} has an unsafe image path"
    );
    Ok(relative)
}

fn observation_kind(name: &str) -> Option<&'static str> {
    if name == "held-part.png" {
        Some("held_initial")
    } else if name.starts_with("held-part-aligned") && name.ends_with(".png") {
        Some("held_aligned")
    } else if name.starts_with("rotation-capture-") && name.ends_with("-held-part.png") {
        Some("rotation_capture")
    } else if name.starts_with("post-release-nozzle")
        && !name.contains("inspection")
        && name.ends_with(".png")
    {
        Some("post_release")
    } else {
        None
    }
}

fn pickup_evidence(
    report: &StripFeederPickupQualificationReport,
    observation_kind: &str,
    artifact_name: &str,
) -> Vec<String> {
    let mut evidence = Vec::new();
    if observation_kind == "rotation_capture" {
        if let Some(capture) = report
            .rotation_captures
            .iter()
            .find(|capture| capture.held_artifact_name == artifact_name)
        {
            evidence.push(if capture.vacuum_held {
                "vacuum_reported_held"
            } else {
                "vacuum_reported_not_held"
            });
            if capture.inspection.detected {
                evidence.push("stored_vision_reported_component");
            }
        }
    } else if observation_kind.starts_with("held") {
        evidence.push(if report.vacuum_held_after_transfer {
            "vacuum_reported_held"
        } else {
            "vacuum_reported_not_held"
        });
        if report
            .inspection
            .as_ref()
            .is_some_and(|value| value.detected)
        {
            evidence.push("stored_vision_reported_component");
        }
    } else if let Some(verification) = &report.nozzle_release_verification {
        evidence.push(if verification.nozzle_clear {
            "release_reported_bare"
        } else {
            "release_reported_component_or_uncertain"
        });
        if let Some(index) = release_attempt_index(artifact_name) {
            if verification
                .observations
                .get(index.saturating_sub(1) as usize)
                .is_some_and(|value| value.detected)
            {
                evidence.push("stored_vision_reported_component");
            }
        }
    }
    evidence.into_iter().map(str::to_string).collect()
}

fn placement_evidence(
    report: &velocitypnp_domain::PlacementExecutionReport,
    observation_kind: &str,
    artifact_name: &str,
) -> Vec<String> {
    let mut evidence = vec![if report.passed {
        "placement_execution_passed".to_string()
    } else {
        "placement_execution_failed".to_string()
    }];
    if let Some(pickup) = &report.pickup_report {
        evidence.extend(pickup_evidence(pickup, observation_kind, artifact_name));
        return evidence;
    }
    if observation_kind.starts_with("held") {
        let attempt = report.pickup_attempts.last();
        if let Some(value) = attempt.and_then(|value| value.vacuum_held_after_transfer) {
            evidence.push(if value {
                "vacuum_reported_held".into()
            } else {
                "vacuum_reported_not_held".into()
            });
        }
        if attempt
            .and_then(|value| value.held_part_inspection.as_ref())
            .is_some_and(|value| value.detected)
        {
            evidence.push("stored_vision_reported_component".into());
        }
    } else if let Some(index) = release_attempt_index(artifact_name) {
        if let Some(attempt) = report
            .placement_release_attempts
            .iter()
            .find(|attempt| attempt.attempt_index == index)
        {
            evidence.push(if attempt.nozzle_clear {
                "release_reported_bare".into()
            } else if attempt.retained_part_detected {
                "release_reported_component".into()
            } else {
                "release_reported_uncertain".into()
            });
        }
    }
    evidence
}

fn expected_presence(evidence: &[String]) -> Option<bool> {
    let positive = evidence
        .iter()
        .any(|value| value == "vacuum_reported_held" || value == "release_reported_component");
    let negative = evidence
        .iter()
        .any(|value| value == "release_reported_bare");
    match (positive, negative) {
        (true, false) => Some(true),
        (false, true) => Some(false),
        _ => None,
    }
}

fn draft_state_priority(state: &str) -> u8 {
    match state {
        "evidence_conflict" => 0,
        "detected_unverified" | "bare_unverified" => 1,
        "evidence_agreement" => 2,
        _ => 3,
    }
}

fn release_attempt_index(name: &str) -> Option<u32> {
    if name == "post-release-nozzle.png" {
        return Some(1);
    }
    name.strip_prefix("post-release-nozzle-attempt-")?
        .strip_suffix(".png")?
        .parse()
        .ok()
}

fn load_bottom_camera_mapping(store: &Store, calibration_id: Uuid) -> Result<[[f64; 2]; 2]> {
    let snapshot = store
        .get_calibration(calibration_id)?
        .with_context(|| format!("calibration {calibration_id} is absent"))?;
    let artifact = snapshot
        .artifacts
        .get(&CalibrationArtifactId::BottomCameraLocalMap)
        .context("bottom-camera local map is absent")?;
    let value = artifact
        .value
        .get("result")
        .cloned()
        .unwrap_or_else(|| artifact.value.clone());
    let report: BottomCameraMappingReport = serde_json::from_value(value)?;
    Ok(report.pixel_error_to_machine_mm)
}

fn centered_crop(frame: &GrayFrame, center: [f64; 2], size: u32) -> Result<(GrayFrame, [u32; 2])> {
    ensure!(
        size > 0 && size <= frame.width && size <= frame.height,
        "held-part crop does not fit the source frame"
    );
    let half = f64::from(size) * 0.5;
    let left = (center[0] - half).round() as i64;
    let top = (center[1] - half).round() as i64;
    ensure!(
        left >= 0
            && top >= 0
            && left + i64::from(size) <= i64::from(frame.width)
            && top + i64::from(size) <= i64::from(frame.height),
        "held-part crop is outside the source frame"
    );
    let left = left as u32;
    let top = top as u32;
    let mut pixels = Vec::with_capacity((size * size) as usize);
    for y in top..top + size {
        let start = (y * frame.width + left) as usize;
        pixels.extend_from_slice(&frame.pixels[start..start + size as usize]);
    }
    Ok((
        GrayFrame {
            width: size,
            height: size,
            pixels,
        },
        [left, top],
    ))
}

fn shift_outline(outline: [[f64; 2]; 4], origin: [u32; 2]) -> [[f64; 2]; 4] {
    outline.map(|point| {
        [
            point[0] - f64::from(origin[0]),
            point[1] - f64::from(origin[1]),
        ]
    })
}

fn seed_outline(part: Option<&Part>, size: u32) -> [[f64; 2]; 4] {
    let scale_px_per_mm = 32.0;
    let width = part
        .and_then(|part| part.body_length_mm)
        .map_or(54.0, |value| value * scale_px_per_mm)
        .clamp(12.0, 0.75 * f64::from(size));
    let height = part
        .and_then(|part| part.body_width_mm)
        .map_or(28.0, |value| value * scale_px_per_mm)
        .clamp(8.0, 0.75 * f64::from(size));
    let center = f64::from(size) * 0.5;
    [
        [center - width * 0.5, center - height * 0.5],
        [center + width * 0.5, center - height * 0.5],
        [center + width * 0.5, center + height * 0.5],
        [center - width * 0.5, center + height * 0.5],
    ]
}

fn draft_marker(
    orientation: PartOrientation,
    report: &HeldPartInspectionReport,
    mapping: [[f64; 2]; 2],
    origin: [u32; 2],
    outline: [[f64; 2]; 4],
) -> (String, Option<[f64; 2]>) {
    if !orientation.is_directed() || !report.detected {
        return ("not_applicable".into(), None);
    }
    let Some(rotation_deg) = report.orientation_rotation_deg else {
        return ("uncertain".into(), None);
    };
    let Some(center_x) = report.center_x_px else {
        return ("uncertain".into(), None);
    };
    let Some(center_y) = report.center_y_px else {
        return ("uncertain".into(), None);
    };
    let machine_axis = [
        rotation_deg.to_radians().cos(),
        rotation_deg.to_radians().sin(),
    ];
    let Some(pixel_axis) = invert_map_vector(mapping, machine_axis) else {
        return ("uncertain".into(), None);
    };
    let extent = outline
        .iter()
        .map(|point| {
            let dx = point[0] + f64::from(origin[0]) - center_x;
            let dy = point[1] + f64::from(origin[1]) - center_y;
            (dx * pixel_axis[0] + dy * pixel_axis[1]).abs()
        })
        .fold(0.0_f64, f64::max);
    (
        "visible_draft".into(),
        Some([
            center_x - f64::from(origin[0]) + extent * pixel_axis[0],
            center_y - f64::from(origin[1]) + extent * pixel_axis[1],
        ]),
    )
}

fn invert_map_vector(mapping: [[f64; 2]; 2], vector: [f64; 2]) -> Option<[f64; 2]> {
    let determinant = mapping[0][0] * mapping[1][1] - mapping[0][1] * mapping[1][0];
    if !determinant.is_finite() || determinant.abs() < 1e-12 {
        return None;
    }
    let pixel = [
        (mapping[1][1] * vector[0] - mapping[0][1] * vector[1]) / determinant,
        (-mapping[1][0] * vector[0] + mapping[0][0] * vector[1]) / determinant,
    ];
    let length = pixel[0].hypot(pixel[1]);
    (length > 1e-9).then(|| [pixel[0] / length, pixel[1] / length])
}

fn hex_sha256(data: &[u8]) -> String {
    Sha256::digest(data)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn review_html(manifest: &DatasetManifest, manifest_json: &str) -> String {
    let template = include_str!("held_part_review.html");
    let title = serde_json::to_string(&manifest.name).expect("dataset title serializes");
    template
        .replace("__MANIFEST__", manifest_json)
        .replace("__DATASET_TITLE__", &title)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn artifact_names_select_only_raw_observations() {
        assert_eq!(observation_kind("held-part.png"), Some("held_initial"));
        assert_eq!(
            observation_kind("held-part-aligned-release-attempt-1.png"),
            Some("held_aligned")
        );
        assert_eq!(
            observation_kind("post-release-nozzle-attempt-2.png"),
            Some("post_release")
        );
        assert_eq!(observation_kind("held-part-inspection.png"), None);
        assert_eq!(observation_kind("source-pocket-before.png"), None);
    }

    #[test]
    fn crop_and_outline_use_the_same_origin() {
        let frame = GrayFrame {
            width: 20,
            height: 16,
            pixels: (0..320).map(|value| value as u8).collect(),
        };
        let (crop, origin) = centered_crop(&frame, [10.0, 8.0], 8).unwrap();
        assert_eq!(origin, [6, 4]);
        assert_eq!([crop.width, crop.height], [8, 8]);
        assert_eq!(crop.pixels[0], frame.pixels[(4 * 20 + 6) as usize]);
        assert_eq!(
            shift_outline([[6.0, 4.0], [8.0, 4.0], [8.0, 6.0], [6.0, 6.0]], origin),
            [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]
        );
    }

    #[test]
    fn conflicting_evidence_has_no_expected_state() {
        assert_eq!(
            expected_presence(&[
                "vacuum_reported_held".into(),
                "release_reported_bare".into()
            ]),
            None
        );
    }

    #[test]
    fn evidence_conflicts_sort_before_automatic_agreements() {
        assert!(
            draft_state_priority("evidence_conflict") < draft_state_priority("evidence_agreement")
        );
    }

    #[test]
    fn carried_images_require_relative_normal_paths() {
        assert_eq!(
            safe_image_path("images/train/pair.png", "pair").unwrap(),
            Path::new("images/train/pair.png")
        );
        assert!(safe_image_path("../pair.png", "pair").is_err());
        assert!(safe_image_path("/tmp/pair.png", "pair").is_err());
    }
}
