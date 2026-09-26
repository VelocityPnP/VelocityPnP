use anyhow::{ensure, Context, Result};
use chrono::Utc;
use serde::{Deserialize, Serialize};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use uuid::Uuid;
use velocitypnp_domain::{
    BottomCameraMappingReport, CalibrationArtifactId, CameraRole, OperationStatus,
    PlacementExecutionReport, PlanarCameraCalibrationReport, PlanarCameraModel,
    PlanarCameraModelKind, Pose, StripFeederInventoryLane, StripFeederInventoryReport,
    StripFeederRegistration, StripPocketOccupancy,
};
use velocitypnp_store::Store;
use velocitypnp_vision::{
    decode_gray_png, draft_strip_feeder_source_component, encode_gray_png,
    StripFeederSourceComponentDraft, STRIP_FEEDER_SOURCE_COMPONENT_DRAFT_ALGORITHM,
};

const SOURCE_ARTIFACT: &str = "source-pocket-before.png";
const MAX_ARCHIVED_FEEDER_MATCH_DISTANCE_MM: f64 = 2.0;

#[derive(Debug, Serialize)]
struct DatasetManifest {
    schema_version: u32,
    name: String,
    generated_at: String,
    detector_algorithm: String,
    source_database: String,
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
    source_sha256: String,
    image_path: String,
    width_px: u32,
    height_px: u32,
    part_key: Option<String>,
    pocket_index: Option<u32>,
    evidence: Vec<String>,
    draft_state: String,
    draft: StripFeederSourceComponentDraft,
}

struct SourceSample {
    source_kind: &'static str,
    source_id: Uuid,
    source_artifact: String,
    registration: StripFeederRegistration,
    calibration_id: Uuid,
    camera_pose: Pose,
    pocket_machine_mm: [f64; 2],
    pocket_index: Option<u32>,
    part_key: Option<String>,
    evidence: Vec<String>,
    image: Vec<u8>,
}

#[derive(Debug, Deserialize)]
struct ReviewedDraftManifest {
    samples: Vec<ReviewedDraftSample>,
}

#[derive(Debug, Deserialize)]
struct ReviewedDraftSample {
    id: String,
    source_sha256: String,
    width_px: u32,
    height_px: u32,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
struct ReviewedGroundTruthSample {
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
struct ReviewedGroundTruthManifest {
    classes: Vec<String>,
    samples: Vec<ReviewedGroundTruthSample>,
}

struct ReviewedDataset {
    root: PathBuf,
    draft_manifest_sha256: String,
    ground_truth_manifest_sha256: String,
    draft_by_id: BTreeMap<String, ReviewedDraftSample>,
    ground_truth_by_id: BTreeMap<String, ReviewedGroundTruthSample>,
}

#[derive(Debug, Deserialize, Serialize)]
struct CarriedGroundTruthManifest {
    schema_version: u32,
    source_dataset: String,
    source_draft_manifest_sha256: String,
    source_ground_truth_manifest_sha256: String,
    classes: Vec<String>,
    samples: Vec<CarriedGroundTruthSample>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct CarriedGroundTruthSample {
    #[serde(flatten)]
    ground_truth: ReviewedGroundTruthSample,
    width_px: u32,
    height_px: u32,
    source_sha256: String,
    image_sha256: String,
}

fn main() -> Result<()> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    let mut positional = Vec::new();
    let mut reviewed_root = None;
    let mut capture_run = None;
    let mut index = 0;
    while index < arguments.len() {
        if arguments[index] == "--carry-reviewed" {
            index += 1;
            let path = arguments
                .get(index)
                .context("--carry-reviewed requires a dataset directory")?;
            reviewed_root = Some(PathBuf::from(path));
        } else if arguments[index] == "--capture-run" {
            index += 1;
            let id = arguments
                .get(index)
                .context("--capture-run requires a source capture UUID")?;
            capture_run = Some(id.parse::<Uuid>()?);
        } else {
            positional.push(arguments[index].clone());
        }
        index += 1;
    }
    ensure!(
        (capture_run.is_some() && positional.len() == 2)
            || (capture_run.is_none() && positional.len() >= 3),
        "usage: export_source_component_dataset DATABASE OUTPUT_DIR \
         EVAL_REGISTRATION_UUID [EVAL_REGISTRATION_UUID ...] \
         [--carry-reviewed DATASET_DIR] | DATABASE OUTPUT_DIR \
         --capture-run CAPTURE_UUID"
    );
    ensure!(
        capture_run.is_none() || reviewed_root.is_none(),
        "a single-capture holdout cannot carry prior training labels"
    );
    let database = PathBuf::from(&positional[0]);
    let output = PathBuf::from(&positional[1]);
    let evaluation_groups = positional[2..]
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
    ensure!(
        !output.join("manifest.draft.json").exists(),
        "output already contains a draft manifest"
    );

    let store = Store::open(&database)?;
    let active_inventory = load_active_strip_feeder_inventory(&store)?;
    let registrations = store.list_strip_feeder_registrations()?;
    let by_id = registrations
        .iter()
        .map(|registration| (registration.id, registration.clone()))
        .collect::<BTreeMap<_, _>>();
    let by_feeder = registrations
        .iter()
        .filter_map(|registration| {
            registration
                .active_feeder_id
                .map(|feeder_id| (feeder_id, registration.clone()))
        })
        .collect::<BTreeMap<_, _>>();
    for group in &evaluation_groups {
        ensure!(
            by_id.contains_key(group),
            "evaluation registration {group} does not exist"
        );
    }

    let mut sources = Vec::new();
    for run in store.list_strip_feeder_pickup_qualifications()? {
        if capture_run.is_some() {
            break;
        }
        let Some((media_type, image)) =
            store.get_strip_feeder_pickup_artifact(run.id, SOURCE_ARTIFACT)?
        else {
            continue;
        };
        ensure!(media_type == "image/png", "source artifact is not a PNG");
        let Some(report) = run.report else {
            continue;
        };
        let registration = by_id
            .get(&run.registration_id)
            .with_context(|| format!("registration {} is absent", run.registration_id))?
            .clone();
        let camera_pose = report
            .return_verification
            .as_ref()
            .and_then(|return_report| return_report.before_camera_pose)
            .unwrap_or(Pose {
                x_mm: report.pocket_machine_mm[0],
                y_mm: report.pocket_machine_mm[1],
                z_mm: 31.5,
                rotation_deg: 0.0,
            });
        let mut evidence = Vec::new();
        if report.vacuum_acquired {
            evidence.push("vacuum_acquired".into());
        }
        if report
            .inspection
            .as_ref()
            .is_some_and(|inspection| inspection.detected && inspection.passed)
        {
            evidence.push("bottom_camera_confirmed_pickup".into());
        }
        if report
            .return_verification
            .as_ref()
            .is_some_and(|return_report| return_report.passed)
        {
            evidence.push("top_camera_confirmed_return".into());
        }
        sources.push(SourceSample {
            source_kind: "pickup_qualification",
            source_id: run.id,
            source_artifact: SOURCE_ARTIFACT.into(),
            registration,
            calibration_id: run.calibration_id,
            camera_pose,
            pocket_machine_mm: report.pocket_machine_mm,
            pocket_index: Some(run.pocket_index),
            part_key: by_id
                .get(&run.registration_id)
                .and_then(|registration| registration.part_key.clone()),
            evidence,
            image,
        });
    }
    for run in store.list_placement_executions()? {
        if capture_run.is_some() {
            break;
        }
        let Some(report) = run.report else {
            continue;
        };
        let Some((media_type, image)) =
            store.get_placement_execution_artifact(run.id, SOURCE_ARTIFACT)?
        else {
            continue;
        };
        ensure!(media_type == "image/png", "source artifact is not a PNG");
        let registration = placement_registration(&report, &by_id, &by_feeder, &registrations)?;
        let camera_pose = report
            .source_pose_inspection
            .as_ref()
            .map(|source_pose| source_pose.candidate_camera_pose)
            .unwrap_or(Pose {
                x_mm: report.feeder_pick.x_mm,
                y_mm: report.feeder_pick.y_mm,
                z_mm: 31.5,
                rotation_deg: report.feeder_pick.rotation_deg,
            });
        let pocket_machine_mm = report
            .source_pose_inspection
            .as_ref()
            .map(|source_pose| source_pose.candidate_nominal_center_machine_mm)
            .unwrap_or([report.feeder_pick.x_mm, report.feeder_pick.y_mm]);
        let mut evidence = Vec::new();
        if report.pickup_attempts.iter().any(|attempt| {
            attempt
                .held_part_inspection
                .as_ref()
                .is_some_and(|inspection| inspection.detected && inspection.passed)
        }) {
            evidence.push("bottom_camera_confirmed_pickup".into());
        }
        if report
            .pickup_report
            .as_ref()
            .and_then(|pickup| pickup.inspection.as_ref())
            .is_some_and(|inspection| inspection.detected && inspection.passed)
        {
            evidence.push("bottom_camera_confirmed_pickup".into());
        }
        evidence.sort();
        evidence.dedup();
        sources.push(SourceSample {
            source_kind: "placement_execution",
            source_id: run.id,
            source_artifact: SOURCE_ARTIFACT.into(),
            registration,
            calibration_id: run.calibration_id,
            camera_pose,
            pocket_machine_mm,
            pocket_index: report.pocket_index,
            part_key: Some(report.part_key),
            evidence,
            image,
        });
    }
    for run in store.list_strip_feeder_live_holdouts()? {
        if capture_run.is_some() {
            break;
        }
        let Some(report) = run.report else {
            continue;
        };
        ensure!(
            report.source_frame_artifact_names.len() == report.source_frame_camera_poses.len(),
            "live holdout {} has mismatched source-frame metadata",
            run.id,
        );
        if report.source_frame_artifact_names.is_empty() || report.sprocket_center_count < 3 {
            continue;
        }
        let registration = by_id
            .get(&run.registration_id)
            .with_context(|| format!("registration {} is absent", run.registration_id))?
            .clone();
        if active_inventory.as_ref().is_some_and(|inventory| {
            !inventory
                .lanes
                .iter()
                .any(|lane| inventory_lane_matches_registration(lane, &registration))
        }) {
            continue;
        }
        let source_index = nearest_source_frame_index(
            &report.source_frame_camera_poses,
            run.predicted_pocket_machine_mm,
        )
        .with_context(|| format!("live holdout {} has no source frames", run.id))?;
        let name = &report.source_frame_artifact_names[source_index];
        let camera_pose = report.source_frame_camera_poses[source_index];
        let (media_type, image) = store
            .get_strip_feeder_holdout_artifact(run.id, name)?
            .with_context(|| format!("live holdout {} is missing artifact {name}", run.id))?;
        ensure!(media_type == "image/png", "source artifact is not a PNG");
        let evidence = if report.occupancy == StripPocketOccupancy::Occupied && report.passed {
            vec!["live_holdout_reported_occupied".into()]
        } else {
            Vec::new()
        };
        sources.push(SourceSample {
            source_kind: "live_holdout",
            source_id: run.id,
            source_artifact: name.clone(),
            registration: registration.clone(),
            calibration_id: run.calibration_id,
            camera_pose,
            pocket_machine_mm: run.predicted_pocket_machine_mm,
            pocket_index: Some(run.pocket_index),
            part_key: registration.part_key.clone(),
            evidence,
            image,
        });
    }
    for run in store.list_strip_feeder_source_captures()? {
        if capture_run.is_some_and(|selected| run.id != selected) {
            continue;
        }
        if capture_run.is_some() {
            ensure!(
                run.status == OperationStatus::Succeeded,
                "selected source capture {} did not succeed",
                run.id
            );
        }
        for sample in run.samples {
            let registration = by_id
                .get(&sample.registration_id)
                .with_context(|| {
                    format!(
                        "source capture registration {} is absent",
                        sample.registration_id
                    )
                })?
                .clone();
            let (media_type, image) = store
                .get_strip_feeder_source_capture_artifact(run.id, &sample.artifact_name)?
                .with_context(|| {
                    format!(
                        "source capture {} is missing artifact {}",
                        run.id, sample.artifact_name
                    )
                })?;
            ensure!(media_type == "image/png", "source artifact is not a PNG");
            let evidence = vec![match sample.expected_occupancy {
                StripPocketOccupancy::Occupied => "inventory_expected_occupied".into(),
                StripPocketOccupancy::Empty => "inventory_expected_empty".into(),
                StripPocketOccupancy::Uncertain => "inventory_expected_uncertain".into(),
            }];
            sources.push(SourceSample {
                source_kind: "inventory_source_capture",
                source_id: run.id,
                source_artifact: sample.artifact_name,
                registration: registration.clone(),
                calibration_id: run.calibration_id,
                camera_pose: sample.camera_pose,
                pocket_machine_mm: sample.pocket_machine_mm,
                pocket_index: Some(sample.pocket_index),
                part_key: registration.part_key.clone(),
                evidence,
                image,
            });
        }
    }
    ensure!(!sources.is_empty(), "no source-pocket images were found");
    sources.sort_by_key(|source| (source.registration.id, source.source_id));

    fs::create_dir_all(&output)?;
    for split in ["train", "eval"] {
        fs::create_dir_all(output.join("images").join(split))?;
    }
    let mut models: BTreeMap<Uuid, (PlanarCameraModel, bool)> = BTreeMap::new();
    let mut samples = Vec::new();
    let mut carried_samples = Vec::new();
    for source in sources {
        let proposed_split =
            if capture_run.is_some() || evaluation_groups.contains(&source.registration.id) {
                "eval"
            } else {
                "train"
            };
        let artifact_stem = source
            .source_artifact
            .strip_suffix(".png")
            .unwrap_or(&source.source_artifact);
        let id = if source.source_artifact == SOURCE_ARTIFACT {
            format!("{}-{}", source.source_kind, source.source_id)
        } else {
            format!(
                "{}-{}-{artifact_stem}",
                source.source_kind, source.source_id
            )
        };
        let source_sha256 = hex_sha256(&source.image);
        let frame = decode_gray_png(&source.image)?;
        let (model, legacy_affine) = if let Some(model) = models.get(&source.calibration_id) {
            model.clone()
        } else {
            let model =
                load_top_camera_model(&store, source.calibration_id, frame.width, frame.height)?;
            models.insert(source.calibration_id, model.clone());
            model
        };
        if legacy_affine {
            if let Some(reviewed) = &reviewed {
                if let Some(carried) =
                    carry_frozen_reviewed_sample(reviewed, &id, &source_sha256, &output)?
                {
                    carried_samples.push(carried);
                    continue;
                }
            }
        }
        let artifacts = draft_strip_feeder_source_component(
            &frame,
            source.camera_pose,
            source.pocket_machine_mm,
            &model,
            &source.registration,
        )?;
        let encoded_crop = encode_gray_png(&artifacts.crop)?;
        let carried = if let Some(reviewed) = &reviewed {
            carry_reviewed_sample(reviewed, &id, &source_sha256, &encoded_crop)?
        } else {
            None
        };
        if let Some(carried) = carried {
            let image_path = &carried.ground_truth.image_path;
            let image_output = output.join(image_path);
            let parent = image_output
                .parent()
                .context("carried image path has no parent")?;
            fs::create_dir_all(parent)?;
            fs::write(image_output, &encoded_crop)?;
            carried_samples.push(carried);
            continue;
        }
        let image_path = format!("images/{proposed_split}/{id}.png");
        fs::write(output.join(&image_path), encoded_crop)?;
        let supported = source
            .evidence
            .iter()
            .any(|item| item == "bottom_camera_confirmed_pickup");
        let draft_state = match (
            artifacts.report.occupancy,
            artifacts.report.outline_px.is_some(),
            supported,
        ) {
            (StripPocketOccupancy::Occupied, true, true) => "positive_supported",
            (StripPocketOccupancy::Occupied, true, false) => "positive_auto",
            (StripPocketOccupancy::Empty, false, _) => "negative_auto",
            _ => "needs_review",
        };
        samples.push(DatasetSample {
            id,
            split: proposed_split.into(),
            group_id: capture_run.unwrap_or(source.registration.id),
            source_kind: source.source_kind.into(),
            source_id: source.source_id,
            source_artifact: source.source_artifact,
            source_sha256,
            image_path,
            width_px: artifacts.crop.width,
            height_px: artifacts.crop.height,
            part_key: source.part_key,
            pocket_index: source.pocket_index,
            evidence: source.evidence,
            draft_state: draft_state.into(),
            draft: artifacts.report,
        });
    }
    ensure!(
        capture_run.is_some()
            || samples.iter().any(|sample| sample.split == "train")
            || carried_samples
                .iter()
                .any(|sample| sample.ground_truth.split == "train"),
        "the train split is empty"
    );
    ensure!(
        samples.iter().any(|sample| sample.split == "eval")
            || carried_samples
                .iter()
                .any(|sample| sample.ground_truth.split == "eval"),
        "the evaluation split is empty"
    );
    samples.sort_by_key(|sample| {
        (
            sample.split != "eval",
            sample.draft_state.clone(),
            sample.group_id,
            sample.source_id,
        )
    });
    carried_samples.sort_by(|first, second| first.ground_truth.id.cmp(&second.ground_truth.id));
    let carried_manifest_name = reviewed
        .as_ref()
        .map(|_| "manifest.carried-ground-truth.json".to_string());
    if let Some(reviewed) = &reviewed {
        let carried = CarriedGroundTruthManifest {
            schema_version: 1,
            source_dataset: reviewed.root.display().to_string(),
            source_draft_manifest_sha256: reviewed.draft_manifest_sha256.clone(),
            source_ground_truth_manifest_sha256: reviewed.ground_truth_manifest_sha256.clone(),
            classes: vec!["component_flat".into(), "component_on_edge".into()],
            samples: carried_samples.clone(),
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
    let carried_count = carried_samples.len();
    let manifest = DatasetManifest {
        schema_version: 1,
        name: if capture_run.is_some() {
            "velocitypnp-source-components-capture-holdout-draft".into()
        } else if reviewed.is_some() {
            "velocitypnp-source-components-v2-draft".into()
        } else {
            "velocitypnp-source-components-v1-draft".into()
        },
        generated_at: Utc::now().to_rfc3339(),
        detector_algorithm: STRIP_FEEDER_SOURCE_COMPONENT_DRAFT_ALGORITHM.into(),
        source_database: database.display().to_string(),
        classes: vec!["component_flat".into(), "component_on_edge".into()],
        evaluation_registration_ids: evaluation_groups.into_iter().collect(),
        split_policy: if capture_run.is_some() {
            "all images from this new capture are evaluation-only".into()
        } else {
            "all images from one feeder registration stay in one split".into()
        },
        carried_ground_truth_manifest: carried_manifest_name,
        carried_sample_count: carried_count,
        notes: vec![
            "draft labels are weak supervision, not ground truth".into(),
            "reviewed labels are carried only when source and crop hashes match".into(),
            "reviewed legacy-calibration images keep their prior crop when the source hash matches"
                .into(),
            "new legacy-calibration images use the retained affine working-plane mapping".into(),
            "legacy placement reports without registration IDs use the nearest pocket geometry"
                .into(),
            "accept or correct every evaluation label before scoring a model".into(),
            "OBB rotation is axis-only and does not resolve 180-degree polarity".into(),
            "operator review assigns on-edge components to a separate class".into(),
            "model confidence is calibrated from accepted evaluation labels".into(),
        ],
        samples,
    };
    let manifest_json = serde_json::to_string_pretty(&manifest)?;
    fs::write(output.join("manifest.draft.json"), &manifest_json)?;
    fs::write(output.join("review.html"), review_html(&manifest)?)?;
    fs::write(
        output.join("README.txt"),
        format!(
            "Open review.html in a browser. Review only the new draft labels.\n\
             {carried_count} prior reviewed labels were carried without another review.\n\
             Use On side for a component that is standing on an edge.\n\
             Use Download decisions to save the review result.\n"
        ),
    )?;

    let train = manifest
        .samples
        .iter()
        .filter(|sample| sample.split == "train")
        .count();
    let evaluation = manifest.samples.len() - train;
    let supported = manifest
        .samples
        .iter()
        .filter(|sample| sample.draft_state == "positive_supported")
        .count();
    println!(
        "exported {} new draft samples and carried {} reviewed samples: {} new train, {} new eval, {} pickup-supported",
        manifest.samples.len(),
        carried_count,
        train,
        evaluation,
        supported,
    );
    println!("review {}", output.join("review.html").display());
    Ok(())
}

fn placement_registration(
    report: &PlacementExecutionReport,
    by_id: &BTreeMap<Uuid, StripFeederRegistration>,
    by_feeder: &BTreeMap<Uuid, StripFeederRegistration>,
    registrations: &[StripFeederRegistration],
) -> Result<StripFeederRegistration> {
    if let Some(registration_id) = report.feeder_registration_id {
        return by_id
            .get(&registration_id)
            .cloned()
            .with_context(|| format!("registration {registration_id} is absent"));
    }
    if let Some(registration) = by_feeder.get(&report.feeder_id) {
        return Ok(registration.clone());
    }
    let mut matches = registrations
        .iter()
        .filter(|registration| registration.part_key.as_deref() == Some(&report.part_key))
        .filter_map(|registration| {
            let reference = report
                .source_pose_inspection
                .as_ref()
                .map(|inspection| inspection.candidate_nominal_center_machine_mm)
                .unwrap_or([report.feeder_pick.x_mm, report.feeder_pick.y_mm]);
            let pocket_distance_mm = registration
                .pocket_centers_machine_mm
                .iter()
                .map(|center| (reference[0] - center[0]).hypot(reference[1] - center[1]))
                .min_by(f64::total_cmp);
            let [start, end] = registration.pick_centerline_machine_mm;
            let delta_x = end[0] - start[0];
            let delta_y = end[1] - start[1];
            let length = delta_x.hypot(delta_y);
            (length > f64::EPSILON).then(|| {
                let relative_x = reference[0] - start[0];
                let relative_y = reference[1] - start[1];
                let centerline_distance_mm =
                    (relative_x * delta_y - relative_y * delta_x).abs() / length;
                (
                    pocket_distance_mm
                        .map(|distance_mm| distance_mm.min(centerline_distance_mm))
                        .unwrap_or(centerline_distance_mm),
                    registration,
                )
            })
        })
        .collect::<Vec<_>>();
    matches.sort_by(|left, right| left.0.total_cmp(&right.0));
    let Some((distance_mm, registration)) = matches.first() else {
        anyhow::bail!(
            "archived feeder {} has no registration with matching part and pickup geometry",
            report.feeder_id
        );
    };
    ensure!(
        *distance_mm <= MAX_ARCHIVED_FEEDER_MATCH_DISTANCE_MM,
        "archived feeder {} is {:.3} mm from its nearest registration pickup geometry",
        report.feeder_id,
        distance_mm
    );
    Ok((*registration).clone())
}

fn load_reviewed_dataset(root: &Path) -> Result<ReviewedDataset> {
    let draft_path = root.join("manifest.draft.json");
    let ground_truth_path = root.join("manifest.ground-truth.json");
    let draft_bytes =
        fs::read(&draft_path).with_context(|| format!("cannot read {}", draft_path.display()))?;
    let ground_truth_bytes = fs::read(&ground_truth_path)
        .with_context(|| format!("cannot read {}", ground_truth_path.display()))?;
    let draft: ReviewedDraftManifest = serde_json::from_slice(&draft_bytes)?;
    let ground_truth: ReviewedGroundTruthManifest = serde_json::from_slice(&ground_truth_bytes)?;
    ensure!(
        ground_truth.classes == ["component_flat", "component_on_edge"],
        "reviewed dataset uses different classes"
    );
    let mut draft_by_id = draft
        .samples
        .into_iter()
        .map(|sample| (sample.id.clone(), sample))
        .collect::<BTreeMap<_, _>>();
    let carried_path = root.join("manifest.carried-ground-truth.json");
    if carried_path.is_file() {
        let carried: CarriedGroundTruthManifest = serde_json::from_slice(
            &fs::read(&carried_path)
                .with_context(|| format!("cannot read {}", carried_path.display()))?,
        )?;
        ensure!(
            carried.classes == ground_truth.classes,
            "reviewed carried labels use different classes"
        );
        for sample in carried.samples {
            let id = sample.ground_truth.id.clone();
            let prior = draft_by_id.insert(
                id.clone(),
                ReviewedDraftSample {
                    id: id.clone(),
                    source_sha256: sample.source_sha256,
                    width_px: sample.width_px,
                    height_px: sample.height_px,
                },
            );
            ensure!(prior.is_none(), "reviewed metadata repeats sample {id}");
        }
    }
    let ground_truth_by_id = ground_truth
        .samples
        .into_iter()
        .map(|sample| (sample.id.clone(), sample))
        .collect::<BTreeMap<_, _>>();
    ensure!(
        draft_by_id.len() == ground_truth_by_id.len(),
        "reviewed source metadata and ground truth have different sample counts"
    );
    ensure!(
        ground_truth_by_id
            .keys()
            .all(|id| draft_by_id.contains_key(id)),
        "reviewed ground truth contains an unknown draft sample"
    );
    Ok(ReviewedDataset {
        root: root.to_path_buf(),
        draft_manifest_sha256: hex_sha256(&draft_bytes),
        ground_truth_manifest_sha256: hex_sha256(&ground_truth_bytes),
        draft_by_id,
        ground_truth_by_id,
    })
}

fn carry_reviewed_sample(
    reviewed: &ReviewedDataset,
    id: &str,
    source_sha256: &str,
    encoded_crop: &[u8],
) -> Result<Option<CarriedGroundTruthSample>> {
    let Some(draft) = reviewed.draft_by_id.get(id) else {
        return Ok(None);
    };
    let ground_truth = reviewed
        .ground_truth_by_id
        .get(id)
        .with_context(|| format!("reviewed sample {id} has no ground truth"))?;
    if draft.source_sha256 != source_sha256 {
        return Ok(None);
    }
    let image_path = Path::new(&ground_truth.image_path);
    ensure!(
        !image_path.is_absolute()
            && image_path
                .components()
                .all(|component| matches!(component, std::path::Component::Normal(_))),
        "reviewed sample {id} has an unsafe image path"
    );
    let prior_image = fs::read(reviewed.root.join(image_path)).with_context(|| {
        format!(
            "cannot read reviewed image {}",
            reviewed.root.join(image_path).display()
        )
    })?;
    if prior_image != encoded_crop {
        return Ok(None);
    }
    Ok(Some(CarriedGroundTruthSample {
        ground_truth: ground_truth.clone(),
        width_px: draft.width_px,
        height_px: draft.height_px,
        source_sha256: source_sha256.into(),
        image_sha256: hex_sha256(encoded_crop),
    }))
}

fn carry_frozen_reviewed_sample(
    reviewed: &ReviewedDataset,
    id: &str,
    source_sha256: &str,
    output: &Path,
) -> Result<Option<CarriedGroundTruthSample>> {
    let Some(draft) = reviewed.draft_by_id.get(id) else {
        return Ok(None);
    };
    if draft.source_sha256 != source_sha256 {
        return Ok(None);
    }
    let ground_truth = reviewed
        .ground_truth_by_id
        .get(id)
        .with_context(|| format!("reviewed sample {id} has no ground truth"))?;
    let image_path = Path::new(&ground_truth.image_path);
    ensure!(
        !image_path.is_absolute()
            && image_path
                .components()
                .all(|component| matches!(component, std::path::Component::Normal(_))),
        "reviewed sample {id} has an unsafe image path"
    );
    let prior_image = fs::read(reviewed.root.join(image_path)).with_context(|| {
        format!(
            "cannot read reviewed image {}",
            reviewed.root.join(image_path).display()
        )
    })?;
    let frame = decode_gray_png(&prior_image)
        .with_context(|| format!("reviewed sample {id} is not a valid PNG"))?;
    ensure!(
        frame.width == draft.width_px && frame.height == draft.height_px,
        "reviewed sample {id} dimensions changed"
    );
    let destination = output.join(image_path);
    fs::create_dir_all(
        destination
            .parent()
            .context("carried image path has no parent")?,
    )?;
    fs::write(destination, &prior_image)?;
    Ok(Some(CarriedGroundTruthSample {
        ground_truth: ground_truth.clone(),
        width_px: draft.width_px,
        height_px: draft.height_px,
        source_sha256: source_sha256.into(),
        image_sha256: hex_sha256(&prior_image),
    }))
}

fn load_active_strip_feeder_inventory(store: &Store) -> Result<Option<StripFeederInventoryReport>> {
    let Some(baseline) = store
        .list_workspace_inventory_baselines()?
        .into_iter()
        .max_by_key(|baseline| baseline.activated_at)
    else {
        return Ok(None);
    };
    let scan = store
        .get_workspace_scan(baseline.scan_id)?
        .with_context(|| format!("baseline workspace scan {} is absent", baseline.scan_id))?;
    let Some(summary) = scan.strip_feeder_inventory else {
        return Ok(None);
    };
    let (media_type, data) = store
        .get_workspace_scan_artifact(scan.id, &summary.report_artifact_name)?
        .with_context(|| format!("workspace scan {} has no feeder inventory report", scan.id))?;
    ensure!(
        media_type == "application/json",
        "feeder inventory report is not JSON"
    );
    let report: StripFeederInventoryReport = serde_json::from_slice(&data)?;
    ensure!(
        report.workspace_scan_id == scan.id,
        "feeder inventory report belongs to another workspace scan"
    );
    Ok(Some(report))
}

fn inventory_lane_matches_registration(
    lane: &StripFeederInventoryLane,
    registration: &StripFeederRegistration,
) -> bool {
    let (Some(width_mm), Some(pitch_mm), Some(centerline)) = (
        lane.proposal.tape_width_mm,
        lane.proposal.component_pitch_mm,
        lane.proposal.pick_centerline_machine_mm,
    ) else {
        return false;
    };
    let angle_error = (lane.proposal.tape_rotation_deg - registration.tape_rotation_deg + 180.0)
        .rem_euclid(360.0)
        - 180.0;
    if (width_mm - registration.tape_width_mm).abs() > 0.25
        || (pitch_mm - registration.tape_pitch_mm).abs() > 0.25
        || angle_error.abs() > 2.0
    {
        return false;
    }
    let direction = [
        registration.pick_centerline_machine_mm[1][0]
            - registration.pick_centerline_machine_mm[0][0],
        registration.pick_centerline_machine_mm[1][1]
            - registration.pick_centerline_machine_mm[0][1],
    ];
    let length = direction[0].hypot(direction[1]);
    if length <= 1e-9 || !length.is_finite() {
        return false;
    }
    let midpoint = [
        0.5 * (centerline[0][0] + centerline[1][0]),
        0.5 * (centerline[0][1] + centerline[1][1]),
    ];
    let delta = [
        midpoint[0] - registration.pick_centerline_machine_mm[0][0],
        midpoint[1] - registration.pick_centerline_machine_mm[0][1],
    ];
    (delta[0] * direction[1] - delta[1] * direction[0]).abs() / length <= 1.5
}

fn load_top_camera_model(
    store: &Store,
    calibration_id: Uuid,
    frame_width: u32,
    frame_height: u32,
) -> Result<(PlanarCameraModel, bool)> {
    let snapshot = store
        .get_calibration(calibration_id)?
        .with_context(|| format!("calibration {calibration_id} is absent"))?;
    if let Some(artifact) = snapshot
        .artifacts
        .get(&CalibrationArtifactId::TopCameraPlanarModel)
    {
        let report: PlanarCameraCalibrationReport = serde_json::from_value(artifact.value.clone())?;
        ensure!(report.passed, "top-camera planar model did not pass");
        return Ok((report.model, false));
    }
    let artifact = snapshot
        .artifacts
        .get(&CalibrationArtifactId::TopCameraWorkingPlaneMap)
        .context("top-camera planar model and working-plane map are absent")?;
    let value = artifact
        .value
        .get("result")
        .cloned()
        .unwrap_or_else(|| artifact.value.clone());
    let report: BottomCameraMappingReport = serde_json::from_value(value)?;
    let center_x_px = f64::from(frame_width) * 0.5;
    let center_y_px = f64::from(frame_height) * 0.5;
    let normalization_x_px = center_x_px.max(1.0);
    let normalization_y_px = center_y_px.max(1.0);
    Ok((
        PlanarCameraModel {
            role: CameraRole::TopCamera,
            center_x_px,
            center_y_px,
            normalization_x_px,
            normalization_y_px,
            correction_x_coefficients_mm: [
                report.pixel_error_to_machine_mm[0][0] * normalization_x_px,
                report.pixel_error_to_machine_mm[0][1] * normalization_y_px,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            correction_y_coefficients_mm: [
                report.pixel_error_to_machine_mm[1][0] * normalization_x_px,
                report.pixel_error_to_machine_mm[1][1] * normalization_y_px,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            supported_min_x_px: 0.0,
            supported_max_x_px: f64::from(frame_width.saturating_sub(1)),
            supported_min_y_px: 0.0,
            supported_max_y_px: f64::from(frame_height.saturating_sub(1)),
            kind: PlanarCameraModelKind::Affine,
        },
        true,
    ))
}

fn hex_sha256(data: &[u8]) -> String {
    Sha256::digest(data)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn nearest_source_frame_index(camera_poses: &[Pose], pocket_machine_mm: [f64; 2]) -> Option<usize> {
    camera_poses
        .iter()
        .enumerate()
        .min_by(|(_, first), (_, second)| {
            let distance = |pose: &Pose| {
                (pose.x_mm - pocket_machine_mm[0]).hypot(pose.y_mm - pocket_machine_mm[1])
            };
            distance(first).total_cmp(&distance(second))
        })
        .map(|(index, _)| index)
}

fn review_html(manifest: &DatasetManifest) -> Result<String> {
    let data = serde_json::to_string(manifest)?;
    let title = json!(manifest.name).to_string();
    Ok(format!(
        r##"<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>VelocityPnP source component review</title>
<style>
body {{ margin: 0; background: #111; color: #eee; font: 16px sans-serif; }}
main {{ max-width: 960px; margin: auto; padding: 16px; }}
#frame {{ position: relative; width: min(92vw, 760px); margin: 12px auto; }}
#image {{ display: block; width: 100%; image-rendering: auto; }}
#overlay {{ position: absolute; inset: 0; width: 100%; height: 100%;
  touch-action: none; }}
#box {{ fill: rgba(0, 255, 128, .12); stroke: #00ff80; stroke-width: 2;
  cursor: move; }}
.handle {{ fill: #00d4ff; stroke: #111; stroke-width: 1; cursor: pointer; }}
#rotate-line {{ stroke: #00d4ff; stroke-width: 1; }}
.controls {{ display: flex; flex-wrap: wrap; gap: 8px; justify-content: center; }}
button, select {{ padding: 10px 14px; font: inherit; }}
button.selected {{ outline: 3px solid #00d4ff; }}
#meta {{ white-space: pre-wrap; background: #222; padding: 12px; }}
textarea {{ width: 100%; min-height: 55px; margin: 10px 0; }}
.warning {{ color: #ffcc66; }}
</style>
</head>
<body>
<main>
<h1>Source component draft review</h1>
<p class="warning">Green is the component box. Drag the box to move it, drag a
corner to resize it, or drag the round handle to rotate it.</p>
<p id="progress"></p>
<div id="frame"><img id="image"><svg id="overlay">
<line id="rotate-line"></line><polygon id="box"></polygon>
<g id="handles"></g></svg></div>
<pre id="meta"></pre>
<textarea id="note" placeholder="Optional correction note"></textarea>
<div class="controls">
<select id="class-name"><option value="component_flat">Flat component</option>
<option value="component_on_edge">Component on edge</option></select>
<button data-decision="accept">A: Accept state and box</button>
<button data-decision="empty">E: Empty</button>
<button data-decision="reject">R: Reject image</button>
<button id="reset">Reset proposal</button>
<button id="previous">Previous</button>
<button id="next">Next</button>
<button id="download">Download decisions</button>
</div>
</main>
<script>
const manifest = {data};
const key = {title} + ":review";
const saved = JSON.parse(localStorage.getItem(key) || "{{}}");
const requested = Number(new URLSearchParams(location.search).get("index") || 0);
let index = Math.max(0, Math.min(manifest.samples.length - 1, requested));
let drag = null, boxState = null;
const image = document.querySelector("#image");
const overlay = document.querySelector("#overlay");
const polygon = document.querySelector("#box");
const handles = document.querySelector("#handles");
const rotateLine = document.querySelector("#rotate-line");
const meta = document.querySelector("#meta");
const note = document.querySelector("#note");
const className = document.querySelector("#class-name");
function store() {{ localStorage.setItem(key, JSON.stringify(saved)); }}
function initialOutline(sample) {{
  if (sample.draft.outline_px) return sample.draft.outline_px;
  const center = sample.draft.center_px ||
    [sample.width_px / 2, sample.height_px / 2];
  return [[center[0] - 32, center[1] - 16],
    [center[0] + 32, center[1] - 16],
    [center[0] + 32, center[1] + 16],
    [center[0] - 32, center[1] + 16]];
}}
function fromOutline(points) {{
  const center = points.reduce((sum, point) =>
    [sum[0] + point[0] / 4, sum[1] + point[1] / 4], [0, 0]);
  return {{cx: center[0], cy: center[1],
    width: Math.hypot(points[1][0] - points[0][0],
      points[1][1] - points[0][1]),
    height: Math.hypot(points[2][0] - points[1][0],
      points[2][1] - points[1][1]),
    angle: Math.atan2(points[1][1] - points[0][1],
      points[1][0] - points[0][0])}};
}}
function points(state = boxState) {{
  const along = [Math.cos(state.angle), Math.sin(state.angle)];
  const across = [-along[1], along[0]];
  const halfWidth = state.width / 2, halfHeight = state.height / 2;
  return [[-halfWidth, -halfHeight], [halfWidth, -halfHeight],
    [halfWidth, halfHeight], [-halfWidth, halfHeight]].map(offset =>
      [state.cx + along[0] * offset[0] + across[0] * offset[1],
        state.cy + along[1] * offset[0] + across[1] * offset[1]]);
}}
function renderBox() {{
  const corners = points();
  polygon.setAttribute("points", corners.map(point => point.join(",")).join(" "));
  handles.innerHTML = "";
  corners.forEach((point, corner) => {{
    const handle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    handle.setAttribute("class", "handle"); handle.setAttribute("r", 5);
    handle.setAttribute("cx", point[0]); handle.setAttribute("cy", point[1]);
    handle.dataset.corner = corner; handles.appendChild(handle);
  }});
  const normal = [-Math.sin(boxState.angle), Math.cos(boxState.angle)];
  const top = [boxState.cx - normal[0] * boxState.height / 2,
    boxState.cy - normal[1] * boxState.height / 2];
  const rotate = [top[0] - normal[0] * 24, top[1] - normal[1] * 24];
  rotateLine.setAttribute("x1", top[0]); rotateLine.setAttribute("y1", top[1]);
  rotateLine.setAttribute("x2", rotate[0]); rotateLine.setAttribute("y2", rotate[1]);
  const handle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
  handle.setAttribute("class", "handle"); handle.setAttribute("r", 6);
  handle.setAttribute("cx", rotate[0]); handle.setAttribute("cy", rotate[1]);
  handle.dataset.rotate = "true"; handles.appendChild(handle);
}}
function current() {{
  const sample = manifest.samples[index];
  const result = saved[sample.id] || {{decision: "unreviewed", note: ""}};
  return {{...result,
    class_name: result.class_name ||
      (result.decision === "on_edge" ? "component_on_edge" : "component_flat"),
    outline_px: result.outline_px || initialOutline(sample)}};
}}
function show() {{
  const sample = manifest.samples[index];
  const decision = current();
  image.src = sample.image_path;
  overlay.setAttribute("viewBox", `0 0 ${{sample.width_px}} ${{sample.height_px}}`);
  boxState = fromOutline(decision.outline_px);
  renderBox();
  note.value = decision.note || "";
  className.value = decision.class_name;
  document.querySelectorAll("[data-decision]").forEach(button =>
    button.classList.toggle("selected", button.dataset.decision === decision.decision));
  const reviewed = Object.values(saved).filter(value => value.decision !== "unreviewed").length;
  document.querySelector("#progress").textContent =
    `${{index + 1}} / ${{manifest.samples.length}}; ${{reviewed}} reviewed`;
  meta.textContent = JSON.stringify({{
    id: sample.id,
    split: sample.split,
    group_id: sample.group_id,
    part_key: sample.part_key,
    pocket_index: sample.pocket_index,
    evidence: sample.evidence,
    draft_state: sample.draft_state,
    occupancy: sample.draft.occupancy,
    confidence: sample.draft.occupancy_confidence,
    measured_size_mm: sample.draft.measured_size_mm
  }}, null, 2);
}}
function persist(decision = current().decision) {{
  const sample = manifest.samples[index];
  saved[sample.id] = {{decision, note: note.value,
    class_name: className.value, outline_px: points()}};
  store();
}}
function decide(value) {{
  persist(value);
  if (index + 1 < manifest.samples.length) index++;
  show();
}}
function eventPoint(event) {{
  const point = overlay.createSVGPoint(); point.x = event.clientX;
  point.y = event.clientY;
  const value = point.matrixTransform(overlay.getScreenCTM().inverse());
  return [value.x, value.y];
}}
overlay.onpointerdown = event => {{
  const point = eventPoint(event), corner = event.target.dataset.corner;
  drag = {{start: point, initial: {{...boxState}},
    mode: event.target.dataset.rotate ? "rotate" :
      corner !== undefined ? "corner" : event.target === polygon ? "move" : "none",
    corner: Number(corner)}};
  overlay.setPointerCapture(event.pointerId);
}};
overlay.onpointermove = event => {{
  if (!drag || drag.mode === "none") return;
  const point = eventPoint(event), dx = point[0] - drag.start[0];
  const dy = point[1] - drag.start[1];
  if (drag.mode === "move") {{
    boxState.cx = drag.initial.cx + dx; boxState.cy = drag.initial.cy + dy;
  }} else if (drag.mode === "rotate") {{
    boxState.angle = Math.atan2(point[1] - boxState.cy,
      point[0] - boxState.cx) + Math.PI / 2;
  }} else {{
    const old = points(drag.initial), opposite = old[(drag.corner + 2) % 4];
    const along = [Math.cos(drag.initial.angle), Math.sin(drag.initial.angle)];
    const across = [-along[1], along[0]];
    const delta = [point[0] - opposite[0], point[1] - opposite[1]];
    boxState.width = Math.max(4,
      Math.abs(delta[0] * along[0] + delta[1] * along[1]));
    boxState.height = Math.max(4,
      Math.abs(delta[0] * across[0] + delta[1] * across[1]));
    boxState.cx = (point[0] + opposite[0]) / 2;
    boxState.cy = (point[1] + opposite[1]) / 2;
  }}
  renderBox();
}};
overlay.onpointerup = () => {{ if (drag) persist(); drag = null; }};
document.querySelectorAll("[data-decision]").forEach(button =>
  button.addEventListener("click", () => decide(button.dataset.decision)));
document.querySelector("#reset").onclick = () => {{
  boxState = fromOutline(initialOutline(manifest.samples[index])); renderBox(); persist();
}};
document.querySelector("#previous").onclick = () => {{
  persist(); index = Math.max(0, index - 1); show();
}};
document.querySelector("#next").onclick = () => {{
  persist(); index = Math.min(manifest.samples.length - 1, index + 1); show();
}};
note.onchange = () => persist();
className.onchange = () => persist();
document.querySelector("#download").onclick = () => {{
  persist();
  const result = {{schema_version: 1, dataset: manifest.name,
    exported_at: new Date().toISOString(), decisions: saved}};
  const link = document.createElement("a");
  link.href = URL.createObjectURL(new Blob([JSON.stringify(result, null, 2)],
    {{type: "application/json"}}));
  link.download = "review-decisions.json";
  link.click();
}};
document.addEventListener("keydown", event => {{
  if (event.target === note || event.target === className) return;
  const action = {{a: "accept", e: "empty", r: "reject"}}[event.key.toLowerCase()];
  if (action) decide(action);
  if (event.key === "ArrowLeft") {{
    persist(); index = Math.max(0, index - 1); show();
  }}
  if (event.key === "ArrowRight") {{
    persist(); index = Math.min(manifest.samples.length - 1, index + 1); show();
  }}
}});
show();
</script>
</body>
</html>
"##
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn source_export_selects_the_pose_nearest_the_requested_pocket() {
        let pose = |x_mm, y_mm| Pose {
            x_mm,
            y_mm,
            z_mm: 31.5,
            rotation_deg: 0.0,
        };
        let poses = [
            pose(96.5, 200.0),
            pose(96.5, 208.0),
            pose(100.0, 204.0),
            pose(103.5, 208.0),
            pose(103.5, 200.0),
        ];
        assert_eq!(nearest_source_frame_index(&poses, [100.0, 204.0]), Some(2));
    }

    #[test]
    fn reviewed_label_carries_only_when_source_and_crop_match() {
        let root =
            std::env::temp_dir().join(format!("velocitypnp-reviewed-carry-{}", Uuid::new_v4()));
        fs::create_dir_all(root.join("images/eval")).unwrap();
        let crop = b"unchanged crop";
        fs::write(root.join("images/eval/prior.png"), crop).unwrap();
        let id = "prior".to_string();
        let reviewed = ReviewedDataset {
            root: root.clone(),
            draft_manifest_sha256: "draft".into(),
            ground_truth_manifest_sha256: "ground-truth".into(),
            draft_by_id: BTreeMap::from([(
                id.clone(),
                ReviewedDraftSample {
                    id: id.clone(),
                    source_sha256: "source".into(),
                    width_px: 20,
                    height_px: 10,
                },
            )]),
            ground_truth_by_id: BTreeMap::from([(
                id.clone(),
                ReviewedGroundTruthSample {
                    id: id.clone(),
                    split: "eval".into(),
                    image_path: "images/eval/prior.png".into(),
                    part_key: Some("R0603".into()),
                    included: true,
                    class_name: Some("component_flat".into()),
                    outline_px: Some([[2.0, 2.0], [18.0, 2.0], [18.0, 8.0], [2.0, 8.0]]),
                    label_source: "second_review".into(),
                    review_note: String::new(),
                },
            )]),
        };

        assert!(carry_reviewed_sample(&reviewed, &id, "source", crop)
            .unwrap()
            .is_some());
        assert!(carry_reviewed_sample(&reviewed, &id, "changed", crop)
            .unwrap()
            .is_none());
        assert!(
            carry_reviewed_sample(&reviewed, &id, "source", b"changed crop")
                .unwrap()
                .is_none()
        );

        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn reviewed_dataset_loads_metadata_carried_by_a_prior_session() {
        let root =
            std::env::temp_dir().join(format!("velocitypnp-reviewed-chain-{}", Uuid::new_v4()));
        fs::create_dir_all(&root).unwrap();
        fs::write(
            root.join("manifest.draft.json"),
            serde_json::to_vec(&json!({
                "samples": [{
                    "id": "new",
                    "source_sha256": "new-source",
                    "width_px": 20,
                    "height_px": 10
                }]
            }))
            .unwrap(),
        )
        .unwrap();
        let ground_truth_sample = |id: &str| {
            json!({
                "id": id,
                "split": "train",
                "image_path": format!("images/train/{id}.png"),
                "part_key": null,
                "included": true,
                "class_name": null,
                "outline_px": null,
                "label_source": "second_review",
                "review_note": ""
            })
        };
        fs::write(
            root.join("manifest.ground-truth.json"),
            serde_json::to_vec(&json!({
                "classes": ["component_flat", "component_on_edge"],
                "samples": [ground_truth_sample("new"), ground_truth_sample("old")]
            }))
            .unwrap(),
        )
        .unwrap();
        fs::write(
            root.join("manifest.carried-ground-truth.json"),
            serde_json::to_vec(&json!({
                "schema_version": 1,
                "source_dataset": "v1",
                "source_draft_manifest_sha256": "draft",
                "source_ground_truth_manifest_sha256": "ground-truth",
                "classes": ["component_flat", "component_on_edge"],
                "samples": [{
                    "id": "old",
                    "split": "train",
                    "image_path": "images/train/old.png",
                    "part_key": null,
                    "included": true,
                    "class_name": null,
                    "outline_px": null,
                    "label_source": "second_review",
                    "review_note": "",
                    "width_px": 20,
                    "height_px": 10,
                    "source_sha256": "old-source",
                    "image_sha256": "old-image"
                }]
            }))
            .unwrap(),
        )
        .unwrap();

        let reviewed = load_reviewed_dataset(&root).unwrap();
        assert_eq!(reviewed.draft_by_id.len(), 2);
        assert_eq!(reviewed.draft_by_id["old"].source_sha256, "old-source");

        fs::remove_dir_all(root).unwrap();
    }
}
