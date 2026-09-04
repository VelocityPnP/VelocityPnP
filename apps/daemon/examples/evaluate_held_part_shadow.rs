use anyhow::{ensure, Context, Result};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use velocitypnp_domain::{HeldPartClass, HeldPartInspectionReport, HeldPartModelPrediction};
use velocitypnp_model::{HeldPartModel, HeldPartModelStatus};
use velocitypnp_vision::{decode_gray_png, encode_rgb_png, resize_gray_frame, GrayFrame, RgbFrame};

const TILE_SIZE: u32 = 192;
const PANEL_GAP: u32 = 4;
const SAMPLE_GAP: u32 = 8;
const SHEET_COLUMNS: u32 = 3;

#[derive(Debug, Deserialize)]
struct DatasetManifest {
    samples: Vec<DatasetSample>,
}

#[derive(Debug, Deserialize)]
struct DatasetSample {
    id: String,
    split: String,
    bare_image_path: String,
    observed_image_path: String,
    width_px: u32,
    height_px: u32,
    deterministic: HeldPartInspectionReport,
    included: bool,
    class_name: Option<String>,
    outline_px: Option<[[f64; 2]; 4]>,
}

#[derive(Debug, Deserialize)]
struct DraftManifest {
    samples: Vec<DraftSample>,
}

#[derive(Debug, Deserialize)]
struct DraftSample {
    id: String,
    draft_outline_px: Option<[[f64; 2]; 4]>,
}

#[derive(Debug, Serialize)]
struct ComparisonReport {
    schema_version: u32,
    model: HeldPartModelStatus,
    sample_count: usize,
    split_counts: BTreeMap<String, usize>,
    learned_state_accuracy: f64,
    learned_occupancy_accuracy: f64,
    deterministic_occupancy_accuracy: f64,
    learned_deterministic_occupancy_agreement: f64,
    learned_confidence_gate_pass_rate: f64,
    learned_mean_center_error_px: Option<f64>,
    deterministic_mean_center_error_px: Option<f64>,
    legend: BTreeMap<String, String>,
    sheet_columns: u32,
    records: Vec<ComparisonRecord>,
}

#[derive(Debug, Serialize)]
struct ComparisonRecord {
    sheet_index: usize,
    id: String,
    split: String,
    ground_truth_class: HeldPartClass,
    ground_truth_outline_px: Option<[[f64; 2]; 4]>,
    deterministic_detected: bool,
    deterministic_passed: bool,
    deterministic_confidence: f64,
    deterministic_outline_px: Option<[[f64; 2]; 4]>,
    deterministic_occupancy_correct: bool,
    deterministic_center_error_px: Option<f64>,
    learned: HeldPartModelPrediction,
    learned_state_correct: bool,
    learned_occupancy_correct: bool,
    learned_deterministic_occupancy_agreement: bool,
    learned_center_error_px: Option<f64>,
}

struct RenderedSample {
    left: RgbFrame,
    right: RgbFrame,
}

fn main() -> Result<()> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    ensure!(
        arguments.len() == 4,
        "usage: evaluate_held_part_shadow DATASET_DIR MODEL_DIR OUTPUT_PNG \
         OUTPUT_JSON"
    );
    let dataset_root = PathBuf::from(&arguments[0]).canonicalize()?;
    let model_root = PathBuf::from(&arguments[1]);
    let output_png = PathBuf::from(&arguments[2]);
    let output_json = PathBuf::from(&arguments[3]);
    let manifest: DatasetManifest = read_json(&dataset_root.join("manifest.ground-truth.json"))?;
    let draft: DraftManifest = read_json(&dataset_root.join("manifest.draft.json"))?;
    let draft_outlines = draft
        .samples
        .into_iter()
        .map(|sample| (sample.id, sample.draft_outline_px))
        .collect::<BTreeMap<_, _>>();
    let model = HeldPartModel::load(&model_root)?;
    let model_status = model.status();

    let mut records = Vec::new();
    let mut rendered = Vec::new();
    let mut split_counts = BTreeMap::new();
    for sample in manifest
        .samples
        .into_iter()
        .filter(|sample| sample.included)
    {
        *split_counts.entry(sample.split.clone()).or_insert(0) += 1;
        ensure!(
            sample.width_px == 512 && sample.height_px == 512,
            "sample {} does not use a 512-pixel crop",
            sample.id
        );
        let bare = read_gray(&dataset_root.join(&sample.bare_image_path))?;
        let observed = read_gray(&dataset_root.join(&sample.observed_image_path))?;
        let learned = model.predict_gray_pair(
            observed.width,
            observed.height,
            &bare.pixels,
            &observed.pixels,
        )?;
        let ground_truth_class = parse_class(sample.class_name.as_deref())?;
        let ground_truth_occupied = ground_truth_class.occupied();
        let deterministic_occupied = sample.deterministic.detected;
        let deterministic_outline_px = draft_outlines
            .get(&sample.id)
            .copied()
            .flatten()
            .filter(|_| deterministic_occupied);
        let deterministic_center_error_px =
            outline_centers(deterministic_outline_px, sample.outline_px);
        let learned_center_error_px = sample.outline_px.map(|outline| {
            let center = outline_center(outline);
            (center[0] - learned.pose.center_px[0]).hypot(center[1] - learned.pose.center_px[1])
        });
        let learned_state_correct = learned.class == ground_truth_class;
        let learned_occupancy_correct = learned.class.occupied() == ground_truth_occupied;
        let deterministic_occupancy_correct = deterministic_occupied == ground_truth_occupied;
        let learned_deterministic_occupancy_agreement =
            learned.class.occupied() == deterministic_occupied;
        let sheet_index = records.len();
        rendered.push(render_sample(
            &observed,
            sample.outline_px,
            deterministic_outline_px,
            &learned,
            deterministic_occupancy_correct,
            learned_state_correct,
        )?);
        records.push(ComparisonRecord {
            sheet_index,
            id: sample.id,
            split: sample.split,
            ground_truth_class,
            ground_truth_outline_px: sample.outline_px,
            deterministic_detected: sample.deterministic.detected,
            deterministic_passed: sample.deterministic.passed,
            deterministic_confidence: sample.deterministic.confidence,
            deterministic_outline_px,
            deterministic_occupancy_correct,
            deterministic_center_error_px,
            learned,
            learned_state_correct,
            learned_occupancy_correct,
            learned_deterministic_occupancy_agreement,
            learned_center_error_px,
        });
    }
    ensure!(!records.is_empty(), "the dataset has no included samples");
    let count = records.len() as f64;
    let learned_state_accuracy = fraction(&records, |record| record.learned_state_correct);
    let learned_occupancy_accuracy = fraction(&records, |record| record.learned_occupancy_correct);
    let deterministic_occupancy_accuracy =
        fraction(&records, |record| record.deterministic_occupancy_correct);
    let learned_deterministic_occupancy_agreement = fraction(&records, |record| {
        record.learned_deterministic_occupancy_agreement
    });
    let learned_confidence_gate_pass_rate = records
        .iter()
        .filter(|record| record.learned.confidence_gate_passed)
        .count() as f64
        / count;
    let report = ComparisonReport {
        schema_version: 1,
        model: model_status,
        sample_count: records.len(),
        split_counts,
        learned_state_accuracy,
        learned_occupancy_accuracy,
        deterministic_occupancy_accuracy,
        learned_deterministic_occupancy_agreement,
        learned_confidence_gate_pass_rate,
        learned_mean_center_error_px: mean(
            records
                .iter()
                .filter_map(|record| record.learned_center_error_px),
        ),
        deterministic_mean_center_error_px: mean(
            records
                .iter()
                .filter_map(|record| record.deterministic_center_error_px),
        ),
        legend: BTreeMap::from([
            ("ground_truth".into(), "magenta outline".into()),
            (
                "deterministic".into(),
                "green outline in each left panel".into(),
            ),
            ("learned".into(), "cyan outline in each right panel".into()),
            ("incorrect".into(), "red panel border".into()),
        ]),
        sheet_columns: SHEET_COLUMNS,
        records,
    };
    let sheet = compose_sheet(&rendered);
    fs::write(&output_png, encode_rgb_png(&sheet)?)
        .with_context(|| format!("cannot write {}", output_png.display()))?;
    fs::write(&output_json, serde_json::to_vec_pretty(&report)?)
        .with_context(|| format!("cannot write {}", output_json.display()))?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

fn read_json<T: for<'de> Deserialize<'de>>(path: &Path) -> Result<T> {
    serde_json::from_slice(
        &fs::read(path).with_context(|| format!("cannot read {}", path.display()))?,
    )
    .with_context(|| format!("cannot decode {}", path.display()))
}

fn read_gray(path: &Path) -> Result<GrayFrame> {
    decode_gray_png(&fs::read(path).with_context(|| format!("cannot read {}", path.display()))?)
}

fn parse_class(value: Option<&str>) -> Result<HeldPartClass> {
    match value {
        None => Ok(HeldPartClass::BareNozzle),
        Some("component_flat") => Ok(HeldPartClass::ComponentFlat),
        Some("component_on_edge") => Ok(HeldPartClass::ComponentOnEdge),
        Some(value) => anyhow::bail!("unsupported held-part class {value}"),
    }
}

fn fraction(records: &[ComparisonRecord], predicate: impl Fn(&ComparisonRecord) -> bool) -> f64 {
    records.iter().filter(|record| predicate(record)).count() as f64 / records.len() as f64
}

fn mean(values: impl Iterator<Item = f64>) -> Option<f64> {
    let values = values.collect::<Vec<_>>();
    (!values.is_empty()).then(|| values.iter().sum::<f64>() / values.len() as f64)
}

fn outline_center(outline: [[f64; 2]; 4]) -> [f64; 2] {
    [
        outline.iter().map(|point| point[0]).sum::<f64>() / 4.0,
        outline.iter().map(|point| point[1]).sum::<f64>() / 4.0,
    ]
}

fn outline_centers(first: Option<[[f64; 2]; 4]>, second: Option<[[f64; 2]; 4]>) -> Option<f64> {
    first.zip(second).map(|(first, second)| {
        let first = outline_center(first);
        let second = outline_center(second);
        (first[0] - second[0]).hypot(first[1] - second[1])
    })
}

fn render_sample(
    observed: &GrayFrame,
    ground_truth: Option<[[f64; 2]; 4]>,
    deterministic: Option<[[f64; 2]; 4]>,
    learned: &HeldPartModelPrediction,
    deterministic_correct: bool,
    learned_correct: bool,
) -> Result<RenderedSample> {
    let resized = resize_gray_frame(observed, TILE_SIZE, TILE_SIZE)?;
    let mut left = grayscale_preview(&resized);
    let mut right = left.clone();
    let scale = f64::from(TILE_SIZE) / f64::from(observed.width);
    if let Some(outline) = ground_truth {
        let outline = scale_outline(outline, scale);
        draw_polygon(&mut left, outline, [255, 0, 255]);
        draw_polygon(&mut right, outline, [255, 0, 255]);
    }
    if let Some(outline) = deterministic {
        draw_polygon(&mut left, scale_outline(outline, scale), [32, 255, 64]);
    }
    if learned.class.occupied() && learned.pose.valid {
        draw_polygon(
            &mut right,
            scale_outline(learned.pose.outline_px, scale),
            [0, 240, 255],
        );
        draw_cross(
            &mut right,
            learned.pose.center_px[0] * scale,
            learned.pose.center_px[1] * scale,
            [0, 240, 255],
        );
    } else {
        draw_cross(
            &mut right,
            f64::from(TILE_SIZE) / 2.0,
            f64::from(TILE_SIZE) / 2.0,
            [255, 48, 32],
        );
    }
    if !deterministic_correct {
        draw_border(&mut left, [255, 32, 16]);
    }
    if !learned_correct {
        draw_border(&mut right, [255, 32, 16]);
    }
    Ok(RenderedSample { left, right })
}

fn grayscale_preview(frame: &GrayFrame) -> RgbFrame {
    let mut pixels = Vec::with_capacity(frame.pixels.len() * 3);
    for value in &frame.pixels {
        pixels.extend_from_slice(&[*value, *value, *value]);
    }
    RgbFrame {
        width: frame.width,
        height: frame.height,
        pixels,
    }
}

fn scale_outline(mut outline: [[f64; 2]; 4], scale: f64) -> [[f64; 2]; 4] {
    for point in &mut outline {
        point[0] *= scale;
        point[1] *= scale;
    }
    outline
}

fn draw_polygon(frame: &mut RgbFrame, outline: [[f64; 2]; 4], color: [u8; 3]) {
    for index in 0..4 {
        draw_line(frame, outline[index], outline[(index + 1) % 4], color);
    }
}

fn draw_line(frame: &mut RgbFrame, start: [f64; 2], end: [f64; 2], color: [u8; 3]) {
    let steps = (end[0] - start[0]).hypot(end[1] - start[1]).ceil().max(1.0) as u32;
    for step in 0..=steps {
        let fraction = f64::from(step) / f64::from(steps);
        set_pixel(
            frame,
            (start[0] + fraction * (end[0] - start[0])).round() as i32,
            (start[1] + fraction * (end[1] - start[1])).round() as i32,
            color,
        );
    }
}

fn draw_cross(frame: &mut RgbFrame, x: f64, y: f64, color: [u8; 3]) {
    for offset in -5..=5 {
        set_pixel(frame, x.round() as i32 + offset, y.round() as i32, color);
        set_pixel(frame, x.round() as i32, y.round() as i32 + offset, color);
    }
}

fn draw_border(frame: &mut RgbFrame, color: [u8; 3]) {
    for inset in 0..3 {
        let maximum = i32::try_from(frame.width).unwrap() - 1 - inset;
        draw_line(
            frame,
            [f64::from(inset), f64::from(inset)],
            [f64::from(maximum), f64::from(inset)],
            color,
        );
        draw_line(
            frame,
            [f64::from(maximum), f64::from(inset)],
            [f64::from(maximum), f64::from(maximum)],
            color,
        );
        draw_line(
            frame,
            [f64::from(maximum), f64::from(maximum)],
            [f64::from(inset), f64::from(maximum)],
            color,
        );
        draw_line(
            frame,
            [f64::from(inset), f64::from(maximum)],
            [f64::from(inset), f64::from(inset)],
            color,
        );
    }
}

fn set_pixel(frame: &mut RgbFrame, x: i32, y: i32, color: [u8; 3]) {
    if x < 0 || y < 0 || x >= frame.width as i32 || y >= frame.height as i32 {
        return;
    }
    let index = (y as usize * frame.width as usize + x as usize) * 3;
    frame.pixels[index..index + 3].copy_from_slice(&color);
}

fn compose_sheet(samples: &[RenderedSample]) -> RgbFrame {
    let cell_width = TILE_SIZE * 2 + PANEL_GAP;
    let cell_height = TILE_SIZE;
    let rows = u32::try_from(samples.len())
        .unwrap()
        .div_ceil(SHEET_COLUMNS);
    let width = SHEET_COLUMNS * cell_width + (SHEET_COLUMNS - 1) * SAMPLE_GAP;
    let height = rows * cell_height + rows.saturating_sub(1) * SAMPLE_GAP;
    let mut sheet = RgbFrame {
        width,
        height,
        pixels: vec![20; (width * height * 3) as usize],
    };
    for (index, sample) in samples.iter().enumerate() {
        let index = u32::try_from(index).unwrap();
        let column = index % SHEET_COLUMNS;
        let row = index / SHEET_COLUMNS;
        let x = column * (cell_width + SAMPLE_GAP);
        let y = row * (cell_height + SAMPLE_GAP);
        blit(&mut sheet, &sample.left, x, y);
        blit(&mut sheet, &sample.right, x + TILE_SIZE + PANEL_GAP, y);
    }
    sheet
}

fn blit(target: &mut RgbFrame, source: &RgbFrame, left: u32, top: u32) {
    for y in 0..source.height {
        let source_start = (y * source.width * 3) as usize;
        let target_start = ((top + y) * target.width * 3 + left * 3) as usize;
        let length = (source.width * 3) as usize;
        target.pixels[target_start..target_start + length]
            .copy_from_slice(&source.pixels[source_start..source_start + length]);
    }
}
