use anyhow::{bail, ensure, Context, Result};
use chrono::Utc;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::path::PathBuf;

#[derive(Debug, Deserialize)]
struct DraftManifest {
    name: String,
    samples: Vec<DraftSample>,
}

#[derive(Debug, Deserialize)]
struct DraftSample {
    id: String,
    split: String,
    image_path: String,
    width_px: u32,
    height_px: u32,
    part_key: Option<String>,
    draft: DraftLabel,
}

#[derive(Debug, Deserialize)]
struct DraftLabel {
    center_px: Option<[f64; 2]>,
    outline_px: Option<[[f64; 2]; 4]>,
}

#[derive(Debug, Deserialize)]
struct ReviewFile {
    schema_version: u32,
    dataset: String,
    decisions: BTreeMap<String, ReviewDecision>,
}

#[derive(Debug, Deserialize)]
struct ReviewDecision {
    decision: String,
    #[serde(default)]
    note: String,
    #[serde(default)]
    class_name: Option<String>,
    #[serde(default)]
    outline_px: Option<[[f64; 2]; 4]>,
}

#[derive(Debug, Deserialize)]
struct CarriedGroundTruthManifest {
    samples: Vec<CarriedGroundTruthSample>,
}

#[derive(Debug, Deserialize)]
struct CarriedGroundTruthSample {
    part_key: Option<String>,
    included: bool,
    class_name: Option<String>,
    outline_px: Option<[[f64; 2]; 4]>,
}

#[derive(Debug, Clone, Serialize)]
struct LearnedExtent {
    part_key: String,
    sample_count: usize,
    width_px: f64,
    height_px: f64,
}

#[derive(Debug, Serialize)]
struct RefinedManifest {
    schema_version: u32,
    name: String,
    generated_at: String,
    source_manifest: String,
    source_manifest_sha256: String,
    source_review: String,
    source_review_sha256: String,
    source_carried_ground_truth: Option<String>,
    source_carried_ground_truth_sha256: Option<String>,
    classes: Vec<String>,
    notes: Vec<String>,
    learned_extents: Vec<LearnedExtent>,
    samples: Vec<RefinedSample>,
}

#[derive(Debug, Serialize)]
struct RefinedSample {
    id: String,
    split: String,
    image_path: String,
    width_px: u32,
    height_px: u32,
    part_key: Option<String>,
    first_decision: String,
    first_note: String,
    class_name: Option<String>,
    outline_px: Option<[[f64; 2]; 4]>,
    proposal_source: String,
    requires_review: bool,
}

type RefinedLabel = (Option<String>, Option<[[f64; 2]; 4]>, String, bool);

fn main() -> Result<()> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    ensure!(
        arguments.len() == 1,
        "usage: refine_source_component_dataset DATASET_DIR"
    );
    let root = PathBuf::from(&arguments[0]);
    let manifest_path = root.join("manifest.draft.json");
    let review_path = root.join("review-decisions.json");
    let manifest_bytes = fs::read(&manifest_path)
        .with_context(|| format!("cannot read {}", manifest_path.display()))?;
    let review_bytes =
        fs::read(&review_path).with_context(|| format!("cannot read {}", review_path.display()))?;
    let carried_path = root.join("manifest.carried-ground-truth.json");
    let carried_bytes = carried_path
        .is_file()
        .then(|| fs::read(&carried_path))
        .transpose()?;
    let manifest: DraftManifest = serde_json::from_slice(&manifest_bytes)?;
    let review: ReviewFile = serde_json::from_slice(&review_bytes)?;
    let carried: Option<CarriedGroundTruthManifest> = carried_bytes
        .as_deref()
        .map(serde_json::from_slice)
        .transpose()?;
    ensure!(review.schema_version == 1, "unsupported review schema");
    ensure!(
        review.dataset == manifest.name,
        "review is for another dataset"
    );
    ensure!(
        review.decisions.len() == manifest.samples.len(),
        "review must contain one decision for each sample"
    );
    for sample in &manifest.samples {
        ensure!(
            review.decisions.contains_key(&sample.id),
            "sample {} has no review decision",
            sample.id
        );
    }
    for id in review.decisions.keys() {
        ensure!(
            manifest.samples.iter().any(|sample| &sample.id == id),
            "review contains unknown sample {id}"
        );
    }

    let learned_extents = learn_accepted_extents(&manifest, &review, carried.as_ref())?;
    let learned_by_part = learned_extents
        .iter()
        .map(|extent| (extent.part_key.clone(), extent.clone()))
        .collect::<BTreeMap<_, _>>();
    let mut samples = Vec::with_capacity(manifest.samples.len());
    for sample in manifest.samples {
        let decision = &review.decisions[&sample.id];
        let (class_name, outline_px, proposal_source, requires_review) =
            refine_sample(&sample, decision, &learned_by_part)?;
        samples.push(RefinedSample {
            id: sample.id,
            split: sample.split,
            image_path: sample.image_path,
            width_px: sample.width_px,
            height_px: sample.height_px,
            part_key: sample.part_key,
            first_decision: decision.decision.clone(),
            first_note: decision.note.clone(),
            class_name,
            outline_px,
            proposal_source,
            requires_review,
        });
    }

    let output = RefinedManifest {
        schema_version: 1,
        name: format!("{}-refined", manifest.name),
        generated_at: Utc::now().to_rfc3339(),
        source_manifest: "manifest.draft.json".into(),
        source_manifest_sha256: hex_sha256(&manifest_bytes),
        source_review: "review-decisions.json".into(),
        source_review_sha256: hex_sha256(&review_bytes),
        source_carried_ground_truth: carried_bytes
            .as_ref()
            .map(|_| "manifest.carried-ground-truth.json".into()),
        source_carried_ground_truth_sha256: carried_bytes.as_deref().map(hex_sha256),
        classes: vec!["component_flat".into(), "component_on_edge".into()],
        notes: vec![
            "accepted boxes and empty decisions are final first-pass labels".into(),
            "first-pass box edits preserve operator position, extent, and rotation".into(),
            "adjusted flat boxes use the median accepted extent for that part".into(),
            "verified carried ground truth can supply prior accepted extents".into(),
            "the existing image detector supplies the proposed center".into(),
            "on-edge and unknown-part boxes remain operator-reviewed drafts".into(),
            "refined draft labels are not ground truth until accepted".into(),
        ],
        learned_extents,
        samples,
    };
    fs::write(
        root.join("manifest.refined.json"),
        serde_json::to_vec_pretty(&output)?,
    )?;
    fs::write(root.join("review-refined.html"), review_html(&output)?)?;

    let learned = output
        .samples
        .iter()
        .filter(|sample| sample.proposal_source == "accepted_extent_and_detected_center")
        .count();
    let unresolved = output
        .samples
        .iter()
        .filter(|sample| sample.requires_review)
        .count();
    if unresolved == 0 {
        let review = serde_json::json!({
            "schema_version": 1,
            "dataset": output.name,
            "decisions": {},
        });
        fs::write(
            root.join("review-refined-decisions.json"),
            serde_json::to_vec_pretty(&review)?,
        )?;
    }
    println!(
        "refined {} samples; {} flat boxes learned; {} require second review",
        output.samples.len(),
        learned,
        unresolved
    );
    if unresolved == 0 {
        println!("no second review is required");
    } else {
        println!("review {}", root.join("review-refined.html").display());
    }
    Ok(())
}

fn learn_accepted_extents(
    manifest: &DraftManifest,
    review: &ReviewFile,
    carried: Option<&CarriedGroundTruthManifest>,
) -> Result<Vec<LearnedExtent>> {
    let mut values: BTreeMap<String, Vec<[f64; 2]>> = BTreeMap::new();
    if let Some(carried) = carried {
        for sample in &carried.samples {
            if !sample.included || sample.class_name.as_deref() != Some("component_flat") {
                continue;
            }
            let (Some(part_key), Some(outline)) = (&sample.part_key, sample.outline_px) else {
                continue;
            };
            values
                .entry(part_key.clone())
                .or_default()
                .push(outline_extent(outline));
        }
    }
    for sample in &manifest.samples {
        let decision = &review.decisions[&sample.id];
        if decision.decision != "accept"
            || decision.class_name.as_deref().unwrap_or("component_flat") != "component_flat"
        {
            continue;
        }
        let Some(part_key) = &sample.part_key else {
            continue;
        };
        let outline = decision
            .outline_px
            .or(sample.draft.outline_px)
            .with_context(|| format!("accepted sample {} has no box", sample.id))?;
        values
            .entry(part_key.clone())
            .or_default()
            .push(outline_extent(outline));
    }
    values
        .into_iter()
        .map(|(part_key, extents)| {
            ensure!(
                extents.len() >= 2,
                "part {part_key} needs two accepted boxes to learn its extent"
            );
            let mut widths = extents.iter().map(|extent| extent[0]).collect::<Vec<_>>();
            let mut heights = extents.iter().map(|extent| extent[1]).collect::<Vec<_>>();
            Ok(LearnedExtent {
                part_key,
                sample_count: extents.len(),
                width_px: median(&mut widths),
                height_px: median(&mut heights),
            })
        })
        .collect()
}

fn refine_sample(
    sample: &DraftSample,
    decision: &ReviewDecision,
    learned: &BTreeMap<String, LearnedExtent>,
) -> Result<RefinedLabel> {
    match decision.decision.as_str() {
        "accept" => {
            let class_name = decision.class_name.as_deref().unwrap_or("component_flat");
            ensure!(
                matches!(class_name, "component_flat" | "component_on_edge"),
                "sample {} has unknown class {class_name}",
                sample.id
            );
            let outline = decision
                .outline_px
                .or(sample.draft.outline_px)
                .with_context(|| format!("accepted sample {} has no box", sample.id))?;
            Ok((
                Some(class_name.into()),
                Some(outline),
                if decision.outline_px.is_some() {
                    "operator_edited".into()
                } else {
                    "operator_accepted".into()
                },
                false,
            ))
        }
        "empty" => Ok((None, None, "operator_empty".into(), false)),
        "reject" => Ok((None, None, "operator_rejected".into(), false)),
        "adjust" => {
            let Some(part_key) = &sample.part_key else {
                return Ok((
                    Some("component_flat".into()),
                    sample.draft.outline_px,
                    "unknown_part_requires_box_edit".into(),
                    true,
                ));
            };
            let extent = learned
                .get(part_key)
                .with_context(|| format!("part {part_key} has no accepted extent"))?;
            let center = sample.draft.center_px.unwrap_or([
                f64::from(sample.width_px) * 0.5,
                f64::from(sample.height_px) * 0.5,
            ]);
            Ok((
                Some("component_flat".into()),
                Some(axis_aligned_box(center, extent.width_px, extent.height_px)),
                "accepted_extent_and_detected_center".into(),
                true,
            ))
        }
        "on_edge" if decision.outline_px.is_some() => Ok((
            Some("component_on_edge".into()),
            decision.outline_px,
            "operator_edited".into(),
            false,
        )),
        "on_edge" => {
            let extent = sample
                .part_key
                .as_ref()
                .and_then(|key| learned.get(key))
                .map(|extent| [extent.width_px, extent.height_px])
                .or_else(|| sample.draft.outline_px.map(outline_extent))
                .unwrap_or([40.0, 22.0]);
            let center = sample.draft.center_px.unwrap_or([
                f64::from(sample.width_px) * 0.5,
                f64::from(sample.height_px) * 0.5,
            ]);
            Ok((
                Some("component_on_edge".into()),
                Some(axis_aligned_box(center, extent[0], extent[1])),
                "on_edge_edit_seed".into(),
                true,
            ))
        }
        value => bail!("sample {} has unknown decision {value}", sample.id),
    }
}

fn outline_extent(outline: [[f64; 2]; 4]) -> [f64; 2] {
    let edge =
        |first: [f64; 2], second: [f64; 2]| (second[0] - first[0]).hypot(second[1] - first[1]);
    [edge(outline[0], outline[1]), edge(outline[1], outline[2])]
}

fn axis_aligned_box(center: [f64; 2], width: f64, height: f64) -> [[f64; 2]; 4] {
    let half_width = width * 0.5;
    let half_height = height * 0.5;
    [
        [center[0] - half_width, center[1] - half_height],
        [center[0] + half_width, center[1] - half_height],
        [center[0] + half_width, center[1] + half_height],
        [center[0] - half_width, center[1] + half_height],
    ]
}

fn median(values: &mut [f64]) -> f64 {
    values.sort_by(f64::total_cmp);
    let middle = values.len() / 2;
    if values.len() % 2 == 0 {
        0.5 * (values[middle - 1] + values[middle])
    } else {
        values[middle]
    }
}

fn hex_sha256(data: &[u8]) -> String {
    Sha256::digest(data)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn review_html(manifest: &RefinedManifest) -> Result<String> {
    let data = serde_json::to_string(manifest)?;
    let title = serde_json::to_string(&manifest.name)?;
    Ok(format!(
        r##"<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>VelocityPnP refined component review</title>
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
<h1>Refined source-component review</h1>
<p class="warning">Drag the box, corners, or round rotation handle. Accept only
after the box covers the visible component.</p>
<p id="progress"></p>
<div id="frame"><img id="image"><svg id="overlay">
<line id="rotate-line"></line><polygon id="box"></polygon>
<g id="handles"></g></svg></div>
<pre id="meta"></pre>
<textarea id="note" placeholder="Optional correction note"></textarea>
<div class="controls">
<select id="class-name"><option>component_flat</option>
<option>component_on_edge</option></select>
<button data-decision="accept">A: Accept box</button>
<button data-decision="empty">E: Empty</button>
<button data-decision="reject">R: Reject image</button>
<button id="reset">Reset box</button>
<button id="previous">Previous</button><button id="next">Next</button>
<button id="download">Download decisions</button>
</div>
</main>
<script>
const manifest = {data};
const samples = manifest.samples.filter(sample => sample.requires_review);
const key = {title} + ":review";
const saved = JSON.parse(localStorage.getItem(key) || "{{}}");
let index = 0, drag = null, boxState = null;
const image = document.querySelector("#image");
const overlay = document.querySelector("#overlay");
const polygon = document.querySelector("#box");
const handles = document.querySelector("#handles");
const rotateLine = document.querySelector("#rotate-line");
const note = document.querySelector("#note");
const className = document.querySelector("#class-name");
function store() {{ localStorage.setItem(key, JSON.stringify(saved)); }}
function fromOutline(points) {{
  const center = points.reduce((a, p) => [a[0] + p[0] / 4,
    a[1] + p[1] / 4], [0, 0]);
  return {{cx: center[0], cy: center[1],
    width: Math.hypot(points[1][0] - points[0][0], points[1][1] - points[0][1]),
    height: Math.hypot(points[2][0] - points[1][0], points[2][1] - points[1][1]),
    angle: Math.atan2(points[1][1] - points[0][1],
      points[1][0] - points[0][0])}};
}}
function points(state = boxState) {{
  const u = [Math.cos(state.angle), Math.sin(state.angle)];
  const v = [-u[1], u[0]], hw = state.width / 2, hh = state.height / 2;
  return [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]].map(offset =>
    [state.cx + u[0] * offset[0] + v[0] * offset[1],
      state.cy + u[1] * offset[0] + v[1] * offset[1]]);
}}
function renderBox() {{
  const corners = points();
  polygon.setAttribute("points", corners.map(p => p.join(",")).join(" "));
  handles.innerHTML = "";
  corners.forEach((point, corner) => {{
    const handle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    handle.setAttribute("class", "handle"); handle.setAttribute("r", 5);
    handle.setAttribute("cx", point[0]); handle.setAttribute("cy", point[1]);
    handle.dataset.corner = corner; handles.appendChild(handle);
  }});
  const v = [-Math.sin(boxState.angle), Math.cos(boxState.angle)];
  const top = [boxState.cx - v[0] * boxState.height / 2,
    boxState.cy - v[1] * boxState.height / 2];
  const rotate = [top[0] - v[0] * 22, top[1] - v[1] * 22];
  rotateLine.setAttribute("x1", top[0]); rotateLine.setAttribute("y1", top[1]);
  rotateLine.setAttribute("x2", rotate[0]); rotateLine.setAttribute("y2", rotate[1]);
  const handle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
  handle.setAttribute("class", "handle"); handle.setAttribute("r", 6);
  handle.setAttribute("cx", rotate[0]); handle.setAttribute("cy", rotate[1]);
  handle.dataset.rotate = "true"; handles.appendChild(handle);
}}
function current() {{
  const sample = samples[index];
  return saved[sample.id] || {{decision: "unreviewed", note: "",
    class_name: sample.class_name, outline_px: sample.outline_px}};
}}
function show() {{
  const sample = samples[index], result = current();
  image.src = sample.image_path;
  overlay.setAttribute("viewBox", `0 0 ${{sample.width_px}} ${{sample.height_px}}`);
  boxState = fromOutline(result.outline_px || sample.outline_px);
  note.value = result.note || ""; className.value = result.class_name;
  renderBox();
  document.querySelectorAll("[data-decision]").forEach(button =>
    button.classList.toggle("selected", button.dataset.decision === result.decision));
  const reviewed = Object.values(saved).filter(value =>
    value.decision !== "unreviewed").length;
  document.querySelector("#progress").textContent =
    `${{index + 1}} / ${{samples.length}}; ${{reviewed}} reviewed`;
  document.querySelector("#meta").textContent = JSON.stringify({{
    id: sample.id, split: sample.split, part_key: sample.part_key,
    first_decision: sample.first_decision, first_note: sample.first_note,
    proposal_source: sample.proposal_source
  }}, null, 2);
}}
function persist(decision) {{
  const sample = samples[index];
  saved[sample.id] = {{decision, note: note.value, class_name: className.value,
    outline_px: points()}}; store();
}}
function decide(value) {{
  persist(value);
  if (index + 1 < samples.length) index++;
  show();
}}
function eventPoint(event) {{
  const point = overlay.createSVGPoint(); point.x = event.clientX; point.y = event.clientY;
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
  const point = eventPoint(event), dx = point[0] - drag.start[0],
    dy = point[1] - drag.start[1];
  if (drag.mode === "move") {{
    boxState.cx = drag.initial.cx + dx; boxState.cy = drag.initial.cy + dy;
  }} else if (drag.mode === "rotate") {{
    boxState.angle = Math.atan2(point[1] - boxState.cy,
      point[0] - boxState.cx) + Math.PI / 2;
  }} else {{
    const old = points(drag.initial), opposite = old[(drag.corner + 2) % 4];
    const u = [Math.cos(drag.initial.angle), Math.sin(drag.initial.angle)];
    const v = [-u[1], u[0]], delta = [point[0] - opposite[0],
      point[1] - opposite[1]];
    boxState.width = Math.max(4, Math.abs(delta[0] * u[0] + delta[1] * u[1]));
    boxState.height = Math.max(4, Math.abs(delta[0] * v[0] + delta[1] * v[1]));
    boxState.cx = (point[0] + opposite[0]) / 2;
    boxState.cy = (point[1] + opposite[1]) / 2;
  }}
  renderBox();
}};
overlay.onpointerup = () => {{ drag = null; persist(current().decision); }};
document.querySelectorAll("[data-decision]").forEach(button =>
  button.onclick = () => decide(button.dataset.decision));
document.querySelector("#reset").onclick = () => {{
  boxState = fromOutline(samples[index].outline_px); renderBox();
}};
document.querySelector("#previous").onclick = () => {{
  persist(current().decision); index = Math.max(0, index - 1); show();
}};
document.querySelector("#next").onclick = () => {{
  persist(current().decision); index = Math.min(samples.length - 1, index + 1); show();
}};
note.onchange = () => persist(current().decision);
className.onchange = () => persist(current().decision);
document.querySelector("#download").onclick = () => {{
  persist(current().decision);
  const result = {{schema_version: 1, dataset: manifest.name,
    exported_at: new Date().toISOString(), decisions: saved}};
  const link = document.createElement("a");
  link.href = URL.createObjectURL(new Blob([JSON.stringify(result, null, 2)],
    {{type: "application/json"}}));
  link.download = "review-refined-decisions.json"; link.click();
}};
document.addEventListener("keydown", event => {{
  if (event.target === note || event.target === className) return;
  const action = {{a: "accept", e: "empty", r: "reject"}}[event.key.toLowerCase()];
  if (action) decide(action);
  if (event.key === "ArrowLeft") {{ index = Math.max(0, index - 1); show(); }}
  if (event.key === "ArrowRight") {{ index = Math.min(samples.length - 1,
    index + 1); show(); }}
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
    fn accepted_extents_use_the_median_for_each_part() {
        let manifest = DraftManifest {
            name: "test".into(),
            samples: vec![
                sample(
                    "first",
                    "part",
                    Some(axis_aligned_box([20.0, 20.0], 40.0, 20.0)),
                ),
                sample(
                    "second",
                    "part",
                    Some(axis_aligned_box([20.0, 20.0], 44.0, 24.0)),
                ),
                sample(
                    "third",
                    "part",
                    Some(axis_aligned_box([20.0, 20.0], 80.0, 50.0)),
                ),
            ],
        };
        let review = ReviewFile {
            schema_version: 1,
            dataset: "test".into(),
            decisions: ["first", "second", "third"]
                .into_iter()
                .map(|id| {
                    (
                        id.into(),
                        ReviewDecision {
                            decision: "accept".into(),
                            note: String::new(),
                            class_name: None,
                            outline_px: None,
                        },
                    )
                })
                .collect(),
        };
        let learned = learn_accepted_extents(&manifest, &review, None).unwrap();
        assert_eq!(learned.len(), 1);
        assert_eq!(learned[0].width_px, 44.0);
        assert_eq!(learned[0].height_px, 24.0);
    }

    #[test]
    fn carried_ground_truth_supplies_prior_accepted_extents() {
        let manifest = DraftManifest {
            name: "test".into(),
            samples: vec![sample("adjust", "part", None)],
        };
        let review = ReviewFile {
            schema_version: 1,
            dataset: "test".into(),
            decisions: BTreeMap::from([(
                "adjust".into(),
                ReviewDecision {
                    decision: "adjust".into(),
                    note: String::new(),
                    class_name: None,
                    outline_px: None,
                },
            )]),
        };
        let carried = CarriedGroundTruthManifest {
            samples: vec![
                carried_sample(axis_aligned_box([20.0, 20.0], 40.0, 20.0)),
                carried_sample(axis_aligned_box([20.0, 20.0], 44.0, 24.0)),
            ],
        };

        let learned = learn_accepted_extents(&manifest, &review, Some(&carried)).unwrap();
        assert_eq!(learned[0].sample_count, 2);
        assert_eq!(learned[0].width_px, 42.0);
        assert_eq!(learned[0].height_px, 22.0);
    }

    #[test]
    fn adjusted_box_uses_learned_extent_and_detected_center() {
        let sample = DraftSample {
            id: "sample".into(),
            split: "train".into(),
            image_path: "image.png".into(),
            width_px: 100,
            height_px: 80,
            part_key: Some("part".into()),
            draft: DraftLabel {
                center_px: Some([48.0, 39.0]),
                outline_px: None,
            },
        };
        let decision = ReviewDecision {
            decision: "adjust".into(),
            note: String::new(),
            class_name: None,
            outline_px: None,
        };
        let learned = [(
            "part".into(),
            LearnedExtent {
                part_key: "part".into(),
                sample_count: 2,
                width_px: 40.0,
                height_px: 20.0,
            },
        )]
        .into_iter()
        .collect();
        let (_, outline, source, review) = refine_sample(&sample, &decision, &learned).unwrap();
        assert_eq!(outline.unwrap(), axis_aligned_box([48.0, 39.0], 40.0, 20.0));
        assert_eq!(source, "accepted_extent_and_detected_center");
        assert!(review);
    }

    #[test]
    fn accepted_edit_preserves_class_and_rotated_outline() {
        let sample = sample("edited", "part", None);
        let outline = [[10.0, 20.0], [30.0, 30.0], [25.0, 40.0], [5.0, 30.0]];
        let decision = ReviewDecision {
            decision: "accept".into(),
            note: String::new(),
            class_name: Some("component_on_edge".into()),
            outline_px: Some(outline),
        };

        let (class_name, resolved, source, requires_review) =
            refine_sample(&sample, &decision, &BTreeMap::new()).unwrap();

        assert_eq!(class_name.as_deref(), Some("component_on_edge"));
        assert_eq!(resolved, Some(outline));
        assert_eq!(source, "operator_edited");
        assert!(!requires_review);
    }

    fn sample(id: &str, part: &str, outline: Option<[[f64; 2]; 4]>) -> DraftSample {
        DraftSample {
            id: id.into(),
            split: "train".into(),
            image_path: format!("{id}.png"),
            width_px: 100,
            height_px: 80,
            part_key: Some(part.into()),
            draft: DraftLabel {
                center_px: Some([20.0, 20.0]),
                outline_px: outline,
            },
        }
    }

    fn carried_sample(outline: [[f64; 2]; 4]) -> CarriedGroundTruthSample {
        CarriedGroundTruthSample {
            part_key: Some("part".into()),
            included: true,
            class_name: Some("component_flat".into()),
            outline_px: Some(outline),
        }
    }
}
