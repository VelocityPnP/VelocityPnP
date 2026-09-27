#!/usr/bin/env python3
"""Create a focused class audit from a reviewed source-component dataset."""

from __future__ import annotations

import argparse
import html
import json
from pathlib import Path
from typing import Any


CLASS_NAMES = ("component_flat", "component_on_edge")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument(
        "--class-name",
        choices=CLASS_NAMES,
        default="component_on_edge",
    )
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def load_samples(
    dataset: Path, class_name: str
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    manifest_path = dataset / "manifest.ground-truth.json"
    manifest = json.loads(manifest_path.read_text())
    samples = []
    for sample in manifest.get("samples", []):
        if not sample.get("included") or sample.get("class_name") != class_name:
            continue
        image_path = Path(sample["image_path"])
        if image_path.is_absolute() or ".." in image_path.parts:
            raise ValueError(f"sample {sample.get('id')} has an unsafe image path")
        if not (dataset / image_path).is_file():
            raise ValueError(f"sample {sample.get('id')} image is missing")
        outline = sample.get("outline_px")
        if not isinstance(outline, list) or len(outline) != 4:
            raise ValueError(f"sample {sample.get('id')} has no reviewed box")
        samples.append(
            {
                "id": sample["id"],
                "image_path": image_path.as_posix(),
                "split": sample["split"],
                "original_class_name": class_name,
                "outline_px": outline,
                "review_note": sample.get("review_note", ""),
            }
        )
    if not samples:
        raise ValueError(f"dataset has no included {class_name} samples")
    return manifest, samples


def json_for_script(value: Any) -> str:
    return json.dumps(value, separators=(",", ":")).replace("</", "<\\/")


def render_review(
    dataset: Path,
    manifest: dict[str, Any],
    samples: list[dict[str, Any]],
    class_name: str,
) -> str:
    dataset_name = manifest.get("name") or dataset.name
    title = f"Source component class audit: {class_name}"
    result = HTML_TEMPLATE
    replacements = {
        "__TITLE__": html.escape(title),
        "__DATASET_JSON__": json_for_script(str(dataset_name)),
        "__CLASS_JSON__": json_for_script(class_name),
        "__SAMPLES_JSON__": json_for_script(samples),
    }
    for marker, value in replacements.items():
        result = result.replace(marker, value)
    return result


def main() -> None:
    args = parse_args()
    dataset = args.dataset.resolve()
    manifest, samples = load_samples(dataset, args.class_name)
    output = args.output or dataset / (
        f"review-{args.class_name.replace('_', '-')}-audit.html"
    )
    output = output.resolve()
    output.write_text(
        render_review(dataset, manifest, samples, args.class_name),
        encoding="utf-8",
    )
    print(f"wrote {output}")
    print(f"review samples: {len(samples)}")


HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>__TITLE__</title>
<style>
body { font: 16px system-ui, sans-serif; margin: 1rem auto; max-width: 980px; padding: 0 1rem; }
h1 { font-size: 1.25rem; }
#status { margin: .5rem 0; }
canvas { display: block; width: 100%; height: auto; background: #222; border: 1px solid #777; }
.controls { display: flex; flex-wrap: wrap; gap: .5rem; margin: .75rem 0; }
button, select { font: inherit; padding: .55rem .8rem; }
button.selected { outline: 3px solid #00a6d6; }
.spacer { flex: 1; }
pre { white-space: pre-wrap; overflow-wrap: anywhere; background: #eee; padding: .75rem; }
</style>
</head>
<body>
<h1>__TITLE__</h1>
<div id="status"></div>
<canvas id="image"></canvas>
<div class="controls">
  <button data-choice="component_flat">1 Flat</button>
  <button data-choice="component_on_edge">2 On edge</button>
  <button data-choice="empty">3 Empty</button>
  <button data-choice="reject">4 Reject</button>
  <span class="spacer"></span>
  <button id="previous">Previous</button>
  <button id="next">Next</button>
</div>
<div class="controls">
  <label>Image <select id="jump"></select></label>
  <button id="download">Download decisions</button>
</div>
<p>Keys: 1 Flat, 2 On edge, 3 Empty, 4 Reject, arrows move.</p>
<h2>Current image metadata</h2>
<pre id="metadata"></pre>
<script>
const dataset = __DATASET_JSON__;
const auditedClass = __CLASS_JSON__;
const samples = __SAMPLES_JSON__;
const storageKey = `velocitypnp-source-class-audit:${dataset}:${auditedClass}`;
const saved = JSON.parse(localStorage.getItem(storageKey) || "{}");
let index = 0;
const canvas = document.getElementById("image");
const context = canvas.getContext("2d");
const image = new Image();
const status = document.getElementById("status");
const metadata = document.getElementById("metadata");
const jump = document.getElementById("jump");

for (let i = 0; i < samples.length; i += 1) {
  const option = document.createElement("option");
  option.value = i;
  option.textContent = `${i + 1}: ${samples[i].id}`;
  jump.appendChild(option);
}

function save() {
  localStorage.setItem(storageKey, JSON.stringify(saved));
}

function reviewedCount() {
  return samples.filter(sample => saved[sample.id]).length;
}

function draw() {
  const sample = samples[index];
  canvas.width = image.naturalWidth;
  canvas.height = image.naturalHeight;
  context.drawImage(image, 0, 0);
  const points = sample.outline_px;
  context.strokeStyle = "#00e5ff";
  context.lineWidth = 3;
  context.beginPath();
  context.moveTo(points[0][0], points[0][1]);
  for (let i = 1; i < points.length; i += 1) context.lineTo(points[i][0], points[i][1]);
  context.closePath();
  context.stroke();
}

function render() {
  const sample = samples[index];
  const decision = saved[sample.id];
  status.textContent = `${index + 1} / ${samples.length}; reviewed ${reviewedCount()} / ${samples.length}`;
  jump.value = index;
  metadata.textContent = JSON.stringify({ ...sample, decision: decision || null }, null, 2);
  document.querySelectorAll("[data-choice]").forEach(button => {
    button.classList.toggle("selected", decision?.choice === button.dataset.choice);
  });
  image.onload = draw;
  image.src = sample.image_path;
}

function choose(choice) {
  const sample = samples[index];
  saved[sample.id] = {
    choice,
    original_class_name: sample.original_class_name,
    class_name: choice.startsWith("component_") ? choice : null,
    included: choice !== "reject",
    outline_px: choice === "empty" || choice === "reject" ? null : sample.outline_px,
  };
  save();
  if (index + 1 < samples.length) index += 1;
  render();
}

document.querySelectorAll("[data-choice]").forEach(button => {
  button.addEventListener("click", () => choose(button.dataset.choice));
});
document.getElementById("previous").onclick = () => { index = Math.max(0, index - 1); render(); };
document.getElementById("next").onclick = () => { index = Math.min(samples.length - 1, index + 1); render(); };
jump.onchange = () => { index = Number(jump.value); render(); };
document.getElementById("download").onclick = () => {
  const output = {
    schema_version: 1,
    dataset,
    audited_class_name: auditedClass,
    decisions: saved,
  };
  const blob = new Blob([JSON.stringify(output, null, 2) + "\n"], { type: "application/json" });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = "source-component-class-review-decisions.json";
  link.click();
  URL.revokeObjectURL(link.href);
};
document.addEventListener("keydown", event => {
  if (event.key >= "1" && event.key <= "4") {
    choose(["component_flat", "component_on_edge", "empty", "reject"][Number(event.key) - 1]);
  } else if (event.key === "ArrowLeft") {
    index = Math.max(0, index - 1); render();
  } else if (event.key === "ArrowRight") {
    index = Math.min(samples.length - 1, index + 1); render();
  }
});
render();
</script>
</body>
</html>
"""


if __name__ == "__main__":
    main()
