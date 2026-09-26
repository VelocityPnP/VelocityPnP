#!/usr/bin/env python3
"""Export linked feeder-to-placement images for one board instance."""

from __future__ import annotations

import argparse
import io
import json
import math
import shutil
import sqlite3
from pathlib import Path
from typing import Any

from PIL import Image


TEMPLATE = Path(__file__).with_name("placement_trace_review.html")
IMAGE_ARTIFACTS = {
    "feeder": "source-pocket-pick-source-component-control.png",
    "held_initial": "held-part-inspection-initial-held-part-shadow.png",
    "held_final": "held-part-inspection-held-part-shadow.png",
    "target_before": "board-placement-before.png",
    "result_after": "board-placement-after.png",
}
JSON_ARTIFACTS = {
    "source": "source-pocket-pick-source-component-control.json",
    "held_initial": "held-part-inspection-initial-held-part-shadow.json",
    "held_final": "held-part-inspection-held-part-shadow.json",
}
REQUIRED_IMAGES = {"feeder", "held_initial", "target_before", "result_after"}
REQUIRED_JSON = {"source", "held_initial"}


def point(value: Any, width: int, height: int, label: str, required: bool = False) -> list[float] | None:
    valid = (
        isinstance(value, (list, tuple))
        and len(value) == 2
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value)
        and all(math.isfinite(item) for item in value)
        and 0 <= value[0] < width
        and 0 <= value[1] < height
    )
    if not valid:
        if required:
            raise ValueError(f"missing or invalid {label}")
        return None
    return [float(value[0]), float(value[1])]


def report_point(report: dict, width: int, height: int, x: str, y: str, label: str) -> list[float]:
    return point([report.get(x), report.get(y)], width, height, label, required=True)


def artifact_map(connection: sqlite3.Connection, identifier: str) -> dict[str, bytes]:
    names = list(IMAGE_ARTIFACTS.values()) + list(JSON_ARTIFACTS.values())
    placeholders = ", ".join("?" for _ in names)
    rows = connection.execute(
        f"SELECT name, data FROM placement_execution_artifacts "
        f"WHERE execution_id = ? AND name IN ({placeholders})",
        (identifier, *names),
    ).fetchall()
    return dict(rows)


def export_trace(
    database: Path,
    board_instance_id: str,
    output: Path,
    prior_notes: dict[str, str] | None = None,
    execution_ids: set[str] | None = None,
) -> dict:
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"output directory is not empty: {output}")
    uri = f"file:{database.resolve()}?mode=ro"
    samples = []
    images_to_write = []
    with sqlite3.connect(uri, uri=True) as connection:
        rows = connection.execute(
            """
            SELECT e.id, e.updated_at, e.document,
                   json_extract(f.document, '$.name')
              FROM entities AS e
              LEFT JOIN entities AS f
                ON f.kind = 'feeder'
               AND f.id = json_extract(e.document, '$.report.feeder_id')
             WHERE e.kind = 'placement_execution'
               AND json_extract(e.document, '$.report.board_instance_id') = ?
               AND EXISTS (
                   SELECT 1 FROM placement_execution_artifacts AS image
                    WHERE image.execution_id = e.id
                      AND image.name = 'board-placement-after.png'
               )
             ORDER BY e.updated_at, e.id
            """,
            (board_instance_id,),
        ).fetchall()
        if not rows:
            raise ValueError(f"no placement result images for board {board_instance_id}")
        references = set()
        for identifier, recorded_at, document, feeder_name in rows:
            if execution_ids is not None and identifier not in execution_ids:
                continue
            execution = json.loads(document)
            report = execution["report"]
            reference = report.get("placement_reference")
            if not isinstance(reference, str) or not reference.isalnum():
                raise ValueError(f"invalid placement reference for {identifier}")
            if reference in references:
                raise ValueError(f"duplicate placement reference {reference}")
            references.add(reference)
            artifacts = artifact_map(connection, identifier)
            images = {}
            for stage, artifact_name in IMAGE_ARTIFACTS.items():
                data = artifacts.get(artifact_name)
                if data is None:
                    if stage in REQUIRED_IMAGES:
                        raise ValueError(f"{reference} is missing {artifact_name}")
                    images[stage] = None
                    continue
                with Image.open(io.BytesIO(data)) as frame:
                    if frame.format != "PNG":
                        raise ValueError(f"{reference} has a non-PNG {artifact_name}")
                    width, height = frame.size
                filename = f"{reference}-{identifier}-{stage}.png"
                images[stage] = {
                    "path": f"images/{filename}", "width": width, "height": height,
                }
                images_to_write.append((filename, data))

            analyses = {}
            for stage, artifact_name in JSON_ARTIFACTS.items():
                data = artifacts.get(artifact_name)
                if data is None and stage in REQUIRED_JSON:
                    raise ValueError(f"{reference} is missing {artifact_name}")
                analyses[stage] = json.loads(data) if data is not None else None

            def center(stage: str, value: Any, label: str, required: bool = False):
                image = images[stage]
                if image is None:
                    return None
                return point(value, image["width"], image["height"], label, required)

            source = analyses["source"]
            held_initial = analyses["held_initial"]
            held_final = analyses["held_final"]
            target = report["board_target_registration"]
            result = report["top_inspection"]
            alignment = report["pickup_report"]["alignment"]
            if result.get("detected") is not True:
                raise ValueError(f"{reference} has no estimated result center")
            centers = {
                "feeder_model": center("feeder", ((source.get("prediction") or {}).get("pose") or {}).get("center_px"), "feeder model"),
                "feeder_cv": center("feeder", source.get("deterministic_center_px"), "feeder CV"),
                "held_initial_model": center("held_initial", ((held_initial.get("prediction") or {}).get("pose") or {}).get("center_px"), "initial held model"),
                "held_initial_cv": center("held_initial", held_initial.get("deterministic_center_px"), "initial held CV"),
                "held_final_model": center("held_final", ((held_final.get("prediction") or {}).get("pose") or {}).get("center_px"), "final held model") if held_final else None,
                "held_final_cv": center("held_final", held_final.get("deterministic_center_px"), "final held CV") if held_final else None,
                "target_before": report_point(target, images["target_before"]["width"], images["target_before"]["height"], "center_x_px", "center_y_px", "target before"),
                "target_after": report_point(result, images["result_after"]["width"], images["result_after"]["height"], "expected_center_x_px", "expected_center_y_px", "target after"),
                "result_after": report_point(result, images["result_after"]["width"], images["result_after"]["height"], "center_x_px", "center_y_px", "placed part"),
            }
            if centers["feeder_model"] is None and centers["feeder_cv"] is None:
                raise ValueError(f"{reference} has no feeder center")
            if centers["held_initial_cv"] is None:
                raise ValueError(f"{reference} has no initial held center")
            samples.append({
                "id": identifier,
                "reference": reference,
                "recorded_at": recorded_at,
                "status": execution["status"],
                "feeder": feeder_name,
                "pocket_index": report.get("pocket_index"),
                "images": images,
                "centers": centers,
                "source_model_controlled": source.get("control_authorized") is True,
                "source_model_cv_delta_px": source.get("center_delta_px"),
                "pickup_center_machine_mm": report["pickup_report"].get("pocket_machine_mm"),
                "held_initial_recenter_mm": [
                    alignment["observations"][0]["inspection"].get("centering_correction_x_mm"),
                    alignment["observations"][0]["inspection"].get("centering_correction_y_mm"),
                ],
                "held_alignment_command_mm": [alignment.get("commanded_x_mm"), alignment.get("commanded_y_mm")],
                "held_final_residual_mm": alignment.get("final_centering_residual_mm"),
                "held_final_model_cv_delta_px": held_final.get("center_delta_px") if held_final else None,
                "held_final_rotation_delta_deg": held_final.get("rotation_delta_deg_mod_180") if held_final else None,
                "target_recenter_mm": [target.get("correction_x_mm"), target.get("correction_y_mm")],
                "target_confidence": target.get("confidence"),
                "intended_surface_machine_mm": report["pickup_report"].get("release_surface_machine_mm"),
                "result_recenter_mm": [result.get("offset_x_mm"), result.get("offset_y_mm")],
                "result_rotation_error_deg": result.get("rotation_error_deg"),
                "inspection_passed": result.get("passed") is True,
                "prior_note": (prior_notes or {}).get(reference, ""),
            })
    if not samples:
        raise ValueError("no selected placement traces have result images")
    manifest = {
        "board_instance_id": board_instance_id,
        "coordinate_note": (
            "Image Y increases down. Stored recenter values are machine commands "
            "to move an observed feature onto its target; do not read them as "
            "signed physical part displacement."
        ),
        "samples": samples,
    }
    image_dir = output / "images"
    image_dir.mkdir(parents=True, exist_ok=True)
    for filename, data in images_to_write:
        (image_dir / filename).write_bytes(data)
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    shutil.copyfile(TEMPLATE, output / "index.html")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("database", type=Path)
    parser.add_argument("board_instance_id")
    parser.add_argument("output", type=Path)
    parser.add_argument("--prior-notes", type=Path)
    parser.add_argument("--execution-id", action="append", default=[])
    args = parser.parse_args()
    notes = json.loads(args.prior_notes.read_text(encoding="utf-8")) if args.prior_notes else None
    if notes is not None and not isinstance(notes, dict):
        parser.error("prior notes must be a JSON object keyed by reference")
    selected = set(args.execution_id) or None
    manifest = export_trace(
        args.database,
        args.board_instance_id,
        args.output,
        notes,
        selected,
    )
    print(f"Exported {len(manifest['samples'])} traces to {args.output}")


if __name__ == "__main__":
    main()
