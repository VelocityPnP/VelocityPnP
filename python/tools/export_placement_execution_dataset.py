#!/usr/bin/env python3
"""Export review crops from retained physical placement executions."""

from __future__ import annotations

import argparse
import io
import json
import math
import sqlite3
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from PIL import Image

from export_placement_verification_snapshot import (
    OUTPUT_SIZE,
    OUTPUT_SPAN_MM,
    normalized_crop_at_pixel,
    render_contact_sheet,
    sha256_file,
)


AFTER_STATES = {
    "good": "component_flat",
    "offset": "component_flat",
    "rotated": "component_flat",
    "tombstoned": "component_on_edge",
    "stacked": "multiple_components",
    "missing": "empty",
    "stuck_on_nozzle": "empty",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("database", type=Path)
    parser.add_argument("output", type=Path)
    return parser.parse_args()


def finite_number(value: Any, name: str) -> float:
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} is absent or invalid")
    return float(value)


def entity(
    connection: sqlite3.Connection, kind: str, entity_id: str
) -> dict[str, Any]:
    row = connection.execute(
        "SELECT document FROM entities WHERE kind = ? AND id = ?",
        (kind, entity_id),
    ).fetchone()
    if row is None:
        raise ValueError(f"{kind} {entity_id} does not exist")
    return json.loads(row[0])


def top_camera_mapping(calibration: dict[str, Any]) -> list[list[float]]:
    value = calibration["artifacts"]["top_camera_working_plane_map"]["value"]
    mapping = value.get("result", value)["pixel_error_to_machine_mm"]
    if (
        not isinstance(mapping, list)
        or len(mapping) != 2
        or any(not isinstance(row, list) or len(row) != 2 for row in mapping)
    ):
        raise ValueError("top-camera mapping has an invalid shape")
    result = [
        [finite_number(value, "top-camera mapping") for value in row]
        for row in mapping
    ]
    determinant = (
        result[0][0] * result[1][1] - result[0][1] * result[1][0]
    )
    if abs(determinant) < 1e-12:
        raise ValueError("top-camera mapping is singular")
    return result


def resolve_board_id(
    connection: sqlite3.Connection, execution: dict[str, Any]
) -> str:
    job = entity(connection, "job", execution["job_id"])
    board_instance_id = execution["report"].get("board_instance_id")
    boards = job.get("boards", [])
    if board_instance_id is not None:
        boards = [board for board in boards if board.get("id") == board_instance_id]
    if len(boards) != 1 or not isinstance(boards[0].get("board_id"), str):
        raise ValueError("execution does not resolve to one job board")
    return boards[0]["board_id"]


def disposition_state(
    outcome: dict[str, Any] | None,
    inspection: dict[str, Any],
    phase: str,
) -> tuple[str, str]:
    operator = outcome.get("operator_disposition") if outcome else None
    automatic = outcome.get("automatic_disposition") if outcome else None
    disposition = operator or automatic
    source = "operator_disposition" if operator else "automatic_disposition"
    if phase == "before":
        if operator == "stacked":
            return "component_flat", "operator_stacked_prior_component"
        return "empty", "placement_target_before_contact"
    if disposition in AFTER_STATES:
        return AFTER_STATES[disposition], source
    detected = inspection.get("detected") is True
    return (
        ("component_flat", "deterministic_detection")
        if detected
        else ("empty", "deterministic_non_detection")
    )


def rotated_outline(
    length_mm: float,
    width_mm: float,
    center_x_px: float,
    center_y_px: float,
    rotation_deg: float,
) -> list[list[float]]:
    scale = OUTPUT_SIZE / OUTPUT_SPAN_MM
    half_length = 0.5 * length_mm * scale
    half_width = 0.5 * width_mm * scale
    radians = math.radians(rotation_deg)
    along = [math.cos(radians), math.sin(radians)]
    across = [-along[1], along[0]]
    return [
        [
            center_x_px + along[0] * x + across[0] * y,
            center_y_px + along[1] * x + across[1] * y,
        ]
        for x, y in (
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width),
        )
    ]


def draft_outline(
    state: str,
    inspection: dict[str, Any],
    length_mm: float,
    width_mm: float,
    target_rotation_deg: float,
) -> list[list[float]] | None:
    if state == "empty":
        return None
    center_x = center_y = OUTPUT_SIZE / 2.0
    offset_x = inspection.get("offset_x_mm")
    offset_y = inspection.get("offset_y_mm")
    if all(isinstance(value, (int, float)) for value in (offset_x, offset_y)):
        radians = math.radians(target_rotation_deg)
        local_x = math.cos(radians) * offset_x + math.sin(radians) * offset_y
        local_y = -math.sin(radians) * offset_x + math.cos(radians) * offset_y
        scale = OUTPUT_SIZE / OUTPUT_SPAN_MM
        center_x += local_x * scale
        center_y += local_y * scale
    rotation = inspection.get("rotation_error_deg")
    if not isinstance(rotation, (int, float)) or not math.isfinite(rotation):
        rotation = 0.0
    return rotated_outline(
        length_mm, width_mm, center_x, center_y, float(rotation)
    )


def execution_rows(connection: sqlite3.Connection) -> list[sqlite3.Row]:
    return connection.execute(
        """
        SELECT execution.document AS execution_document,
               outcome.document AS outcome_document,
               before_image.data AS before_data,
               after_image.data AS after_data
          FROM entities AS execution
          JOIN placement_execution_artifacts AS before_image
            ON before_image.execution_id = execution.id
           AND before_image.name = 'board-placement-before.png'
          JOIN placement_execution_artifacts AS after_image
            ON after_image.execution_id = execution.id
           AND after_image.name = 'board-placement-after.png'
          LEFT JOIN entities AS outcome
            ON outcome.kind = 'placement_outcome'
           AND outcome.id = execution.id
         WHERE execution.kind = 'placement_execution'
           AND json_extract(execution.document, '$.report.simulated') = 0
         ORDER BY execution.updated_at, execution.id
        """
    ).fetchall()


def prepare_output(output: Path) -> Path:
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"output directory is not empty: {output}")
    images = output / "images"
    images.mkdir(parents=True, exist_ok=True)
    return images


def export_dataset(
    database: Path,
    output: Path,
    generated_at: str | None = None,
) -> dict[str, Any]:
    images = prepare_output(output)
    connection = sqlite3.connect(database)
    connection.row_factory = sqlite3.Row
    calibration_cache: dict[str, list[list[float]]] = {}
    samples: list[dict[str, Any]] = []
    skipped: list[dict[str, str]] = []
    board_ids: set[str] = set()
    calibration_ids: set[str] = set()
    board_instances: set[str] = set()
    try:
        for row in execution_rows(connection):
            execution = json.loads(row["execution_document"])
            report = execution.get("report", {})
            execution_id = str(execution.get("id", "unknown"))
            reference = str(report.get("placement_reference", "unknown"))
            try:
                inspection = report["top_inspection"]
                center = [
                    finite_number(
                        inspection.get("expected_center_x_px"),
                        "expected target X",
                    ),
                    finite_number(
                        inspection.get("expected_center_y_px"),
                        "expected target Y",
                    ),
                ]
                target_rotation_deg = finite_number(
                    inspection.get("expected_rotation_deg"),
                    "expected target rotation",
                )
                width_mm = finite_number(
                    inspection.get("expected_width_mm"), "expected width"
                )
                length_mm = finite_number(
                    inspection.get("expected_length_mm"), "expected length"
                )
                calibration_id = str(execution["calibration_id"])
                if calibration_id not in calibration_cache:
                    calibration_cache[calibration_id] = top_camera_mapping(
                        entity(connection, "calibration", calibration_id)
                    )
                mapping = calibration_cache[calibration_id]
                board_id = resolve_board_id(connection, execution)
                outcome = (
                    json.loads(row["outcome_document"])
                    if row["outcome_document"]
                    else None
                )
                target = report["board_place"]
                target_machine_mm = [
                    finite_number(target.get("x_mm"), "board target X"),
                    finite_number(target.get("y_mm"), "board target Y"),
                ]
                package = (
                    outcome.get("package") if outcome else None
                ) or str(report.get("part_key", "unknown")).rsplit(":", 1)[0]
                for phase, data in (
                    ("before", row["before_data"]),
                    ("after", row["after_data"]),
                ):
                    with Image.open(io.BytesIO(data)) as source:
                        crop = normalized_crop_at_pixel(
                            source.convert("L"),
                            center,
                            mapping,
                            math.radians(target_rotation_deg),
                        )
                    image_path = images / f"{execution_id}-{phase}.png"
                    crop.save(image_path)
                    state, label_source = disposition_state(
                        outcome, inspection, phase
                    )
                    samples.append(
                        {
                            "id": f"placement_execution-{execution_id}-{phase}",
                            "group_id": f"placement_execution-{execution_id}",
                            "reference": reference,
                            "package": package,
                            "image_path": image_path.relative_to(output).as_posix(),
                            "image_sha256": sha256_file(image_path),
                            "width_px": OUTPUT_SIZE,
                            "height_px": OUTPUT_SIZE,
                            "target_machine_mm": target_machine_mm,
                            "target_rotation_deg": target_rotation_deg,
                            "source_capture": execution_id,
                            "source_execution_id": execution_id,
                            "source_state": phase,
                            "source_created_at": execution.get("created_at"),
                            "draft_state": state,
                            "draft_label_source": label_source,
                            "draft_outline_px": draft_outline(
                                state,
                                inspection,
                                length_mm,
                                width_mm,
                                target_rotation_deg,
                            ),
                            "expected_size_mm": [width_mm, length_mm],
                            "operator_disposition": (
                                outcome.get("operator_disposition")
                                if outcome
                                else None
                            ),
                            "automatic_disposition": (
                                outcome.get("automatic_disposition")
                                if outcome
                                else None
                            ),
                            "operator_note": (
                                outcome.get("operator_note") if outcome else None
                            ),
                        }
                    )
                board_ids.add(board_id)
                calibration_ids.add(calibration_id)
                instance = report.get("board_instance_id")
                if isinstance(instance, str):
                    board_instances.add(instance)
            except (KeyError, TypeError, ValueError, OSError) as error:
                skipped.append(
                    {
                        "execution_id": execution_id,
                        "reference": reference,
                        "reason": str(error),
                    }
                )
    finally:
        connection.close()

    if not samples:
        raise ValueError("no usable physical placement executions were found")
    if len(board_ids) != 1:
        raise ValueError(f"executions span {len(board_ids)} boards")
    if len(calibration_ids) != 1:
        raise ValueError(f"executions span {len(calibration_ids)} calibrations")
    timestamp = generated_at or datetime.now(UTC).isoformat()
    manifest = {
        "schema_version": 1,
        "name": output.name,
        "generated_at": timestamp,
        "registration_id": "placement-execution-history",
        "board_id": next(iter(board_ids)),
        "calibration_id": next(iter(calibration_ids)),
        "source_board_instance_ids": sorted(board_instances),
        "normalization": {
            "output_size_px": OUTPUT_SIZE,
            "output_span_mm": OUTPUT_SPAN_MM,
            "target_center_px": [OUTPUT_SIZE / 2.0, OUTPUT_SIZE / 2.0],
            "target_long_axis": "positive_x",
        },
        "label_policy": (
            "all labels are weak proposals from retained execution evidence; "
            "operator review is required"
        ),
        "samples": samples,
        "skipped": skipped,
    }
    (output / "manifest.draft.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    template = Path(__file__).with_name(
        "placement_verification_review.html"
    ).read_text()
    review = template.replace("__MANIFEST__", json.dumps(manifest)).replace(
        "__DATASET_TITLE__", json.dumps(output.name)
    )
    (output / "index.html").write_text(review)
    render_contact_sheet(output, samples)
    return manifest


def main() -> None:
    args = parse_args()
    manifest = export_dataset(args.database, args.output)
    counts = {
        state: sum(sample["draft_state"] == state for sample in manifest["samples"])
        for state in (
            "empty",
            "component_flat",
            "component_on_edge",
            "multiple_components",
        )
    }
    print(
        json.dumps(
            {
                "output": str(args.output),
                "sample_count": len(manifest["samples"]),
                "draft_counts": counts,
                "skipped_count": len(manifest["skipped"]),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
