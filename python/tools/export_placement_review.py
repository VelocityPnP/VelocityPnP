#!/usr/bin/env python3
"""Export a static review page for one board's retained placements."""

from __future__ import annotations

import argparse
import io
import json
import math
import shutil
import sqlite3
from pathlib import Path

from PIL import Image


TEMPLATE = Path(__file__).with_name("placement_review.html")


def finite_coordinate(inspection: dict, key: str, limit: int) -> float:
    value = inspection.get(key)
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"missing or invalid {key}")
    if not 0 <= value < limit:
        raise ValueError(f"{key} is outside the image")
    return float(value)


def export_review(database: Path, board_instance_id: str, output: Path) -> dict:
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"output directory is not empty: {output}")
    uri = f"file:{database.resolve()}?mode=ro"
    with sqlite3.connect(uri, uri=True) as connection:
        rows = connection.execute(
            """
            SELECT execution.id, execution.updated_at, execution.document,
                   image.data
              FROM entities AS execution
              JOIN placement_execution_artifacts AS image
                ON image.execution_id = execution.id
               AND image.name = 'board-placement-after.png'
             WHERE execution.kind = 'placement_execution'
               AND json_extract(
                   execution.document, '$.report.board_instance_id'
               ) = ?
             ORDER BY execution.updated_at, execution.id
            """,
            (board_instance_id,),
        ).fetchall()
    if not rows:
        raise ValueError(f"no result images for board instance {board_instance_id}")

    samples = []
    images = []
    references = set()
    for identifier, recorded_at, document, data in rows:
        execution = json.loads(document)
        report = execution["report"]
        inspection = report.get("top_inspection") or {}
        reference = report.get("placement_reference")
        if not isinstance(reference, str) or not reference.isalnum():
            raise ValueError(f"invalid placement reference: {identifier}")
        if reference in references:
            raise ValueError(f"more than one result image for {reference}")
        references.add(reference)
        with Image.open(io.BytesIO(data)) as frame:
            if frame.format != "PNG":
                raise ValueError(f"result image is not PNG: {identifier}")
            width, height = frame.size
        target = [
            finite_coordinate(inspection, "expected_center_x_px", width),
            finite_coordinate(inspection, "expected_center_y_px", height),
        ]
        detected = inspection.get("detected") is True
        estimated = (
            [
                finite_coordinate(inspection, "center_x_px", width),
                finite_coordinate(inspection, "center_y_px", height),
            ]
            if detected
            else None
        )
        image_name = f"{reference}-{identifier}.png"
        images.append((image_name, data))
        samples.append(
            {
                "id": identifier,
                "reference": reference,
                "recorded_at": recorded_at,
                "status": execution["status"],
                "inspection_passed": inspection.get("passed") is True,
                "part_detected": detected,
                "image": f"images/{image_name}",
                "width": width,
                "height": height,
                "target_px": target,
                "estimated_px": estimated,
                "offset_x_mm": inspection.get("offset_x_mm"),
                "offset_y_mm": inspection.get("offset_y_mm"),
                "offset_mm": inspection.get("offset_mm"),
                "rotation_error_deg": inspection.get("rotation_error_deg"),
            }
        )

    manifest = {
        "board_instance_id": board_instance_id,
        "note": (
            "Target is the locally registered pad center in the after-image "
            "frame. Estimated center is from stored top-camera inspection; "
            "it is not an independent measurement."
        ),
        "samples": samples,
    }
    image_dir = output / "images"
    image_dir.mkdir(parents=True, exist_ok=True)
    for image_name, data in images:
        (image_dir / image_name).write_bytes(data)
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    shutil.copyfile(TEMPLATE, output / "index.html")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("database", type=Path)
    parser.add_argument("board_instance_id")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    manifest = export_review(args.database, args.board_instance_id, args.output)
    print(f"Exported {len(manifest['samples'])} placements to {args.output}")


if __name__ == "__main__":
    main()
