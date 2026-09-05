#!/usr/bin/env python3
"""Move reviewed outlines into a corrected placement crop basis."""

from __future__ import annotations

import argparse
import json
import math
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_ground_truth", type=Path)
    parser.add_argument("target_draft", type=Path)
    parser.add_argument("output", type=Path)
    return parser.parse_args()


def remap_point(
    point: list[float],
    center: list[float],
    source_rotation_deg: float,
    target_rotation_deg: float,
) -> list[float]:
    """Keep one physical point fixed while the crop basis rotates."""

    angle = math.radians(source_rotation_deg - target_rotation_deg)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    x = float(point[0]) - center[0]
    y = float(point[1]) - center[1]
    return [
        center[0] + cosine * x - sine * y,
        center[1] + sine * x + cosine * y,
    ]


def migrate_review(
    source: dict[str, Any], target: dict[str, Any]
) -> dict[str, Any]:
    source_samples = {sample["id"]: sample for sample in source["samples"]}
    target_samples = {sample["id"]: sample for sample in target["samples"]}
    if source_samples.keys() != target_samples.keys():
        raise ValueError("source and target sample IDs differ")
    source_normalization = source["normalization"]
    target_normalization = target["normalization"]
    if source_normalization["output_span_mm"] != target_normalization[
        "output_span_mm"
    ]:
        raise ValueError("source and target crop spans differ")
    center = [float(value) for value in target_normalization["target_center_px"]]
    decisions = {}
    for sample_id, new in target_samples.items():
        old = source_samples[sample_id]
        if not old.get("included"):
            raise ValueError(f"source sample {sample_id} was not included")
        if old["reference"] != new["reference"]:
            raise ValueError(f"sample {sample_id} changed reference")
        if any(
            abs(float(first) - float(second)) > 1e-6
            for first, second in zip(
                old["target_machine_mm"], new["target_machine_mm"], strict=True
            )
        ):
            raise ValueError(f"sample {sample_id} changed machine target")
        outline = old.get("outline_px")
        if outline is not None:
            outline = [
                remap_point(
                    point,
                    center,
                    float(old["target_rotation_deg"]),
                    float(new["target_rotation_deg"]),
                )
                for point in outline
            ]
        decisions[sample_id] = {
            "decision": "accept",
            "note": old.get("review_note", ""),
            "class_name": old.get("class_name") or "empty",
            "outline_px": outline,
        }
    return {
        "schema_version": 1,
        "dataset": target["name"],
        "exported_at": datetime.now(UTC).isoformat(),
        "source_manifest_generated_at": target["generated_at"],
        "decisions": decisions,
    }


def main() -> None:
    args = parse_args()
    source = json.loads(args.source_ground_truth.read_text())
    target = json.loads(args.target_draft.read_text())
    migrated = migrate_review(source, target)
    args.output.write_text(json.dumps(migrated, indent=2) + "\n")
    print(
        json.dumps(
            {
                "output": str(args.output),
                "sample_count": len(migrated["decisions"]),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
