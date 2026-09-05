#!/usr/bin/env python3
"""Validate placement review decisions and write immutable ground truth."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


CLASS_NAMES = (
    "component_flat",
    "component_on_edge",
    "multiple_components",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("review", type=Path)
    parser.add_argument(
        "--evaluation-capture",
        action="append",
        default=[],
        help="source capture kept out of training; repeat as needed",
    )
    return parser.parse_args()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def resolve_review_path(dataset: Path, review: Path) -> Path:
    return review if review.is_absolute() else dataset / review


def validate_outline(
    sample_id: str,
    outline: Any,
    width: int,
    height: int,
) -> list[list[float]]:
    if not isinstance(outline, list) or len(outline) != 4:
        raise ValueError(f"sample {sample_id} needs a four-point outline")
    points: list[list[float]] = []
    for point in outline:
        if not isinstance(point, list) or len(point) != 2:
            raise ValueError(f"sample {sample_id} has an invalid outline point")
        converted = [float(point[0]), float(point[1])]
        if not all(math.isfinite(axis) for axis in converted):
            raise ValueError(f"sample {sample_id} has a non-finite outline")
        if not 0.0 <= converted[0] < width:
            raise ValueError(f"sample {sample_id} outline is outside the image")
        if not 0.0 <= converted[1] < height:
            raise ValueError(f"sample {sample_id} outline is outside the image")
        points.append(converted)

    edges = [
        [
            points[(index + 1) % 4][axis] - points[index][axis]
            for axis in range(2)
        ]
        for index in range(4)
    ]
    lengths = [math.hypot(*edge) for edge in edges]
    if min(lengths) < 4.0:
        raise ValueError(f"sample {sample_id} outline is too small")
    for first, second in ((0, 2), (1, 3)):
        relative_error = abs(lengths[first] - lengths[second]) / max(
            lengths[first], lengths[second]
        )
        if relative_error > 0.02:
            raise ValueError(f"sample {sample_id} outline is not rectangular")
    for index in range(4):
        adjacent = edges[(index + 1) % 4]
        dot = sum(edges[index][axis] * adjacent[axis] for axis in range(2))
        if abs(dot) / (lengths[index] * lengths[(index + 1) % 4]) > 0.02:
            raise ValueError(f"sample {sample_id} outline is not rectangular")
    signed_area = 0.5 * sum(
        points[index][0] * points[(index + 1) % 4][1]
        - points[(index + 1) % 4][0] * points[index][1]
        for index in range(4)
    )
    if abs(signed_area) < 16.0:
        raise ValueError(f"sample {sample_id} outline has no area")
    return points


def validate_review(
    draft: dict[str, Any], review: dict[str, Any]
) -> dict[str, dict[str, Any]]:
    if review.get("schema_version") != 1:
        raise ValueError("unsupported review schema")
    if review.get("dataset") != draft.get("name"):
        raise ValueError("review is for another dataset")
    if review.get("source_manifest_generated_at") != draft.get("generated_at"):
        raise ValueError("review is for another draft generation")
    decisions = review.get("decisions")
    if not isinstance(decisions, dict):
        raise ValueError("review decisions are absent")
    expected = {sample["id"] for sample in draft["samples"]}
    if set(decisions) != expected:
        missing = sorted(expected - set(decisions))
        extra = sorted(set(decisions) - expected)
        raise ValueError(f"review set differs: missing={missing}, extra={extra}")
    return decisions


def finalize_sample(
    root: Path,
    sample: dict[str, Any],
    decision: dict[str, Any],
    evaluation_captures: set[str],
    registration_id: str,
) -> dict[str, Any]:
    result = decision.get("decision")
    if result not in {"accept", "reject"}:
        raise ValueError(f'sample {sample["id"]} is not accepted or rejected')
    class_name = decision.get("class_name")
    if class_name == "empty":
        resolved_class = None
        if decision.get("outline_px") is not None:
            raise ValueError(f'sample {sample["id"]} empty label has an outline')
        outline = None
    else:
        if class_name not in CLASS_NAMES:
            raise ValueError(
                f'sample {sample["id"]} has unsupported class {class_name!r}'
            )
        resolved_class = class_name
        outline = validate_outline(
            sample["id"],
            decision.get("outline_px"),
            sample["width_px"],
            sample["height_px"],
        )
    image_path = root / sample["image_path"]
    if not image_path.is_file():
        raise ValueError(f'sample {sample["id"]} image is absent')
    if sha256_file(image_path) != sample["image_sha256"]:
        raise ValueError(f'sample {sample["id"]} image changed after review')
    source_capture = sample["source_capture"]
    split = "eval" if source_capture in evaluation_captures else "train"
    return {
        **sample,
        "split": split,
        "group_id": f"{registration_id}:{source_capture}",
        "included": result == "accept",
        "class_name": resolved_class,
        "outline_px": outline,
        "label_source": "operator_review",
        "review_note": str(decision.get("note", "")),
    }


def main() -> None:
    args = parse_args()
    root = args.dataset.resolve()
    output = root / "manifest.ground-truth.json"
    if output.exists():
        raise ValueError(f"ground-truth manifest already exists: {output}")
    draft_path = root / "manifest.draft.json"
    review_path = resolve_review_path(root, args.review)
    draft_bytes = draft_path.read_bytes()
    review_bytes = review_path.read_bytes()
    draft = json.loads(draft_bytes)
    review = json.loads(review_bytes)
    decisions = validate_review(draft, review)
    evaluation_captures = set(args.evaluation_capture)
    available_captures = {
        sample["source_capture"] for sample in draft["samples"]
    }
    unknown_captures = evaluation_captures - available_captures
    if unknown_captures:
        raise ValueError(
            "unknown evaluation captures: " + ", ".join(sorted(unknown_captures))
        )
    registration_id = draft["registration_id"]
    samples = [
        finalize_sample(
            root,
            sample,
            decisions[sample["id"]],
            evaluation_captures,
            registration_id,
        )
        for sample in draft["samples"]
    ]
    for split in ("train", "eval"):
        has_samples = any(
            sample["included"] and sample["split"] == split
            for sample in samples
        )
        if not has_samples:
            raise ValueError(f"{split} split is empty")
    groups_by_split = {
        split: {
            sample["group_id"]
            for sample in samples
            if sample["included"] and sample["split"] == split
        }
        for split in ("train", "eval")
    }
    if groups_by_split["train"] & groups_by_split["eval"]:
        raise ValueError("one source capture occurs in both splits")
    review_copy = root / "review-decisions.json"
    review_copy.write_bytes(review_bytes)
    present_classes = {
        sample["class_name"]
        for sample in samples
        if sample["included"] and sample["class_name"] is not None
    }
    ground_truth = {
        "schema_version": 1,
        "name": f'{draft["name"]}-ground-truth',
        "generated_at": datetime.now(UTC).isoformat(),
        "source_draft_manifest": draft_path.name,
        "source_draft_manifest_sha256": sha256_bytes(draft_bytes),
        "source_review": review_copy.name,
        "source_review_sha256": sha256_bytes(review_bytes),
        "registration_id": registration_id,
        "board_id": draft["board_id"],
        "calibration_id": draft["calibration_id"],
        "classes": list(CLASS_NAMES),
        "evaluation_capture_names": sorted(evaluation_captures),
        "split_policy": "source camera capture groups do not cross splits",
        "normalization": draft["normalization"],
        "notes": [
            "every included label passed explicit operator review",
            "empty targets remain included with no package outline",
            "rejected images remain as excluded provenance records",
            "this first board session is not an independent-machine gate",
        ],
        "training_class_limitations": sorted(set(CLASS_NAMES) - present_classes),
        "samples": samples,
    }
    output.write_text(json.dumps(ground_truth, indent=2) + "\n")
    counts = {
        name: sum(
            sample["included"] and sample["class_name"] == name
            for sample in samples
        )
        for name in (None, *CLASS_NAMES)
    }
    print(
        json.dumps(
            {
                "output": str(output),
                "train": sum(
                    sample["included"] and sample["split"] == "train"
                    for sample in samples
                ),
                "eval": sum(
                    sample["included"] and sample["split"] == "eval"
                    for sample in samples
                ),
                "empty": counts[None],
                **{name: counts[name] for name in CLASS_NAMES},
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
