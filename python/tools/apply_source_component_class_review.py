#!/usr/bin/env python3
"""Apply a focused class audit to a copy of a reviewed source dataset."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import UTC, datetime
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
from typing import Any


CLASS_NAMES = ("component_flat", "component_on_edge")
CHOICES = (*CLASS_NAMES, "empty", "reject")


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def read_json(path: Path) -> tuple[dict[str, Any], bytes]:
    data = path.read_bytes()
    return json.loads(data), data


def apply_review(
    source: Path,
    review_path: Path,
    output: Path,
) -> dict[str, Any]:
    source = source.resolve()
    review_path = review_path.resolve()
    output = output.resolve()
    if output.exists():
        raise ValueError(f"output already exists: {output}")
    if not output.parent.is_dir():
        raise ValueError(f"output parent does not exist: {output.parent}")

    manifest, manifest_bytes = read_json(source / "manifest.ground-truth.json")
    review, review_bytes = read_json(review_path)
    audited_class = review.get("audited_class_name")
    if audited_class not in CLASS_NAMES:
        raise ValueError("review has an invalid audited class")
    decisions = review.get("decisions")
    if not isinstance(decisions, dict):
        raise ValueError("review decisions must be an object")

    expected_ids = {
        sample["id"]
        for sample in manifest.get("samples", [])
        if sample.get("included") and sample.get("class_name") == audited_class
    }
    decision_ids = set(decisions)
    missing = sorted(expected_ids - decision_ids)
    extra = sorted(decision_ids - expected_ids)
    if missing or extra:
        raise ValueError(
            f"review sample mismatch: {len(missing)} missing, {len(extra)} extra"
        )

    staged = Path(tempfile.mkdtemp(prefix=f".{output.name}-", dir=output.parent))
    changes: Counter[str] = Counter()
    try:
        samples = []
        for original in manifest["samples"]:
            sample = dict(original)
            decision = decisions.get(sample["id"])
            if decision is not None:
                choice = decision.get("choice")
                if choice not in CHOICES:
                    raise ValueError(
                        f"sample {sample['id']} has invalid choice {choice!r}"
                    )
                old_class = sample.get("class_name") or "empty"
                if choice in CLASS_NAMES:
                    outline = decision.get("outline_px")
                    if not isinstance(outline, list) or len(outline) != 4:
                        raise ValueError(f"sample {sample['id']} has no box")
                    sample["included"] = True
                    sample["class_name"] = choice
                    sample["outline_px"] = outline
                    new_class = choice
                elif choice == "empty":
                    sample["included"] = True
                    sample["class_name"] = None
                    sample["outline_px"] = None
                    new_class = "empty"
                else:
                    sample["included"] = False
                    sample["class_name"] = None
                    sample["outline_px"] = None
                    new_class = "rejected"
                sample["label_source"] = "focused_class_review"
                sample["review_note"] = (
                    f"focused audit changed {old_class} to {new_class}"
                    if old_class != new_class
                    else f"focused audit retained {new_class}"
                )
                changes[f"{old_class}->{new_class}"] += 1
            samples.append(sample)

        for sample in samples:
            relative = Path(sample["image_path"])
            source_image = (source / relative).resolve()
            if not source_image.is_relative_to(source):
                raise ValueError(f"image path leaves dataset: {relative}")
            destination = staged / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_image, destination)

        corrected = dict(manifest)
        corrected.update(
            {
                "name": "velocitypnp-source-components-class-corrected",
                "generated_at": datetime.now(UTC).isoformat(),
                "source_manifest_sha256": sha256(manifest_bytes),
                "class_review_sha256": sha256(review_bytes),
                "samples": samples,
            }
        )
        notes = list(manifest.get("notes", []))
        notes.append(
            f"all included {audited_class} labels received a focused operator audit"
        )
        corrected["notes"] = notes
        (staged / "manifest.ground-truth.json").write_text(
            json.dumps(corrected, indent=2) + "\n"
        )
        shutil.copy2(source / "manifest.draft.json", staged / "manifest.draft.json")
        (staged / "source-component-class-review-decisions.json").write_bytes(
            review_bytes
        )
        staged.rename(output)
    except BaseException:
        shutil.rmtree(staged)
        raise

    counts = {
        split: Counter(
            (sample.get("class_name") or "empty")
            for sample in samples
            if sample.get("included") and sample.get("split") == split
        )
        for split in ("train", "eval")
    }
    return {
        "output": str(output),
        "reviewed": len(decisions),
        "changes": dict(sorted(changes.items())),
        "included": {split: dict(counts[split]) for split in counts},
        "source_manifest_sha256": sha256(manifest_bytes),
        "class_review_sha256": sha256(review_bytes),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("review", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    print(
        json.dumps(
            apply_review(arguments.source, arguments.review, arguments.output),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
