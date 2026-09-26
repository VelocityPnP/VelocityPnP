"""Combine reviewed feeder data with a class-complete validation group."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import tempfile
from collections import Counter, defaultdict
from datetime import UTC, datetime
from pathlib import Path


CLASSES = ("empty", "component_flat", "component_on_edge")


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def read_manifest(root: Path, name: str) -> tuple[dict, bytes]:
    data = (root / name).read_bytes()
    return json.loads(data), data


def draft_groups(root: Path) -> dict[str, str]:
    draft, _ = read_manifest(root, "manifest.draft.json")
    return {sample["id"]: sample["group_id"] for sample in draft["samples"]}


def checked_image(root: Path, image_path: str) -> bytes:
    path = (root / image_path).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"image path leaves dataset: {image_path}")
    return path.read_bytes()


def build(base: Path, recent: Path, output: Path, eval_group: str) -> dict:
    base = base.resolve()
    recent = recent.resolve()
    output = output.resolve()
    if output.exists():
        raise ValueError(f"output already exists: {output}")
    if not output.parent.is_dir():
        raise ValueError(f"output parent does not exist: {output.parent}")

    base_manifest, base_bytes = read_manifest(base, "manifest.ground-truth.json")
    recent_manifest, recent_bytes = read_manifest(
        recent, "manifest.ground-truth.json"
    )
    if base_manifest["classes"] != recent_manifest["classes"]:
        raise ValueError("source datasets use different classes")
    if tuple(base_manifest["classes"]) != CLASSES[1:]:
        raise ValueError("source classes do not match the model contract")
    base_groups = draft_groups(base)
    recent_groups = draft_groups(recent)
    selected = [
        sample
        for sample in base_manifest["samples"]
        if base_groups.get(sample["id"]) == eval_group
    ]
    if not selected or any(sample["split"] != "train" for sample in selected):
        raise ValueError("validation group must exist entirely in base train")
    if any(sample["split"] != "eval" for sample in recent_manifest["samples"]):
        raise ValueError("recent dataset must be evaluation-only")
    if len(set(recent_groups.values())) != 1:
        raise ValueError("recent dataset must contain one capture group")

    staged = Path(tempfile.mkdtemp(prefix=f".{output.name}-", dir=output.parent))
    try:
        samples: list[dict] = []
        seen_ids: set[str] = set()
        image_splits: dict[str, set[str]] = defaultdict(set)
        group_splits: dict[str, set[str]] = defaultdict(set)
        for source_name, root, manifest, groups in (
            ("base", base, base_manifest, base_groups),
            ("recent", recent, recent_manifest, recent_groups),
        ):
            for original in manifest["samples"]:
                sample = dict(original)
                sample_id = sample["id"]
                if sample_id in seen_ids:
                    raise ValueError(f"duplicate sample id: {sample_id}")
                seen_ids.add(sample_id)
                group_id = groups.get(sample_id)
                if source_name == "recent":
                    sample["split"] = "train"
                elif group_id == eval_group:
                    sample["split"] = "eval"
                split = sample["split"]
                image = checked_image(root, sample["image_path"])
                image_sha256 = sha256(image)
                relative = Path("images") / split / f"{sample_id}.png"
                destination = staged / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(image)
                sample["image_path"] = relative.as_posix()
                sample["source_dataset"] = source_name
                sample["source_image_sha256"] = image_sha256
                if group_id is not None:
                    sample["source_group_id"] = group_id
                    group_splits[group_id].add(split)
                image_splits[image_sha256].add(split)
                samples.append(sample)

        crossed_groups = [
            group for group, splits in group_splits.items() if len(splits) != 1
        ]
        if crossed_groups:
            raise ValueError(f"capture groups cross splits: {crossed_groups}")
        crossed_images = [
            digest for digest, splits in image_splits.items() if len(splits) != 1
        ]
        if crossed_images:
            raise ValueError(f"identical images cross splits: {crossed_images}")

        counts = {
            split: Counter(
                sample["class_name"] or "empty"
                for sample in samples
                if sample["included"] and sample["split"] == split
            )
            for split in ("train", "eval")
        }
        if any(counts[split][name] == 0 for split in counts for name in CLASSES):
            raise ValueError(f"each split must contain every class: {counts}")
        output_manifest = {
            "schema_version": 1,
            "name": "velocitypnp-source-components-retrain-reviewed",
            "generated_at": datetime.now(UTC).isoformat(),
            "classes": base_manifest["classes"],
            "source_manifests": [
                {"dataset": "base", "sha256": sha256(base_bytes)},
                {"dataset": "recent", "sha256": sha256(recent_bytes)},
            ],
            "split_policy": (
                "keep source capture groups together; move the selected marked "
                "group to evaluation and the recent capture to training"
            ),
            "evaluation_group_id": eval_group,
            "notes": [
                "all labels come from finalized operator reviews",
                "recent images are no longer an independent model test",
                "collect a new independent capture before deployment",
            ],
            "samples": samples,
        }
        (staged / "manifest.ground-truth.json").write_text(
            json.dumps(output_manifest, indent=2) + "\n"
        )
        staged.rename(output)
    except BaseException:
        shutil.rmtree(staged)
        raise
    return {
        "output": str(output),
        "samples": len(samples),
        "included": {split: dict(counts[split]) for split in counts},
        "validation_group": eval_group,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("base", type=Path)
    parser.add_argument("recent", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--eval-group", required=True)
    arguments = parser.parse_args()
    print(
        json.dumps(
            build(
                arguments.base,
                arguments.recent,
                arguments.output,
                arguments.eval_group,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
