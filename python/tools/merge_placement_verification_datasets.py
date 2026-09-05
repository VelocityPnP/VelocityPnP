#!/usr/bin/env python3
"""Merge reviewed placement datasets without capture or target leakage."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
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
    parser.add_argument("output", type=Path)
    parser.add_argument("sources", type=Path, nargs="+")
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare_output(output: Path) -> Path:
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"output directory is not empty: {output}")
    images = output / "images"
    images.mkdir(parents=True, exist_ok=True)
    return images


class Groups:
    def __init__(self, size: int) -> None:
        self.parent = list(range(size))

    def find(self, item: int) -> int:
        while self.parent[item] != item:
            self.parent[item] = self.parent[self.parent[item]]
            item = self.parent[item]
        return item

    def union(self, first: int, second: int) -> None:
        first_root = self.find(first)
        second_root = self.find(second)
        if first_root != second_root:
            self.parent[second_root] = first_root


def validated_manifest(root: Path) -> dict[str, Any]:
    path = root / "manifest.ground-truth.json"
    if not path.is_file():
        raise ValueError(f"missing {path}")
    manifest = json.loads(path.read_text())
    if tuple(manifest.get("classes", [])) != CLASS_NAMES:
        raise ValueError(f"class contract differs in {path}")
    return manifest


def merge_datasets(
    sources: list[Path],
    output: Path,
    generated_at: str | None = None,
) -> dict[str, Any]:
    if len(sources) < 2:
        raise ValueError("at least two source datasets are required")
    images = prepare_output(output)
    loaded = [(root.resolve(), validated_manifest(root.resolve())) for root in sources]
    normalizations = [manifest.get("normalization") for _, manifest in loaded]
    if any(value != normalizations[0] for value in normalizations[1:]):
        raise ValueError("source normalization contracts differ")

    records: list[dict[str, Any]] = []
    source_manifests = []
    for source_index, (root, manifest) in enumerate(loaded):
        manifest_path = root / "manifest.ground-truth.json"
        source_manifests.append(
            {
                "name": manifest.get("name", root.name),
                "path": str(manifest_path),
                "sha256": sha256_file(manifest_path),
            }
        )
        for sample in manifest.get("samples", []):
            if not sample.get("included"):
                continue
            relative = Path(sample["image_path"])
            if relative.is_absolute() or ".." in relative.parts:
                raise ValueError(f"sample {sample.get('id')} has an unsafe path")
            source_image = root / relative
            expected_hash = sample.get("image_sha256")
            actual_hash = sha256_file(source_image)
            if expected_hash and expected_hash != actual_hash:
                raise ValueError(f"sample {sample.get('id')} image changed")
            merged_id = f"source-{source_index + 1}:{sample['id']}"
            suffix = source_image.suffix.lower() or ".png"
            filename = hashlib.sha256(merged_id.encode()).hexdigest()[:24] + suffix
            target_image = images / filename
            shutil.copyfile(source_image, target_image)
            records.append(
                {
                    **sample,
                    "id": merged_id,
                    "image_path": target_image.relative_to(output).as_posix(),
                    "image_sha256": sha256_file(target_image),
                    "source_dataset": manifest.get("name", root.name),
                    "source_group_id": str(sample["group_id"]),
                    "source_split": sample["split"],
                    "source_board_id": manifest.get("board_id"),
                }
            )

    groups = Groups(len(records))
    first_by_capture: dict[tuple[str, str], int] = {}
    first_by_target: dict[tuple[str, str], int] = {}
    for index, sample in enumerate(records):
        capture_key = (sample["source_dataset"], sample["source_group_id"])
        if capture_key in first_by_capture:
            groups.union(index, first_by_capture[capture_key])
        else:
            first_by_capture[capture_key] = index
        board_id = sample.get("source_board_id")
        reference = sample.get("reference")
        if isinstance(board_id, str) and isinstance(reference, str):
            target_key = (board_id, reference)
            if target_key in first_by_target:
                groups.union(index, first_by_target[target_key])
            else:
                first_by_target[target_key] = index

    members: dict[int, list[int]] = {}
    for index in range(len(records)):
        members.setdefault(groups.find(index), []).append(index)
    for component in members.values():
        split = (
            "eval"
            if any(records[index]["source_split"] == "eval" for index in component)
            else "train"
        )
        digest = hashlib.sha256()
        for sample_id in sorted(records[index]["id"] for index in component):
            digest.update(sample_id.encode())
            digest.update(b"\0")
        group_id = f"merged-{digest.hexdigest()[:20]}"
        for index in component:
            records[index]["split"] = split
            records[index]["group_id"] = group_id

    for split in ("train", "eval"):
        if not any(sample["split"] == split for sample in records):
            raise ValueError(f"merged {split} split is empty")
    present_classes = {
        sample.get("class_name") for sample in records if sample.get("class_name")
    }
    manifest = {
        "schema_version": 1,
        "name": output.name,
        "generated_at": generated_at or datetime.now(UTC).isoformat(),
        "classes": list(CLASS_NAMES),
        "normalization": normalizations[0],
        "source_manifests": source_manifests,
        "split_policy": (
            "source capture groups and matching board-reference targets do not "
            "cross splits; any prior evaluation membership propagates"
        ),
        "training_class_limitations": sorted(set(CLASS_NAMES) - present_classes),
        "samples": records,
    }
    (output / "manifest.ground-truth.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    return manifest


def main() -> None:
    args = parse_args()
    manifest = merge_datasets(args.sources, args.output)
    counts = {
        split: sum(sample["split"] == split for sample in manifest["samples"])
        for split in ("train", "eval")
    }
    classes = {
        name: sum(
            (sample.get("class_name") or "empty") == name
            for sample in manifest["samples"]
        )
        for name in ("empty", *CLASS_NAMES)
    }
    print(
        json.dumps(
            {
                "output": str(args.output),
                "sample_count": len(manifest["samples"]),
                "split_counts": counts,
                "class_counts": classes,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
