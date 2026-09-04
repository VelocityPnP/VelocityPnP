"""Dataset support for one-component tape-pocket images."""

from __future__ import annotations

import hashlib
import json
import math
import random
import re
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch
from PIL import Image, ImageEnhance
from torch.utils.data import Dataset


CLASS_NAMES = ("empty", "component_flat", "component_on_edge")
SOURCE_CLASSES = CLASS_NAMES[1:]
INVENTORY_GROUP = re.compile(
    r"^(inventory_source_capture-[0-9a-f-]+-lane-[0-9]+)-pocket-[0-9]+$"
)
RUN_GROUP = re.compile(
    r"^((?:live_holdout|pickup_qualification|placement_execution)-[0-9a-f-]+)"
)


@dataclass(frozen=True)
class Sample:
    """One reviewed source-component image and its ground truth."""

    sample_id: str
    split: str
    image_path: Path
    class_id: int
    outline_px: tuple[tuple[float, float], ...] | None
    part_key: str | None
    group_id: str


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _sample_group(sample_id: str) -> str:
    match = INVENTORY_GROUP.match(sample_id) or RUN_GROUP.match(sample_id)
    return match.group(1) if match else sample_id


def load_samples(root: Path) -> tuple[dict[str, Any], list[Sample]]:
    """Load and validate the reviewed manifest without using detector labels."""

    root = root.resolve()
    manifest_path = root / "manifest.ground-truth.json"
    if not manifest_path.is_file():
        raise ValueError(f"missing {manifest_path}")
    manifest = json.loads(manifest_path.read_text())
    if tuple(manifest.get("classes", [])) != SOURCE_CLASSES:
        raise ValueError("ground-truth classes do not match the model contract")

    samples: list[Sample] = []
    seen: set[str] = set()
    for record in manifest.get("samples", []):
        if not record.get("included"):
            continue
        sample_id = record.get("id")
        split = record.get("split")
        if not isinstance(sample_id, str) or not sample_id:
            raise ValueError("ground-truth sample has no id")
        if sample_id in seen:
            raise ValueError(f"duplicate ground-truth sample {sample_id}")
        seen.add(sample_id)
        if split not in {"train", "eval"}:
            raise ValueError(f"sample {sample_id} has invalid split {split!r}")
        image_path = root / record["image_path"]
        if not image_path.is_file():
            raise ValueError(f"missing image {image_path}")

        class_name = record.get("class_name")
        outline = record.get("outline_px")
        if class_name is None:
            if outline is not None:
                raise ValueError(f"empty sample {sample_id} has an outline")
            class_id = 0
            points = None
        else:
            if class_name not in SOURCE_CLASSES:
                raise ValueError(
                    f"sample {sample_id} has invalid class {class_name!r}"
                )
            if not isinstance(outline, list) or len(outline) != 4:
                raise ValueError(f"sample {sample_id} needs a four-point outline")
            points_list = []
            for point in outline:
                if not isinstance(point, list) or len(point) != 2:
                    raise ValueError(f"sample {sample_id} has an invalid outline")
                x, y = float(point[0]), float(point[1])
                if not math.isfinite(x) or not math.isfinite(y):
                    raise ValueError(f"sample {sample_id} has a non-finite outline")
                points_list.append((x, y))
            points = tuple(points_list)
            class_id = SOURCE_CLASSES.index(class_name) + 1

        samples.append(
            Sample(
                sample_id=sample_id,
                split=split,
                image_path=image_path,
                class_id=class_id,
                outline_px=points,
                part_key=record.get("part_key"),
                group_id=_sample_group(sample_id),
            )
        )
    if not samples:
        raise ValueError("ground-truth manifest has no included samples")
    return manifest, samples


def dataset_content_sha256(root: Path, samples: list[Sample]) -> str:
    """Hash the manifest and exact image bytes used by a training run."""

    digest = hashlib.sha256()
    manifest_path = root.resolve() / "manifest.ground-truth.json"
    for path in [manifest_path, *(sample.image_path for sample in samples)]:
        relative = path.resolve().relative_to(root.resolve())
        digest.update(str(relative).encode())
        digest.update(b"\0")
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def dataset_summary(root: Path) -> dict[str, Any]:
    """Return class coverage, content identity, and split leakage checks."""

    manifest, samples = load_samples(root)
    split_counts: dict[str, dict[str, int]] = {}
    groups: dict[str, set[str]] = {"train": set(), "eval": set()}
    for split in ("train", "eval"):
        selected = [sample for sample in samples if sample.split == split]
        counts = Counter(CLASS_NAMES[sample.class_id] for sample in selected)
        split_counts[split] = {
            name: counts.get(name, 0) for name in CLASS_NAMES
        }
        groups[split] = {sample.group_id for sample in selected}
    if not all(split_counts["train"].values()):
        raise ValueError("training split must contain all occupancy classes")
    if not any(split_counts["eval"].values()):
        raise ValueError("evaluation split is empty")
    overlap = sorted(groups["train"] & groups["eval"])
    if overlap:
        raise ValueError(f"capture groups occur in both splits: {overlap}")
    return {
        "schema_version": manifest.get("schema_version"),
        "manifest_sha256": sha256_file(
            root.resolve() / "manifest.ground-truth.json"
        ),
        "content_sha256": dataset_content_sha256(root, samples),
        "sample_count": len(samples),
        "split_counts": {
            split: sum(counts.values()) for split, counts in split_counts.items()
        },
        "class_counts": split_counts,
        "group_counts": {
            split: len(split_groups) for split, split_groups in groups.items()
        },
        "evaluation_limitations": [
            name for name, count in split_counts["eval"].items() if count == 0
        ],
    }


def polygon_to_pose(points: np.ndarray, image_size: int) -> np.ndarray:
    """Convert a rectangle to normalized center, size, and doubled angle."""

    points = np.asarray(points, dtype=np.float32)
    if points.shape != (4, 2):
        raise ValueError("component outline must have four points")
    edges = np.roll(points, -1, axis=0) - points
    lengths = np.linalg.norm(edges, axis=1)
    longest = int(np.argmax(lengths))
    width = float(lengths[longest])
    height = float(lengths[(longest + 1) % 4])
    if width <= 0.0 or height <= 0.0:
        raise ValueError("component outline has no area")
    axis = edges[longest]
    theta = math.atan2(float(axis[1]), float(axis[0]))
    center = points.mean(axis=0) / float(image_size)
    return np.asarray(
        [
            center[0],
            center[1],
            width / image_size,
            height / image_size,
            math.cos(2.0 * theta),
            math.sin(2.0 * theta),
        ],
        dtype=np.float32,
    )


def pose_to_polygon(pose: np.ndarray, image_size: int) -> np.ndarray:
    """Convert a normalized pose to four image-space corners."""

    center = np.asarray(pose[:2], dtype=np.float32) * image_size
    width, height = np.asarray(pose[2:4], dtype=np.float32) * image_size
    theta = 0.5 * math.atan2(float(pose[5]), float(pose[4]))
    cosine, sine = math.cos(theta), math.sin(theta)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]], np.float32)
    local = np.asarray(
        [
            [-width / 2, -height / 2],
            [width / 2, -height / 2],
            [width / 2, height / 2],
            [-width / 2, height / 2],
        ],
        np.float32,
    )
    return local @ rotation.T + center


def _letterbox(
    image: Image.Image,
    points: tuple[tuple[float, float], ...] | None,
    image_size: int,
) -> tuple[Image.Image, np.ndarray | None]:
    width, height = image.size
    scale = min(image_size / width, image_size / height)
    resized_width = max(1, math.floor(width * scale + 0.5))
    resized_height = max(1, math.floor(height * scale + 0.5))
    left = (image_size - resized_width) // 2
    top = (image_size - resized_height) // 2
    resized = _resize_half_pixel_bilinear_u8(image, resized_width, resized_height)
    output = Image.new("RGB", (image_size, image_size), (127, 127, 127))
    output.paste(resized, (left, top))
    if points is None:
        return output, None
    transformed = np.asarray(points, np.float32)
    transformed[:, 0] = transformed[:, 0] * (resized_width / width) + left
    transformed[:, 1] = transformed[:, 1] * (resized_height / height) + top
    return output, transformed


def _resize_half_pixel_bilinear_u8(
    image: Image.Image, width: int, height: int
) -> Image.Image:
    """Resize RGB bytes with the model contract's portable interpolation."""

    source = np.asarray(image, dtype=np.uint8)
    source_height, source_width, channels = source.shape
    if channels != 3 or min(width, height, source_width, source_height) < 1:
        raise ValueError("model resize requires a non-empty RGB image")
    target_x = np.arange(width, dtype=np.float64)
    target_y = np.arange(height, dtype=np.float64)
    source_x = np.clip(
        (target_x + 0.5) * source_width / width - 0.5,
        0.0,
        source_width - 1,
    )
    source_y = np.clip(
        (target_y + 0.5) * source_height / height - 0.5,
        0.0,
        source_height - 1,
    )
    x0 = np.floor(source_x).astype(np.intp)
    y0 = np.floor(source_y).astype(np.intp)
    x1 = np.minimum(x0 + 1, source_width - 1)
    y1 = np.minimum(y0 + 1, source_height - 1)
    x_fraction = (source_x - x0)[None, :, None]
    y_fraction = (source_y - y0)[:, None, None]
    top = (
        source[y0[:, None], x0[None, :]].astype(np.float64)
        * (1.0 - x_fraction)
        + source[y0[:, None], x1[None, :]].astype(np.float64) * x_fraction
    )
    bottom = (
        source[y1[:, None], x0[None, :]].astype(np.float64)
        * (1.0 - x_fraction)
        + source[y1[:, None], x1[None, :]].astype(np.float64) * x_fraction
    )
    resized = np.floor(top * (1.0 - y_fraction) + bottom * y_fraction + 0.5)
    return Image.fromarray(np.clip(resized, 0, 255).astype(np.uint8))


def _augment_geometry(
    image: Image.Image,
    points: np.ndarray | None,
    rng: random.Random,
) -> tuple[Image.Image, np.ndarray | None]:
    size = image.width
    scale = rng.uniform(0.94, 1.06)
    translate_x = rng.uniform(-0.05, 0.05) * size
    translate_y = rng.uniform(-0.05, 0.05) * size
    if points is not None:
        center = np.asarray([size / 2, size / 2], np.float32)
        candidate = (points - center) * scale + center
        candidate += np.asarray([translate_x, translate_y], np.float32)
        if candidate.min() < 2 or candidate.max() > size - 2:
            scale, translate_x, translate_y = 1.0, 0.0, 0.0
    inverse_scale = 1.0 / scale
    center_value = size / 2
    offset_x = center_value - (center_value + translate_x) * inverse_scale
    offset_y = center_value - (center_value + translate_y) * inverse_scale
    image = image.transform(
        image.size,
        Image.Transform.AFFINE,
        (inverse_scale, 0.0, offset_x, 0.0, inverse_scale, offset_y),
        Image.Resampling.BILINEAR,
        fillcolor=(127, 127, 127),
    )
    if points is not None:
        center = np.asarray([center_value, center_value], np.float32)
        points = (points - center) * scale + center
        points += np.asarray([translate_x, translate_y], np.float32)

    if rng.random() < 0.5:
        image = image.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
        if points is not None:
            points[:, 0] = size - points[:, 0]
    if rng.random() < 0.5:
        image = image.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
        if points is not None:
            points[:, 1] = size - points[:, 1]
    return image, points


class SourceComponentDataset(Dataset):
    """Torch dataset for a fixed split of reviewed pocket images."""

    def __init__(
        self,
        root: Path,
        split: str,
        image_size: int,
        augment: bool = False,
        seed: int = 0,
    ) -> None:
        if image_size < 64 or image_size % 8:
            raise ValueError("image size must be at least 64 and divisible by 8")
        _, all_samples = load_samples(root)
        self.samples = [sample for sample in all_samples if sample.split == split]
        if not self.samples:
            raise ValueError(f"dataset has no {split} samples")
        self.image_size = image_size
        self.augment = augment
        self.rng = random.Random(seed)

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int) -> tuple[torch.Tensor, int, torch.Tensor, str]:
        sample = self.samples[index]
        with Image.open(sample.image_path) as source:
            image, points = _letterbox(
                source.convert("RGB"), sample.outline_px, self.image_size
            )
        if self.augment:
            image, points = _augment_geometry(image, points, self.rng)
            image = ImageEnhance.Brightness(image).enhance(
                self.rng.uniform(0.85, 1.15)
            )
            image = ImageEnhance.Contrast(image).enhance(
                self.rng.uniform(0.85, 1.15)
            )
        array = np.asarray(image, dtype=np.float32).copy() / 255.0
        tensor = torch.from_numpy(array).permute(2, 0, 1)
        if self.augment:
            noise = torch.randn_like(tensor) * self.rng.uniform(0.0, 0.02)
            tensor = (tensor + noise).clamp(0.0, 1.0)
        pose = (
            polygon_to_pose(points, self.image_size)
            if points is not None
            else np.zeros(6, dtype=np.float32)
        )
        return tensor, sample.class_id, torch.from_numpy(pose), sample.sample_id
