"""Paired-image data and model support for held-part inspection."""

from __future__ import annotations

import hashlib
import json
import math
import random
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch
from PIL import Image, ImageEnhance
from torch import nn
from torch.nn import functional as F
from torch.utils.data import Dataset

from .dataset import _letterbox, polygon_to_pose

HELD_PART_CLASS_NAMES = (
    "bare_nozzle",
    "component_flat",
    "component_on_edge",
)
HELD_PART_SOURCE_CLASSES = HELD_PART_CLASS_NAMES[1:]


@dataclass(frozen=True)
class HeldPartSample:
    """One reviewed bare-nozzle and observed-image pair."""

    sample_id: str
    split: str
    bare_image_path: Path
    observed_image_path: Path
    class_id: int
    outline_px: tuple[tuple[float, float], ...] | None
    directed_end_px: tuple[float, float] | None
    part_key: str | None
    group_id: str


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _image_path(root: Path, record: dict[str, Any], key: str) -> Path:
    relative = Path(record[key])
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"sample {record.get('id')} has an unsafe {key}")
    path = root / relative
    if not path.is_file():
        raise ValueError(f"missing image {path}")
    expected = record.get(key.replace("_path", "_sha256"))
    if expected and _sha256_file(path) != expected:
        raise ValueError(f"sample {record.get('id')} changed after review")
    return path


def _outline(record: dict[str, Any]) -> tuple[tuple[float, float], ...] | None:
    value = record.get("outline_px")
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError(f"sample {record.get('id')} needs a four-point outline")
    points = tuple((float(point[0]), float(point[1])) for point in value)
    if any(not math.isfinite(axis) for point in points for axis in point):
        raise ValueError(f"sample {record.get('id')} has a non-finite outline")
    return points


def load_held_part_samples(
    root: Path,
) -> tuple[dict[str, Any], list[HeldPartSample]]:
    """Load reviewed pairs without using deterministic detector results."""

    root = root.resolve()
    manifest_path = root / "manifest.ground-truth.json"
    if not manifest_path.is_file():
        raise ValueError(f"missing {manifest_path}")
    manifest = json.loads(manifest_path.read_text())
    declared = set(manifest.get("classes", []))
    required = set(HELD_PART_SOURCE_CLASSES)
    if not required.issubset(declared):
        raise ValueError("ground-truth classes do not match the model contract")

    samples: list[HeldPartSample] = []
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

        class_name = record.get("class_name")
        outline = _outline(record)
        if class_name is None:
            if outline is not None:
                raise ValueError(f"bare sample {sample_id} has an outline")
            class_id = 0
        else:
            if class_name not in HELD_PART_SOURCE_CLASSES:
                raise ValueError(
                    f"sample {sample_id} uses unsupported class {class_name!r}"
                )
            if outline is None:
                raise ValueError(f"sample {sample_id} has no outline")
            class_id = HELD_PART_CLASS_NAMES.index(class_name)

        marker_status = record.get("marker_status", "not_applicable")
        marker = record.get("marker_end_px")
        if marker_status == "visible":
            if class_id == 0 or not isinstance(marker, list) or len(marker) != 2:
                raise ValueError(f"sample {sample_id} has an invalid directed end")
            directed_end = (float(marker[0]), float(marker[1]))
            if any(not math.isfinite(axis) for axis in directed_end):
                raise ValueError(f"sample {sample_id} has a non-finite directed end")
        else:
            if marker is not None:
                raise ValueError(f"sample {sample_id} has a hidden directed end")
            directed_end = None

        samples.append(
            HeldPartSample(
                sample_id=sample_id,
                split=split,
                bare_image_path=_image_path(root, record, "bare_image_path"),
                observed_image_path=_image_path(root, record, "observed_image_path"),
                class_id=class_id,
                outline_px=outline,
                directed_end_px=directed_end,
                part_key=record.get("part_key"),
                group_id=str(record.get("group_id", sample_id)),
            )
        )
    if not samples:
        raise ValueError("ground-truth manifest has no included samples")
    return manifest, samples


def held_part_dataset_summary(root: Path) -> dict[str, Any]:
    """Report coverage and prove that capture groups do not cross splits."""

    manifest, samples = load_held_part_samples(root)
    split_classes: dict[str, dict[str, int]] = {}
    split_groups: dict[str, set[str]] = {}
    directed: dict[str, int] = {}
    for split in ("train", "eval"):
        selected = [sample for sample in samples if sample.split == split]
        counts = Counter(HELD_PART_CLASS_NAMES[sample.class_id] for sample in selected)
        split_classes[split] = {
            name: counts.get(name, 0) for name in HELD_PART_CLASS_NAMES
        }
        split_groups[split] = {sample.group_id for sample in selected}
        directed[split] = sum(sample.directed_end_px is not None for sample in selected)
    if not all(split_classes["train"].values()):
        raise ValueError("training split must contain all held-part classes")
    if not sum(split_classes["eval"].values()):
        raise ValueError("evaluation split is empty")
    overlap = split_groups["train"] & split_groups["eval"]
    if overlap:
        raise ValueError(f"capture groups occur in both splits: {sorted(overlap)}")

    digest = hashlib.sha256()
    paths = [
        root.resolve() / "manifest.ground-truth.json",
        *(
            path
            for sample in samples
            for path in (
                sample.bare_image_path,
                sample.observed_image_path,
            )
        ),
    ]
    for path in paths:
        digest.update(str(path.resolve().relative_to(root.resolve())).encode())
        digest.update(b"\0")
        digest.update(bytes.fromhex(_sha256_file(path)))
    return {
        "schema_version": manifest.get("schema_version"),
        "content_sha256": digest.hexdigest(),
        "sample_count": len(samples),
        "split_counts": {
            split: sum(counts.values()) for split, counts in split_classes.items()
        },
        "class_counts": split_classes,
        "group_counts": {split: len(groups) for split, groups in split_groups.items()},
        "directed_end_counts": directed,
        "evaluation_limitations": [
            name for name, count in split_classes["eval"].items() if count == 0
        ],
    }


def _transform_point(
    point: tuple[float, float], width: int, height: int, image_size: int
) -> np.ndarray:
    scale = min(image_size / width, image_size / height)
    resized_width = max(1, math.floor(width * scale + 0.5))
    resized_height = max(1, math.floor(height * scale + 0.5))
    left = (image_size - resized_width) // 2
    top = (image_size - resized_height) // 2
    return np.asarray(
        [
            point[0] * resized_width / width + left,
            point[1] * resized_height / height + top,
        ],
        np.float32,
    )


def _augment_pair(
    bare: Image.Image,
    observed: Image.Image,
    points: np.ndarray | None,
    marker: np.ndarray | None,
    rng: random.Random,
) -> tuple[Image.Image, Image.Image, np.ndarray | None, np.ndarray | None]:
    size = bare.width
    scale = rng.uniform(0.94, 1.06)
    shift = np.asarray(
        [rng.uniform(-0.05, 0.05) * size, rng.uniform(-0.05, 0.05) * size],
        np.float32,
    )
    candidates = []
    center = np.asarray([size / 2, size / 2], np.float32)
    if points is not None:
        candidates.extend((points - center) * scale + center + shift)
    if marker is not None:
        candidates.append((marker - center) * scale + center + shift)
    if candidates and (np.min(candidates) < 2 or np.max(candidates) > size - 2):
        scale = 1.0
        shift[:] = 0.0
    inverse = 1.0 / scale
    offset = center - (center + shift) * inverse
    affine = (inverse, 0.0, float(offset[0]), 0.0, inverse, float(offset[1]))
    bare = bare.transform(
        bare.size,
        Image.Transform.AFFINE,
        affine,
        Image.Resampling.BILINEAR,
        fillcolor=(127, 127, 127),
    )
    observed = observed.transform(
        observed.size,
        Image.Transform.AFFINE,
        affine,
        Image.Resampling.BILINEAR,
        fillcolor=(127, 127, 127),
    )
    if points is not None:
        points = (points - center) * scale + center + shift
    if marker is not None:
        marker = (marker - center) * scale + center + shift
    if rng.random() < 0.5:
        bare = bare.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
        observed = observed.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
        if points is not None:
            points[:, 0] = size - points[:, 0]
        if marker is not None:
            marker[0] = size - marker[0]
    if rng.random() < 0.5:
        bare = bare.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
        observed = observed.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
        if points is not None:
            points[:, 1] = size - points[:, 1]
        if marker is not None:
            marker[1] = size - marker[1]
    factor_brightness = rng.uniform(0.85, 1.15)
    factor_contrast = rng.uniform(0.85, 1.15)
    bare = ImageEnhance.Brightness(bare).enhance(factor_brightness)
    observed = ImageEnhance.Brightness(observed).enhance(factor_brightness)
    bare = ImageEnhance.Contrast(bare).enhance(factor_contrast)
    observed = ImageEnhance.Contrast(observed).enhance(factor_contrast)
    return bare, observed, points, marker


class HeldPartDataset(Dataset):
    """Torch dataset for paired bare and observed bottom-camera images."""

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
        _, samples = load_held_part_samples(root)
        self.samples = [sample for sample in samples if sample.split == split]
        if not self.samples:
            raise ValueError(f"dataset has no {split} samples")
        self.image_size = image_size
        self.augment = augment
        self.rng = random.Random(seed)

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(
        self, index: int
    ) -> tuple[torch.Tensor, int, torch.Tensor, torch.Tensor, float, str]:
        sample = self.samples[index]
        with Image.open(sample.bare_image_path) as source:
            bare_source = source.convert("RGB")
        with Image.open(sample.observed_image_path) as source:
            observed_source = source.convert("RGB")
        if bare_source.size != observed_source.size:
            raise ValueError(f"sample {sample.sample_id} image sizes do not match")
        width, height = observed_source.size
        bare, _ = _letterbox(bare_source, None, self.image_size)
        observed, points = _letterbox(
            observed_source, sample.outline_px, self.image_size
        )
        marker = (
            _transform_point(sample.directed_end_px, width, height, self.image_size)
            if sample.directed_end_px is not None
            else None
        )
        if self.augment:
            bare, observed, points, marker = _augment_pair(
                bare, observed, points, marker, self.rng
            )

        tensors = []
        for image in (bare, observed):
            array = np.asarray(image, dtype=np.float32).copy() / 255.0
            tensors.append(torch.from_numpy(array).permute(2, 0, 1))
        image_pair = torch.cat(tensors, dim=0)
        if self.augment:
            image_pair = (
                image_pair + torch.randn_like(image_pair) * self.rng.uniform(0.0, 0.015)
            ).clamp(0.0, 1.0)
        pose = (
            polygon_to_pose(points, self.image_size)
            if points is not None
            else np.zeros(6, np.float32)
        )
        if marker is not None and points is not None:
            direction = marker - points.mean(axis=0)
            length = float(np.linalg.norm(direction))
            if length <= 1e-6:
                raise ValueError(f"sample {sample.sample_id} has no directed axis")
            direction /= length
            directed = 1.0
        else:
            direction = np.zeros(2, np.float32)
            directed = 0.0
        return (
            image_pair,
            sample.class_id,
            torch.from_numpy(pose),
            torch.from_numpy(direction.astype(np.float32)),
            directed,
            sample.sample_id,
        )


@dataclass(frozen=True)
class HeldPartModelConfig:
    """Shape configuration stored with a held-part model artifact."""

    image_size: int = 192
    input_channels: int = 6
    class_count: int = 3
    feature_channels: int = 96

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


class _ConvBlock(nn.Module):
    def __init__(self, input_channels: int, output_channels: int, stride: int) -> None:
        super().__init__()
        self.layers = nn.Sequential(
            nn.Conv2d(
                input_channels, output_channels, 3, stride=stride, padding=1, bias=False
            ),
            nn.BatchNorm2d(output_channels),
            nn.SiLU(),
        )

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return self.layers(inputs)


class _ResidualBlock(nn.Module):
    def __init__(self, channels: int) -> None:
        super().__init__()
        self.layers = nn.Sequential(
            _ConvBlock(channels, channels, 1),
            nn.Conv2d(channels, channels, 3, padding=1, bias=False),
            nn.BatchNorm2d(channels),
        )
        self.activation = nn.SiLU()

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return self.activation(inputs + self.layers(inputs))


class HeldPartPoseModel(nn.Module):
    """Predict held-part state, OBB, and optional directed package axis."""

    def __init__(self, config: HeldPartModelConfig | None = None) -> None:
        super().__init__()
        self.config = config or HeldPartModelConfig()
        if self.config.input_channels != 6 or self.config.class_count != 3:
            raise ValueError(
                "held-part v1 requires six input channels and three classes"
            )
        if self.config.image_size % 8:
            raise ValueError("model image size must be divisible by 8")
        self.stem = nn.Sequential(
            nn.Conv2d(6, 24, 5, stride=2, padding=2, bias=False),
            nn.BatchNorm2d(24),
            nn.SiLU(),
        )
        self.shallow = nn.Sequential(
            _ConvBlock(24, 48, 2), _ResidualBlock(48), _ResidualBlock(48)
        )
        self.deep = nn.Sequential(
            _ConvBlock(48, self.config.feature_channels, 2),
            _ResidualBlock(self.config.feature_channels),
            _ResidualBlock(self.config.feature_channels),
        )
        self.top_down = nn.Conv2d(self.config.feature_channels, 64, 1, bias=False)
        self.lateral = nn.Conv2d(48, 64, 1, bias=False)
        self.local = nn.Sequential(_ConvBlock(64, 64, 1), _ResidualBlock(64))
        self.center_head = nn.Conv2d(64, 1, 1)
        self.pose_head = nn.Conv2d(64, 4, 1)
        self.direction_head = nn.Conv2d(64, 2, 1)
        self.classifier = nn.Sequential(
            nn.Linear(self.config.feature_channels, 64),
            nn.SiLU(),
            nn.Dropout(0.1),
            nn.Linear(64, self.config.class_count),
        )
        with torch.no_grad():
            self.pose_head.bias[:2].fill_(-2.0)
        feature_size = self.config.image_size // 4
        coordinates = torch.arange(feature_size, dtype=torch.float32) + 0.5
        coordinates /= feature_size
        grid_y, grid_x = torch.meshgrid(coordinates, coordinates, indexing="ij")
        self.register_buffer(
            "center_grid", torch.stack((grid_x.flatten(), grid_y.flatten()), dim=1)
        )

    def _predict(
        self, inputs: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        inputs = (inputs - 0.5) / 0.25
        shallow = self.shallow(self.stem(inputs))
        deep = self.deep(shallow)
        state = F.log_softmax(self.classifier(deep.mean(dim=(2, 3))), dim=1)
        local = self.local(
            F.interpolate(self.top_down(deep), scale_factor=2, mode="nearest")
            + self.lateral(shallow)
        )
        center_logits = self.center_head(local).flatten(1)
        weights = F.softmax(center_logits, dim=1)
        grid = self.center_grid.unsqueeze(0).expand(inputs.shape[0], -1, -1)
        center = torch.bmm(weights.unsqueeze(1), grid).squeeze(1)
        raw_pose = torch.bmm(
            self.pose_head(local).flatten(2), weights.unsqueeze(2)
        ).squeeze(2)
        size = torch.sigmoid(raw_pose[:, :2])
        angle = F.normalize(raw_pose[:, 2:], dim=1, eps=1e-6)
        direction = torch.bmm(
            self.direction_head(local).flatten(2), weights.unsqueeze(2)
        ).squeeze(2)
        direction = F.normalize(direction, dim=1, eps=1e-6)
        return state, torch.cat((center, size, angle), dim=1), direction, center_logits

    def forward(
        self, inputs: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        state, pose, direction, _ = self._predict(inputs)
        return state, pose, direction

    def forward_training(
        self, inputs: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        return self._predict(inputs)
