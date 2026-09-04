#!/usr/bin/env python3
"""Render component-only held-part rotation examples for review."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageChops, ImageDraw

REPOSITORY = Path(__file__).resolve().parents[2]
TRAINING_PACKAGE = REPOSITORY / "python" / "training"
sys.path.insert(0, str(TRAINING_PACKAGE))

from velocitypnp_training import load_held_part_samples  # noqa: E402
from velocitypnp_training.dataset import _letterbox  # noqa: E402
from velocitypnp_training.held_part import _rotate_component_only  # noqa: E402

IMAGE_SIZE = 192
ANGLES_DEGREES = (
    -75.0,
    -60.0,
    -45.0,
    -30.0,
    -15.0,
    15.0,
    30.0,
    45.0,
    60.0,
    75.0,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--count", type=int, default=16)
    return parser.parse_args()


def draw_outline(image: Image.Image, points: np.ndarray, color: str) -> None:
    ImageDraw.Draw(image).polygon(
        [tuple(float(axis) for axis in point) for point in points],
        outline=color,
        width=2,
    )


def label(image: Image.Image, text: str) -> None:
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, image.width, 18), fill="black")
    draw.text((3, 3), text, fill="white")


def main() -> None:
    args = parse_args()
    if args.count < 1:
        raise ValueError("count must be positive")
    _, samples = load_held_part_samples(args.dataset)
    occupied = [sample for sample in samples if sample.outline_px is not None]
    if not occupied:
        raise ValueError("dataset has no occupied held-part samples")
    selected = occupied[: min(args.count, len(occupied))]
    rows: list[tuple[Image.Image, Image.Image, Image.Image]] = []
    for index, sample in enumerate(selected):
        with Image.open(sample.bare_image_path) as source:
            bare, _ = _letterbox(source.convert("RGB"), None, IMAGE_SIZE)
        with Image.open(sample.observed_image_path) as source:
            observed, points = _letterbox(
                source.convert("RGB"), sample.outline_px, IMAGE_SIZE
            )
        if points is None:
            raise ValueError(f"occupied sample {sample.sample_id} has no outline")
        angle = ANGLES_DEGREES[index % len(ANGLES_DEGREES)]
        synthetic, rotated_points = _rotate_component_only(
            bare, observed, points, angle
        )
        original = observed.copy()
        rotated = synthetic.copy()
        difference = ImageChops.difference(bare, synthetic)
        draw_outline(original, points, "lime")
        draw_outline(rotated, rotated_points, "cyan")
        draw_outline(difference, rotated_points, "cyan")
        label(original, "original")
        label(rotated, f"component {angle:+.0f} deg")
        label(difference, "bare / synthetic difference")
        rows.append((original, rotated, difference))

    sheet = Image.new("RGB", (3 * IMAGE_SIZE, len(rows) * IMAGE_SIZE), "black")
    for row, images in enumerate(rows):
        for column, image in enumerate(images):
            sheet.paste(image, (column * IMAGE_SIZE, row * IMAGE_SIZE))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(args.output)
    print(f"wrote {len(rows)} component-only rotations to {args.output}")


if __name__ == "__main__":
    main()
