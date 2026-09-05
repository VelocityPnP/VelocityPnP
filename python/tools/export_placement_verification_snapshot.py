#!/usr/bin/env python3
"""Export target-normalized placement verification crops from a board survey."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sqlite3
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw


OUTPUT_SIZE = 256
OUTPUT_SPAN_MM = 4.0
RESAMPLING = Image.Resampling.BILINEAR


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("database", type=Path)
    parser.add_argument("capture_manifest", type=Path)
    parser.add_argument("output", type=Path)
    return parser.parse_args()


def load_entity(
    connection: sqlite3.Connection, kind: str, entity_id: str
) -> dict[str, Any]:
    row = connection.execute(
        "SELECT document FROM entities WHERE kind = ? AND id = ?",
        (kind, entity_id),
    ).fetchone()
    if row is None:
        raise ValueError(f"{kind} {entity_id} does not exist")
    return json.loads(row[0])


def load_entities(connection: sqlite3.Connection, kind: str) -> list[dict[str, Any]]:
    return [
        json.loads(row[0])
        for row in connection.execute(
            "SELECT document FROM entities WHERE kind = ?",
            (kind,),
        )
    ]


def part_dimensions(
    placement: dict[str, Any], parts: list[dict[str, Any]]
) -> tuple[float, float]:
    expected_key = f'{placement["package"]}:{placement["value"]}'
    exact = [part for part in parts if part.get("key") == expected_key]
    candidates = exact or [
        part for part in parts if part.get("package") == placement["package"]
    ]
    if len(candidates) != 1:
        raise ValueError(
            f'{placement["reference"]} has {len(candidates)} matching parts'
        )
    width = candidates[0].get("body_width_mm")
    length = candidates[0].get("body_length_mm")
    if not all(
        isinstance(value, (int, float)) and math.isfinite(value) and value > 0
        for value in (width, length)
    ):
        raise ValueError(
            f'{placement["reference"]} has no valid package dimensions'
        )
    return float(width), float(length)


def reference_sort_key(reference: str) -> tuple[str, int, str]:
    prefix = reference.rstrip("0123456789")
    suffix = reference[len(prefix) :]
    return prefix, int(suffix) if suffix else -1, reference


def inverse_2x2(matrix: list[list[float]]) -> list[list[float]]:
    determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
    if not math.isfinite(determinant) or abs(determinant) < 1e-12:
        raise ValueError("camera mapping is singular")
    return [
        [matrix[1][1] / determinant, -matrix[0][1] / determinant],
        [-matrix[1][0] / determinant, matrix[0][0] / determinant],
    ]


def multiply_2x2(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1],
    ]


def transform_point(matrix: list[float], point: list[float]) -> list[float]:
    return [
        matrix[0] * point[0] + matrix[1] * point[1] + matrix[2],
        matrix[3] * point[0] + matrix[4] * point[1] + matrix[5],
    ]


def machine_rotation(matrix: list[float], design_rotation_deg: float) -> float:
    board_rotation = math.atan2(matrix[3], matrix[0])
    return board_rotation + math.radians(design_rotation_deg)


def mean_frames(paths: list[Path]) -> Image.Image:
    if not paths:
        raise ValueError("capture has no frames")
    images = []
    for path in paths:
        with Image.open(path) as source:
            images.append(source.convert("L"))
    size = images[0].size
    if any(image.size != size for image in images):
        raise ValueError("capture frames have different sizes")
    mean = images[0]
    for index, image in enumerate(images[1:], start=2):
        mean = Image.blend(mean, image, 1.0 / index)
    return mean


def normalized_crop_coefficients(
    camera_center_px: list[float],
    pixel_to_machine: list[list[float]],
    camera_machine_mm: list[float],
    target_machine_mm: list[float],
    target_rotation_rad: float,
) -> tuple[float, float, float, float, float, float]:
    machine_to_pixel = inverse_2x2(pixel_to_machine)
    target_error_px = multiply_2x2(
        machine_to_pixel,
        [
            target_machine_mm[0] - camera_machine_mm[0],
            target_machine_mm[1] - camera_machine_mm[1],
        ],
    )
    target_px = [
        camera_center_px[0] + target_error_px[0],
        camera_center_px[1] + target_error_px[1],
    ]
    scale = OUTPUT_SPAN_MM / OUTPUT_SIZE
    cosine = math.cos(target_rotation_rad)
    sine = math.sin(target_rotation_rad)
    output_x_machine = [scale * cosine, scale * sine]
    output_y_machine = [-scale * sine, scale * cosine]
    output_x_pixel = multiply_2x2(machine_to_pixel, output_x_machine)
    output_y_pixel = multiply_2x2(machine_to_pixel, output_y_machine)
    half = OUTPUT_SIZE / 2.0
    return (
        output_x_pixel[0],
        output_y_pixel[0],
        target_px[0] - half * (output_x_pixel[0] + output_y_pixel[0]),
        output_x_pixel[1],
        output_y_pixel[1],
        target_px[1] - half * (output_x_pixel[1] + output_y_pixel[1]),
    )


def crop_fits_image(
    image: Image.Image,
    coefficients: tuple[float, float, float, float, float, float],
) -> bool:
    width, height = image.size
    for output_x, output_y in (
        (0.0, 0.0),
        (OUTPUT_SIZE - 1.0, 0.0),
        (OUTPUT_SIZE - 1.0, OUTPUT_SIZE - 1.0),
        (0.0, OUTPUT_SIZE - 1.0),
    ):
        source_x = (
            coefficients[0] * output_x
            + coefficients[1] * output_y
            + coefficients[2]
        )
        source_y = (
            coefficients[3] * output_x
            + coefficients[4] * output_y
            + coefficients[5]
        )
        if not (1.0 <= source_x < width - 1.0):
            return False
        if not (1.0 <= source_y < height - 1.0):
            return False
    return True


def normalized_crop(
    image: Image.Image,
    camera_center_px: list[float],
    pixel_to_machine: list[list[float]],
    camera_machine_mm: list[float],
    target_machine_mm: list[float],
    target_rotation_rad: float,
) -> Image.Image:
    coefficients = normalized_crop_coefficients(
        camera_center_px,
        pixel_to_machine,
        camera_machine_mm,
        target_machine_mm,
        target_rotation_rad,
    )
    return image.transform(
        (OUTPUT_SIZE, OUTPUT_SIZE),
        Image.Transform.AFFINE,
        coefficients,
        resample=RESAMPLING,
        fillcolor=127,
    )


def expected_outline(length_mm: float, width_mm: float) -> list[list[float]]:
    pixels_per_mm = OUTPUT_SIZE / OUTPUT_SPAN_MM
    half_length = 0.5 * length_mm * pixels_per_mm
    half_width = 0.5 * width_mm * pixels_per_mm
    center = OUTPUT_SIZE / 2.0
    return [
        [center - half_length, center - half_width],
        [center + half_length, center - half_width],
        [center + half_length, center + half_width],
        [center - half_length, center + half_width],
    ]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def render_contact_sheet(output: Path, samples: list[dict[str, Any]]) -> None:
    columns = 8
    tile = 192
    label_height = 24
    rows = math.ceil(len(samples) / columns)
    sheet = Image.new("RGB", (columns * tile, rows * (tile + label_height)), "black")
    draw = ImageDraw.Draw(sheet)
    scale = tile / OUTPUT_SIZE
    for index, sample in enumerate(samples):
        column = index % columns
        row = index // columns
        left = column * tile
        top = row * (tile + label_height)
        with Image.open(output / sample["image_path"]) as source:
            image = source.convert("RGB").resize((tile, tile), RESAMPLING)
        sheet.paste(image, (left, top))
        outline = sample["draft_outline_px"]
        if outline:
            points = [
                (left + point[0] * scale, top + point[1] * scale)
                for point in outline
            ]
            draw.line(points + [points[0]], fill=(0, 255, 128), width=2)
        center = left + tile // 2, top + tile // 2
        draw.line(
            [(center[0] - 5, center[1]), (center[0] + 5, center[1])],
            fill=(255, 190, 0),
            width=1,
        )
        draw.line(
            [(center[0], center[1] - 5), (center[0], center[1] + 5)],
            fill=(255, 190, 0),
            width=1,
        )
        state = "placed" if sample["draft_state"] == "component_flat" else "empty"
        draw.text(
            (left + 4, top + tile + 5),
            f'{sample["reference"]}  {state}',
            fill="white",
        )
    sheet.save(output / "contact-sheet.png")


def main() -> None:
    args = parse_args()
    specification = json.loads(args.capture_manifest.read_text())
    connection = sqlite3.connect(args.database)
    registration = load_entity(
        connection, "board_registration", specification["registration_id"]
    )
    board = load_entity(connection, "board", registration["board_id"])
    calibration = load_entity(connection, "calibration", registration["calibration_id"])
    parts = load_entities(connection, "part")
    connection.close()
    camera_value = calibration["artifacts"]["top_camera_planar_model"]["value"]
    camera_model = camera_value.get("result", camera_value)["model"]
    mapping_value = calibration["artifacts"]["top_camera_working_plane_map"][
        "value"
    ]
    pixel_to_machine = mapping_value.get("result", mapping_value)[
        "pixel_error_to_machine_mm"
    ]
    camera_center_px = [camera_model["center_x_px"], camera_model["center_y_px"]]
    capture_root = args.capture_manifest.parent
    captures = []
    for capture in specification["captures"]:
        frames = [capture_root / path for path in capture["frames"]]
        image = mean_frames(frames)
        captures.append((capture, image))

    output_images = args.output / "images"
    output_images.mkdir(parents=True, exist_ok=True)
    placed = set(specification["placed_references"])
    transform = registration["design_to_machine"]["matrix"]
    if transform and isinstance(transform[0], list):
        transform = [value for row in transform for value in row]
    samples = []
    all_placements = [
        placement
        for placement in board["placements"]
        if placement["enabled"] and not placement["fiducial"]
    ]
    requested_references = specification.get("references")
    reference_prefixes = specification.get("reference_prefixes")
    if requested_references is not None and reference_prefixes is not None:
        raise ValueError("capture manifest cannot use both reference selectors")
    if requested_references is not None:
        requested_references = set(requested_references)
        placements = [
            placement
            for placement in all_placements
            if placement["reference"] in requested_references
        ]
        missing_references = requested_references - {
            placement["reference"] for placement in placements
        }
        if missing_references:
            raise ValueError(
                "requested references are not enabled board placements: "
                + ", ".join(sorted(missing_references))
            )
    elif reference_prefixes is not None:
        if not all(
            isinstance(prefix, str) and prefix for prefix in reference_prefixes
        ):
            raise ValueError("reference prefixes must be non-empty strings")
        placements = [
            placement
            for placement in all_placements
            if any(
                placement["reference"].startswith(prefix)
                for prefix in reference_prefixes
            )
        ]
    else:
        placements = all_placements
    references = {placement["reference"] for placement in placements}
    unknown_placed = placed - references
    if unknown_placed:
        raise ValueError(
            "placed references are not enabled board placements: "
            + ", ".join(sorted(unknown_placed))
        )
    for placement in sorted(
        placements,
        key=lambda placement: reference_sort_key(placement["reference"]),
    ):
        target_machine_mm = transform_point(
            transform,
            [placement["pose"]["x_mm"], placement["pose"]["y_mm"]],
        )
        rotation_rad = machine_rotation(
            transform, placement["pose"]["rotation_deg"]
        )
        valid_captures = [
            item
            for item in captures
            if crop_fits_image(
                item[1],
                normalized_crop_coefficients(
                    camera_center_px,
                    pixel_to_machine,
                    item[0]["camera_machine_mm"],
                    target_machine_mm,
                    rotation_rad,
                ),
            )
        ]
        if not valid_captures:
            raise ValueError(
                f'{placement["reference"]} is not fully covered by a capture'
            )
        capture, source = min(
            valid_captures,
            key=lambda item: (
                target_machine_mm[0] - item[0]["camera_machine_mm"][0]
            )
            ** 2
            + (
                target_machine_mm[1] - item[0]["camera_machine_mm"][1]
            )
            ** 2,
        )
        crop = normalized_crop(
            source,
            camera_center_px,
            pixel_to_machine,
            capture["camera_machine_mm"],
            target_machine_mm,
            rotation_rad,
        )
        image_path = output_images / f'{placement["reference"].lower()}.png'
        crop.save(image_path)
        width_mm, length_mm = part_dimensions(placement, parts)
        state = (
            "component_flat"
            if placement["reference"] in placed
            else "empty"
        )
        samples.append(
            {
                "id": (
                    f'{registration["id"]}-{placement["reference"].lower()}'
                ),
                "group_id": registration["id"],
                "reference": placement["reference"],
                "package": placement["package"],
                "image_path": image_path.relative_to(args.output).as_posix(),
                "image_sha256": sha256_file(image_path),
                "width_px": OUTPUT_SIZE,
                "height_px": OUTPUT_SIZE,
                "target_machine_mm": target_machine_mm,
                "target_rotation_deg": math.degrees(rotation_rad),
                "source_capture": capture["name"],
                "draft_state": state,
                "draft_outline_px": (
                    expected_outline(length_mm, width_mm)
                    if state == "component_flat"
                    else None
                ),
                "expected_size_mm": [width_mm, length_mm],
            }
        )

    manifest = {
        "schema_version": 1,
        "name": args.output.name,
        "generated_at": datetime.now(UTC).isoformat(),
        "registration_id": registration["id"],
        "board_id": board["id"],
        "calibration_id": registration["calibration_id"],
        "normalization": {
            "output_size_px": OUTPUT_SIZE,
            "output_span_mm": OUTPUT_SPAN_MM,
            "target_center_px": [OUTPUT_SIZE / 2.0, OUTPUT_SIZE / 2.0],
            "target_long_axis": "positive_x",
        },
        "samples": samples,
    }
    (args.output / "manifest.draft.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    template = Path(__file__).with_name(
        "placement_verification_review.html"
    ).read_text()
    review = template.replace("__MANIFEST__", json.dumps(manifest)).replace(
        "__DATASET_TITLE__", json.dumps(args.output.name)
    )
    (args.output / "index.html").write_text(review)
    render_contact_sheet(args.output, samples)
    print(
        json.dumps(
            {
                "output": str(args.output),
                "sample_count": len(samples),
                "placed_count": sum(
                    sample["draft_state"] == "component_flat"
                    for sample in samples
                ),
                "empty_count": sum(
                    sample["draft_state"] == "empty" for sample in samples
                ),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
