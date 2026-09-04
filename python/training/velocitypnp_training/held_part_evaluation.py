"""Evaluation and overlays for held-part pose models."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
import torch
from PIL import Image, ImageDraw
from torch.utils.data import DataLoader

from .dataset import pose_to_polygon
from .evaluation import polygon_iou
from .held_part import HELD_PART_CLASS_NAMES, HeldPartDataset


def evaluate_held_part(
    model: torch.nn.Module,
    dataset: HeldPartDataset,
    device: torch.device,
    batch_size: int,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Evaluate state and 180-degree-symmetric pose predictions."""

    loader = DataLoader(dataset, batch_size=batch_size, shuffle=False)
    confusion = np.zeros((3, 3), np.int64)
    center_errors: list[float] = []
    angle_errors: list[float] = []
    overlaps: list[float] = []
    records: list[dict[str, Any]] = []
    model.eval()
    with torch.inference_mode():
        for images, classes, targets, sample_ids in loader:
            logits, predictions = model(images.to(device))
            probabilities = logits.exp().cpu()
            predictions = predictions.cpu()
            predicted_classes = probabilities.argmax(dim=1)
            for item, sample_id in enumerate(sample_ids):
                actual = int(classes[item])
                predicted = int(predicted_classes[item])
                confusion[actual, predicted] += 1
                target = targets[item].numpy()
                prediction = predictions[item].numpy()
                record: dict[str, Any] = {
                    "sample_id": sample_id,
                    "actual_class": HELD_PART_CLASS_NAMES[actual],
                    "predicted_class": HELD_PART_CLASS_NAMES[predicted],
                    "probabilities": {
                        name: float(probabilities[item, class_id])
                        for class_id, name in enumerate(HELD_PART_CLASS_NAMES)
                    },
                    "target_pose": target.tolist() if actual else None,
                    "predicted_pose": prediction.tolist(),
                }
                if actual:
                    center_error = float(
                        np.linalg.norm(target[:2] - prediction[:2]) * dataset.image_size
                    )
                    angle_dot = float(
                        np.clip(np.dot(target[4:6], prediction[4:6]), -1, 1)
                    )
                    angle_error = math.degrees(0.5 * math.acos(angle_dot))
                    overlap = polygon_iou(
                        pose_to_polygon(target, dataset.image_size),
                        pose_to_polygon(prediction, dataset.image_size),
                    )
                    center_errors.append(center_error)
                    angle_errors.append(angle_error)
                    overlaps.append(overlap)
                    record.update(
                        {
                            "center_error_input_px": center_error,
                            "angle_error_degrees_mod_180": angle_error,
                            "oriented_iou": overlap,
                        }
                    )
                records.append(record)

    correct = int(np.trace(confusion))
    occupied_actual = int(confusion[1:, :].sum())
    occupied_true = int(confusion[1:, 1:].sum())
    occupied_predicted = int(confusion[:, 1:].sum())
    return {
        "sample_count": len(dataset),
        "state_accuracy": correct / len(dataset),
        "occupancy": {
            "precision": (
                occupied_true / occupied_predicted if occupied_predicted else 0.0
            ),
            "recall": occupied_true / occupied_actual if occupied_actual else 0.0,
        },
        "class_confusion": {
            actual: {
                predicted: int(confusion[actual_id, predicted_id])
                for predicted_id, predicted in enumerate(HELD_PART_CLASS_NAMES)
            }
            for actual_id, actual in enumerate(HELD_PART_CLASS_NAMES)
        },
        "pose_on_ground_truth_occupied": {
            "sample_count": len(center_errors),
            "mean_center_error_input_px": (
                float(np.mean(center_errors)) if center_errors else None
            ),
            "maximum_center_error_input_px": (
                max(center_errors) if center_errors else None
            ),
            "mean_angle_error_degrees_mod_180": (
                float(np.mean(angle_errors)) if angle_errors else None
            ),
            "mean_oriented_iou": float(np.mean(overlaps)) if overlaps else None,
        },
    }, records


def render_held_part_evaluation(
    dataset: HeldPartDataset,
    records: list[dict[str, Any]],
    output: Path,
) -> None:
    """Write observed-image diagnostics and one contact sheet."""

    output.mkdir(parents=True, exist_ok=True)
    by_id = {record["sample_id"]: record for record in records}
    cells: list[Image.Image] = []
    for index, sample in enumerate(dataset.samples):
        pair, _, target, _ = dataset[index]
        observed = pair[3:6]
        image = Image.fromarray(
            np.rint(observed.permute(1, 2, 0).numpy() * 255)
            .clip(0, 255)
            .astype(np.uint8)
        )
        draw = ImageDraw.Draw(image)
        record = by_id[sample.sample_id]
        if sample.class_id:
            target_polygon = pose_to_polygon(target.numpy(), dataset.image_size)
            draw.polygon(
                [tuple(point) for point in target_polygon], outline="lime", width=2
            )
        if record["predicted_class"] != "bare_nozzle":
            predicted = np.asarray(record["predicted_pose"], np.float32)
            predicted_polygon = pose_to_polygon(predicted, dataset.image_size)
            draw.polygon(
                [tuple(point) for point in predicted_polygon],
                outline="cyan",
                width=2,
            )
        label = f"GT {record['actual_class']}  P {record['predicted_class']}"
        draw.rectangle((0, 0, image.width, 18), fill="black")
        draw.text((3, 3), label, fill="white")
        image.save(output / f"{sample.sample_id}.png")
        cells.append(image)

    columns = 4
    rows = (len(cells) + columns - 1) // columns
    sheet = Image.new(
        "RGB", (columns * dataset.image_size, rows * dataset.image_size), "black"
    )
    for index, image in enumerate(cells):
        sheet.paste(
            image,
            (
                (index % columns) * dataset.image_size,
                (index // columns) * dataset.image_size,
            ),
        )
    sheet.save(output / "evaluation-contact-sheet.png")
