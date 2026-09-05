"""Metrics and overlays for the placement-verification model."""

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
from .placement_verification import PLACEMENT_CLASS_NAMES, PlacementDataset


def evaluate_placements(
    model: torch.nn.Module,
    dataset: PlacementDataset,
    device: torch.device,
    batch_size: int,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Measure physical-state classification and conditional package pose."""

    loader = DataLoader(dataset, batch_size=batch_size, shuffle=False)
    class_count = len(PLACEMENT_CLASS_NAMES)
    confusion = np.zeros((class_count, class_count), dtype=np.int64)
    records: list[dict[str, Any]] = []
    center_errors: list[float] = []
    extent_errors: list[float] = []
    angle_errors: list[float] = []
    overlaps: list[float] = []
    model.eval()
    with torch.inference_mode():
        for images, classes, targets, sample_ids in loader:
            log_probabilities, predictions = model(images.to(device))
            probabilities = log_probabilities.exp().cpu()
            predictions = predictions.cpu()
            predicted_classes = probabilities.argmax(dim=1)
            for index, sample_id in enumerate(sample_ids):
                actual = int(classes[index])
                predicted = int(predicted_classes[index])
                confusion[actual, predicted] += 1
                target = targets[index].numpy()
                prediction = predictions[index].numpy()
                record: dict[str, Any] = {
                    "sample_id": sample_id,
                    "actual_class": PLACEMENT_CLASS_NAMES[actual],
                    "predicted_class": PLACEMENT_CLASS_NAMES[predicted],
                    "probabilities": {
                        name: float(probabilities[index, class_index])
                        for class_index, name in enumerate(PLACEMENT_CLASS_NAMES)
                    },
                    "target_pose": target.tolist() if actual else None,
                    "predicted_pose": prediction.tolist(),
                }
                if actual:
                    center_error = float(
                        np.linalg.norm(target[:2] - prediction[:2])
                        * dataset.image_size
                    )
                    extent_error = float(
                        np.mean(np.abs(target[2:4] - prediction[2:4]))
                        * dataset.image_size
                    )
                    angle_dot = float(
                        np.clip(np.dot(target[4:6], prediction[4:6]), -1, 1)
                    )
                    angle_error = math.degrees(0.5 * math.acos(angle_dot))
                    target_polygon = pose_to_polygon(target, dataset.image_size)
                    predicted_polygon = pose_to_polygon(
                        prediction, dataset.image_size
                    )
                    overlap = polygon_iou(target_polygon, predicted_polygon)
                    center_errors.append(center_error)
                    extent_errors.append(extent_error)
                    angle_errors.append(angle_error)
                    overlaps.append(overlap)
                    record.update(
                        {
                            "center_error_input_px": center_error,
                            "center_error_mm": (
                                center_error
                                * dataset.output_span_mm
                                / dataset.image_size
                            ),
                            "mean_extent_error_input_px": extent_error,
                            "angle_error_degrees_mod_180": angle_error,
                            "oriented_iou": overlap,
                        }
                    )
                records.append(record)

    correct = int(np.trace(confusion))
    sample_count = int(confusion.sum())
    occupied_actual = confusion[1:, :].sum()
    occupied_predicted = confusion[:, 1:].sum()
    occupied_true = confusion[1:, 1:].sum()
    false_positive = int(confusion[0, 1:].sum())
    false_negative = int(confusion[1:, 0].sum())
    return {
        "sample_count": sample_count,
        "classification_accuracy": correct / sample_count if sample_count else 0.0,
        "occupancy": {
            "true_positives": int(occupied_true),
            "false_positives": false_positive,
            "false_negatives": false_negative,
            "precision": (
                float(occupied_true / occupied_predicted)
                if occupied_predicted
                else 0.0
            ),
            "recall": (
                float(occupied_true / occupied_actual) if occupied_actual else 0.0
            ),
        },
        "class_confusion": {
            actual: {
                predicted: int(confusion[actual_index, predicted_index])
                for predicted_index, predicted in enumerate(
                    PLACEMENT_CLASS_NAMES
                )
            }
            for actual_index, actual in enumerate(PLACEMENT_CLASS_NAMES)
        },
        "pose_on_ground_truth_component": {
            "sample_count": len(center_errors),
            "mean_center_error_input_px": (
                float(np.mean(center_errors)) if center_errors else None
            ),
            "mean_center_error_mm": (
                float(np.mean(center_errors))
                * dataset.output_span_mm
                / dataset.image_size
                if center_errors
                else None
            ),
            "maximum_center_error_mm": (
                max(center_errors) * dataset.output_span_mm / dataset.image_size
                if center_errors
                else None
            ),
            "mean_extent_error_input_px": (
                float(np.mean(extent_errors)) if extent_errors else None
            ),
            "mean_angle_error_degrees_mod_180": (
                float(np.mean(angle_errors)) if angle_errors else None
            ),
            "mean_oriented_iou": float(np.mean(overlaps)) if overlaps else None,
        },
    }, records


def render_placement_evaluation(
    dataset: PlacementDataset,
    records: list[dict[str, Any]],
    output: Path,
) -> None:
    """Write annotated samples and one comparison sheet."""

    output.mkdir(parents=True, exist_ok=True)
    by_id = {record["sample_id"]: record for record in records}
    cells: list[Image.Image] = []
    for index, sample in enumerate(dataset.samples):
        record = by_id[sample.sample_id]
        image_tensor, _, target, _ = dataset[index]
        image = Image.fromarray(
            np.rint(image_tensor.permute(1, 2, 0).numpy() * 255)
            .clip(0, 255)
            .astype(np.uint8)
        )
        draw = ImageDraw.Draw(image)
        center = dataset.image_size // 2
        draw.line((center - 5, center, center + 5, center), fill="orange")
        draw.line((center, center - 5, center, center + 5), fill="orange")
        if sample.class_id:
            target_polygon = pose_to_polygon(target.numpy(), dataset.image_size)
            draw.polygon(
                [tuple(point) for point in target_polygon],
                outline="magenta",
                width=2,
            )
        if record["predicted_class"] != "empty":
            predicted = np.asarray(record["predicted_pose"], np.float32)
            predicted_polygon = pose_to_polygon(predicted, dataset.image_size)
            draw.polygon(
                [tuple(point) for point in predicted_polygon],
                outline="cyan",
                width=2,
            )
        if record["predicted_class"] != record["actual_class"]:
            draw.rectangle(
                (1, 1, image.width - 2, image.height - 2),
                outline="red",
                width=3,
            )
        label = (
            f"{sample.reference} GT {record['actual_class']} "
            f"P {record['predicted_class']}"
        )
        draw.rectangle((0, 0, image.width, 18), fill="black")
        draw.text((3, 3), label, fill="white")
        image.save(output / f"{sample.sample_id}.png")
        cells.append(image)

    columns = 4
    size = dataset.image_size
    rows = (len(cells) + columns - 1) // columns
    sheet = Image.new("RGB", (columns * size, rows * size), "black")
    for index, image in enumerate(cells):
        sheet.paste(image, ((index % columns) * size, (index // columns) * size))
    sheet.save(output / "evaluation-contact-sheet.png")
