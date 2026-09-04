"""Evaluation and diagnostic rendering for source-component pose models."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
import torch
from PIL import Image, ImageDraw
from torch.utils.data import DataLoader

from .dataset import CLASS_NAMES, SourceComponentDataset, pose_to_polygon


def _polygon_area(points: np.ndarray) -> float:
    shifted = np.roll(points, -1, axis=0)
    return abs(float(np.sum(points[:, 0] * shifted[:, 1] - shifted[:, 0] * points[:, 1]))) / 2


def _inside(point: np.ndarray, first: np.ndarray, second: np.ndarray) -> bool:
    edge = second - first
    relative = point - first
    return float(edge[0] * relative[1] - edge[1] * relative[0]) >= -1e-6


def _intersection(
    start: np.ndarray,
    end: np.ndarray,
    clip_start: np.ndarray,
    clip_end: np.ndarray,
) -> np.ndarray:
    first = end - start
    second = clip_end - clip_start
    denominator = first[0] * second[1] - first[1] * second[0]
    if abs(float(denominator)) < 1e-8:
        return end
    offset = clip_start - start
    parameter = (offset[0] * second[1] - offset[1] * second[0]) / denominator
    return start + parameter * first


def polygon_iou(first: np.ndarray, second: np.ndarray) -> float:
    """Calculate intersection over union for two convex rectangles."""

    output = [point.copy() for point in first]
    for index, clip_start in enumerate(second):
        clip_end = second[(index + 1) % len(second)]
        input_points = output
        output = []
        if not input_points:
            break
        start = input_points[-1]
        for end in input_points:
            end_inside = _inside(end, clip_start, clip_end)
            start_inside = _inside(start, clip_start, clip_end)
            if end_inside:
                if not start_inside:
                    output.append(_intersection(start, end, clip_start, clip_end))
                output.append(end)
            elif start_inside:
                output.append(_intersection(start, end, clip_start, clip_end))
            start = end
    intersection = _polygon_area(np.asarray(output)) if output else 0.0
    union = _polygon_area(first) + _polygon_area(second) - intersection
    return intersection / union if union > 0 else 0.0


def evaluate(
    model: torch.nn.Module,
    dataset: SourceComponentDataset,
    device: torch.device,
    batch_size: int,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Evaluate fixed argmax occupancy and conditional pose quality."""

    loader = DataLoader(dataset, batch_size=batch_size, shuffle=False)
    confusion = np.zeros((len(CLASS_NAMES), len(CLASS_NAMES)), dtype=np.int64)
    records: list[dict[str, Any]] = []
    center_errors: list[float] = []
    size_errors: list[float] = []
    angle_errors: list[float] = []
    occupied_ious: list[float] = []
    true_positives = false_positives = false_negatives = 0
    model.eval()
    with torch.inference_mode():
        for images, classes, targets, sample_ids in loader:
            logits, predictions = model(images.to(device))
            probabilities = torch.softmax(logits, dim=1).cpu()
            predictions = predictions.cpu()
            predicted_classes = probabilities.argmax(dim=1)
            for index, sample_id in enumerate(sample_ids):
                actual = int(classes[index])
                predicted = int(predicted_classes[index])
                confusion[actual, predicted] += 1
                actual_occupied = actual != 0
                predicted_occupied = predicted != 0
                if actual_occupied and predicted_occupied:
                    true_positives += 1
                elif actual_occupied:
                    false_negatives += 1
                elif predicted_occupied:
                    false_positives += 1

                target = targets[index].numpy()
                prediction = predictions[index].numpy()
                record: dict[str, Any] = {
                    "sample_id": sample_id,
                    "actual_class": CLASS_NAMES[actual],
                    "predicted_class": CLASS_NAMES[predicted],
                    "probabilities": {
                        name: float(probabilities[index, class_index])
                        for class_index, name in enumerate(CLASS_NAMES)
                    },
                    "target_pose": target.tolist() if actual_occupied else None,
                    "predicted_pose": prediction.tolist(),
                }
                if actual_occupied:
                    center_error = float(
                        np.linalg.norm(target[:2] - prediction[:2])
                        * dataset.image_size
                    )
                    size_error = float(
                        np.mean(np.abs(target[2:4] - prediction[2:4]))
                        * dataset.image_size
                    )
                    angle_dot = float(np.clip(np.dot(target[4:6], prediction[4:6]), -1, 1))
                    angle_error = math.degrees(0.5 * math.acos(angle_dot))
                    target_polygon = pose_to_polygon(target, dataset.image_size)
                    predicted_polygon = pose_to_polygon(prediction, dataset.image_size)
                    overlap = polygon_iou(target_polygon, predicted_polygon)
                    center_errors.append(center_error)
                    size_errors.append(size_error)
                    angle_errors.append(angle_error)
                    occupied_ious.append(overlap)
                    record.update(
                        {
                            "center_error_input_px": center_error,
                            "mean_size_error_input_px": size_error,
                            "angle_error_degrees_mod_180": angle_error,
                            "oriented_iou": overlap,
                        }
                    )
                records.append(record)

    precision = (
        true_positives / (true_positives + false_positives)
        if true_positives + false_positives
        else 0.0
    )
    recall = (
        true_positives / (true_positives + false_negatives)
        if true_positives + false_negatives
        else 0.0
    )
    metrics = {
        "sample_count": len(dataset),
        "occupancy": {
            "true_positives": true_positives,
            "false_positives": false_positives,
            "false_negatives": false_negatives,
            "precision": precision,
            "recall": recall,
        },
        "class_confusion": {
            actual: {
                predicted: int(confusion[actual_index, predicted_index])
                for predicted_index, predicted in enumerate(CLASS_NAMES)
            }
            for actual_index, actual in enumerate(CLASS_NAMES)
        },
        "pose_on_ground_truth_occupied": {
            "sample_count": len(center_errors),
            "mean_center_error_input_px": (
                float(np.mean(center_errors)) if center_errors else None
            ),
            "maximum_center_error_input_px": (
                max(center_errors) if center_errors else None
            ),
            "mean_size_error_input_px": (
                float(np.mean(size_errors)) if size_errors else None
            ),
            "mean_angle_error_degrees_mod_180": (
                float(np.mean(angle_errors)) if angle_errors else None
            ),
            "mean_oriented_iou": (
                float(np.mean(occupied_ious)) if occupied_ious else None
            ),
        },
    }
    return metrics, records


def render_evaluation(
    dataset: SourceComponentDataset,
    records: list[dict[str, Any]],
    output: Path,
) -> None:
    """Write per-sample overlays and one compact contact sheet."""

    output.mkdir(parents=True, exist_ok=True)
    by_id = {record["sample_id"]: record for record in records}
    cells: list[Image.Image] = []
    for sample in dataset.samples:
        record = by_id[sample.sample_id]
        image_tensor, _, target, _ = dataset[dataset.samples.index(sample)]
        image = Image.fromarray(
            np.rint(image_tensor.permute(1, 2, 0).numpy() * 255)
            .clip(0, 255)
            .astype(np.uint8)
        )
        draw = ImageDraw.Draw(image)
        if sample.class_id:
            target_polygon = pose_to_polygon(target.numpy(), dataset.image_size)
            draw.polygon([tuple(point) for point in target_polygon], outline="lime", width=2)
        prediction = np.asarray(record["predicted_pose"], np.float32)
        predicted_polygon = pose_to_polygon(prediction, dataset.image_size)
        color = "cyan" if record["predicted_class"] != "empty" else "red"
        if record["predicted_class"] != "empty":
            draw.polygon(
                [tuple(point) for point in predicted_polygon], outline=color, width=2
            )
        label = f"GT {record['actual_class']}  P {record['predicted_class']}"
        draw.rectangle((0, 0, image.width, 18), fill="black")
        draw.text((3, 3), label, fill="white")
        image.save(output / f"{sample.sample_id}.png")
        cells.append(image)

    columns = 4
    cell_size = dataset.image_size
    rows = (len(cells) + columns - 1) // columns
    sheet = Image.new("RGB", (columns * cell_size, rows * cell_size), "black")
    for index, image in enumerate(cells):
        sheet.paste(image, ((index % columns) * cell_size, (index // columns) * cell_size))
    sheet.save(output / "evaluation-contact-sheet.png")
