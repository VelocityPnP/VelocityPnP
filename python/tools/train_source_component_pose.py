#!/usr/bin/env python3
"""Train, evaluate, and export the VelocityPnP pocket-pose model."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import random
import shutil
import subprocess
import sys
import time
from collections import Counter
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F
from torch.utils.data import DataLoader, WeightedRandomSampler


REPOSITORY = Path(__file__).resolve().parents[2]
TRAINING_PACKAGE = REPOSITORY / "python" / "training"
sys.path.insert(0, str(TRAINING_PACKAGE))

from velocitypnp_training import (  # noqa: E402
    CLASS_NAMES,
    ModelConfig,
    SourceComponentDataset,
    SourceComponentPoseModel,
    dataset_summary,
)
from velocitypnp_training.evaluation import evaluate, render_evaluation  # noqa: E402


CONTRACT_VERSION = "velocitypnp.source-component-pose.v2"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_revision() -> dict[str, Any]:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=REPOSITORY,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=REPOSITORY,
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        )
        return {"commit": commit, "dirty": dirty}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": None, "dirty": None}


def source_sha256() -> str:
    digest = hashlib.sha256()
    paths = [
        Path(__file__),
        *(TRAINING_PACKAGE / "velocitypnp_training").glob("*.py"),
    ]
    for path in sorted(paths):
        digest.update(str(path.relative_to(REPOSITORY)).encode())
        digest.update(b"\0")
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def configure_determinism(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.use_deterministic_algorithms(True, warn_only=True)
    if hasattr(torch.backends, "cudnn"):
        torch.backends.cudnn.benchmark = False


def classification_balance(
    dataset: SourceComponentDataset,
    device: torch.device,
    sample_weights: list[float] | None = None,
) -> torch.Tensor:
    weights = sample_weights or [1.0] * len(dataset.samples)
    if len(weights) != len(dataset.samples):
        raise ValueError("classification weights do not match the dataset")
    counts: Counter[int] = Counter()
    for sample, weight in zip(dataset.samples, weights, strict=True):
        counts[sample.class_id] += weight
    empty = counts[0]
    flat = counts[1]
    on_edge = counts[2]
    if min(empty, flat, on_edge) <= 0:
        raise ValueError("classification balance requires all three classes")
    return torch.tensor(
        [empty / (flat + on_edge), flat / on_edge],
        dtype=torch.float32,
        device=device,
    )


def classification_loss(
    state_logits: torch.Tensor,
    classes: torch.Tensor,
    balance: torch.Tensor,
) -> torch.Tensor:
    occupied = (classes != 0).to(torch.float32)
    occupancy_loss = F.binary_cross_entropy_with_logits(
        state_logits[:, 0], occupied, pos_weight=balance[0]
    )
    occupied_mask = classes != 0
    if not occupied_mask.any():
        return occupancy_loss
    on_edge = (classes[occupied_mask] == 2).to(torch.float32)
    edge_loss = F.binary_cross_entropy_with_logits(
        state_logits[occupied_mask, 1],
        on_edge,
        pos_weight=balance[1],
    )
    return occupancy_loss + edge_loss


def classifier_selection_score(
    policy: str,
    eval_result: dict[str, float | int],
    train_result: dict[str, float | int],
    combined_error_rate: float,
    selection_loss: float,
) -> tuple[float, ...]:
    eval_false_positives = int(eval_result["false_positives"])
    train_false_positives = int(train_result["false_positives"])
    if policy == "all-zero-fp":
        return (
            eval_false_positives + train_false_positives,
            combined_error_rate,
            selection_loss,
        )
    if policy == "eval-first":
        return (
            eval_false_positives,
            int(eval_result["errors"]),
            train_false_positives,
            int(train_result["errors"]),
            selection_loss,
        )
    raise ValueError(f"unknown classifier selection policy: {policy}")


def calculate_loss(
    model: SourceComponentPoseModel,
    images: torch.Tensor,
    classes: torch.Tensor,
    targets: torch.Tensor,
    balance: torch.Tensor,
) -> tuple[torch.Tensor, dict[str, float]]:
    logits, predictions, center_logits, state_logits = model.forward_training(images)
    classification = classification_loss(state_logits, classes, balance)
    occupied = classes != 0
    if occupied.any():
        selected_predictions = predictions[occupied]
        selected_targets = targets[occupied]
        image_size = model.config.image_size
        center = F.smooth_l1_loss(
            selected_predictions[:, :2] * image_size,
            selected_targets[:, :2] * image_size,
        )
        size = F.smooth_l1_loss(
            selected_predictions[:, 2:4] * image_size,
            selected_targets[:, 2:4] * image_size,
        )
        angle = 1.0 - (
            selected_predictions[:, 4:6] * selected_targets[:, 4:6]
        ).sum(dim=1).mean()
        grid = model.center_grid.unsqueeze(0)
        distances = (
            grid - selected_targets[:, None, :2]
        ).square().sum(dim=2)
        target_heatmap = F.softmax(-distances / (2.0 * 0.02**2), dim=1)
        heatmap = -(
            target_heatmap * F.log_softmax(center_logits[occupied], dim=1)
        ).sum(dim=1).mean()
    else:
        zero = logits.sum() * 0.0
        center = size = angle = heatmap = zero
    total = classification + 0.25 * center + 0.1 * size + angle + 0.1 * heatmap
    parts = {
        "total": float(total.detach()),
        "classification": float(classification.detach()),
        "center": float(center.detach()),
        "size": float(size.detach()),
        "angle": float(angle.detach()),
        "heatmap": float(heatmap.detach()),
    }
    return total, parts


def epoch_loss(
    model: SourceComponentPoseModel,
    loader: DataLoader,
    device: torch.device,
    balance: torch.Tensor,
    optimizer: torch.optim.Optimizer | None,
) -> dict[str, float]:
    training = optimizer is not None
    model.train(training)
    totals: Counter[str] = Counter()
    batches = 0
    context = torch.enable_grad() if training else torch.no_grad()
    with context:
        for images, classes, targets, _ in loader:
            images = images.to(device)
            classes = classes.to(device)
            targets = targets.to(device)
            if optimizer is not None:
                optimizer.zero_grad(set_to_none=True)
            loss, parts = calculate_loss(model, images, classes, targets, balance)
            if optimizer is not None:
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), 5.0)
                optimizer.step()
            totals.update(parts)
            batches += 1
    return {key: value / batches for key, value in totals.items()}


def classifier_epoch(
    model: SourceComponentPoseModel,
    loader: DataLoader,
    device: torch.device,
    balance: torch.Tensor,
    optimizer: torch.optim.Optimizer | None,
) -> dict[str, float | int]:
    """Fit only the occupancy head while the learned pose features stay fixed."""

    model.eval()
    if optimizer is not None:
        model.occupancy_features.train()
        model.classifier.train()
    total = 0.0
    batches = 0
    errors = 0
    false_positives = 0
    false_negatives = 0
    context = torch.enable_grad() if optimizer is not None else torch.no_grad()
    with context:
        for images, classes, _, _ in loader:
            images = images.to(device)
            classes = classes.to(device)
            if optimizer is not None:
                optimizer.zero_grad(set_to_none=True)
            logits, _, _, state_logits = model.forward_training(images)
            loss = (
                classification_loss(state_logits, classes, balance)
                if optimizer is not None
                else F.nll_loss(logits, classes)
            )
            if optimizer is not None:
                loss.backward()
                optimizer.step()
            total += float(loss.detach())
            predicted_classes = logits.argmax(dim=1)
            errors += int((predicted_classes != classes).sum().detach())
            false_positives += int(
                ((classes == 0) & (predicted_classes != 0)).sum().detach()
            )
            false_negatives += int(
                ((classes != 0) & (predicted_classes == 0)).sum().detach()
            )
            batches += 1
    return {
        "loss": total / batches,
        "errors": errors,
        "false_positives": false_positives,
        "false_negatives": false_negatives,
    }


def export_onnx(
    model: SourceComponentPoseModel,
    destination: Path,
    image_size: int,
) -> None:
    model = model.cpu().eval()
    sample = torch.zeros(1, 3, image_size, image_size)
    torch.onnx.export(
        model,
        sample,
        destination,
        input_names=["images"],
        output_names=["occupancy_logits", "pose"],
        dynamic_axes={
            "images": {0: "batch"},
            "occupancy_logits": {0: "batch"},
            "pose": {0: "batch"},
        },
        opset_version=17,
        do_constant_folding=True,
        dynamo=False,
    )
    import onnx

    artifact = onnx.load(destination)
    onnx.checker.check_model(artifact)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--name", default="source-component-pose-v2")
    parser.add_argument("--image-size", type=int, default=192)
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--patience", type=int, default=40)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--learning-rate", type=float, default=0.002)
    parser.add_argument("--classifier-epochs", type=int, default=100)
    parser.add_argument("--classifier-patience", type=int, default=25)
    parser.add_argument("--classifier-learning-rate", type=float, default=0.001)
    parser.add_argument("--hard-negative-occupied-probability", type=float, default=0.2)
    parser.add_argument("--hard-negative-weight", type=float, default=8.0)
    parser.add_argument("--classifier-train-selection-weight", type=float, default=0.5)
    parser.add_argument(
        "--classifier-selection",
        choices=("all-zero-fp", "eval-first"),
        default="all-zero-fp",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--validate-only", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not 0.0 <= args.hard_negative_occupied_probability <= 1.0:
        raise ValueError("hard-negative occupied probability must be from 0 to 1")
    if args.hard_negative_weight < 1.0:
        raise ValueError("hard-negative weight must be at least 1")
    if args.classifier_train_selection_weight < 0.0:
        raise ValueError("classifier train selection weight must not be negative")
    dataset_root = args.dataset.resolve()
    summary = dataset_summary(dataset_root)
    print(json.dumps(summary, indent=2))
    if args.validate_only:
        return

    output = (args.output or dataset_root / "training" / args.name).resolve()
    if output.exists():
        raise ValueError(f"training output already exists: {output}")
    output.mkdir(parents=True)
    configure_determinism(args.seed)
    if args.device == "auto":
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    else:
        device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise ValueError("CUDA was requested but is unavailable")

    config = ModelConfig(image_size=args.image_size)
    model = SourceComponentPoseModel(config).to(device)
    train_dataset = SourceComponentDataset(
        dataset_root, "train", args.image_size, augment=True, seed=args.seed
    )
    eval_dataset = SourceComponentDataset(
        dataset_root, "eval", args.image_size, augment=False, seed=args.seed
    )
    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        generator=torch.Generator().manual_seed(args.seed),
    )
    eval_loader = DataLoader(
        eval_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
    )
    balance = classification_balance(train_dataset, device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=args.learning_rate, weight_decay=0.01
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=args.epochs, eta_min=args.learning_rate * 0.02
    )

    started_at = datetime.now(UTC)
    start_time = time.monotonic()
    history: list[dict[str, Any]] = []
    best_loss = math.inf
    best_epoch = 0
    best_state: dict[str, torch.Tensor] | None = None
    stale_epochs = 0
    for epoch in range(1, args.epochs + 1):
        train_losses = epoch_loss(model, train_loader, device, balance, optimizer)
        eval_losses = epoch_loss(model, eval_loader, device, balance, None)
        record = {
            "epoch": epoch,
            "learning_rate": optimizer.param_groups[0]["lr"],
            "train": train_losses,
            "eval": eval_losses,
        }
        history.append(record)
        print(json.dumps(record), flush=True)
        if eval_losses["total"] < best_loss - 1e-5:
            best_loss = eval_losses["total"]
            best_epoch = epoch
            best_state = {
                key: value.detach().cpu().clone()
                for key, value in model.state_dict().items()
            }
            stale_epochs = 0
        else:
            stale_epochs += 1
        scheduler.step()
        if stale_epochs >= args.patience:
            break
    if best_state is None:
        raise RuntimeError("training did not produce a checkpoint")
    model.load_state_dict(best_state)
    model.to(device)

    train_evaluation_dataset = SourceComponentDataset(
        dataset_root, "train", args.image_size, augment=False, seed=args.seed
    )
    pre_calibration_metrics, pre_calibration_records = evaluate(
        model,
        train_evaluation_dataset,
        device,
        batch_size=args.batch_size,
    )
    hard_negative_ids = {
        record["sample_id"]
        for record in pre_calibration_records
        if record["actual_class"] == "empty"
        and 1.0 - record["probabilities"]["empty"]
        >= args.hard_negative_occupied_probability
    }
    classifier_sample_weights = [
        args.hard_negative_weight if sample.sample_id in hard_negative_ids else 1.0
        for sample in train_dataset.samples
    ]
    classifier_sampler = WeightedRandomSampler(
        classifier_sample_weights,
        num_samples=round(sum(classifier_sample_weights)),
        replacement=True,
        generator=torch.Generator().manual_seed(args.seed + 1),
    )
    classifier_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        sampler=classifier_sampler,
        num_workers=args.workers,
    )
    train_evaluation_loader = DataLoader(
        train_evaluation_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        generator=torch.Generator().manual_seed(args.seed + 2),
    )
    train_evaluation_check_loader = DataLoader(
        train_evaluation_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
    )
    classifier_balance = classification_balance(
        train_dataset, device, classifier_sample_weights
    )

    for parameter in model.parameters():
        parameter.requires_grad = False
    for parameter in model.occupancy_features.parameters():
        parameter.requires_grad = True
    for parameter in model.classifier.parameters():
        parameter.requires_grad = True
    classifier_optimizer = torch.optim.AdamW(
        [
            *model.occupancy_features.parameters(),
            *model.classifier.parameters(),
        ],
        lr=args.classifier_learning_rate,
        weight_decay=0.01,
    )
    classifier_history: list[dict[str, Any]] = []
    best_classifier_score: tuple[float, ...] | None = None
    best_classifier_epoch = 0
    best_classifier_state: dict[str, torch.Tensor] | None = None
    stale_classifier_epochs = 0
    for epoch in range(1, args.classifier_epochs + 1):
        train_augmented = classifier_epoch(
            model,
            classifier_loader,
            device,
            classifier_balance,
            classifier_optimizer,
        )
        train_exact = classifier_epoch(
            model,
            train_evaluation_loader,
            device,
            balance,
            classifier_optimizer,
        )
        train_unaugmented = classifier_epoch(
            model, train_evaluation_check_loader, device, balance, None
        )
        eval_classifier = classifier_epoch(
            model, eval_loader, device, balance, None
        )
        selection_loss = (
            float(eval_classifier["loss"])
            + args.classifier_train_selection_weight
            * float(train_unaugmented["loss"])
        )
        combined_error_rate = (
            float(eval_classifier["errors"]) / len(eval_dataset)
            + float(train_unaugmented["errors"]) / len(train_evaluation_dataset)
        )
        selection_score = classifier_selection_score(
            args.classifier_selection,
            eval_classifier,
            train_unaugmented,
            combined_error_rate,
            selection_loss,
        )
        classifier_history.append(
            {
                "epoch": epoch,
                "train_augmented_classification": train_augmented,
                "train_exact_classification": train_exact,
                "train_unaugmented_classification": train_unaugmented,
                "eval_classification": eval_classifier,
                "selection_loss": selection_loss,
                "combined_error_rate": combined_error_rate,
                "selection_score": list(selection_score),
            }
        )
        if best_classifier_score is None or selection_score < best_classifier_score:
            best_classifier_score = selection_score
            best_classifier_epoch = epoch
            best_classifier_state = {
                key: value.detach().cpu().clone()
                for key, value in model.state_dict().items()
            }
            stale_classifier_epochs = 0
        else:
            stale_classifier_epochs += 1
        if stale_classifier_epochs >= args.classifier_patience:
            break
    if best_classifier_state is None:
        raise RuntimeError("classifier calibration did not produce a checkpoint")
    selected_classifier = classifier_history[best_classifier_epoch - 1]
    model.load_state_dict(best_classifier_state)
    for parameter in model.parameters():
        parameter.requires_grad = True
    best_state = {
        key: value.detach().cpu().clone() for key, value in model.state_dict().items()
    }

    training_metrics, training_records = evaluate(
        model,
        train_evaluation_dataset,
        device,
        batch_size=args.batch_size,
    )
    metrics, records = evaluate(
        model, eval_dataset, device, batch_size=args.batch_size
    )
    render_evaluation(
        train_evaluation_dataset,
        training_records,
        output / "training-evaluation",
    )
    render_evaluation(eval_dataset, records, output / "evaluation")
    remaining_hard_negative_false_positives = [
        record["sample_id"]
        for record in training_records
        if record["sample_id"] in hard_negative_ids
        and record["predicted_class"] != "empty"
    ]
    checkpoint = output / "best.pt"
    torch.save(
        {
            "contract_version": CONTRACT_VERSION,
            "model_config": config.to_dict(),
            "class_names": list(CLASS_NAMES),
            "state_dict": best_state,
        },
        checkpoint,
    )
    onnx_path = output / "best.onnx"
    export_onnx(model, onnx_path, args.image_size)

    report = {
        "schema_version": 1,
        "contract_version": CONTRACT_VERSION,
        "started_at": started_at.isoformat(),
        "completed_at": datetime.now(UTC).isoformat(),
        "duration_seconds": time.monotonic() - start_time,
        "dataset": summary,
        "source": {
            "git": git_revision(),
            "training_source_sha256": source_sha256(),
        },
        "environment": {
            "hostname": platform.node(),
            "platform": platform.platform(),
            "python": platform.python_version(),
            "torch": torch.__version__,
            "cuda_available": torch.cuda.is_available(),
            "cuda_version": torch.version.cuda,
            "device": str(device),
            "device_name": (
                torch.cuda.get_device_name(device)
                if device.type == "cuda"
                else platform.processor()
            ),
        },
        "training": {
            "requested_epochs": args.epochs,
            "completed_epochs": len(history),
            "best_epoch": best_epoch,
            "early_stopping_patience": args.patience,
            "batch_size": args.batch_size,
            "learning_rate": args.learning_rate,
            "seed": args.seed,
            "parameter_count": sum(parameter.numel() for parameter in model.parameters()),
            "pretrained_weights": False,
            "classifier_calibration": {
                "selection_policy": args.classifier_selection,
                "requested_epochs": args.classifier_epochs,
                "completed_epochs": len(classifier_history),
                "best_epoch": best_classifier_epoch,
                "early_stopping_patience": args.classifier_patience,
                "learning_rate": args.classifier_learning_rate,
                "train_selection_weight": args.classifier_train_selection_weight,
                "selection_false_positives": (
                    selected_classifier["eval_classification"]["false_positives"]
                    + selected_classifier["train_unaugmented_classification"][
                        "false_positives"
                    ]
                ),
                "selection_eval_false_positives": selected_classifier[
                    "eval_classification"
                ]["false_positives"],
                "selection_train_false_positives": selected_classifier[
                    "train_unaugmented_classification"
                ]["false_positives"],
                "selection_combined_error_rate": selected_classifier[
                    "combined_error_rate"
                ],
                "selection_errors": {
                    "eval": int(
                        classifier_history[best_classifier_epoch - 1][
                            "eval_classification"
                        ]["errors"]
                    ),
                    "train": int(
                        classifier_history[best_classifier_epoch - 1][
                            "train_unaugmented_classification"
                        ]["errors"]
                    ),
                },
                "selection_loss": selected_classifier["selection_loss"],
            },
            "hard_negative_mining": {
                "occupied_probability": args.hard_negative_occupied_probability,
                "sample_weight": args.hard_negative_weight,
                "sample_count": len(hard_negative_ids),
                "sample_ids": sorted(hard_negative_ids),
                "pre_calibration_metrics": pre_calibration_metrics,
                "remaining_false_positive_count": len(
                    remaining_hard_negative_false_positives
                ),
                "remaining_false_positive_ids": remaining_hard_negative_false_positives,
            },
        },
        "model": {
            "config": config.to_dict(),
            "classes": list(CLASS_NAMES),
            "input": {
                "name": "images",
                "shape": ["batch", 3, args.image_size, args.image_size],
                "type": "float32",
                "range": [0.0, 1.0],
                "resize": "velocitypnp.half-pixel-bilinear-u8.v1",
                "letterbox_value": 127,
            },
            "outputs": {
                "occupancy_logits": ["batch", len(CLASS_NAMES)],
                "pose": {
                    "shape": ["batch", 6],
                    "fields": [
                        "center_x_normalized",
                        "center_y_normalized",
                        "width_normalized",
                        "height_normalized",
                        "cos_2theta",
                        "sin_2theta",
                    ],
                },
            },
            "angle_period_degrees": 180,
            "polarity_supported": False,
        },
        "license": {
            "artifact": "Apache-2.0",
            "training_code": "Apache-2.0",
            "training_framework": "BSD-3-Clause",
        },
        "artifacts": {
            "checkpoint": {
                "path": checkpoint.name,
                "sha256": sha256_file(checkpoint),
                "size_bytes": checkpoint.stat().st_size,
            },
            "onnx": {
                "path": onnx_path.name,
                "sha256": sha256_file(onnx_path),
                "size_bytes": onnx_path.stat().st_size,
                "opset": 17,
            },
        },
        "metrics": metrics,
        "training_metrics": training_metrics,
        "limitations": [
            "The evaluation split is too small for a production gate.",
            *(
                [
                    "The evaluation split has no examples of: "
                    + ", ".join(summary["evaluation_limitations"])
                    + "."
                ]
                if summary["evaluation_limitations"]
                else []
            ),
            "The data comes from one machine and a small number of sessions.",
            "The v2 contract does not predict component polarity.",
        ],
    }
    (output / "training-history.json").write_text(json.dumps(history, indent=2) + "\n")
    (output / "classifier-history.json").write_text(
        json.dumps(classifier_history, indent=2) + "\n"
    )
    (output / "evaluation-records.json").write_text(
        json.dumps(records, indent=2) + "\n"
    )
    (output / "training-evaluation-records.json").write_text(
        json.dumps(training_records, indent=2) + "\n"
    )
    (output / "model-manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
