#!/usr/bin/env python3
"""Train, evaluate, and export the placement-verification model."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import random
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
    PLACEMENT_CLASS_NAMES,
    PlacementDataset,
    PlacementModelConfig,
    PlacementVerificationModel,
    placement_dataset_summary,
    placement_sampling_weights,
)
from velocitypnp_training.placement_evaluation import (  # noqa: E402
    evaluate_placements,
    render_placement_evaluation,
)


CONTRACT_VERSION = "velocitypnp.placement-verification.v1"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


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


def configure_determinism(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.use_deterministic_algorithms(True, warn_only=True)
    if hasattr(torch.backends, "cudnn"):
        torch.backends.cudnn.benchmark = False


def balanced_sampler(
    dataset: PlacementDataset, seed: int
) -> WeightedRandomSampler:
    weights = placement_sampling_weights(dataset.samples)
    return WeightedRandomSampler(
        weights,
        num_samples=len(dataset),
        replacement=True,
        generator=torch.Generator().manual_seed(seed),
    )


def class_weights(
    dataset: PlacementDataset, device: torch.device
) -> torch.Tensor:
    counts = Counter(sample.class_id for sample in dataset.samples)
    present = len(counts)
    total = len(dataset)
    return torch.tensor(
        [
            total / (present * counts[class_id]) if counts[class_id] else 0.0
            for class_id in range(len(PLACEMENT_CLASS_NAMES))
        ],
        dtype=torch.float32,
        device=device,
    )


def calculate_loss(
    model: PlacementVerificationModel,
    images: torch.Tensor,
    classes: torch.Tensor,
    targets: torch.Tensor,
    weights: torch.Tensor,
) -> tuple[torch.Tensor, dict[str, float]]:
    log_probabilities, predictions, center_logits = model.forward_training(images)
    classification = F.nll_loss(log_probabilities, classes, weight=weights)
    components = classes != 0
    if components.any():
        selected_predictions = predictions[components]
        selected_targets = targets[components]
        size = model.config.image_size
        center = F.smooth_l1_loss(
            selected_predictions[:, :2] * size,
            selected_targets[:, :2] * size,
        )
        extent = F.smooth_l1_loss(
            selected_predictions[:, 2:4] * size,
            selected_targets[:, 2:4] * size,
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
            target_heatmap * F.log_softmax(center_logits[components], dim=1)
        ).sum(dim=1).mean()
    else:
        zero = log_probabilities.sum() * 0.0
        center = extent = angle = heatmap = zero
    total = classification + 0.25 * center + 0.1 * extent + angle + 0.1 * heatmap
    return total, {
        "total": float(total.detach()),
        "classification": float(classification.detach()),
        "center": float(center.detach()),
        "extent": float(extent.detach()),
        "angle": float(angle.detach()),
        "heatmap": float(heatmap.detach()),
    }


def epoch_loss(
    model: PlacementVerificationModel,
    loader: DataLoader,
    device: torch.device,
    weights: torch.Tensor,
    optimizer: torch.optim.Optimizer | None,
) -> dict[str, float]:
    model.train(optimizer is not None)
    totals: Counter[str] = Counter()
    batches = 0
    context = torch.enable_grad() if optimizer is not None else torch.no_grad()
    with context:
        for images, classes, targets, _ in loader:
            images = images.to(device)
            classes = classes.to(device)
            targets = targets.to(device)
            if optimizer is not None:
                optimizer.zero_grad(set_to_none=True)
            loss, parts = calculate_loss(model, images, classes, targets, weights)
            if optimizer is not None:
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), 5.0)
                optimizer.step()
            totals.update(parts)
            batches += 1
    return {key: value / batches for key, value in totals.items()}


def export_onnx(
    model: PlacementVerificationModel,
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
        output_names=["state_log_probabilities", "pose"],
        dynamic_axes={
            "images": {0: "batch"},
            "state_log_probabilities": {0: "batch"},
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
    parser.add_argument("--name", default="placement-verification-v1")
    parser.add_argument("--image-size", type=int, default=192)
    parser.add_argument("--epochs", type=int, default=240)
    parser.add_argument("--patience", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--learning-rate", type=float, default=0.002)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--validate-only", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = args.dataset.resolve()
    summary = placement_dataset_summary(root)
    print(json.dumps(summary, indent=2), flush=True)
    if args.validate_only:
        return
    output = (args.output or root / "training" / args.name).resolve()
    if output.exists():
        raise ValueError(f"training output already exists: {output}")
    output.mkdir(parents=True)
    configure_determinism(args.seed)
    device = torch.device(
        "cuda"
        if args.device == "auto" and torch.cuda.is_available()
        else "cpu" if args.device == "auto" else args.device
    )
    if device.type == "cuda" and not torch.cuda.is_available():
        raise ValueError("CUDA was requested but is unavailable")

    config = PlacementModelConfig(image_size=args.image_size)
    model = PlacementVerificationModel(config).to(device)
    training = PlacementDataset(
        root, "train", args.image_size, augment=True, seed=args.seed
    )
    training_exact = PlacementDataset(root, "train", args.image_size)
    evaluation = PlacementDataset(root, "eval", args.image_size)
    train_loader = DataLoader(
        training,
        batch_size=args.batch_size,
        sampler=balanced_sampler(training, args.seed),
        num_workers=args.workers,
    )
    eval_loader = DataLoader(
        evaluation,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
    )
    weights = class_weights(training, device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=args.learning_rate, weight_decay=0.01
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer,
        T_max=args.epochs,
        eta_min=args.learning_rate * 0.02,
    )

    started_at = datetime.now(UTC)
    start_time = time.monotonic()
    history: list[dict[str, Any]] = []
    best_loss = math.inf
    best_epoch = 0
    best_state: dict[str, torch.Tensor] | None = None
    stale = 0
    for epoch in range(1, args.epochs + 1):
        record = {
            "epoch": epoch,
            "learning_rate": optimizer.param_groups[0]["lr"],
            "train": epoch_loss(
                model, train_loader, device, weights, optimizer
            ),
            "eval": epoch_loss(model, eval_loader, device, weights, None),
        }
        record["selection_loss"] = (
            record["eval"]["total"] + 0.1 * record["train"]["total"]
        )
        history.append(record)
        print(json.dumps(record), flush=True)
        if record["selection_loss"] < best_loss - 1e-5:
            best_loss = record["selection_loss"]
            best_epoch = epoch
            best_state = {
                key: value.detach().cpu().clone()
                for key, value in model.state_dict().items()
            }
            stale = 0
        else:
            stale += 1
        scheduler.step()
        if stale >= args.patience:
            break
    if best_state is None:
        raise RuntimeError("training did not produce a checkpoint")
    model.load_state_dict(best_state)
    model.to(device)
    training_metrics, training_records = evaluate_placements(
        model, training_exact, device, args.batch_size
    )
    metrics, records = evaluate_placements(
        model, evaluation, device, args.batch_size
    )
    render_placement_evaluation(
        training_exact, training_records, output / "training-evaluation"
    )
    render_placement_evaluation(evaluation, records, output / "evaluation")

    checkpoint = output / "best.pt"
    torch.save(
        {
            "contract_version": CONTRACT_VERSION,
            "model_config": config.to_dict(),
            "class_names": list(PLACEMENT_CLASS_NAMES),
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
            "parameter_count": sum(
                parameter.numel() for parameter in model.parameters()
            ),
            "pretrained_weights": False,
        },
        "model": {
            "config": config.to_dict(),
            "classes": list(PLACEMENT_CLASS_NAMES),
            "input": {
                "name": "images",
                "shape": ["batch", 3, args.image_size, args.image_size],
                "type": "float32",
                "range": [0.0, 1.0],
                "resize": "velocitypnp.half-pixel-bilinear-u8.v1",
                "letterbox_value": 127,
                "normalization_span_mm": training.output_span_mm,
                "target_center_normalized": [0.5, 0.5],
                "target_long_axis": "positive_x",
            },
            "outputs": {
                "state_log_probabilities": [
                    "batch",
                    len(PLACEMENT_CLASS_NAMES),
                ],
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
            "This is a one-machine, one-board-session development split.",
            "The evaluation split is too small for a production gate.",
            "The training set has no on-edge or multiple-component examples.",
            "The model artifact does not define daemon acceptance policy.",
        ],
    }
    (output / "training-history.json").write_text(
        json.dumps(history, indent=2) + "\n"
    )
    (output / "evaluation-records.json").write_text(
        json.dumps(records, indent=2) + "\n"
    )
    (output / "training-evaluation-records.json").write_text(
        json.dumps(training_records, indent=2) + "\n"
    )
    (output / "model-manifest.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
