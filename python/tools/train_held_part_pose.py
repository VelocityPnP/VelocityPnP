#!/usr/bin/env python3
"""Train and export the paired-image held-part pose model."""

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
    HELD_PART_CLASS_NAMES,
    HeldPartDataset,
    HeldPartModelConfig,
    HeldPartPoseModel,
    held_part_dataset_summary,
)
from velocitypnp_training.held_part_evaluation import (  # noqa: E402
    evaluate_held_part,
    render_held_part_evaluation,
)

CONTRACT_VERSION = "velocitypnp.held-part-pose.v1"


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


def balanced_sampler(dataset: HeldPartDataset, seed: int) -> WeightedRandomSampler:
    counts = Counter(sample.class_id for sample in dataset.samples)
    if set(counts) != set(range(len(HELD_PART_CLASS_NAMES))):
        raise ValueError("training split does not contain every model class")
    weights = [1.0 / counts[sample.class_id] for sample in dataset.samples]
    return WeightedRandomSampler(
        weights,
        num_samples=len(dataset),
        replacement=True,
        generator=torch.Generator().manual_seed(seed),
    )


def calculate_loss(
    model: HeldPartPoseModel,
    images: torch.Tensor,
    classes: torch.Tensor,
    targets: torch.Tensor,
    directions: torch.Tensor,
    directed: torch.Tensor,
) -> tuple[torch.Tensor, dict[str, float]]:
    logits, predictions, predicted_directions, center_logits = model.forward_training(
        images
    )
    classification = F.nll_loss(logits, classes)
    occupied = classes != 0
    if occupied.any():
        selected_predictions = predictions[occupied]
        selected_targets = targets[occupied]
        size = model.config.image_size
        center = F.smooth_l1_loss(
            selected_predictions[:, :2] * size,
            selected_targets[:, :2] * size,
        )
        extent = F.smooth_l1_loss(
            selected_predictions[:, 2:4] * size,
            selected_targets[:, 2:4] * size,
        )
        angle = (
            1.0
            - (selected_predictions[:, 4:6] * selected_targets[:, 4:6])
            .sum(dim=1)
            .mean()
        )
        grid = model.center_grid.unsqueeze(0)
        distances = (grid - selected_targets[:, None, :2]).square().sum(dim=2)
        target_heatmap = F.softmax(-distances / (2.0 * 0.02**2), dim=1)
        heatmap = (
            -(target_heatmap * F.log_softmax(center_logits[occupied], dim=1))
            .sum(dim=1)
            .mean()
        )
    else:
        zero = logits.sum() * 0.0
        center = extent = angle = heatmap = zero
    directed_mask = directed.bool()
    if directed_mask.any():
        direction = (
            1.0
            - (predicted_directions[directed_mask] * directions[directed_mask])
            .sum(dim=1)
            .mean()
        )
    else:
        direction = logits.sum() * 0.0
    total = (
        classification
        + 0.25 * center
        + 0.1 * extent
        + angle
        + 0.1 * heatmap
        + direction
    )
    return total, {
        "total": float(total.detach()),
        "classification": float(classification.detach()),
        "center": float(center.detach()),
        "extent": float(extent.detach()),
        "angle": float(angle.detach()),
        "heatmap": float(heatmap.detach()),
        "direction": float(direction.detach()),
    }


def epoch_loss(
    model: HeldPartPoseModel,
    loader: DataLoader,
    device: torch.device,
    optimizer: torch.optim.Optimizer | None,
) -> dict[str, float]:
    model.train(optimizer is not None)
    totals: Counter[str] = Counter()
    batches = 0
    context = torch.enable_grad() if optimizer is not None else torch.no_grad()
    with context:
        for images, classes, targets, directions, directed, _ in loader:
            values = [images, classes, targets, directions, directed]
            images, classes, targets, directions, directed = [
                value.to(device) for value in values
            ]
            if optimizer is not None:
                optimizer.zero_grad(set_to_none=True)
            loss, parts = calculate_loss(
                model, images, classes, targets, directions, directed
            )
            if optimizer is not None:
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), 5.0)
                optimizer.step()
            totals.update(parts)
            batches += 1
    return {key: value / batches for key, value in totals.items()}


def export_onnx(model: HeldPartPoseModel, destination: Path, image_size: int) -> None:
    model = model.cpu().eval()
    sample = torch.zeros(1, 6, image_size, image_size)
    torch.onnx.export(
        model,
        sample,
        destination,
        input_names=["image_pair"],
        output_names=["state_log_probabilities", "pose", "directed_axis"],
        dynamic_axes={
            "image_pair": {0: "batch"},
            "state_log_probabilities": {0: "batch"},
            "pose": {0: "batch"},
            "directed_axis": {0: "batch"},
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
    parser.add_argument("--name", default="held-part-pose-v1")
    parser.add_argument("--image-size", type=int, default=192)
    parser.add_argument("--epochs", type=int, default=240)
    parser.add_argument("--patience", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=24)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--learning-rate", type=float, default=0.002)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--validate-only", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = args.dataset.resolve()
    summary = held_part_dataset_summary(root)
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

    config = HeldPartModelConfig(image_size=args.image_size)
    model = HeldPartPoseModel(config).to(device)
    train = HeldPartDataset(
        root, "train", args.image_size, augment=True, seed=args.seed
    )
    train_exact = HeldPartDataset(root, "train", args.image_size)
    evaluation = HeldPartDataset(root, "eval", args.image_size)
    train_loader = DataLoader(
        train,
        batch_size=args.batch_size,
        sampler=balanced_sampler(train, args.seed),
        num_workers=args.workers,
    )
    eval_loader = DataLoader(
        evaluation, batch_size=args.batch_size, shuffle=False, num_workers=args.workers
    )
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
    stale = 0
    for epoch in range(1, args.epochs + 1):
        record = {
            "epoch": epoch,
            "learning_rate": optimizer.param_groups[0]["lr"],
            "train": epoch_loss(model, train_loader, device, optimizer),
            "eval": epoch_loss(model, eval_loader, device, None),
        }
        record["selection_loss"] = (
            record["eval"]["total"] + 0.1 * record["train"]["total"]
        )
        history.append(record)
        print(json.dumps(record), flush=True)
        loss = record["selection_loss"]
        if loss < best_loss - 1e-5:
            best_loss = loss
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

    train_metrics, train_records = evaluate_held_part(
        model, train_exact, device, args.batch_size
    )
    metrics, records = evaluate_held_part(model, evaluation, device, args.batch_size)
    render_held_part_evaluation(
        train_exact, train_records, output / "training-evaluation"
    )
    render_held_part_evaluation(evaluation, records, output / "evaluation")
    checkpoint = output / "best.pt"
    torch.save(
        {
            "contract_version": CONTRACT_VERSION,
            "model_config": config.to_dict(),
            "class_names": list(HELD_PART_CLASS_NAMES),
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
        "source": {"git": git_revision(), "training_source_sha256": source_sha256()},
        "environment": {
            "hostname": platform.node(),
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
            "best_selection_loss": best_loss,
            "train_selection_weight": 0.1,
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
            "classes": list(HELD_PART_CLASS_NAMES),
            "input": {
                "name": "image_pair",
                "shape": ["batch", 6, args.image_size, args.image_size],
                "channels": [
                    "bare_red",
                    "bare_green",
                    "bare_blue",
                    "observed_red",
                    "observed_green",
                    "observed_blue",
                ],
                "type": "float32",
                "range": [0.0, 1.0],
                "resize": "velocitypnp.half-pixel-bilinear-u8.v1",
                "letterbox_value": 127,
            },
            "outputs": {
                "state_log_probabilities": ["batch", 3],
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
                "directed_axis": {
                    "shape": ["batch", 2],
                    "fields": ["cos_theta", "sin_theta"],
                },
            },
            "polarity_supported": True,
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
        "training_metrics": train_metrics,
        "limitations": [
            "The four-sample evaluation split is too small for a production gate.",
            "The evaluation split has no on-edge held parts.",
            "Only 12 reviewed images have visible directed-end labels.",
            "No multiple-component examples are available for this contract.",
            "The data comes from one machine and a small number of sessions.",
            "Model results must remain in shadow mode until independent validation.",
        ],
    }
    (output / "training-history.json").write_text(json.dumps(history, indent=2) + "\n")
    (output / "evaluation-records.json").write_text(
        json.dumps(records, indent=2) + "\n"
    )
    (output / "training-evaluation-records.json").write_text(
        json.dumps(train_records, indent=2) + "\n"
    )
    (output / "model-manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
