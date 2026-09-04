#!/usr/bin/env python3
"""Compare PyTorch and ONNX source-component model outputs."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch


REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "python" / "training"))

from velocitypnp_training import (  # noqa: E402
    ModelConfig,
    SourceComponentDataset,
    SourceComponentPoseModel,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("onnx", type=Path)
    parser.add_argument("--split", default="eval", choices=("train", "eval"))
    parser.add_argument("--absolute-tolerance", type=float, default=1e-4)
    parser.add_argument("--report", type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    checkpoint = torch.load(
        args.checkpoint.resolve(), map_location="cpu", weights_only=True
    )
    config = ModelConfig(**checkpoint["model_config"])
    model = SourceComponentPoseModel(config)
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()

    dataset = SourceComponentDataset(
        args.dataset.resolve(), args.split, config.image_size, augment=False
    )
    images = np.stack([dataset[index][0].numpy() for index in range(len(dataset))])
    with torch.inference_mode():
        torch_logits, torch_pose = model(torch.from_numpy(images))
    torch_outputs = {
        "occupancy_logits": torch_logits.numpy(),
        "pose": torch_pose.numpy(),
    }

    session = ort.InferenceSession(
        str(args.onnx.resolve()), providers=["CPUExecutionProvider"]
    )
    onnx_values = session.run(None, {"images": images})
    onnx_outputs = dict(zip((output.name for output in session.get_outputs()), onnx_values))

    differences = {
        name: float(np.max(np.abs(torch_outputs[name] - onnx_outputs[name])))
        for name in torch_outputs
    }
    torch_classes = torch_outputs["occupancy_logits"].argmax(axis=1)
    onnx_classes = onnx_outputs["occupancy_logits"].argmax(axis=1)
    class_mismatches = int(np.count_nonzero(torch_classes != onnx_classes))
    passed = (
        all(value <= args.absolute_tolerance for value in differences.values())
        and class_mismatches == 0
    )
    report = {
        "schema_version": 1,
        "contract_version": checkpoint["contract_version"],
        "split": args.split,
        "sample_count": len(dataset),
        "absolute_tolerance": args.absolute_tolerance,
        "maximum_absolute_difference": differences,
        "class_mismatches": class_mismatches,
        "passed": passed,
    }
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
