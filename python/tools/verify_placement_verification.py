#!/usr/bin/env python3
"""Compare PyTorch and ONNX placement-verification outputs."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

try:
    import onnxruntime as ort
except ModuleNotFoundError:
    ort = None
    import onnx
    from onnx.reference import ReferenceEvaluator


REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "python" / "training"))

from velocitypnp_training import (  # noqa: E402
    PlacementDataset,
    PlacementModelConfig,
    PlacementVerificationModel,
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
    config = PlacementModelConfig(**checkpoint["model_config"])
    model = PlacementVerificationModel(config)
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()
    dataset = PlacementDataset(
        args.dataset.resolve(), args.split, config.image_size, augment=False
    )
    tensors = [dataset[index][0].numpy() for index in range(len(dataset))]
    images = np.stack(tensors)
    with torch.inference_mode():
        torch_state, torch_pose = model(torch.from_numpy(images))
    torch_outputs = {
        "state_log_probabilities": torch_state.numpy(),
        "pose": torch_pose.numpy(),
    }

    if ort is not None:
        session = ort.InferenceSession(
            str(args.onnx.resolve()), providers=["CPUExecutionProvider"]
        )
        values = session.run(None, {"images": images})
        output_names = [output.name for output in session.get_outputs()]
        runtime = "onnxruntime"
    else:
        artifact = onnx.load(args.onnx.resolve())
        session = ReferenceEvaluator(artifact)
        values = session.run(None, {"images": images})
        output_names = [output.name for output in artifact.graph.output]
        runtime = "onnx.reference"
    onnx_outputs = dict(zip(output_names, values, strict=True))
    differences = {
        name: float(np.max(np.abs(torch_outputs[name] - onnx_outputs[name])))
        for name in torch_outputs
    }
    torch_classes = torch_outputs["state_log_probabilities"].argmax(axis=1)
    onnx_classes = onnx_outputs["state_log_probabilities"].argmax(axis=1)
    class_mismatches = int(np.count_nonzero(torch_classes != onnx_classes))
    passed = (
        all(value <= args.absolute_tolerance for value in differences.values())
        and class_mismatches == 0
    )
    report = {
        "schema_version": 1,
        "contract_version": checkpoint["contract_version"],
        "onnx_runtime": runtime,
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
