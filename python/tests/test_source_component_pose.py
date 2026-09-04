import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

try:
    import numpy as np
    import torch
    from PIL import Image
except ModuleNotFoundError:
    np = None
    torch = None
    Image = None


TRAINING_ROOT = Path(__file__).resolve().parents[1] / "training"
if torch is not None:
    sys.path.insert(0, str(TRAINING_ROOT))
    from velocitypnp_training import (
        ModelConfig,
        SourceComponentDataset,
        SourceComponentPoseModel,
        dataset_summary,
        polygon_to_pose,
    )
    from velocitypnp_training.evaluation import polygon_iou


@unittest.skipIf(torch is None, "training dependencies are not installed")
class SourceComponentPoseTests(unittest.TestCase):
    def make_dataset(self, root: Path) -> None:
        records = []
        cases = [
            ("train", "flat", "component_flat"),
            ("train", "edge", "component_on_edge"),
            ("train", "empty", None),
            ("eval", "flat", "component_flat"),
            ("eval", "empty", None),
        ]
        for split, name, class_name in cases:
            image_path = Path("images") / split / f"{name}.png"
            destination = root / image_path
            destination.parent.mkdir(parents=True, exist_ok=True)
            Image.new("RGB", (350, 300), (128, 128, 128)).save(destination)
            records.append(
                {
                    "id": f"{split}-{name}",
                    "split": split,
                    "image_path": str(image_path),
                    "included": True,
                    "class_name": class_name,
                    "outline_px": (
                        [[150, 130], [200, 130], [200, 160], [150, 160]]
                        if class_name
                        else None
                    ),
                }
            )
        (root / "manifest.ground-truth.json").write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "classes": ["component_flat", "component_on_edge"],
                    "samples": records,
                }
            )
        )

    def test_dataset_summary_reports_missing_eval_class(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_dataset(root)
            summary = dataset_summary(root)
            self.assertEqual(summary["sample_count"], 5)
            self.assertEqual(
                summary["evaluation_limitations"], ["component_on_edge"]
            )

    def test_dataset_returns_normalized_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_dataset(root)
            dataset = SourceComponentDataset(root, "eval", 192)
            image, class_id, pose, sample_id = dataset[0]
            self.assertEqual(tuple(image.shape), (3, 192, 192))
            self.assertEqual(class_id, 1)
            self.assertEqual(sample_id, "eval-flat")
            self.assertTrue(torch.all((pose[:4] >= 0) & (pose[:4] <= 1)))
            self.assertAlmostEqual(float(torch.linalg.norm(pose[4:])), 1.0, places=5)

    def test_polygon_pose_uses_undirected_long_axis(self):
        horizontal = np.asarray([[1, 2], [9, 2], [9, 6], [1, 6]], np.float32)
        reversed_points = horizontal[::-1].copy()
        first = polygon_to_pose(horizontal, 10)
        second = polygon_to_pose(reversed_points, 10)
        np.testing.assert_allclose(first, second, atol=1e-6)
        self.assertAlmostEqual(float(first[4]), 1.0, places=6)
        self.assertAlmostEqual(float(first[5]), 0.0, places=6)

    def test_model_contract_shapes_and_ranges(self):
        model = SourceComponentPoseModel(ModelConfig(image_size=64)).eval()
        logits, pose = model(torch.zeros(2, 3, 64, 64))
        self.assertEqual(tuple(logits.shape), (2, 3))
        self.assertEqual(tuple(pose.shape), (2, 6))
        torch.testing.assert_close(
            logits.exp().sum(dim=1), torch.ones(2), atol=1e-6, rtol=1e-6
        )
        self.assertTrue(torch.all((pose[:, :4] >= 0) & (pose[:, :4] <= 1)))
        torch.testing.assert_close(
            torch.linalg.norm(pose[:, 4:], dim=1), torch.ones(2)
        )

    def test_polygon_iou(self):
        first = np.asarray([[0, 0], [2, 0], [2, 2], [0, 2]], np.float32)
        overlap = np.asarray([[1, 0], [3, 0], [3, 2], [1, 2]], np.float32)
        self.assertAlmostEqual(polygon_iou(first, first), 1.0)
        self.assertAlmostEqual(polygon_iou(first, overlap), 1 / 3)


if __name__ == "__main__":
    unittest.main()
