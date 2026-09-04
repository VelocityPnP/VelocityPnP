import json
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
        HeldPartDataset,
        HeldPartModelConfig,
        HeldPartPoseModel,
        held_part_dataset_summary,
    )
    from velocitypnp_training.held_part import _rotate_component_only


@unittest.skipIf(torch is None, "training dependencies are not installed")
class HeldPartPoseTests(unittest.TestCase):
    def make_dataset(self, root: Path) -> None:
        cases = [
            ("train", "flat", "component_flat", True),
            ("train", "edge", "component_on_edge", False),
            ("train", "empty", None, False),
            ("eval", "flat", "component_flat", False),
            ("eval", "empty", None, False),
        ]
        records = []
        for split, name, class_name, directed in cases:
            directory = root / "images" / split
            directory.mkdir(parents=True, exist_ok=True)
            bare_path = directory / f"{name}-bare.png"
            observed_path = directory / f"{name}-observed.png"
            Image.new("RGB", (80, 64), (32, 32, 32)).save(bare_path)
            Image.new("RGB", (80, 64), (96, 96, 96)).save(observed_path)
            records.append(
                {
                    "id": f"{split}-{name}",
                    "split": split,
                    "group_id": f"{split}-{name}-group",
                    "bare_image_path": str(bare_path.relative_to(root)),
                    "observed_image_path": str(observed_path.relative_to(root)),
                    "included": True,
                    "class_name": class_name,
                    "outline_px": (
                        [[25, 25], [55, 25], [55, 39], [25, 39]] if class_name else None
                    ),
                    "marker_status": "visible" if directed else "not_applicable",
                    "marker_end_px": [58, 32] if directed else None,
                }
            )
        (root / "manifest.ground-truth.json").write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "classes": [
                        "component_flat",
                        "component_on_edge",
                        "multiple_components",
                    ],
                    "samples": records,
                }
            )
        )

    def test_summary_reports_class_and_directed_end_coverage(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_dataset(root)
            summary = held_part_dataset_summary(root)
            self.assertEqual(summary["sample_count"], 5)
            self.assertEqual(summary["directed_end_counts"]["train"], 1)
            self.assertEqual(summary["evaluation_limitations"], ["component_on_edge"])

    def test_dataset_returns_pair_and_undirected_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_dataset(root)
            dataset = HeldPartDataset(root, "train", 64)
            pair, class_id, pose, sample_id = dataset[0]
            self.assertEqual(tuple(pair.shape), (6, 64, 64))
            self.assertEqual(class_id, 1)
            self.assertEqual(sample_id, "train-flat")
            self.assertAlmostEqual(float(torch.linalg.norm(pose[4:])), 1.0)

    def test_fixed_component_rotation_changes_only_the_pose_axis(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_dataset(root)
            original = HeldPartDataset(root, "train", 64)
            rotated = HeldPartDataset(
                root,
                "train",
                64,
                fixed_component_rotation_degrees=90.0,
            )

            _, _, original_pose, _ = original[0]
            _, _, rotated_pose, sample_id = rotated[0]

            torch.testing.assert_close(rotated_pose[:4], original_pose[:4])
            torch.testing.assert_close(
                rotated_pose[4:],
                -original_pose[4:],
                atol=1e-5,
                rtol=1e-5,
            )
            self.assertTrue(sample_id.endswith("@rotation-+90-degrees"))

    def test_model_contract_shapes_and_ranges(self):
        model = HeldPartPoseModel(HeldPartModelConfig(image_size=64)).eval()
        state, pose = model(torch.zeros(2, 6, 64, 64))
        self.assertEqual(tuple(state.shape), (2, 3))
        self.assertEqual(tuple(pose.shape), (2, 6))
        torch.testing.assert_close(
            state.exp().sum(dim=1), torch.ones(2), atol=1e-6, rtol=1e-6
        )
        self.assertTrue(torch.all((pose[:, :4] >= 0) & (pose[:, :4] <= 1)))
        torch.testing.assert_close(torch.linalg.norm(pose[:, 4:], dim=1), torch.ones(2))

    def test_component_rotation_keeps_fixed_camera_features(self):
        bare = np.full((64, 64, 3), 32, np.uint8)
        for left, top in [(5, 5), (54, 5), (5, 54), (54, 54)]:
            bare[top : top + 5, left : left + 5] = 180
        observed = bare.copy()
        observed[28:36, 20:44] = 220
        points = np.asarray(
            [[20.0, 28.0], [44.0, 28.0], [44.0, 36.0], [20.0, 36.0]],
            np.float32,
        )

        rotated, rotated_points = _rotate_component_only(
            Image.fromarray(bare),
            Image.fromarray(observed),
            points,
            90.0,
        )
        pixels = np.asarray(rotated)

        np.testing.assert_array_equal(pixels[5:10, 5:10], bare[5:10, 5:10])
        self.assertLess(int(pixels[32, 21, 0]), 80)
        self.assertGreater(int(pixels[21, 32, 0]), 180)
        np.testing.assert_allclose(
            rotated_points,
            [[28.0, 44.0], [28.0, 20.0], [36.0, 20.0], [36.0, 44.0]],
            atol=1e-5,
        )


if __name__ == "__main__":
    unittest.main()
