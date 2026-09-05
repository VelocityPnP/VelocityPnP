import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest

try:
    import torch
    from PIL import Image
except ModuleNotFoundError:
    torch = None
    Image = None


TRAINING_ROOT = Path(__file__).resolve().parents[1] / "training"
if torch is not None:
    sys.path.insert(0, str(TRAINING_ROOT))
    from velocitypnp_training import (
        PlacementDataset,
        PlacementModelConfig,
        PlacementSample,
        PlacementVerificationModel,
        placement_dataset_summary,
        placement_sampling_weights,
    )
    from velocitypnp_training.placement_verification import (
        augment_placement_appearance,
    )


@unittest.skipIf(torch is None, "training dependencies are not installed")
class PlacementVerificationTests(unittest.TestCase):
    def make_dataset(self, root: Path, overlap: bool = False) -> None:
        records = []
        cases = [
            ("train", "empty", None),
            ("train", "flat", "component_flat"),
            ("eval", "empty", None),
            ("eval", "flat", "component_flat"),
        ]
        for split, name, class_name in cases:
            path = root / "images" / f"{split}-{name}.png"
            path.parent.mkdir(parents=True, exist_ok=True)
            Image.new("RGB", (256, 256), (96, 96, 96)).save(path)
            records.append(
                {
                    "id": f"{split}-{name}",
                    "split": split,
                    "group_id": "shared" if overlap else f"{split}-group",
                    "image_path": str(path.relative_to(root)),
                    "image_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "included": True,
                    "class_name": class_name,
                    "outline_px": (
                        [[76.8, 102.4], [179.2, 102.4],
                         [179.2, 153.6], [76.8, 153.6]]
                        if class_name
                        else None
                    ),
                    "reference": "R1",
                    "package": "R0603",
                    "target_machine_mm": [100.0, 200.0],
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
                    "normalization": {"output_span_mm": 4.0},
                    "samples": records,
                }
            )
        )

    def test_summary_reports_missing_condition_classes(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_dataset(root)

            summary = placement_dataset_summary(root)

        self.assertEqual(summary["sample_count"], 4)
        self.assertEqual(
            summary["training_limitations"],
            ["component_on_edge", "multiple_components"],
        )

    def test_summary_rejects_capture_group_leakage(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_dataset(root, overlap=True)

            with self.assertRaisesRegex(ValueError, "both splits"):
                placement_dataset_summary(root)

    def test_dataset_returns_target_pose(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_dataset(root)
            dataset = PlacementDataset(root, "train", 192)

            image, class_id, pose, sample_id = dataset[1]

        self.assertEqual(tuple(image.shape), (3, 192, 192))
        self.assertEqual(class_id, 1)
        self.assertEqual(sample_id, "train-flat")
        self.assertAlmostEqual(float(pose[0]), 0.5, delta=1e-6)
        self.assertAlmostEqual(float(pose[1]), 0.5, delta=1e-6)

    def test_model_contract_shapes_and_ranges(self):
        model = PlacementVerificationModel(
            PlacementModelConfig(image_size=64)
        ).eval()

        state, pose = model(torch.zeros(2, 3, 64, 64))

        self.assertEqual(tuple(state.shape), (2, 4))
        self.assertEqual(tuple(pose.shape), (2, 6))
        torch.testing.assert_close(
            state.exp().sum(dim=1),
            torch.ones(2),
            atol=1e-6,
            rtol=1e-6,
        )
        self.assertTrue(torch.all((pose[:, :4] >= 0) & (pose[:, :4] <= 1)))
        torch.testing.assert_close(
            torch.linalg.norm(pose[:, 4:], dim=1), torch.ones(2)
        )

    def test_sampling_balances_flat_angle_bins(self):
        samples = [
            PlacementSample(
                sample_id="empty",
                split="train",
                group_id="empty",
                image_path=Path("empty.png"),
                class_id=0,
                outline_px=None,
                reference="R1",
                package="R0603",
                target_machine_mm=(0.0, 0.0),
            ),
            *[
                PlacementSample(
                    sample_id=f"horizontal-{index}",
                    split="train",
                    group_id=f"horizontal-{index}",
                    image_path=Path("flat.png"),
                    class_id=1,
                    outline_px=((0, 0), (20, 0), (20, 10), (0, 10)),
                    reference="R2",
                    package="R0603",
                    target_machine_mm=(0.0, 0.0),
                )
                for index in range(2)
            ],
            PlacementSample(
                sample_id="vertical",
                split="train",
                group_id="vertical",
                image_path=Path("flat.png"),
                class_id=1,
                outline_px=((0, 0), (0, 20), (-10, 20), (-10, 0)),
                reference="R3",
                package="R0603",
                target_machine_mm=(0.0, 0.0),
            ),
        ]

        weights = placement_sampling_weights(samples)

        self.assertEqual(weights, [1.0, 0.25, 0.25, 0.5])

    def test_appearance_augmentation_keeps_image_shape(self):
        import random

        image = Image.new("RGB", (192, 192), (96, 128, 160))

        result = augment_placement_appearance(image, random.Random(7))

        self.assertEqual(result.size, image.size)
        self.assertEqual(result.mode, image.mode)


if __name__ == "__main__":
    unittest.main()
