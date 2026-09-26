import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest


TOOLS_ROOT = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS_ROOT))

from build_source_component_retrain_dataset import build  # noqa: E402


class BuildSourceComponentRetrainDatasetTests(unittest.TestCase):
    def make_dataset(
        self,
        root: Path,
        samples: list[tuple[str, str, str | None, str, int]],
    ) -> None:
        root.mkdir()
        ground_truth = []
        draft = []
        for sample_id, split, class_name, group_id, pixel in samples:
            image_path = Path("images") / split / f"{sample_id}.png"
            path = root / image_path
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(bytes([pixel]))
            ground_truth.append(
                {
                    "id": sample_id,
                    "split": split,
                    "class_name": class_name,
                    "outline_px": None
                    if class_name is None
                    else [[1, 1], [3, 1], [3, 2], [1, 2]],
                    "image_path": image_path.as_posix(),
                    "included": True,
                    "part_key": "R0603-1K",
                }
            )
            draft.append({"id": sample_id, "group_id": group_id})
        (root / "manifest.ground-truth.json").write_text(
            json.dumps(
                {
                    "classes": ["component_flat", "component_on_edge"],
                    "samples": ground_truth,
                }
            )
        )
        (root / "manifest.draft.json").write_text(
            json.dumps({"samples": draft})
        )

    def test_moves_complete_marked_group_and_recent_capture(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            base = root / "base"
            recent = root / "recent"
            output = root / "merged"
            self.make_dataset(
                base,
                [
                    ("marked-flat", "train", "component_flat", "marked", 1),
                    ("marked-edge", "train", "component_on_edge", "marked", 2),
                    ("old-empty", "eval", None, "old-eval", 3),
                    ("old-flat", "eval", "component_flat", "old-eval", 4),
                    ("train-empty", "train", None, "old-train", 5),
                    ("train-edge", "train", "component_on_edge", "old-train", 6),
                ],
            )
            self.make_dataset(
                recent,
                [("recent-flat", "eval", "component_flat", "recent", 7)],
            )

            report = build(base, recent, output, "marked")
            manifest = json.loads((output / "manifest.ground-truth.json").read_text())
            splits = {sample["id"]: sample["split"] for sample in manifest["samples"]}

            self.assertEqual(report["samples"], 7)
            self.assertEqual(splits["marked-flat"], "eval")
            self.assertEqual(splits["marked-edge"], "eval")
            self.assertEqual(splits["recent-flat"], "train")
            copied = output / "images/train/recent-flat.png"
            self.assertEqual(copied.read_bytes(), b"\x07")

    def test_rejects_identical_images_across_splits(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            base = root / "base"
            recent = root / "recent"
            output = root / "merged"
            self.make_dataset(
                base,
                [
                    ("marked-flat", "train", "component_flat", "marked", 1),
                    ("marked-edge", "train", "component_on_edge", "marked", 2),
                    ("old-empty", "eval", None, "old-eval", 3),
                    ("train-empty", "train", None, "old-train", 4),
                    ("train-edge", "train", "component_on_edge", "old-train", 5),
                ],
            )
            self.make_dataset(
                recent,
                [("recent-flat", "eval", "component_flat", "recent", 1)],
            )

            with self.assertRaisesRegex(ValueError, "identical images cross splits"):
                build(base, recent, output, "marked")
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
