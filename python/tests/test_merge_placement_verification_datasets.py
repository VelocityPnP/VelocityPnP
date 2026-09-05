import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest

from PIL import Image


TOOLS_ROOT = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS_ROOT))

from merge_placement_verification_datasets import merge_datasets  # noqa: E402


class MergePlacementVerificationDatasetsTests(unittest.TestCase):
    def make_dataset(
        self,
        root: Path,
        name: str,
        samples: list[tuple[str, str, str, str]],
    ) -> None:
        records = []
        for index, (sample_id, reference, group_id, split) in enumerate(samples):
            path = root / "images" / f"{sample_id}.png"
            path.parent.mkdir(parents=True, exist_ok=True)
            Image.new("L", (256, 256), 40 + index).save(path)
            records.append(
                {
                    "id": sample_id,
                    "group_id": group_id,
                    "split": split,
                    "included": True,
                    "reference": reference,
                    "package": "R0603",
                    "image_path": path.relative_to(root).as_posix(),
                    "image_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "width_px": 256,
                    "height_px": 256,
                    "target_machine_mm": [100.0, 200.0],
                    "class_name": "component_flat",
                    "outline_px": [[80, 100], [176, 100], [176, 150], [80, 150]],
                }
            )
        manifest = {
            "schema_version": 1,
            "name": name,
            "board_id": "board",
            "classes": [
                "component_flat",
                "component_on_edge",
                "multiple_components",
            ],
            "normalization": {"output_span_mm": 4.0},
            "samples": records,
        }
        (root / "manifest.ground-truth.json").write_text(json.dumps(manifest))

    def test_merge_propagates_eval_across_capture_and_target(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            first = root / "first"
            second = root / "second"
            self.make_dataset(
                first,
                "first",
                [
                    ("r1", "R1", "capture-a", "eval"),
                    ("r2", "R2", "capture-a", "eval"),
                    ("r3", "R3", "capture-b", "train"),
                ],
            )
            self.make_dataset(
                second,
                "second",
                [
                    ("r2-before", "R2", "execution-r2", "train"),
                    ("r2-after", "R2", "execution-r2", "train"),
                    ("r4", "R4", "execution-r4", "train"),
                ],
            )

            manifest = merge_datasets(
                [first, second], root / "merged", "2026-01-01T00:00:00Z"
            )

        splits = {
            sample["id"]: sample["split"] for sample in manifest["samples"]
        }
        self.assertEqual(splits["source-1:r1"], "eval")
        self.assertEqual(splits["source-1:r2"], "eval")
        self.assertEqual(splits["source-2:r2-before"], "eval")
        self.assertEqual(splits["source-2:r2-after"], "eval")
        self.assertEqual(splits["source-1:r3"], "train")
        self.assertEqual(splits["source-2:r4"], "train")

    def test_merge_refuses_different_normalization(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            first = root / "first"
            second = root / "second"
            self.make_dataset(first, "first", [("r1", "R1", "a", "eval")])
            self.make_dataset(second, "second", [("r2", "R2", "b", "train")])
            manifest_path = second / "manifest.ground-truth.json"
            manifest = json.loads(manifest_path.read_text())
            manifest["normalization"]["output_span_mm"] = 5.0
            manifest_path.write_text(json.dumps(manifest))

            with self.assertRaisesRegex(ValueError, "normalization"):
                merge_datasets([first, second], root / "merged")


if __name__ == "__main__":
    unittest.main()
