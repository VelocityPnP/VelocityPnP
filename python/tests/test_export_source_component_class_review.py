import json
import subprocess
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest


REPOSITORY = Path(__file__).resolve().parents[2]
TOOL = REPOSITORY / "python" / "tools" / "export_source_component_class_review.py"


class ExportSourceComponentClassReviewTests(unittest.TestCase):
    def test_exports_only_selected_included_class(self) -> None:
        with TemporaryDirectory() as directory:
            dataset = Path(directory) / "dataset"
            (dataset / "images" / "train").mkdir(parents=True)
            for name in ("edge.png", "flat.png", "excluded.png"):
                (dataset / "images" / "train" / name).write_bytes(b"image")
            manifest = {
                "name": "test-source-components",
                "samples": [
                    _sample("edge", "component_on_edge", True),
                    _sample("flat", "component_flat", True),
                    _sample("excluded", "component_on_edge", False),
                ],
            }
            (dataset / "manifest.ground-truth.json").write_text(
                json.dumps(manifest)
            )

            result = subprocess.run(
                [sys.executable, TOOL, dataset],
                check=True,
                capture_output=True,
                text=True,
            )

            output = dataset / "review-component-on-edge-audit.html"
            page = output.read_text()
            self.assertIn("review samples: 1", result.stdout)
            self.assertIn('\"id\":\"edge\"', page)
            self.assertNotIn('\"id\":\"flat\"', page)
            self.assertNotIn('\"id\":\"excluded\"', page)
            self.assertIn("Current image metadata", page)


def _sample(sample_id: str, class_name: str, included: bool) -> dict:
    return {
        "id": sample_id,
        "split": "train",
        "image_path": f"images/train/{sample_id}.png",
        "included": included,
        "class_name": class_name,
        "outline_px": [[1, 1], [2, 1], [2, 2], [1, 2]],
    }


if __name__ == "__main__":
    unittest.main()
