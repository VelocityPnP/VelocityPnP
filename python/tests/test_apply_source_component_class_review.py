import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest


TOOLS_ROOT = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS_ROOT))

from apply_source_component_class_review import apply_review  # noqa: E402


class ApplySourceComponentClassReviewTests(unittest.TestCase):
    def test_applies_complete_audit_without_changing_source(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source"
            source.mkdir()
            (source / "images/train").mkdir(parents=True)
            for name in ("edge", "flat"):
                (source / f"images/train/{name}.png").write_bytes(name.encode())
            manifest = {
                "classes": ["component_flat", "component_on_edge"],
                "samples": [
                    _sample("edge", "component_on_edge"),
                    _sample("flat", "component_flat"),
                ],
            }
            (source / "manifest.ground-truth.json").write_text(
                json.dumps(manifest)
            )
            (source / "manifest.draft.json").write_text(
                json.dumps({"samples": []})
            )
            review = root / "review.json"
            review.write_text(
                json.dumps(
                    {
                        "audited_class_name": "component_on_edge",
                        "decisions": {
                            "edge": {
                                "choice": "component_flat",
                                "outline_px": [[1, 1], [2, 1], [2, 2], [1, 2]],
                            }
                        },
                    }
                )
            )
            output = root / "corrected"

            report = apply_review(source, review, output)

            corrected = json.loads(
                (output / "manifest.ground-truth.json").read_text()
            )
            classes = {
                sample["id"]: sample["class_name"]
                for sample in corrected["samples"]
            }
            source_manifest = json.loads(
                (source / "manifest.ground-truth.json").read_text()
            )
            self.assertEqual(
                classes,
                {"edge": "component_flat", "flat": "component_flat"},
            )
            self.assertEqual(
                source_manifest["samples"][0]["class_name"],
                "component_on_edge",
            )
            self.assertEqual(
                report["changes"],
                {"component_on_edge->component_flat": 1},
            )
            self.assertEqual(
                (output / "images/train/edge.png").read_bytes(), b"edge"
            )

    def test_rejects_partial_audit(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source"
            source.mkdir()
            (source / "manifest.ground-truth.json").write_text(
                json.dumps(
                    {
                        "samples": [
                            {
                                "id": "edge",
                                "included": True,
                                "class_name": "component_on_edge",
                            }
                        ]
                    }
                )
            )
            review = root / "review.json"
            review.write_text(
                json.dumps(
                    {
                        "audited_class_name": "component_on_edge",
                        "decisions": {},
                    }
                )
            )

            with self.assertRaisesRegex(ValueError, "1 missing"):
                apply_review(source, review, root / "corrected")


def _sample(sample_id: str, class_name: str) -> dict:
    return {
        "id": sample_id,
        "split": "train",
        "image_path": f"images/train/{sample_id}.png",
        "included": True,
        "class_name": class_name,
        "outline_px": [[1, 1], [2, 1], [2, 2], [1, 2]],
    }


if __name__ == "__main__":
    unittest.main()
