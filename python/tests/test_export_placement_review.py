import io
import json
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest

from PIL import Image


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from export_placement_review import export_review, finite_coordinate  # noqa: E402


class PlacementReviewTests(unittest.TestCase):
    def test_finite_coordinate_rejects_missing_and_outside_values(self):
        self.assertEqual(finite_coordinate({"x": 4.5}, "x", 10), 4.5)
        for value in (None, float("nan"), -1, 10):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    finite_coordinate({"x": value}, "x", 10)

    def test_export_keeps_flagged_physical_placement(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            database = root / "test.db"
            with sqlite3.connect(database) as connection:
                connection.execute(
                    "CREATE TABLE entities (kind TEXT, id TEXT, updated_at TEXT, "
                    "document TEXT)"
                )
                connection.execute(
                    "CREATE TABLE placement_execution_artifacts "
                    "(execution_id TEXT, name TEXT, data BLOB)"
                )
                report = {
                    "board_instance_id": "board-1",
                    "placement_reference": "R19",
                    "top_inspection": {
                        "detected": True,
                        "passed": False,
                        "expected_center_x_px": 18.0,
                        "expected_center_y_px": 17.0,
                        "center_x_px": 19.5,
                        "center_y_px": 15.0,
                        "offset_x_mm": 0.02,
                        "offset_y_mm": -0.18,
                        "offset_mm": 0.181,
                        "rotation_error_deg": 4.2,
                    },
                }
                connection.execute(
                    "INSERT INTO entities VALUES (?, ?, ?, ?)",
                    (
                        "placement_execution",
                        "execution-1",
                        "2026-09-20T23:00:00Z",
                        json.dumps({"status": "failed", "report": report}),
                    ),
                )
                image = io.BytesIO()
                Image.new("RGB", (40, 30)).save(image, format="PNG")
                connection.execute(
                    "INSERT INTO placement_execution_artifacts VALUES (?, ?, ?)",
                    ("execution-1", "board-placement-after.png", image.getvalue()),
                )
            output = root / "review"
            manifest = export_review(database, "board-1", output)
            self.assertEqual(len(manifest["samples"]), 1)
            sample = manifest["samples"][0]
            self.assertEqual(sample["target_px"], [18.0, 17.0])
            self.assertEqual(sample["estimated_px"], [19.5, 15.0])
            self.assertFalse(sample["inspection_passed"])
            self.assertTrue((output / sample["image"]).is_file())
            self.assertTrue((output / "index.html").is_file())
            with self.assertRaises(ValueError):
                export_review(database, "board-1", output)

    def test_export_keeps_result_when_part_is_not_detected(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            database = root / "test.db"
            with sqlite3.connect(database) as connection:
                connection.execute(
                    "CREATE TABLE entities (kind TEXT, id TEXT, updated_at TEXT, "
                    "document TEXT)"
                )
                connection.execute(
                    "CREATE TABLE placement_execution_artifacts "
                    "(execution_id TEXT, name TEXT, data BLOB)"
                )
                report = {
                    "board_instance_id": "board-1",
                    "placement_reference": "R17",
                    "top_inspection": {
                        "detected": False,
                        "passed": False,
                        "expected_center_x_px": 18.0,
                        "expected_center_y_px": 17.0,
                    },
                }
                connection.execute(
                    "INSERT INTO entities VALUES (?, ?, ?, ?)",
                    (
                        "placement_execution",
                        "execution-1",
                        "2026-09-20T23:00:00Z",
                        json.dumps({"status": "failed", "report": report}),
                    ),
                )
                image = io.BytesIO()
                Image.new("RGB", (40, 30)).save(image, format="PNG")
                connection.execute(
                    "INSERT INTO placement_execution_artifacts VALUES (?, ?, ?)",
                    ("execution-1", "board-placement-after.png", image.getvalue()),
                )
            manifest = export_review(database, "board-1", root / "review")
            sample = manifest["samples"][0]
            self.assertFalse(sample["part_detected"])
            self.assertIsNone(sample["estimated_px"])
            self.assertEqual(sample["target_px"], [18.0, 17.0])


if __name__ == "__main__":
    unittest.main()
