import io
import json
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest

from PIL import Image


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from export_placement_trace_review import export_trace, point  # noqa: E402


def png(width, height):
    output = io.BytesIO()
    Image.new("RGB", (width, height)).save(output, format="PNG")
    return output.getvalue()


class PlacementTraceReviewTests(unittest.TestCase):
    def test_point_checks_image_bounds(self):
        self.assertEqual(point([5, 6], 10, 10, "center"), [5.0, 6.0])
        self.assertIsNone(point([10, 6], 10, 10, "center"))
        with self.assertRaises(ValueError):
            point([10, 6], 10, 10, "center", required=True)

    def test_export_links_stages_and_allows_no_final_held_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            database = root / "test.db"
            with sqlite3.connect(database) as connection:
                connection.execute(
                    "CREATE TABLE entities (kind TEXT, id TEXT, updated_at TEXT, document TEXT)"
                )
                connection.execute(
                    "CREATE TABLE placement_execution_artifacts "
                    "(execution_id TEXT, name TEXT, data BLOB)"
                )
                report = {
                    "board_instance_id": "board-1",
                    "placement_reference": "R8",
                    "feeder_id": "feeder-1",
                    "pocket_index": 4,
                    "board_target_registration": {
                        "center_x_px": 18, "center_y_px": 19,
                        "correction_x_mm": 0.01, "correction_y_mm": -0.02,
                        "confidence": 0.9,
                    },
                    "top_inspection": {
                        "detected": True, "passed": False,
                        "expected_center_x_px": 18, "expected_center_y_px": 18,
                        "center_x_px": 20, "center_y_px": 15,
                        "offset_x_mm": 0.03, "offset_y_mm": -0.04,
                        "rotation_error_deg": 4.4,
                    },
                    "pickup_report": {
                        "pocket_machine_mm": [10, 20],
                        "release_surface_machine_mm": [30, 40],
                        "alignment": {
                            "observations": [{"inspection": {
                                "centering_correction_x_mm": 0.1,
                                "centering_correction_y_mm": -0.2,
                            }}],
                            "commanded_x_mm": 0, "commanded_y_mm": 0,
                            "final_centering_residual_mm": 0.03,
                        },
                    },
                }
                connection.execute(
                    "INSERT INTO entities VALUES (?, ?, ?, ?)",
                    ("placement_execution", "execution-1", "2026-09-20T23:00:00Z",
                     json.dumps({"status": "failed", "report": report})),
                )
                connection.execute(
                    "INSERT INTO entities VALUES (?, ?, ?, ?)",
                    ("feeder", "feeder-1", "2026-09-20T23:00:00Z",
                     json.dumps({"name": "Feeder 8"})),
                )
                artifacts = {
                    "source-pocket-pick-source-component-control.png": png(30, 30),
                    "held-part-inspection-initial-held-part-shadow.png": png(30, 30),
                    "board-placement-before.png": png(40, 30),
                    "board-placement-after.png": png(40, 30),
                    "source-pocket-pick-source-component-control.json": json.dumps({
                        "control_authorized": True,
                        "deterministic_center_px": [14, 15],
                        "prediction": {"pose": {"center_px": [15, 15]}},
                    }).encode(),
                    "held-part-inspection-initial-held-part-shadow.json": json.dumps({
                        "deterministic_center_px": [14, 15],
                        "prediction": {"pose": {"center_px": [15, 15]}},
                    }).encode(),
                }
                connection.executemany(
                    "INSERT INTO placement_execution_artifacts VALUES (?, ?, ?)",
                    [("execution-1", name, data) for name, data in artifacts.items()],
                )
            output = root / "review"
            manifest = export_trace(database, "board-1", output, {"R8": "On its side"})
            self.assertEqual(len(manifest["samples"]), 1)
            sample = manifest["samples"][0]
            self.assertEqual(sample["feeder"], "Feeder 8")
            self.assertIsNone(sample["images"]["held_final"])
            self.assertEqual(sample["centers"]["target_after"], [18.0, 18.0])
            self.assertEqual(sample["centers"]["result_after"], [20.0, 15.0])
            self.assertEqual(sample["prior_note"], "On its side")
            self.assertTrue((output / sample["images"]["result_after"]["path"]).is_file())
            self.assertTrue((output / "index.html").is_file())
            with self.assertRaises(ValueError):
                export_trace(database, "board-1", output)


if __name__ == "__main__":
    unittest.main()
