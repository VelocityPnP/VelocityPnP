import io
import json
from pathlib import Path
import sqlite3
import sys
from tempfile import TemporaryDirectory
import unittest

from PIL import Image


TOOLS_ROOT = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS_ROOT))

from export_placement_execution_dataset import (  # noqa: E402
    draft_outline,
    disposition_state,
    export_dataset,
)


class PlacementExecutionDatasetTests(unittest.TestCase):
    def add_entity(
        self,
        connection: sqlite3.Connection,
        kind: str,
        entity_id: str,
        document: dict,
        updated_at: str,
    ) -> None:
        connection.execute(
            "INSERT INTO entities VALUES (?, ?, ?, ?)",
            (kind, entity_id, updated_at, json.dumps(document)),
        )

    def image_bytes(self, value: int) -> bytes:
        output = io.BytesIO()
        Image.new("L", (512, 512), value).save(output, format="PNG")
        return output.getvalue()

    def make_database(self, path: Path) -> None:
        connection = sqlite3.connect(path)
        connection.executescript(
            """
            CREATE TABLE entities (
                kind TEXT NOT NULL,
                id TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                document TEXT NOT NULL,
                PRIMARY KEY (kind, id)
            );
            CREATE TABLE placement_execution_artifacts (
                execution_id TEXT NOT NULL,
                name TEXT NOT NULL,
                media_type TEXT NOT NULL,
                data BLOB NOT NULL,
                created_at TEXT NOT NULL,
                PRIMARY KEY (execution_id, name)
            );
            """
        )
        calibration = {
            "id": "calibration",
            "artifacts": {
                "top_camera_working_plane_map": {
                    "value": {
                        "result": {
                            "pixel_error_to_machine_mm": [
                                [0.01, 0.0],
                                [0.0, 0.01],
                            ]
                        }
                    }
                }
            },
        }
        job = {
            "id": "job",
            "boards": [
                {
                    "id": "board-instance",
                    "board_id": "board",
                }
            ],
        }
        self.add_entity(
            connection, "calibration", "calibration", calibration, "2026-01-01"
        )
        self.add_entity(connection, "job", "job", job, "2026-01-01")
        for index, (execution_id, center) in enumerate(
            (("valid", [256.0, 256.0]), ("invalid", [None, 256.0]))
        ):
            report = {
                "simulated": False,
                "board_instance_id": "board-instance",
                "placement_reference": f"R{index + 1}",
                "part_key": "R0603:10k",
                "board_place": {
                    "x_mm": 100.0 + index,
                    "y_mm": 200.0,
                },
                "top_inspection": {
                    "detected": True,
                    "expected_center_x_px": center[0],
                    "expected_center_y_px": center[1],
                    "expected_width_mm": 0.8,
                    "expected_length_mm": 1.6,
                    "expected_rotation_deg": 0.0,
                    "offset_x_mm": 0.1,
                    "offset_y_mm": -0.2,
                    "rotation_error_deg": 5.0,
                },
            }
            execution = {
                "id": execution_id,
                "job_id": "job",
                "calibration_id": "calibration",
                "created_at": f"2026-01-0{index + 2}",
                "report": report,
            }
            outcome = {
                "id": execution_id,
                "package": "R0603",
                "operator_disposition": "good",
                "automatic_disposition": "offset",
                "operator_note": "reviewed",
            }
            timestamp = f"2026-01-0{index + 2}"
            self.add_entity(
                connection,
                "placement_execution",
                execution_id,
                execution,
                timestamp,
            )
            self.add_entity(
                connection,
                "placement_outcome",
                execution_id,
                outcome,
                timestamp,
            )
            for name, value in (
                ("board-placement-before.png", 50),
                ("board-placement-after.png", 200),
            ):
                connection.execute(
                    "INSERT INTO placement_execution_artifacts "
                    "VALUES (?, ?, 'image/png', ?, ?)",
                    (execution_id, name, self.image_bytes(value), timestamp),
                )
        connection.commit()
        connection.close()

    def test_export_keeps_before_and_after_in_one_execution_group(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            database = root / "test.db"
            output = root / "review"
            self.make_database(database)

            manifest = export_dataset(
                database, output, "2026-01-05T00:00:00Z"
            )

            self.assertEqual(len(manifest["samples"]), 2)
            before, after = manifest["samples"]
            self.assertEqual(before["source_state"], "before")
            self.assertEqual(before["draft_state"], "empty")
            self.assertEqual(after["source_state"], "after")
            self.assertEqual(after["draft_state"], "component_flat")
            self.assertEqual(before["group_id"], after["group_id"])
            self.assertIsNone(before["draft_outline_px"])
            self.assertEqual(len(after["draft_outline_px"]), 4)
            for sample in manifest["samples"]:
                with Image.open(output / sample["image_path"]) as image:
                    self.assertEqual(image.size, (256, 256))
            self.assertEqual(manifest["skipped"][0]["execution_id"], "invalid")
            self.assertIn("expected target X", manifest["skipped"][0]["reason"])

    def test_disposition_proposals_preserve_special_conditions(self):
        inspection = {"detected": False}

        before = disposition_state(
            {"operator_disposition": "stacked"}, inspection, "before"
        )
        edge = disposition_state(
            {"operator_disposition": "tombstoned"}, inspection, "after"
        )
        missing = disposition_state(
            {"automatic_disposition": "stuck_on_nozzle"}, inspection, "after"
        )

        self.assertEqual(before[0], "component_flat")
        self.assertEqual(edge[0], "component_on_edge")
        self.assertEqual(missing[0], "empty")

    def test_draft_outline_rotates_machine_offset_into_target_axis(self):
        outline = draft_outline(
            "component_flat",
            {
                "offset_x_mm": 0.1,
                "offset_y_mm": -0.2,
                "rotation_error_deg": 0.0,
            },
            length_mm=1.6,
            width_mm=0.8,
            target_rotation_deg=90.0,
        )

        center_x = sum(point[0] for point in outline) / 4
        center_y = sum(point[1] for point in outline) / 4
        self.assertAlmostEqual(center_x, 115.2)
        self.assertAlmostEqual(center_y, 121.6)

    def test_export_refuses_to_replace_review_data(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            database = root / "test.db"
            output = root / "review"
            self.make_database(database)
            output.mkdir()
            (output / "decision.json").write_text("{}")

            with self.assertRaisesRegex(ValueError, "not empty"):
                export_dataset(database, output)


if __name__ == "__main__":
    unittest.main()
