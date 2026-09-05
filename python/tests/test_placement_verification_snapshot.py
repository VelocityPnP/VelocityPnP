import sys
from pathlib import Path
import unittest

from PIL import Image


TOOLS_ROOT = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS_ROOT))

from export_placement_verification_snapshot import (  # noqa: E402
    OUTPUT_SIZE,
    crop_fits_image,
    expected_outline,
    mean_frames,
    normalized_crop,
    normalized_crop_coefficients,
    part_dimensions,
    reference_sort_key,
)
from finalize_placement_verification_dataset import (  # noqa: E402
    validate_outline,
    validate_review,
)


class PlacementVerificationSnapshotTests(unittest.TestCase):
    def test_part_dimensions_prefer_exact_value_key(self):
        placement = {
            "reference": "R1",
            "package": "Resistor_SMD:R_0603_1608Metric",
            "value": "10k",
        }
        parts = [
            {
                "key": "Resistor_SMD:R_0603_1608Metric:1k",
                "package": placement["package"],
                "body_width_mm": 0.7,
                "body_length_mm": 1.5,
            },
            {
                "key": "Resistor_SMD:R_0603_1608Metric:10k",
                "package": placement["package"],
                "body_width_mm": 0.8,
                "body_length_mm": 1.6,
            },
        ]

        self.assertEqual(part_dimensions(placement, parts), (0.8, 1.6))

    def test_part_dimensions_reject_ambiguous_package_fallback(self):
        placement = {
            "reference": "R1",
            "package": "R0603",
            "value": "unknown",
        }
        parts = [
            {
                "key": "R0603:1k",
                "package": "R0603",
                "body_width_mm": 0.8,
                "body_length_mm": 1.6,
            },
            {
                "key": "R0603:10k",
                "package": "R0603",
                "body_width_mm": 0.8,
                "body_length_mm": 1.6,
            },
        ]

        with self.assertRaisesRegex(ValueError, "2 matching parts"):
            part_dimensions(placement, parts)

    def test_normalized_crop_centers_the_machine_target(self):
        source = Image.new("L", (512, 512))
        for x in range(275, 278):
            for y in range(245, 248):
                source.putpixel((x, y), 255)

        crop = normalized_crop(
            source,
            camera_center_px=[256.0, 256.0],
            pixel_to_machine=[[0.01, 0.0], [0.0, 0.01]],
            camera_machine_mm=[100.0, 200.0],
            target_machine_mm=[100.2, 199.9],
            target_rotation_rad=0.0,
        )

        self.assertGreater(crop.getpixel((OUTPUT_SIZE // 2, OUTPUT_SIZE // 2)), 240)

    def test_crop_bounds_reject_filled_edge_data(self):
        image = Image.new("L", (512, 512))
        centered = normalized_crop_coefficients(
            [256.0, 256.0],
            [[0.01, 0.0], [0.0, 0.01]],
            [100.0, 200.0],
            [100.0, 200.0],
            0.0,
        )
        clipped = normalized_crop_coefficients(
            [256.0, 256.0],
            [[0.01, 0.0], [0.0, 0.01]],
            [100.0, 200.0],
            [102.0, 200.0],
            0.0,
        )

        self.assertTrue(crop_fits_image(image, centered))
        self.assertFalse(crop_fits_image(image, clipped))

    def test_expected_outline_uses_configured_physical_size(self):
        self.assertEqual(
            expected_outline(1.6, 0.8),
            [[76.8, 102.4], [179.2, 102.4], [179.2, 153.6], [76.8, 153.6]],
        )

    def test_reference_sort_key_orders_numeric_suffixes(self):
        references = ["R10", "R2", "D3", "R1"]
        self.assertEqual(
            sorted(references, key=reference_sort_key),
            ["D3", "R1", "R2", "R10"],
        )

    def test_frame_mean_has_equal_weights(self):
        from tempfile import TemporaryDirectory

        with TemporaryDirectory() as directory:
            paths = []
            for index, value in enumerate((0, 60, 120)):
                path = Path(directory) / f"{index}.png"
                Image.new("L", (4, 4), value).save(path)
                paths.append(path)

            result = mean_frames(paths)

        self.assertEqual(result.getpixel((0, 0)), 60)

    def test_review_requires_the_exact_draft_generation(self):
        draft = {
            "name": "placement-review",
            "generated_at": "2026-09-04T00:00:00Z",
            "samples": [{"id": "r1"}],
        }
        review = {
            "schema_version": 1,
            "dataset": "placement-review",
            "source_manifest_generated_at": "2026-09-04T00:00:01Z",
            "decisions": {"r1": {"decision": "accept"}},
        }

        with self.assertRaisesRegex(ValueError, "another draft generation"):
            validate_review(draft, review)

    def test_outline_validation_accepts_a_rotated_rectangle(self):
        result = validate_outline(
            "r1",
            [[80, 100], [180, 110], [175, 160], [75, 150]],
            256,
            256,
        )

        self.assertEqual(len(result), 4)

    def test_outline_validation_rejects_a_trapezoid(self):
        with self.assertRaisesRegex(ValueError, "not rectangular"):
            validate_outline(
                "r1",
                [[80, 100], [180, 100], [160, 160], [80, 160]],
                256,
                256,
            )


if __name__ == "__main__":
    unittest.main()
