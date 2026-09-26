import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from analyze_placement_accuracy import correlation, percentile, summarize  # noqa: E402


class AnalyzePlacementAccuracyTests(unittest.TestCase):
    def test_percentile_interpolates(self):
        self.assertEqual(percentile([0.4, 0.1, 0.3, 0.2], 0.5), 0.25)
        self.assertIsNone(percentile([], 0.5))

    def test_correlation_needs_spread(self):
        self.assertAlmostEqual(correlation([1, 2, 3], [3, 2, 1]), -1)
        self.assertIsNone(correlation([1, 1, 1], [1, 2, 3]))

    def test_summary_separates_bias_from_scatter(self):
        records = [
            {"dx_mm": 0.1, "dy_mm": -0.2, "rotation_error_deg": 1.0},
            {"dx_mm": 0.1, "dy_mm": -0.2, "rotation_error_deg": -1.0},
        ]
        result = summarize(records)
        self.assertEqual(result["count"], 2)
        self.assertEqual(result["count_dy_negative"], 2)
        self.assertAlmostEqual(result["mean_bias_magnitude_mm"], math.hypot(0.1, 0.2))
        self.assertAlmostEqual(result["rms_after_mean_bias_removed_mm"], 0)


if __name__ == "__main__":
    unittest.main()
