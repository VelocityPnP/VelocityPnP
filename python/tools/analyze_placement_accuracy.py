#!/usr/bin/env python3
"""Summarize signed placement residuals from a retained physical run.

Only executions with a result image in the supplied CSV are used. The
measurement is the stored top-camera inspection, not an independent gauge.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sqlite3
import statistics
from collections import Counter, defaultdict
from pathlib import Path


def mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def correlation(first: list[float], second: list[float]) -> float | None:
    if len(first) < 3 or len(first) != len(second):
        return None
    first_mean, second_mean = mean(first), mean(second)
    numerator = sum((x - first_mean) * (y - second_mean) for x, y in zip(first, second))
    first_spread = sum((x - first_mean) ** 2 for x in first)
    second_spread = sum((y - second_mean) ** 2 for y in second)
    if first_spread == 0 or second_spread == 0:
        return None
    return numerator / math.sqrt(first_spread * second_spread)


def summarize(records: list[dict]) -> dict:
    if not records:
        raise ValueError("no measured placements")
    dx = [record["dx_mm"] for record in records]
    dy = [record["dy_mm"] for record in records]
    radius = [math.hypot(x, y) for x, y in zip(dx, dy)]
    mean_x, mean_y = mean(dx), mean(dy)
    return {
        "count": len(records),
        "mean_dx_mm": mean_x,
        "mean_dy_mm": mean_y,
        "count_dx_positive": sum(value > 0 for value in dx),
        "count_dy_negative": sum(value < 0 for value in dy),
        "mean_bias_magnitude_mm": math.hypot(mean_x, mean_y),
        "median_radial_error_mm": percentile(radius, 0.5),
        "p95_radial_error_mm": percentile(radius, 0.95),
        "max_radial_error_mm": max(radius),
        "rms_radial_error_mm": math.sqrt(mean([value * value for value in radius])),
        "rms_after_mean_bias_removed_mm": math.sqrt(
            mean([(x - mean_x) ** 2 + (y - mean_y) ** 2 for x, y in zip(dx, dy)])
        ),
        "mean_absolute_rotation_error_deg": mean(
            [abs(record["rotation_error_deg"]) for record in records]
        ),
        "count_radial_over_0_30_mm": sum(value > 0.30 for value in radius),
    }


def entity(connection: sqlite3.Connection, kind: str, identifier: str) -> dict:
    row = connection.execute(
        "SELECT document FROM entities WHERE kind = ? AND id = ?",
        (kind, identifier),
    ).fetchone()
    if row is None:
        raise ValueError(f"missing {kind} {identifier}")
    return json.loads(row[0])


def analyze(csv_path: Path, database_path: Path) -> dict:
    with csv_path.open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    contacts = [row for row in rows if row["result_top"] == "1"]
    identifiers = [row["execution_id"] for row in contacts]
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("result CSV repeats an execution")
    uri = f"file:{database_path.resolve()}?mode=ro"
    records = []
    with sqlite3.connect(uri, uri=True) as connection:
        for row in contacts:
            identifier = row["execution_id"]
            run = entity(connection, "placement_execution", identifier)
            outcome = entity(connection, "placement_outcome", identifier)
            x = outcome["measured_offset_x_mm"]
            y = outcome["measured_offset_y_mm"]
            rotation = outcome["measured_rotation_error_deg"]
            if x is None or y is None or rotation is None:
                raise ValueError(f"result image has no measured offset: {identifier}")
            if row["offset_mm"] and abs(math.hypot(x, y) - float(row["offset_mm"])) > 1e-6:
                raise ValueError(f"CSV and stored offset disagree: {identifier}")
            report = run["report"]
            registration = report.get("board_target_registration") or {}
            pickup = report.get("pickup_report") or {}
            alignment = pickup.get("alignment") or {}
            comparison = report.get("top_inspection_comparison") or {}
            inspection = report.get("top_inspection") or {}
            records.append(
                {
                    "id": identifier,
                    "reference": row["reference"],
                    "created_at": row["created_at"],
                    "status": row["execution_status"],
                    "feeder_id": report.get("feeder_id"),
                    "nozzle_id": report.get("nozzle_id"),
                    "pocket_index": report.get("pocket_index"),
                    "board_x_mm": (report.get("board_place") or {}).get("x_mm"),
                    "board_y_mm": (report.get("board_place") or {}).get("y_mm"),
                    "dx_mm": x,
                    "dy_mm": y,
                    "rotation_error_deg": rotation,
                    "target_correction_x_mm": registration.get("correction_x_mm"),
                    "target_correction_y_mm": registration.get("correction_y_mm"),
                    "held_final_residual_mm": alignment.get("final_centering_residual_mm"),
                    "inspection_algorithm": inspection.get("algorithm_version"),
                    "model_cv_center_delta_mm": comparison.get("center_delta_mm"),
                }
            )
    records.sort(key=lambda record: record["created_at"])
    by_feeder = defaultdict(list)
    by_inspection = defaultdict(list)
    for record in records:
        by_feeder[record["feeder_id"]].append(record)
        by_inspection[record["inspection_algorithm"]].append(record)
    center_deltas = [
        record["model_cv_center_delta_mm"]
        for record in records
        if record["model_cv_center_delta_mm"] is not None
    ]
    paired_held = [record for record in records if record["held_final_residual_mm"] is not None]
    paired_target_x = [record for record in records if record["target_correction_x_mm"] is not None]
    paired_target_y = [record for record in records if record["target_correction_y_mm"] is not None]
    return {
        "source_csv": str(csv_path),
        "measurement_note": "Stored top-camera results are model/CV estimates, not independent metrology.",
        "csv_attempts": len(rows),
        "physical_contacts_with_result_image": len(contacts),
        "measured": summarize(records),
        "first_half": summarize(records[: len(records) // 2]),
        "second_half": summarize(records[len(records) // 2 :]),
        "by_feeder_id": {
            identifier: summarize(group) for identifier, group in sorted(by_feeder.items())
        },
        "by_inspection_algorithm": {
            identifier: summarize(group)
            for identifier, group in sorted(by_inspection.items())
        },
        "inspection_algorithms": dict(Counter(record["inspection_algorithm"] for record in records)),
        "mean_target_correction_mm": mean(
            [
                math.hypot(record["target_correction_x_mm"], record["target_correction_y_mm"])
                for record in records
                if record["target_correction_x_mm"] is not None
                and record["target_correction_y_mm"] is not None
            ]
        ),
        "mean_held_final_residual_mm": mean(
            [record["held_final_residual_mm"] for record in paired_held]
        ),
        "model_cv_center_delta_mm": {
            "count": len(center_deltas),
            "median": percentile(center_deltas, 0.5),
            "p95": percentile(center_deltas, 0.95),
            "maximum": max(center_deltas) if center_deltas else None,
        },
        "exploratory_correlations": {
            "dx_vs_target_correction_x": correlation(
                [record["dx_mm"] for record in paired_target_x],
                [record["target_correction_x_mm"] for record in paired_target_x],
            ),
            "dy_vs_target_correction_y": correlation(
                [record["dy_mm"] for record in paired_target_y],
                [record["target_correction_y_mm"] for record in paired_target_y],
            ),
            "radial_error_vs_held_final_residual": correlation(
                [math.hypot(record["dx_mm"], record["dy_mm"]) for record in paired_held],
                [record["held_final_residual_mm"] for record in paired_held],
            ),
            "dx_vs_board_x": correlation(
                [record["dx_mm"] for record in records],
                [record["board_x_mm"] for record in records],
            ),
            "dy_vs_board_y": correlation(
                [record["dy_mm"] for record in records],
                [record["board_y_mm"] for record in records],
            ),
        },
        "largest_errors": sorted(
            [
                {
                    "reference": record["reference"],
                    "id": record["id"],
                    "dx_mm": record["dx_mm"],
                    "dy_mm": record["dy_mm"],
                    "radial_mm": math.hypot(record["dx_mm"], record["dy_mm"]),
                }
                for record in records
            ],
            key=lambda item: item["radial_mm"],
            reverse=True,
        )[:5],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("database", type=Path)
    arguments = parser.parse_args()
    print(json.dumps(analyze(arguments.csv, arguments.database), indent=2))


if __name__ == "__main__":
    main()
