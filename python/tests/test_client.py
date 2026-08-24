import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from velocitypnp import Client, OperationFailed


class ClientTests(unittest.TestCase):
    def test_wait_returns_successful_operation(self):
        client = Client(timeout=1)
        pending = {"id": "op", "status": "pending", "message": None}
        complete = {"id": "op", "status": "succeeded", "message": "done"}
        with patch.object(client, "_request", return_value=complete):
            self.assertEqual(client.wait(pending, poll_interval=0), complete)

    def test_wait_raises_operation_message(self):
        client = Client(timeout=1)
        operation = {"id": "op", "status": "failed", "message": "outside bounds"}
        with self.assertRaisesRegex(OperationFailed, "outside bounds"):
                client.wait(operation, poll_interval=0)

    def test_import_board_builds_multipart_request(self):
        client = Client(timeout=1)
        with TemporaryDirectory() as directory:
            pcb = Path(directory, "fixture.kicad_pcb")
            pcb.write_text("(kicad_pcb)", encoding="utf-8")
            with patch.object(client, "_request_data", return_value={"name": "fixture"}) as send:
                result = client.import_board(pcb)
        self.assertEqual(result["name"], "fixture")
        method, path, payload, content_type = send.call_args.args
        self.assertEqual((method, path), ("POST", "/boards/import"))
        self.assertIn(b"(kicad_pcb)", payload)
        self.assertTrue(content_type.startswith("multipart/form-data; boundary="))

    def test_camera_snapshot_uses_role_scoped_png_endpoint(self):
        client = Client(timeout=1)
        with patch.object(client, "_request_bytes", return_value=b"png") as send:
            self.assertEqual(client.camera_snapshot("top_camera"), b"png")
        send.assert_called_once_with("GET", "/cameras/top_camera/snapshot", "image/png")
        with self.assertRaisesRegex(ValueError, "camera role"):
            client.camera_snapshot("laptop_camera")

    def test_camera_controls_are_role_scoped_and_ordered(self):
        client = Client(timeout=1)
        response = {"controls": [{"id": 10094849, "current": 1}]}
        with patch.object(client, "_request", return_value=response) as send:
            controls = client.set_camera_controls(
                "bottom_camera", {10094849: 1, 10094851: False, 10094850: 200},
            )
        self.assertEqual(controls, response["controls"])
        send.assert_called_once_with("PUT", "/cameras/bottom_camera/controls", {
            "controls": [
                {"id": 10094849, "value": 1},
                {"id": 10094851, "value": False},
                {"id": 10094850, "value": 200},
            ],
        })

    def test_camera_profile_and_reproducibility_are_role_scoped(self):
        client = Client(timeout=1)
        profile = {"role": "top_camera", "settle_frames": 5}
        with patch.object(client, "_request", side_effect=[profile, profile, {"report": {}}]) as send:
            self.assertEqual(client.camera_profile("top_camera"), profile)
            self.assertEqual(client.set_camera_profile("top_camera", profile), profile)
            self.assertEqual(
                client.camera_reproducibility(
                    "top_camera",
                    "00000000-0000-0000-0000-000000000000",
                ),
                {"report": {}},
            )
        self.assertEqual(send.call_args_list[0].args, ("GET", "/cameras/top_camera/profile"))
        self.assertEqual(
            send.call_args_list[1].args,
            ("PUT", "/cameras/top_camera/profile", profile),
        )
        self.assertEqual(send.call_args_list[2].args, (
            "POST",
            "/cameras/top_camera/reproducibility",
            {
                "lease_id": "00000000-0000-0000-0000-000000000000",
                "sample_count": 30,
            },
        ))
        with self.assertRaisesRegex(ValueError, "sample_count"):
            client.camera_reproducibility("top_camera", "lease", sample_count=0)

    def test_workspace_survey_estimates_then_starts_with_explicit_safety(self):
        client = Client(timeout=1)
        estimate = {"planned_tiles": 1886, "estimated_retained_bytes": 12000000}
        started = {"scan": {"id": "scan"}, "operation": {"id": "operation"}}
        with patch.object(client, "_request", side_effect=[estimate, started]) as send:
            self.assertEqual(
                client.estimate_workspace_scan(
                    "calibration", region_preset="installed_plates",
                    feed_mm_per_min=900.0,
                ),
                estimate,
            )
            self.assertEqual(
                client.start_workspace_scan(
                    "calibration",
                    "lease",
                    region_preset="installed_plates",
                    feed_mm_per_min=900.0,
                    safety_checks_confirmed=True,
                ),
                started,
            )
        specification = {
            "calibration_id": "calibration",
            "mode": "survey",
            "artifact_retention": "mosaic_only",
            "region_preset": "installed_plates",
            "feed_mm_per_min": 900.0,
        }
        self.assertEqual(
            send.call_args_list[0].args,
            ("POST", "/workspace-scans/estimate", specification),
        )
        self.assertEqual(send.call_args_list[1].args, (
            "POST",
            "/workspace-scans",
            {
                "lease_id": "lease",
                "confirmed_external_safety_checks": True,
                **specification,
            },
        ))

        crop = {"x_min_mm": 0.0, "x_max_mm": 80.0,
                "y_min_mm": 80.0, "y_max_mm": 300.0}
        with patch.object(client, "_request", return_value=estimate) as send:
            client.estimate_workspace_scan(
                "calibration",
                region=crop,
                region_preset="installed_plates",
                artifact_retention="transient_sources",
            )
        send.assert_called_once_with("POST", "/workspace-scans/estimate", {
            "calibration_id": "calibration",
            "mode": "survey",
            "artifact_retention": "transient_sources",
            "region": crop,
            "region_preset": "installed_plates",
        })

        with patch.object(client, "_request", return_value=estimate) as send:
            client.estimate_workspace_scan(
                "calibration",
                region_preset="installed_plates",
                mode="reconnaissance",
            )
        send.assert_called_once_with("POST", "/workspace-scans/estimate", {
            "calibration_id": "calibration",
            "mode": "reconnaissance",
            "artifact_retention": "mosaic_only",
            "region_preset": "installed_plates",
        })

        with patch.object(client, "_request") as send:
            with self.assertRaisesRegex(ValueError, "explicit confirmation"):
                client.start_workspace_scan(
                    "calibration", "lease", region_preset="installed_plates",
                )
            with self.assertRaisesRegex(ValueError, "region or region_preset"):
                client.estimate_workspace_scan("calibration")
            with self.assertRaisesRegex(ValueError, "above zero"):
                client.estimate_workspace_scan(
                    "calibration",
                    region_preset="installed_plates",
                    feed_mm_per_min=0.0,
                )
        send.assert_not_called()

    def test_workspace_refinement_uses_parent_target_and_transient_sources(self):
        client = Client(timeout=1)
        targets = {
            "algorithm_version": "workspace_seam_health.v2",
            "target_regions": [{"x_min_mm": 346.0}],
        }
        estimate = {"planned_tiles": 800}
        started = {"scan": {"mode": "refinement"}}
        with patch.object(
            client, "_request", side_effect=[targets, estimate, started],
        ) as send:
            self.assertEqual(client.workspace_refinement_targets("parent"), targets)
            self.assertEqual(
                client.estimate_workspace_refinement(
                    "calibration", "parent", 6,
                ),
                estimate,
            )
            self.assertEqual(
                client.start_workspace_refinement(
                    "calibration",
                    "parent",
                    6,
                    "lease",
                    safety_checks_confirmed=True,
                ),
                started,
            )
        specification = {
            "calibration_id": "calibration",
            "mode": "refinement",
            "artifact_retention": "transient_sources",
            "parent_scan_id": "parent",
            "refinement_target_index": 6,
        }
        self.assertEqual(
            send.call_args_list[0].args,
            ("GET", "/workspace-scans/parent/refinement-targets"),
        )
        self.assertEqual(
            send.call_args_list[1].args,
            ("POST", "/workspace-scans/estimate", specification),
        )
        self.assertEqual(send.call_args_list[2].args, (
            "POST",
            "/workspace-scans",
            {
                "lease_id": "lease",
                "confirmed_external_safety_checks": True,
                **specification,
            },
        ))

        with patch.object(client, "_request") as send:
            with self.assertRaisesRegex(ValueError, "target index cannot be negative"):
                client.estimate_workspace_refinement(
                    "calibration", "parent", -1,
                )
            with self.assertRaisesRegex(ValueError, "explicit confirmation"):
                client.start_workspace_refinement(
                    "calibration", "parent", 0, "lease",
                )
        send.assert_not_called()

    def test_workspace_refinement_batch_requires_safety_and_sets_limits(self):
        client = Client(timeout=1)
        started = {
            "operation": {"id": "batch"},
            "items": [{"target": {"target_index": 2, "priority": 1}}],
        }
        with patch.object(client, "_request", return_value=started) as send:
            self.assertEqual(
                client.start_workspace_refinement_batch(
                    "calibration",
                    "parent",
                    "lease",
                    max_targets=3,
                    feed_mm_per_min=1200.0,
                    safety_checks_confirmed=True,
                ),
                started,
            )
        send.assert_called_once_with(
            "POST",
            "/workspace-scans/parent/refinements",
            {
                "lease_id": "lease",
                "calibration_id": "calibration",
                "confirmed_external_safety_checks": True,
                "max_targets": 3,
                "feed_mm_per_min": 1200.0,
            },
        )

        with patch.object(client, "_request") as send:
            with self.assertRaisesRegex(ValueError, "explicit confirmation"):
                client.start_workspace_refinement_batch(
                    "calibration", "parent", "lease",
                )
            with self.assertRaisesRegex(ValueError, "1 through 32"):
                client.start_workspace_refinement_batch(
                    "calibration",
                    "parent",
                    "lease",
                    max_targets=0,
                    safety_checks_confirmed=True,
                )
        send.assert_not_called()

    def test_workspace_refinement_composite_and_artifacts(self):
        client = Client(timeout=1)
        quality = {"accepted": True, "accepted_target_count": 8}
        with patch.object(client, "_request", return_value=quality) as send:
            self.assertEqual(
                client.build_workspace_refinement_composite("parent"), quality,
            )
        send.assert_called_once_with(
            "POST",
            "/workspace-scans/parent/refinement-composite",
            {},
        )

        with patch.object(client, "_request", return_value=quality) as send:
            self.assertEqual(
                client.workspace_scan_refinement_quality("parent"), quality,
            )
        send.assert_called_once_with(
            "GET",
            "/workspace-scans/parent/artifacts/refinement-quality.json",
        )

        for method, artifact_name in [
            (client.workspace_scan_refined_mosaic, "mosaic-refined.png"),
            (
                client.workspace_scan_refinement_quality_preview,
                "mosaic-refined-quality.png",
            ),
        ]:
            with patch.object(client, "_request_bytes", return_value=b"png") as send:
                self.assertEqual(method("parent"), b"png")
            send.assert_called_once_with(
                "GET",
                f"/workspace-scans/parent/artifacts/{artifact_name}",
                "image/png",
            )

    def test_workspace_board_discovery_and_artifacts(self):
        client = Client(timeout=1)
        report = {"algorithm_version": "workspace_board_discovery.v1"}
        with patch.object(client, "_request", return_value=report) as send:
            self.assertEqual(client.discover_workspace_boards("scan"), report)
        send.assert_called_once_with(
            "POST", "/workspace-scans/scan/board-discovery", {},
        )

        with patch.object(client, "_request", return_value=report) as send:
            self.assertEqual(client.discover_workspace_objects("scan"), report)
        send.assert_called_once_with(
            "POST", "/workspace-scans/scan/object-proposals", {},
        )

        feeder_report = {"algorithm_version": "strip_feeder_vision.v1"}
        with patch.object(client, "_request", return_value=feeder_report) as send:
            self.assertEqual(
                client.analyze_strip_feeder("scan", 2), feeder_report,
            )
        send.assert_called_once_with(
            "POST",
            "/workspace-scans/scan/strip-feeder-vision",
            {"candidate_index": 2},
        )
        with self.assertRaises(ValueError):
            client.analyze_strip_feeder("scan", -1)

        draft = {"id": "draft", "status": "vision_draft"}
        with patch.object(client, "_request", return_value=draft) as send:
            self.assertEqual(
                client.create_strip_feeder_draft(
                    "scan", 2, 1, name="R strip", part_key="R0603:10k",
                    part_rotation_deg=90.0,
                ),
                draft,
            )
        send.assert_called_once_with(
            "POST",
            "/strip-feeder-registrations",
            {
                "scan_id": "scan",
                "candidate_index": 2,
                "proposal_index": 1,
                "name": "R strip",
                "part_key": "R0603:10k",
                "part_rotation_deg": 90.0,
            },
        )
        with self.assertRaises(ValueError):
            client.create_strip_feeder_draft("scan", 0, -1)

        with patch.object(
            client, "_request", return_value={"registrations": [draft]},
        ) as send:
            self.assertEqual(client.strip_feeder_drafts(), [draft])
        send.assert_called_once_with("GET", "/strip-feeder-registrations")

        with patch.object(client, "_request", return_value=draft) as send:
            self.assertEqual(client.strip_feeder_draft("draft"), draft)
        send.assert_called_once_with(
            "GET", "/strip-feeder-registrations/draft",
        )

        with patch.object(client, "_request", return_value=report) as send:
            self.assertEqual(
                client.workspace_board_discovery_report("scan"), report,
            )
        send.assert_called_once_with(
            "GET",
            "/workspace-scans/scan/artifacts/board-discovery.json",
        )

        with patch.object(client, "_request_bytes", return_value=b"png") as send:
            self.assertEqual(
                client.workspace_board_discovery_preview("scan"), b"png",
            )
        send.assert_called_once_with(
            "GET",
            "/workspace-scans/scan/artifacts/mosaic-board-candidates.png",
            "image/png",
        )

        with patch.object(client, "_request_bytes", return_value=b"crop") as send:
            self.assertEqual(client.workspace_object_evidence("scan", 2), b"crop")
        send.assert_called_once_with(
            "GET",
            "/workspace-scans/scan/artifacts/object-evidence-002.png",
            "image/png",
        )
        with self.assertRaises(ValueError):
            client.workspace_object_evidence("scan", -1)

        with patch.object(client, "_request_bytes", return_value=b"preview") as send:
            self.assertEqual(
                client.strip_feeder_vision_preview("scan", 2), b"preview",
            )
        send.assert_called_once_with(
            "GET",
            "/workspace-scans/scan/artifacts/strip-feeder-vision-002.png",
            "image/png",
        )
        with self.assertRaises(ValueError):
            client.strip_feeder_vision_preview("scan", -1)

        baseline = {"id": "baseline", "scan_id": "scan"}
        with patch.object(client, "_request", return_value=baseline) as send:
            self.assertEqual(
                client.set_workspace_inventory_baseline("scan"), baseline,
            )
        send.assert_called_once_with(
            "POST", "/workspace-scans/scan/inventory-baseline", {},
        )

        with patch.object(client, "_request", return_value=baseline) as send:
            self.assertEqual(client.workspace_inventory_baseline(), baseline)
        send.assert_called_once_with("GET", "/workspace-inventory/baseline")

        changes = {"modified_count": 1, "target_regions": [{}]}
        with patch.object(client, "_request", return_value=changes) as send:
            self.assertEqual(
                client.compare_workspace_observation(
                    "scan", baseline_id="baseline",
                ),
                changes,
            )
        send.assert_called_once_with(
            "POST",
            "/workspace-scans/scan/change-set",
            {"baseline_id": "baseline"},
        )

        with patch.object(client, "_request", return_value=changes) as send:
            self.assertEqual(client.workspace_change_set_report("scan"), changes)
        send.assert_called_once_with(
            "GET",
            "/workspace-scans/scan/artifacts/workspace-change-set.json",
        )

    def test_board_registration_requires_confirmation_and_returns_report(self):
        client = Client(timeout=1)
        with patch.object(client, "_request") as send:
            with self.assertRaisesRegex(ValueError, "explicit confirmation"):
                client.start_board_registration("scan", 0, "board", "lease")
        send.assert_not_called()

        started = {
            "registration": {"id": "registration"},
            "operation": {"id": "operation", "status": "succeeded"},
        }
        completed = {"id": "registration", "passed": True}
        with patch.object(
            client, "_request", side_effect=[started, completed],
        ) as send:
            with patch.object(client, "wait"):
                report = client.register_board(
                    "scan",
                    2,
                    "board",
                    "lease",
                    label="FTP panel",
                    safety_checks_confirmed=True,
                )
        self.assertEqual(report, completed)
        self.assertEqual(send.call_args_list[0].args, (
            "POST",
            "/workspace-scans/scan/board-registrations",
            {
                "lease_id": "lease",
                "candidate_index": 2,
                "board_id": "board",
                "label": "FTP panel",
                "sample_count": 3,
                "confirmed_external_safety_checks": True,
            },
        ))

    def test_workspace_mosaic_selects_a_calibrated_height_layer(self):
        client = Client(timeout=1)
        layered_report = {
            "mosaic_layers": [{
                "surface_model": "installed_plate_planes",
                "artifact_name": "mosaic-installed-plate-planes.png",
            }],
        }
        with (
            patch.object(client, "_request", return_value=layered_report),
            patch.object(client, "_request_bytes", return_value=b"png") as send,
        ):
            result = client.workspace_scan_mosaic(
                "scan", surface_model="installed_plate_planes",
            )
        self.assertEqual(result, b"png")
        send.assert_called_once_with(
            "GET",
            "/workspace-scans/scan/artifacts/"
            "mosaic-installed-plate-planes.png",
            "image/png",
        )

        reconnaissance_report = {
            "mosaic_layers": [{
                "surface_model": "installed_plate_planes",
                "artifact_name": "mosaic.png",
            }],
        }
        with (
            patch.object(client, "_request", return_value=reconnaissance_report),
            patch.object(client, "_request_bytes", return_value=b"recon") as send,
        ):
            result = client.workspace_scan_mosaic(
                "scan", surface_model="installed_plate_planes",
            )
        self.assertEqual(result, b"recon")
        send.assert_called_once_with(
            "GET",
            "/workspace-scans/scan/artifacts/mosaic.png",
            "image/png",
        )

        with self.assertRaisesRegex(ValueError, "surface_model"):
            client.workspace_scan_mosaic("scan", surface_model="unknown")

        heights = {"accepted_observations": 42, "observations": []}
        with patch.object(client, "_request", return_value=heights) as send:
            self.assertEqual(client.workspace_scan_surface_heights("scan"), heights)
        send.assert_called_once_with(
            "GET",
            "/workspace-scans/scan/artifacts/"
            "surface-height-observations.json",
        )

        height_map = {"cell_size_mm": 2.0, "cells": []}
        with patch.object(client, "_request", return_value=height_map) as send:
            self.assertEqual(client.workspace_scan_height_map("scan"), height_map)
        send.assert_called_once_with(
            "GET", "/workspace-scans/scan/artifacts/surface-height-map.json",
        )

        image_methods = [
            (
                client.workspace_scan_height_map_preview,
                "surface-height-map.png",
            ),
            (
                client.workspace_scan_height_confidence,
                "surface-height-confidence.png",
            ),
            (
                client.workspace_scan_height_corrected_mosaic,
                "mosaic-height-corrected.png",
            ),
            (
                client.workspace_scan_height_overlay,
                "surface-height-overlay.png",
            ),
            (
                client.workspace_scan_height_class_preview,
                "mosaic-height-classes.png",
            ),
        ]
        for method, artifact_name in image_methods:
            with patch.object(client, "_request_bytes", return_value=b"png") as send:
                self.assertEqual(method("scan"), b"png")
            send.assert_called_once_with(
                "GET",
                f"/workspace-scans/scan/artifacts/{artifact_name}",
                "image/png",
            )

        seam_health = {"recommended_action": "targeted_refinement"}
        with patch.object(client, "_request", return_value=seam_health) as send:
            self.assertEqual(client.workspace_scan_seam_health("scan"), seam_health)
        send.assert_called_once_with(
            "GET", "/workspace-scans/scan/artifacts/seam-health.json",
        )

        seam_image_methods = [
            (client.workspace_scan_disagreement, "mosaic-disagreement.png"),
            (client.workspace_scan_overlap_confidence, "overlap-confidence.png"),
            (client.workspace_scan_seam_overlay, "seam-health-overlay.png"),
            (client.workspace_scan_seam_preview, "mosaic-seam-health.png"),
        ]
        for method, artifact_name in seam_image_methods:
            with patch.object(client, "_request_bytes", return_value=b"png") as send:
                self.assertEqual(method("scan"), b"png")
            send.assert_called_once_with(
                "GET",
                f"/workspace-scans/scan/artifacts/{artifact_name}",
                "image/png",
            )

    def test_home_requires_explicit_physical_confirmation(self):
        client = Client(timeout=1)
        with patch.object(client, "_request") as send:
            with self.assertRaisesRegex(ValueError, "explicit confirmation"):
                client.home("00000000-0000-0000-0000-000000000000")
        send.assert_not_called()

    def test_jog_preserves_rotation_when_omitted(self):
        client = Client(timeout=1)
        operation = {"id": "op", "status": "succeeded", "message": None}
        with patch.object(client, "_request", return_value=operation) as send:
            client.jog(
                "00000000-0000-0000-0000-000000000000",
                x_mm=10.0,
                y_mm=20.0,
                z_mm=31.5,
            )
        send.assert_called_once_with("POST", "/machine/jog", {
            "lease_id": "00000000-0000-0000-0000-000000000000",
            "target": {"x_mm": 10.0, "y_mm": 20.0, "z_mm": 31.5},
            "feed_mm_per_min": 300.0,
        })

    def test_sensor_read_uses_lease_and_profile_id(self):
        client = Client(timeout=1)
        reading = {"sensor_id": "vacuum_1", "raw_value": -42}
        with patch.object(client, "_request", return_value=reading) as send:
            self.assertEqual(client.read_sensor("lease", "vacuum_1"), reading)
        send.assert_called_once_with(
            "POST",
            "/machine/sensors/vacuum_1/read",
            {"lease_id": "lease"},
        )

    def test_visual_home_defaults_to_observation_at_seed(self):
        client = Client(timeout=1)
        observation = {"applied": False, "initial": {"error_x_px": 1.0}}
        with patch.object(client, "_request", return_value=observation) as send:
            self.assertEqual(client.visual_home("lease"), observation)
        send.assert_called_once_with("POST", "/machine/visual-home", {
            "lease_id": "lease",
            "apply_offset": False,
            "move_to_seed": True,
        })
        with self.assertRaisesRegex(ValueError, "without moving"):
            client.visual_home("lease", apply_offset=True, move_to_seed=False)

    def test_calibration_run_requires_safety_confirmation_and_is_machine_driven(self):
        client = Client(timeout=1)
        with patch.object(client, "_request") as send:
            with self.assertRaisesRegex(ValueError, "explicit confirmation"):
                client.start_calibration_run("calibration", "lease")
        send.assert_not_called()

        started = {"run": {"id": "run"}, "operation": {"id": "op"}}
        with patch.object(client, "_request", return_value=started) as send:
            self.assertEqual(
                client.start_calibration_run(
                    "calibration", "lease", safety_checks_confirmed=True,
                ),
                started,
            )
        send.assert_called_once_with("POST", "/calibrations/calibration/runs", {
            "lease_id": "lease",
            "confirmed_external_safety_checks": True,
            "visual_homing_sample_count": 5,
        })

    def test_calibration_artifacts_plan_recompute_and_targeted_capture(self):
        client = Client(timeout=1)
        with patch.object(client, "_request", return_value={"artifacts": [{"id": "x"}]}) as send:
            self.assertEqual(client.calibration_artifacts(), [{"id": "x"}])
        send.assert_called_once_with("GET", "/calibration/artifacts")

        with patch.object(client, "_request", return_value={"requires_motion": False}) as send:
            client.calibration_plan("calibration", ["camera_ray_transfer"])
        send.assert_called_once_with(
            "POST", "/calibrations/calibration/artifact-plan",
            {"targets": ["camera_ray_transfer"]},
        )

        with patch.object(client, "_request", return_value={"recomputed": []}) as send:
            client.recompute_calibration_artifacts("calibration")
        send.assert_called_once_with(
            "POST", "/calibrations/calibration/derived-artifacts/recompute",
            {"targets": []},
        )

        started = {
            "run": {"id": "run"},
            "operation": {"id": "op", "status": "succeeded"},
        }
        completed = {"id": "run", "status": "succeeded"}
        with patch.object(client, "_request", side_effect=[started, completed]) as send:
            result = client.capture_calibration_artifacts(
                "calibration", "lease", ["nozzle_height_n1"],
                safety_checks_confirmed=True,
            )
        self.assertEqual(result, completed)
        self.assertEqual(send.call_args_list[0].args, (
            "POST", "/calibrations/calibration/artifact-runs", {
                "lease_id": "lease",
                "confirmed_external_safety_checks": True,
                "targets": ["nozzle_height_n1"],
            },
        ))

    def test_vacuum_probe_requires_confirmation_and_returns_persisted_report(self):
        client = Client(timeout=1)
        with patch.object(client, "_request") as send:
            with self.assertRaisesRegex(ValueError, "explicit confirmation"):
                client.vacuum_probe("calibration", "lease", "N1")
            with self.assertRaisesRegex(ValueError, "N1 or N2"):
                client.vacuum_probe(
                    "calibration", "lease", "N3", safety_checks_confirmed=True,
                )
        send.assert_not_called()

        operation = {"id": "op", "status": "succeeded", "message": "seal"}
        snapshot = {"values": {"vacuum_probe": {"N1": {"passed": True}}}}
        with patch.object(client, "_request", side_effect=[operation, snapshot]) as send:
            report = client.vacuum_probe(
                "calibration", "lease", "N1", safety_checks_confirmed=True,
            )
        self.assertEqual(report, {"passed": True})
        self.assertEqual(send.call_args_list[0].args, (
            "POST",
            "/calibrations/calibration/vacuum-probe",
            {
                "lease_id": "lease",
                "nozzle_id": "N1",
                "confirmed_external_safety_checks": True,
            },
        ))
        self.assertEqual(
            send.call_args_list[1].args,
            ("GET", "/calibrations/calibration"),
        )

    def test_vacuum_characterization_returns_persisted_report(self):
        client = Client(timeout=1)
        with patch.object(client, "_request") as send:
            with self.assertRaisesRegex(ValueError, "explicit confirmation"):
                client.characterize_vacuum("calibration", "lease", "N1")
            with self.assertRaisesRegex(ValueError, "N1 or N2"):
                client.characterize_vacuum(
                    "calibration", "lease", "N3", safety_checks_confirmed=True,
                )
        send.assert_not_called()

        operation = {"id": "op", "status": "succeeded", "message": "characterized"}
        snapshot = {
            "values": {
                "vacuum_sensor_characterization": {"N1": {"passed": True}},
            },
        }
        with patch.object(client, "_request", side_effect=[operation, snapshot]) as send:
            report = client.characterize_vacuum(
                "calibration", "lease", "N1", safety_checks_confirmed=True,
            )
        self.assertEqual(report, {"passed": True})
        self.assertEqual(send.call_args_list[0].args, (
            "POST",
            "/calibrations/calibration/vacuum-characterization",
            {
                "lease_id": "lease",
                "nozzle_id": "N1",
                "confirmed_external_safety_checks": True,
            },
        ))

    def test_fine_home_requires_confirmation_and_waits_for_completion(self):
        client = Client(timeout=1)
        with patch.object(client, "_request") as send:
            with self.assertRaisesRegex(ValueError, "explicit confirmation"):
                client.fine_home("calibration", "lease")
            with self.assertRaisesRegex(ValueError, "N1 or N2"):
                client.fine_home(
                    "calibration", "lease", "N3", safety_checks_confirmed=True,
                )
        send.assert_not_called()

        operation = {"id": "op", "status": "succeeded", "message": "fine homed"}
        with patch.object(client, "_request", return_value=operation) as send:
            result = client.fine_home(
                "calibration", "lease", safety_checks_confirmed=True,
            )
        self.assertEqual(result, operation)
        send.assert_called_once_with(
            "POST",
            "/calibrations/calibration/fine-home",
            {
                "lease_id": "lease",
                "nozzle_id": "N2",
                "confirmed_external_safety_checks": True,
            },
        )

    def test_xy_direction_response_requires_confirmation_and_returns_report(self):
        client = Client(timeout=1)
        with patch.object(client, "_request") as send:
            with self.assertRaisesRegex(ValueError, "explicit confirmation"):
                client.calibrate_xy_direction_response(
                    "calibration", "lease",
                )
        send.assert_not_called()

        operation = {"id": "op", "status": "succeeded", "message": "measured"}
        snapshot = {
            "values": {"local_xy_direction_response": {"passed": True}},
        }
        with patch.object(client, "_request", side_effect=[operation, snapshot]) as send:
            report = client.calibrate_xy_direction_response(
                "calibration", "lease", safety_checks_confirmed=True,
            )
        self.assertEqual(report, {"passed": True})
        self.assertEqual(send.call_args_list[0].args, (
            "POST",
            "/calibrations/calibration/xy-direction-response",
            {
                "lease_id": "lease",
                "confirmed_external_safety_checks": True,
            },
        ))

    def test_installed_geometry_requires_confirmation_and_returns_run(self):
        client = Client(timeout=1)
        with patch.object(client, "_request") as send:
            with self.assertRaisesRegex(ValueError, "explicit confirmation"):
                client.calibrate_installed_geometry("calibration", "lease")
        send.assert_not_called()

        started = {
            "run": {"id": "run"},
            "operation": {"id": "op", "status": "succeeded"},
        }
        completed = {"id": "run", "status": "succeeded"}
        with patch.object(client, "_request", side_effect=[started, completed]) as send:
            with patch.object(client, "wait"):
                report = client.calibrate_installed_geometry(
                    "calibration", "lease", safety_checks_confirmed=True,
                )
        self.assertEqual(report, completed)
        self.assertEqual(send.call_args_list[0].args, (
            "POST",
            "/calibrations/calibration/installed-geometry-runs",
            {
                "lease_id": "lease",
                "confirmed_external_safety_checks": True,
            },
        ))

    def test_nozzle_height_observation_records_bounded_working_plane_error(self):
        client = Client(timeout=1)
        snapshot = {"values": {"nozzle_height_model": {"models": []}}}
        with patch.object(client, "_request", return_value=snapshot) as send:
            result = client.observe_nozzle_height(
                "calibration", "N1", 0.5, -0.5, 0.3, nozzle_z_mm=3.5,
            )
        self.assertEqual(result, snapshot)
        send.assert_called_once_with(
            "POST",
            "/calibrations/calibration/nozzle-height-observation",
            {
                "nozzle_id": "N1",
                "nozzle_z_mm": 3.5,
                "observed_error_x_mm": 0.5,
                "observed_error_y_mm": -0.5,
                "measurement_uncertainty_mm": 0.3,
                "post_correction_residual": False,
            },
        )
        with self.assertRaisesRegex(ValueError, "N1 or N2"):
            client.observe_nozzle_height("calibration", "N3", 0.0, 0.0, 0.3)

    def test_job_simulation_selects_calibration_explicitly(self):
        client = Client(timeout=1)
        operation = {"id": "op", "status": "succeeded", "message": "done"}
        with patch.object(client, "_request", return_value=operation) as send:
            self.assertEqual(
                client.simulate_job("job", "lease", "calibration"),
                operation,
            )
        send.assert_called_once_with("POST", "/jobs/job/simulate", {
            "lease_id": "lease",
            "calibration_id": "calibration",
        })

        with patch.object(client, "_request", return_value=operation) as send:
            client.simulate_job("job", "lease")
        send.assert_called_once_with("POST", "/jobs/job/simulate", {
            "lease_id": "lease",
        })


if __name__ == "__main__":
    unittest.main()
