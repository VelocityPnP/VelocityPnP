from __future__ import annotations

import json
import math
import time
import uuid
from pathlib import Path
from typing import Any
from urllib.error import HTTPError
from urllib.request import Request, build_opener


class ApiError(RuntimeError):
    """The daemon rejected an API request."""

    def __init__(self, status: int, message: str):
        super().__init__(f"VelocityPnP API returned {status}: {message}")
        self.status = status
        self.message = message


class OperationFailed(RuntimeError):
    """An accepted asynchronous machine operation failed."""


class Client:
    def __init__(self, base_url: str = "http://127.0.0.1:4080/api/v1", timeout: float = 90.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self._opener = build_opener()

    def __enter__(self) -> "Client":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def close(self) -> None:
        self._opener.close()

    def status(self) -> dict[str, Any]:
        return self._request("GET", "/machine/status")["machine"]

    def devices(self) -> list[dict[str, Any]]:
        return self._request("GET", "/system/devices")["devices"]

    def camera_snapshot(self, role: str) -> bytes:
        self._validate_camera_role(role)
        return self._request_bytes("GET", f"/cameras/{role}/snapshot", "image/png")

    def camera_controls(self, role: str) -> list[dict[str, Any]]:
        self._validate_camera_role(role)
        return self._request("GET", f"/cameras/{role}/controls")["controls"]

    def set_camera_controls(
        self, role: str, controls: dict[int, int | bool],
    ) -> list[dict[str, Any]]:
        self._validate_camera_role(role)
        payload = {"controls": [
            {"id": control_id, "value": value}
            for control_id, value in controls.items()
        ]}
        return self._request("PUT", f"/cameras/{role}/controls", payload)["controls"]

    def camera_profile(self, role: str) -> dict[str, Any]:
        self._validate_camera_role(role)
        return self._request("GET", f"/cameras/{role}/profile")

    def set_camera_profile(
        self, role: str, profile: dict[str, Any],
    ) -> dict[str, Any]:
        self._validate_camera_role(role)
        return self._request("PUT", f"/cameras/{role}/profile", profile)

    def camera_reproducibility(
        self,
        role: str,
        lease_id: str | uuid.UUID,
        *,
        sample_count: int = 30,
    ) -> dict[str, Any]:
        self._validate_camera_role(role)
        if not 1 <= sample_count <= 60:
            raise ValueError("camera sample_count must be between 1 and 60")
        return self._request(
            "POST",
            f"/cameras/{role}/reproducibility",
            {"lease_id": str(lease_id), "sample_count": sample_count},
        )

    def estimate_workspace_scan(
        self,
        calibration_id: str | uuid.UUID,
        *,
        region: dict[str, float] | None = None,
        region_preset: str | None = None,
        mode: str = "survey",
        artifact_retention: str = "mosaic_only",
        overlap_fraction: float | None = None,
        output_mm_per_pixel: float | None = None,
        feed_mm_per_min: float | None = None,
        artifact_quota_bytes: int | None = None,
        parent_scan_id: str | uuid.UUID | None = None,
        refinement_target_index: int | None = None,
    ) -> dict[str, Any]:
        specification = self._workspace_scan_specification(
            calibration_id,
            region=region,
            region_preset=region_preset,
            mode=mode,
            artifact_retention=artifact_retention,
            overlap_fraction=overlap_fraction,
            output_mm_per_pixel=output_mm_per_pixel,
            feed_mm_per_min=feed_mm_per_min,
            artifact_quota_bytes=artifact_quota_bytes,
            parent_scan_id=parent_scan_id,
            refinement_target_index=refinement_target_index,
        )
        return self._request("POST", "/workspace-scans/estimate", specification)

    def start_workspace_scan(
        self,
        calibration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        region: dict[str, float] | None = None,
        region_preset: str | None = None,
        mode: str = "survey",
        artifact_retention: str = "mosaic_only",
        overlap_fraction: float | None = None,
        output_mm_per_pixel: float | None = None,
        feed_mm_per_min: float | None = None,
        artifact_quota_bytes: int | None = None,
        parent_scan_id: str | uuid.UUID | None = None,
        refinement_target_index: int | None = None,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        if not safety_checks_confirmed:
            raise ValueError(
                "workspace scanning requires explicit confirmation that both nozzles "
                "are level, the region and envelope are clear, and power isolation "
                "is reachable"
            )
        specification = self._workspace_scan_specification(
            calibration_id,
            region=region,
            region_preset=region_preset,
            mode=mode,
            artifact_retention=artifact_retention,
            overlap_fraction=overlap_fraction,
            output_mm_per_pixel=output_mm_per_pixel,
            feed_mm_per_min=feed_mm_per_min,
            artifact_quota_bytes=artifact_quota_bytes,
            parent_scan_id=parent_scan_id,
            refinement_target_index=refinement_target_index,
        )
        return self._request("POST", "/workspace-scans", {
            "lease_id": str(lease_id),
            "confirmed_external_safety_checks": True,
            **specification,
        })

    def workspace_scans(self) -> list[dict[str, Any]]:
        return self._request("GET", "/workspace-scans")["scans"]

    def workspace_scan(self, scan_id: str | uuid.UUID) -> dict[str, Any]:
        return self._request("GET", f"/workspace-scans/{scan_id}")

    def workspace_refinement_targets(
        self, scan_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "GET", f"/workspace-scans/{scan_id}/refinement-targets",
        )

    def estimate_workspace_refinement(
        self,
        calibration_id: str | uuid.UUID,
        parent_scan_id: str | uuid.UUID,
        target_index: int,
        *,
        overlap_fraction: float | None = None,
        output_mm_per_pixel: float | None = None,
        feed_mm_per_min: float | None = None,
        artifact_quota_bytes: int | None = None,
    ) -> dict[str, Any]:
        return self.estimate_workspace_scan(
            calibration_id,
            mode="refinement",
            artifact_retention="transient_sources",
            parent_scan_id=parent_scan_id,
            refinement_target_index=target_index,
            overlap_fraction=overlap_fraction,
            output_mm_per_pixel=output_mm_per_pixel,
            feed_mm_per_min=feed_mm_per_min,
            artifact_quota_bytes=artifact_quota_bytes,
        )

    def start_workspace_refinement(
        self,
        calibration_id: str | uuid.UUID,
        parent_scan_id: str | uuid.UUID,
        target_index: int,
        lease_id: str | uuid.UUID,
        *,
        overlap_fraction: float | None = None,
        output_mm_per_pixel: float | None = None,
        feed_mm_per_min: float | None = None,
        artifact_quota_bytes: int | None = None,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        return self.start_workspace_scan(
            calibration_id,
            lease_id,
            mode="refinement",
            artifact_retention="transient_sources",
            parent_scan_id=parent_scan_id,
            refinement_target_index=target_index,
            overlap_fraction=overlap_fraction,
            output_mm_per_pixel=output_mm_per_pixel,
            feed_mm_per_min=feed_mm_per_min,
            artifact_quota_bytes=artifact_quota_bytes,
            safety_checks_confirmed=safety_checks_confirmed,
        )

    def start_workspace_refinement_batch(
        self,
        calibration_id: str | uuid.UUID,
        parent_scan_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        max_targets: int | None = None,
        overlap_fraction: float | None = None,
        output_mm_per_pixel: float | None = None,
        feed_mm_per_min: float | None = None,
        artifact_quota_bytes: int | None = None,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        if not safety_checks_confirmed:
            raise ValueError(
                "workspace refinement requires explicit confirmation that all "
                "target regions and the travel envelope are clear, both nozzles "
                "are level, and power isolation is reachable"
            )
        if max_targets is not None and not 1 <= max_targets <= 32:
            raise ValueError("workspace refinement max_targets must be 1 through 32")
        for name, value in {
            "overlap_fraction": overlap_fraction,
            "output_mm_per_pixel": output_mm_per_pixel,
            "feed_mm_per_min": feed_mm_per_min,
            "artifact_quota_bytes": artifact_quota_bytes,
        }.items():
            if value is not None and value <= 0:
                raise ValueError(f"workspace refinement {name} must be above zero")
        request: dict[str, Any] = {
            "lease_id": str(lease_id),
            "calibration_id": str(calibration_id),
            "confirmed_external_safety_checks": True,
        }
        for name, value in {
            "max_targets": max_targets,
            "overlap_fraction": overlap_fraction,
            "output_mm_per_pixel": output_mm_per_pixel,
            "feed_mm_per_min": feed_mm_per_min,
            "artifact_quota_bytes": artifact_quota_bytes,
        }.items():
            if value is not None:
                request[name] = value
        return self._request(
            "POST", f"/workspace-scans/{parent_scan_id}/refinements", request,
        )

    def build_workspace_refinement_composite(
        self, parent_scan_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "POST",
            f"/workspace-scans/{parent_scan_id}/refinement-composite",
            {},
        )

    def discover_workspace_boards(
        self, scan_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "POST",
            f"/workspace-scans/{scan_id}/board-discovery",
            {},
        )

    def discover_workspace_objects(
        self, scan_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "POST",
            f"/workspace-scans/{scan_id}/object-proposals",
            {},
        )

    def analyze_strip_feeder(
        self, scan_id: str | uuid.UUID, candidate_index: int,
    ) -> dict[str, Any]:
        """Measure strip-feeder lanes from retained workspace evidence.

        This operation causes no machine motion and does not activate a
        feeder.
        """
        if candidate_index < 0:
            raise ValueError("candidate_index must not be negative")
        return self._request(
            "POST",
            f"/workspace-scans/{scan_id}/strip-feeder-vision",
            {"candidate_index": candidate_index},
        )

    def build_strip_feeder_inventory(
        self,
        workspace_scan_id: str | uuid.UUID,
        evidence_sources: list[tuple[str | uuid.UUID, int]],
    ) -> dict[str, Any]:
        """Combine feeder evidence and annotate one workspace mosaic."""
        if not evidence_sources:
            raise ValueError("evidence_sources must not be empty")
        if any(candidate_index < 0 for _, candidate_index in evidence_sources):
            raise ValueError("candidate indexes must not be negative")
        return self._request(
            "POST",
            f"/workspace-scans/{workspace_scan_id}/strip-feeder-inventory",
            {
                "evidence_sources": [
                    {
                        "scan_id": str(scan_id),
                        "candidate_index": candidate_index,
                    }
                    for scan_id, candidate_index in evidence_sources
                ],
            },
        )

    def promote_strip_feeder_inventory(
        self, workspace_scan_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        """Create or reuse one persistent registration for each inventory lane."""
        return self._request(
            "POST",
            f"/workspace-scans/{workspace_scan_id}/"
            "strip-feeder-inventory/registrations",
            {},
        )

    def start_strip_feeder_source_capture(
        self,
        workspace_scan_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        occupied_count: int = 12,
        empty_count: int = 12,
        lane_indices: list[int] | None = None,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        """Capture centered safe-Z images from persisted inventory pockets."""
        if not safety_checks_confirmed:
            raise ValueError(
                "feeder source capture requires explicit confirmation that the "
                "tape is fixed, both nozzles are level, the envelope is clear, "
                "and power isolation is reachable"
            )
        if occupied_count < 0 or empty_count < 0:
            raise ValueError("source capture counts must not be negative")
        if occupied_count + empty_count == 0 or occupied_count + empty_count > 48:
            raise ValueError("source capture requires between 1 and 48 images")
        lanes = lane_indices or []
        if len(lanes) != len(set(lanes)) or any(index < 0 for index in lanes):
            raise ValueError("lane_indices must be unique and not negative")
        return self._request(
            "POST",
            f"/workspace-scans/{workspace_scan_id}/"
            "strip-feeder-inventory/source-captures",
            {
                "lease_id": str(lease_id),
                "occupied_count": occupied_count,
                "empty_count": empty_count,
                "lane_indices": lanes,
                "confirmed_external_safety_checks": True,
            },
        )

    def strip_feeder_source_capture(
        self, capture_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "GET", f"/strip-feeder-source-captures/{capture_id}",
        )

    def reanalyze_strip_feeder_source_capture_model(
        self, capture_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        """Run the configured learned model on retained source images."""
        return self._request(
            "POST",
            f"/strip-feeder-source-captures/{capture_id}/reanalyze-model",
        )

    def create_strip_feeder_draft(
        self,
        scan_id: str | uuid.UUID,
        candidate_index: int,
        proposal_index: int,
        *,
        name: str | None = None,
        part_key: str | None = None,
        part_rotation_deg: float | None = None,
    ) -> dict[str, Any]:
        if candidate_index < 0 or proposal_index < 0:
            raise ValueError("candidate and proposal indexes must not be negative")
        return self._request(
            "POST",
            "/strip-feeder-registrations",
            {
                "scan_id": str(scan_id),
                "candidate_index": candidate_index,
                "proposal_index": proposal_index,
                "name": name,
                "part_key": part_key,
                "part_rotation_deg": part_rotation_deg,
            },
        )

    def strip_feeder_drafts(self) -> list[dict[str, Any]]:
        return self._request(
            "GET", "/strip-feeder-registrations",
        )["registrations"]

    def strip_feeder_draft(
        self, registration_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "GET", f"/strip-feeder-registrations/{registration_id}",
        )

    def start_strip_feeder_live_holdout(
        self,
        registration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        pocket_index: int | None = None,
        sample_count: int = 3,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        """Start a fresh top-camera check of one feeder draft."""
        if not safety_checks_confirmed:
            raise ValueError(
                "live pocket holdout requires explicit confirmation that the "
                "tape is fixed, both nozzles are level, the envelope is clear, "
                "and power isolation is reachable"
            )
        if pocket_index is not None and pocket_index < 0:
            raise ValueError("pocket_index must not be negative")
        if not 3 <= sample_count <= 5:
            raise ValueError("sample_count must be between 3 and 5")
        return self._request(
            "POST",
            f"/strip-feeder-registrations/{registration_id}/live-holdouts",
            {
                "lease_id": str(lease_id),
                "pocket_index": pocket_index,
                "sample_count": sample_count,
                "confirmed_external_safety_checks": True,
            },
        )

    def strip_feeder_live_holdout(
        self, holdout_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "GET", f"/strip-feeder-live-holdouts/{holdout_id}",
        )

    def strip_feeder_live_holdout_preview(
        self, holdout_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/strip-feeder-live-holdouts/{holdout_id}/artifacts/"
            "live-pocket-holdout.png",
            "image/png",
        )

    def start_strip_feeder_pickup_qualification(
        self,
        registration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        nozzle_id: str = "N1",
        pocket_index: int | None = None,
        fallback_drop_surface_machine_mm: tuple[float, float] | None = None,
        rotation_capture_offsets_deg: tuple[float, ...] | list[float] = (),
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        """Pick, inspect, and return one component from a validated strip."""
        if not safety_checks_confirmed:
            raise ValueError(
                "pickup qualification requires explicit confirmation that the "
                "tape is fixed and uncovered, both nozzles are level, the "
                "envelope and fallback target are clear, and power isolation "
                "is reachable"
            )
        if not nozzle_id.strip():
            raise ValueError("nozzle_id must not be empty")
        if pocket_index is not None and pocket_index < 0:
            raise ValueError("pocket_index must not be negative")
        rotation_offsets = [float(value) for value in rotation_capture_offsets_deg]
        if len(rotation_offsets) > 12:
            raise ValueError("rotation capture permits at most 12 angles")
        if any(not math.isfinite(value) or abs(value) < 1e-6 for value in rotation_offsets):
            raise ValueError("rotation capture angles must be finite and nonzero")
        if len({round(value, 6) for value in rotation_offsets}) != len(rotation_offsets):
            raise ValueError("rotation capture angles must not repeat")
        fallback = None
        if fallback_drop_surface_machine_mm is not None:
            fallback = list(fallback_drop_surface_machine_mm)
            if len(fallback) != 2 or not all(math.isfinite(v) for v in fallback):
                raise ValueError("fallback drop point must contain two finite values")
        return self._request(
            "POST",
            f"/strip-feeder-registrations/{registration_id}/"
            "pickup-qualifications",
            {
                "lease_id": str(lease_id),
                "nozzle_id": nozzle_id,
                "pocket_index": pocket_index,
                "fallback_drop_surface_machine_mm": fallback,
                "rotation_capture_offsets_deg": rotation_offsets,
                "confirmed_external_safety_checks": True,
            },
        )

    def qualify_strip_feeder_sequence(
        self,
        registration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        cycle_count: int = 3,
        nozzle_id: str = "N1",
        fallback_drop_surface_machine_mm: tuple[float, float] | None = None,
        safety_checks_confirmed: bool = False,
        poll_interval: float = 0.1,
    ) -> dict[str, Any]:
        """Run three to five pickup-return checks on different pockets."""
        if not 3 <= cycle_count <= 5:
            raise ValueError("cycle_count must be between 3 and 5")
        registration = self.strip_feeder_draft(registration_id)
        pockets = [
            pocket for pocket in registration.get("pockets", [])
            if pocket.get("status") == "available"
            and pocket.get("occupancy") == "occupied"
        ]
        if len(pockets) < cycle_count:
            raise ValueError(
                f"sequence needs {cycle_count} distinct available occupied pockets"
            )
        runs: list[dict[str, Any]] = []
        for pocket in pockets[:cycle_count]:
            started = self.start_strip_feeder_pickup_qualification(
                registration_id,
                lease_id,
                nozzle_id=nozzle_id,
                pocket_index=int(pocket["pocket_index"]),
                fallback_drop_surface_machine_mm=fallback_drop_surface_machine_mm,
                safety_checks_confirmed=safety_checks_confirmed,
            )
            try:
                self.wait(started["operation"], poll_interval=poll_interval)
            except OperationFailed:
                pass
            run = self.strip_feeder_pickup_qualification(
                started["qualification"]["id"]
            )
            runs.append(run)
            if (run.get("report") or {}).get("passed") is not True:
                break
        return {
            "registration_id": str(registration_id),
            "requested_cycle_count": cycle_count,
            "completed_cycle_count": len(runs),
            "passed": len(runs) == cycle_count
            and all((run.get("report") or {}).get("passed") is True for run in runs),
            "qualifications": runs,
        }

    def strip_feeder_pickup_qualification(
        self, qualification_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "GET",
            f"/strip-feeder-pickup-qualifications/{qualification_id}",
        )

    def reanalyze_strip_feeder_return(
        self, qualification_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        """Recheck retained pocket-return images without machine motion."""
        return self._request(
            "POST",
            f"/strip-feeder-pickup-qualifications/{qualification_id}/"
            "reanalyze-return",
        )

    def strip_feeder_pickup_preview(
        self, qualification_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/strip-feeder-pickup-qualifications/{qualification_id}/"
            "artifacts/held-part-inspection.png",
            "image/png",
        )

    def strip_feeder_return_preview(
        self, qualification_id: str | uuid.UUID,
    ) -> bytes:
        """Get the before/after top-camera pocket-return evidence."""
        return self._request_bytes(
            "GET",
            f"/strip-feeder-pickup-qualifications/{qualification_id}/"
            "artifacts/pocket-return-inspection.png",
            "image/png",
        )

    def start_board_registration(
        self,
        scan_id: str | uuid.UUID,
        candidate_index: int,
        board_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        label: str | None = None,
        sample_count: int = 3,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        if not safety_checks_confirmed:
            raise ValueError(
                "board registration requires explicit confirmation that the PCB "
                "is fixed, both nozzles are level, the envelope is clear, and "
                "power isolation is reachable"
            )
        if candidate_index < 0:
            raise ValueError("candidate_index must not be negative")
        if not 3 <= sample_count <= 5:
            raise ValueError("sample_count must be between 3 and 5")
        return self._request(
            "POST",
            f"/workspace-scans/{scan_id}/board-registrations",
            {
                "lease_id": str(lease_id),
                "candidate_index": candidate_index,
                "board_id": str(board_id),
                "label": label,
                "sample_count": sample_count,
                "confirmed_external_safety_checks": True,
            },
        )

    def register_board(
        self,
        scan_id: str | uuid.UUID,
        candidate_index: int,
        board_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        label: str | None = None,
        sample_count: int = 3,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        started = self.start_board_registration(
            scan_id,
            candidate_index,
            board_id,
            lease_id,
            label=label,
            sample_count=sample_count,
            safety_checks_confirmed=safety_checks_confirmed,
        )
        self.wait(started["operation"])
        return self.board_registration(started["registration"]["id"])

    def board_registrations(self) -> list[dict[str, Any]]:
        return self._request("GET", "/board-registrations")["registrations"]

    def board_registration(
        self, registration_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request("GET", f"/board-registrations/{registration_id}")

    def board_registration_artifact(
        self, registration_id: str | uuid.UUID, artifact_name: str,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/board-registrations/{registration_id}/artifacts/{artifact_name}",
            "image/png",
        )

    def start_board_surface_measurement(
        self,
        registration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        nozzle_id: str = "N1",
        sample_count: int = 3,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        if not safety_checks_confirmed:
            raise ValueError(
                "board surface measurement requires explicit confirmation that "
                "the PCB is fixed, its fiducials are visible, both nozzles are "
                "level, the envelope is clear, and power isolation is reachable"
            )
        if not 3 <= sample_count <= 5:
            raise ValueError("sample_count must be between 3 and 5")
        return self._request(
            "POST",
            f"/board-registrations/{registration_id}/surface-measurements",
            {
                "lease_id": str(lease_id),
                "nozzle_id": nozzle_id,
                "sample_count": sample_count,
                "confirmed_external_safety_checks": True,
            },
        )

    def measure_board_surface(
        self,
        registration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        nozzle_id: str = "N1",
        sample_count: int = 3,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        started = self.start_board_surface_measurement(
            registration_id,
            lease_id,
            nozzle_id=nozzle_id,
            sample_count=sample_count,
            safety_checks_confirmed=safety_checks_confirmed,
        )
        self.wait(started["operation"])
        return self.board_surface_measurement(started["measurement"]["id"])

    def board_surface_measurements(self) -> list[dict[str, Any]]:
        return self._request("GET", "/board-surface-measurements")["measurements"]

    def board_surface_measurement(
        self, measurement_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "GET", f"/board-surface-measurements/{measurement_id}"
        )

    def board_surface_measurement_artifact(
        self, measurement_id: str | uuid.UUID, artifact_name: str,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/board-surface-measurements/{measurement_id}/artifacts/{artifact_name}",
            "image/png" if artifact_name.endswith(".png") else "application/json",
        )

    def set_workspace_inventory_baseline(
        self, scan_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "POST",
            f"/workspace-scans/{scan_id}/inventory-baseline",
            {},
        )

    def workspace_inventory_baseline(self) -> dict[str, Any]:
        return self._request("GET", "/workspace-inventory/baseline")

    def compare_workspace_observation(
        self,
        scan_id: str | uuid.UUID,
        *,
        baseline_id: str | uuid.UUID | None = None,
    ) -> dict[str, Any]:
        request = {
            "baseline_id": str(baseline_id) if baseline_id is not None else None,
        }
        return self._request(
            "POST",
            f"/workspace-scans/{scan_id}/change-set",
            request,
        )

    def workspace_change_set_report(
        self, scan_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/workspace-change-set.json",
        )

    def workspace_board_discovery_report(
        self, scan_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/board-discovery.json",
        )

    def workspace_board_discovery_preview(
        self, scan_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/"
            "mosaic-board-candidates.png",
            "image/png",
        )

    def workspace_object_evidence(
        self, scan_id: str | uuid.UUID, candidate_index: int,
    ) -> bytes:
        if candidate_index < 0:
            raise ValueError("workspace candidate index cannot be negative")
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/"
            f"object-evidence-{candidate_index:03}.png",
            "image/png",
        )

    def strip_feeder_vision_preview(
        self, scan_id: str | uuid.UUID, candidate_index: int,
    ) -> bytes:
        if candidate_index < 0:
            raise ValueError("candidate_index must not be negative")
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/"
            f"strip-feeder-vision-{candidate_index:03}.png",
            "image/png",
        )

    def workspace_scan_refined_mosaic(
        self, scan_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/mosaic-refined.png",
            "image/png",
        )

    def workspace_scan_refinement_quality(
        self, scan_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/refinement-quality.json",
        )

    def workspace_scan_refinement_quality_preview(
        self, scan_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/mosaic-refined-quality.png",
            "image/png",
        )

    def workspace_scan_mosaic(
        self,
        scan_id: str | uuid.UUID,
        surface_model: str = "top_camera_working_plane",
    ) -> bytes:
        fallback_artifacts = {
            "top_camera_working_plane": "mosaic.png",
            "installed_plate_planes": "mosaic-installed-plate-planes.png",
        }
        if surface_model not in fallback_artifacts:
            raise ValueError(
                "surface_model must be top_camera_working_plane or "
                "installed_plate_planes"
            )
        report = self.workspace_scan(scan_id)
        artifact_name = next(
            (
                layer["artifact_name"]
                for layer in report.get("mosaic_layers", [])
                if layer.get("surface_model") == surface_model
            ),
            fallback_artifacts[surface_model],
        )
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/{artifact_name}",
            "image/png",
        )

    def workspace_scan_surface_heights(
        self, scan_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/"
            "surface-height-observations.json",
        )

    def workspace_scan_height_map(
        self, scan_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/surface-height-map.json",
        )

    def workspace_scan_height_map_preview(
        self, scan_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/surface-height-map.png",
            "image/png",
        )

    def workspace_scan_height_confidence(
        self, scan_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/"
            "surface-height-confidence.png",
            "image/png",
        )

    def workspace_scan_height_corrected_mosaic(
        self, scan_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/"
            "mosaic-height-corrected.png",
            "image/png",
        )

    def workspace_scan_height_overlay(
        self, scan_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/"
            "surface-height-overlay.png",
            "image/png",
        )

    def workspace_scan_height_class_preview(
        self, scan_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/"
            "mosaic-height-classes.png",
            "image/png",
        )

    def workspace_scan_seam_health(
        self, scan_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/seam-health.json",
        )

    def workspace_scan_disagreement(
        self, scan_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/mosaic-disagreement.png",
            "image/png",
        )

    def workspace_scan_overlap_confidence(
        self, scan_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/overlap-confidence.png",
            "image/png",
        )

    def workspace_scan_seam_overlay(
        self, scan_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/seam-health-overlay.png",
            "image/png",
        )

    def workspace_scan_seam_preview(
        self, scan_id: str | uuid.UUID,
    ) -> bytes:
        return self._request_bytes(
            "GET",
            f"/workspace-scans/{scan_id}/artifacts/mosaic-seam-health.png",
            "image/png",
        )

    @staticmethod
    def _workspace_scan_specification(
        calibration_id: str | uuid.UUID,
        *,
        region: dict[str, float] | None,
        region_preset: str | None,
        mode: str,
        artifact_retention: str,
        overlap_fraction: float | None,
        output_mm_per_pixel: float | None,
        feed_mm_per_min: float | None,
        artifact_quota_bytes: int | None,
        parent_scan_id: str | uuid.UUID | None,
        refinement_target_index: int | None,
    ) -> dict[str, Any]:
        refinement = mode == "refinement"
        if refinement:
            if parent_scan_id is None or refinement_target_index is None:
                raise ValueError(
                    "workspace refinement requires parent_scan_id and target index"
                )
            if region is not None or region_preset is not None:
                raise ValueError(
                    "workspace refinement selects its region from the parent scan"
                )
            if refinement_target_index < 0:
                raise ValueError("workspace refinement target index cannot be negative")
        elif region is None and region_preset is None:
            raise ValueError("workspace scan requires region or region_preset")
        if region_preset is not None and region_preset != "installed_plates":
            raise ValueError("workspace region_preset must be installed_plates")
        if mode not in {"precision", "survey", "reconnaissance", "refinement"}:
            raise ValueError(
                "workspace scan mode must be precision, survey, reconnaissance, "
                "or refinement"
            )
        if artifact_retention not in {
            "full_sources", "transient_sources", "thumbnails", "mosaic_only",
        }:
            raise ValueError(
                "workspace artifact retention must be full_sources, "
                "transient_sources, thumbnails, or mosaic_only"
            )
        if feed_mm_per_min is not None and feed_mm_per_min <= 0:
            raise ValueError("workspace feed_mm_per_min must be above zero")
        specification: dict[str, Any] = {
            "calibration_id": str(calibration_id),
            "mode": mode,
            "artifact_retention": artifact_retention,
        }
        if region is not None:
            specification["region"] = region
        if region_preset is not None:
            specification["region_preset"] = region_preset
        if parent_scan_id is not None:
            specification["parent_scan_id"] = str(parent_scan_id)
        if refinement_target_index is not None:
            specification["refinement_target_index"] = refinement_target_index
        for name, value in {
            "overlap_fraction": overlap_fraction,
            "output_mm_per_pixel": output_mm_per_pixel,
            "feed_mm_per_min": feed_mm_per_min,
            "artifact_quota_bytes": artifact_quota_bytes,
        }.items():
            if value is not None:
                specification[name] = value
        return specification

    @staticmethod
    def _validate_camera_role(role: str) -> None:
        if role not in {"top_camera", "bottom_camera"}:
            raise ValueError("camera role must be top_camera or bottom_camera")

    def connect(self) -> dict[str, Any]:
        return self.wait(self._request("POST", "/machine/connect"))

    def disconnect(self) -> dict[str, Any]:
        return self.wait(self._request("POST", "/machine/disconnect"))

    def complete_preflight(self) -> dict[str, Any]:
        return self.wait(self._request("POST", "/machine/preflight", {"confirmed": True}))

    def acquire_lease(self, client_name: str, duration_seconds: int = 900) -> dict[str, Any]:
        return self._request("POST", "/control/lease", {
            "client_name": client_name,
            "duration_seconds": duration_seconds,
        })["lease"]

    def renew_lease(
        self, lease_id: str | uuid.UUID, duration_seconds: int = 3600,
    ) -> dict[str, Any]:
        return self._request("POST", "/control/lease/renew", {
            "lease_id": str(lease_id),
            "duration_seconds": duration_seconds,
        })["lease"]

    def home(self, lease_id: str | uuid.UUID, *, safety_checks_confirmed: bool = False) -> dict[str, Any]:
        if not safety_checks_confirmed:
            raise ValueError(
                "homing requires explicit confirmation that both nozzles are level, "
                "the envelope is clear, and power isolation is reachable"
            )
        return self.wait(self._request("POST", "/machine/home", {
            "lease_id": str(lease_id),
            "confirmed": True,
        }))

    def jog(
        self,
        lease_id: str | uuid.UUID,
        *,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        rotation_deg: float | None = None,
        feed_mm_per_min: float = 300.0,
    ) -> dict[str, Any]:
        target: dict[str, Any] = {
            "x_mm": x_mm,
            "y_mm": y_mm,
            "z_mm": z_mm,
        }
        if rotation_deg is not None:
            target["rotation_deg"] = rotation_deg
        return self.wait(self._request("POST", "/machine/jog", {
            "lease_id": str(lease_id),
            "target": target,
            "feed_mm_per_min": feed_mm_per_min,
        }))

    def actuate(self, lease_id: str | uuid.UUID, actuator_id: str, enabled: bool) -> dict[str, Any]:
        return self.wait(self._request("POST", f"/machine/actuators/{actuator_id}", {
            "lease_id": str(lease_id), "enabled": enabled,
        }))

    def read_sensor(
        self, lease_id: str | uuid.UUID, sensor_id: str,
    ) -> dict[str, Any]:
        return self._request(
            "POST",
            f"/machine/sensors/{sensor_id}/read",
            {"lease_id": str(lease_id)},
        )

    def visual_home(
        self,
        lease_id: str | uuid.UUID,
        *,
        apply_offset: bool = False,
        move_to_seed: bool = True,
    ) -> dict[str, Any]:
        if apply_offset and not move_to_seed:
            raise ValueError("visual homing cannot apply an offset without moving to the seed")
        return self._request(
            "POST",
            "/machine/visual-home",
            {
                "lease_id": str(lease_id),
                "apply_offset": apply_offset,
                "move_to_seed": move_to_seed,
            },
        )

    def emergency_stop(self) -> dict[str, Any]:
        return self.wait(self._request("POST", "/machine/emergency-stop"))

    def boards(self) -> list[dict[str, Any]]:
        return self._request("GET", "/boards")["boards"]

    def import_board(
        self,
        pcb: str | Path,
        *,
        bom: str | Path | None = None,
        name: str | None = None,
    ) -> dict[str, Any]:
        pcb_path = Path(pcb)
        boundary = f"velocitypnp-{uuid.uuid4().hex}"
        parts: list[bytes] = []

        def add_field(field_name: str, value: bytes, filename: str | None = None) -> None:
            disposition = f'form-data; name="{field_name}"'
            if filename is not None:
                disposition += f'; filename="{filename}"'
            parts.extend([
                f"--{boundary}\r\nContent-Disposition: {disposition}\r\n\r\n".encode(),
                value,
                b"\r\n",
            ])

        add_field("pcb", pcb_path.read_bytes(), pcb_path.name)
        if bom is not None:
            bom_path = Path(bom)
            add_field("bom", bom_path.read_bytes(), bom_path.name)
        add_field("name", (name or pcb_path.stem).encode())
        parts.append(f"--{boundary}--\r\n".encode())
        return self._request_data(
            "POST",
            "/boards/import",
            b"".join(parts),
            f"multipart/form-data; boundary={boundary}",
        )

    def import_openpnp(
        self, machine_xml: str | Path, *, parts_xml: str | Path | None = None,
    ) -> dict[str, Any]:
        machine_path = Path(machine_xml)
        files = [("machine", machine_path.name, machine_path.read_bytes())]
        if parts_xml is not None:
            parts_path = Path(parts_xml)
            files.append(("parts", parts_path.name, parts_path.read_bytes()))
        boundary = f"velocitypnp-{uuid.uuid4().hex}"
        parts: list[bytes] = []
        for field_name, filename, value in files:
            parts.extend([
                f"--{boundary}\r\nContent-Disposition: form-data; name=\"{field_name}\"; filename=\"{filename}\"\r\n\r\n".encode(),
                value,
                b"\r\n",
            ])
        parts.append(f"--{boundary}--\r\n".encode())
        return self._request_data(
            "POST",
            "/import/openpnp",
            b"".join(parts),
            f"multipart/form-data; boundary={boundary}",
        )["import"]

    def create_calibration(self, machine_serial: str) -> dict[str, Any]:
        return self._request("POST", "/calibrations", {"machine_serial": machine_serial})

    def calibration(self, calibration_id: str | uuid.UUID) -> dict[str, Any]:
        return self._request("GET", f"/calibrations/{calibration_id}")

    def calibration_artifacts(self) -> list[dict[str, Any]]:
        return self._request("GET", "/calibration/artifacts")["artifacts"]

    def calibration_plan(
        self,
        calibration_id: str | uuid.UUID,
        targets: list[str] | None = None,
    ) -> dict[str, Any]:
        return self._request(
            "POST",
            f"/calibrations/{calibration_id}/artifact-plan",
            {"targets": targets or []},
        )

    def recompute_calibration_artifacts(
        self,
        calibration_id: str | uuid.UUID,
        targets: list[str] | None = None,
    ) -> dict[str, Any]:
        return self._request(
            "POST",
            f"/calibrations/{calibration_id}/derived-artifacts/recompute",
            {"targets": targets or []},
        )

    def capture_calibration_artifacts(
        self,
        calibration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        targets: list[str],
        *,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        supported = {
            "nozzle_axis_runout_n1", "nozzle_axis_runout_n2",
            "nozzle_height_n1", "nozzle_height_n2",
        }
        if not targets or not set(targets) <= supported:
            raise ValueError("targets must name supported nozzle runout or height artifacts")
        if not safety_checks_confirmed:
            raise ValueError(
                "artifact capture requires explicit confirmation that both nozzles are "
                "level, the envelope is clear, and power isolation is reachable"
            )
        started = self._request(
            "POST",
            f"/calibrations/{calibration_id}/artifact-runs",
            {
                "lease_id": str(lease_id),
                "confirmed_external_safety_checks": True,
                "targets": targets,
            },
        )
        self.wait(started["operation"])
        return self.calibration_run(started["run"]["id"])

    def calibration_runs(
        self, calibration_id: str | uuid.UUID,
    ) -> list[dict[str, Any]]:
        return self._request(
            "GET", f"/calibrations/{calibration_id}/runs",
        )["runs"]

    def calibration_run(self, run_id: str | uuid.UUID) -> dict[str, Any]:
        return self._request("GET", f"/calibration-runs/{run_id}")

    def start_calibration_run(
        self,
        calibration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        safety_checks_confirmed: bool = False,
        visual_homing_sample_count: int = 5,
    ) -> dict[str, Any]:
        if not safety_checks_confirmed:
            raise ValueError(
                "calibration requires explicit confirmation that both nozzles are level, "
                "the envelope is clear, and power isolation is reachable"
            )
        if not 3 <= visual_homing_sample_count <= 100:
            raise ValueError("visual_homing_sample_count must be between 3 and 100")
        return self._request(
            "POST",
            f"/calibrations/{calibration_id}/runs",
            {
                "lease_id": str(lease_id),
                "confirmed_external_safety_checks": True,
                "visual_homing_sample_count": visual_homing_sample_count,
            },
        )

    def calibrate(
        self,
        calibration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        safety_checks_confirmed: bool = False,
        visual_homing_sample_count: int = 5,
    ) -> dict[str, Any]:
        started = self.start_calibration_run(
            calibration_id,
            lease_id,
            safety_checks_confirmed=safety_checks_confirmed,
            visual_homing_sample_count=visual_homing_sample_count,
        )
        self.wait(started["operation"])
        return self.calibration_run(started["run"]["id"])

    def vacuum_probe(
        self,
        calibration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        nozzle_id: str,
        *,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        if nozzle_id not in {"N1", "N2"}:
            raise ValueError("nozzle_id must be N1 or N2")
        if not safety_checks_confirmed:
            raise ValueError(
                "vacuum probing requires explicit confirmation that the datum board "
                "and sealing target are clean and fixed, both nozzles are level, "
                "the envelope is clear, and power isolation is reachable"
            )
        operation = self._request(
            "POST",
            f"/calibrations/{calibration_id}/vacuum-probe",
            {
                "lease_id": str(lease_id),
                "nozzle_id": nozzle_id,
                "confirmed_external_safety_checks": True,
            },
        )
        self.wait(operation)
        return self.calibration(calibration_id)["values"]["vacuum_probe"][nozzle_id]

    def characterize_vacuum(
        self,
        calibration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        nozzle_id: str,
        *,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        if nozzle_id not in {"N1", "N2"}:
            raise ValueError("nozzle_id must be N1 or N2")
        if not safety_checks_confirmed:
            raise ValueError(
                "vacuum characterization requires explicit confirmation that the "
                "datum board and sealing target are clean and fixed, both nozzles "
                "are level, the envelope is clear, and power isolation is reachable"
            )
        operation = self._request(
            "POST",
            f"/calibrations/{calibration_id}/vacuum-characterization",
            {
                "lease_id": str(lease_id),
                "nozzle_id": nozzle_id,
                "confirmed_external_safety_checks": True,
            },
        )
        self.wait(operation)
        return self.calibration(calibration_id)["values"][
            "vacuum_sensor_characterization"
        ][nozzle_id]

    def fine_home(
        self,
        calibration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        nozzle_id: str = "N2",
        *,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        if nozzle_id not in {"N1", "N2"}:
            raise ValueError("nozzle_id must be N1 or N2")
        if not safety_checks_confirmed:
            raise ValueError(
                "fine homing requires explicit confirmation that the datum board "
                "and sealing target are clean and fixed, both nozzles are level, "
                "the envelope is clear, and power isolation is reachable"
            )
        operation = self._request(
            "POST",
            f"/calibrations/{calibration_id}/fine-home",
            {
                "lease_id": str(lease_id),
                "nozzle_id": nozzle_id,
                "confirmed_external_safety_checks": True,
            },
        )
        return self.wait(operation)

    def calibrate_xy_direction_response(
        self,
        calibration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        if not safety_checks_confirmed:
            raise ValueError(
                "local XY direction-response calibration requires explicit "
                "confirmation that both nozzles are level, the datum board is "
                "fixed, the envelope is clear, and power isolation is reachable"
            )
        operation = self._request(
            "POST",
            f"/calibrations/{calibration_id}/xy-direction-response",
            {
                "lease_id": str(lease_id),
                "confirmed_external_safety_checks": True,
            },
        )
        self.wait(operation)
        return self.calibration(calibration_id)["values"][
            "local_xy_direction_response"
        ]

    def calibrate_installed_geometry(
        self,
        calibration_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        *,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        if not safety_checks_confirmed:
            raise ValueError(
                "installed geometry calibration requires explicit confirmation "
                "that both nozzles are level, installed hardware is secure, "
                "tube routing and the envelope are clear, and power isolation "
                "is reachable"
            )
        started = self._request(
            "POST",
            f"/calibrations/{calibration_id}/installed-geometry-runs",
            {
                "lease_id": str(lease_id),
                "confirmed_external_safety_checks": True,
            },
        )
        self.wait(started["operation"])
        return self.calibration_run(started["run"]["id"])

    def observe_nozzle_height(
        self,
        calibration_id: str | uuid.UUID,
        nozzle_id: str,
        observed_error_x_mm: float,
        observed_error_y_mm: float,
        measurement_uncertainty_mm: float,
        *,
        nozzle_z_mm: float | None = None,
        post_correction_residual: bool = False,
    ) -> dict[str, Any]:
        """Record nozzle-center minus target-center XY error at a known Z.

        Set post_correction_residual after inspecting a pose that already used
        the stored correction. That residual updates only the selected nozzle.
        """
        if nozzle_id not in {"N1", "N2"}:
            raise ValueError("nozzle_id must be N1 or N2")
        return self._request(
            "POST",
            f"/calibrations/{calibration_id}/nozzle-height-observation",
            {
                "nozzle_id": nozzle_id,
                "nozzle_z_mm": nozzle_z_mm,
                "observed_error_x_mm": observed_error_x_mm,
                "observed_error_y_mm": observed_error_y_mm,
                "measurement_uncertainty_mm": measurement_uncertainty_mm,
                "post_correction_residual": post_correction_residual,
            },
        )

    def feeders(self) -> list[dict[str, Any]]:
        return self._request("GET", "/feeders")["feeders"]

    def put_feeder(self, feeder: dict[str, Any]) -> dict[str, Any]:
        return self._request("POST", "/feeders", feeder)

    def parts(self) -> list[dict[str, Any]]:
        return self._request("GET", "/parts")["parts"]

    def put_part(self, part: dict[str, Any]) -> dict[str, Any]:
        return self._request("POST", "/parts", part)

    def part(self, part_id: str | uuid.UUID) -> dict[str, Any]:
        return self._request("GET", f"/parts/{part_id}")

    def jobs(self) -> list[dict[str, Any]]:
        return self._request("GET", "/jobs")["jobs"]

    def put_job(self, job: dict[str, Any]) -> dict[str, Any]:
        return self._request("POST", "/jobs", job)

    def plan_job(self, job_id: str | uuid.UUID) -> dict[str, Any]:
        return self._request("GET", f"/jobs/{job_id}/plan")["plan"]

    def simulate_job(
        self,
        job_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        calibration_id: str | uuid.UUID | None = None,
    ) -> dict[str, Any]:
        request = {
            "lease_id": str(lease_id),
        }
        if calibration_id is not None:
            request["calibration_id"] = str(calibration_id)
        return self.wait(
            self._request("POST", f"/jobs/{job_id}/simulate", request),
        )

    def start_guarded_single_placement(
        self,
        job_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        calibration_id: str | uuid.UUID,
        *,
        step_index: int = 0,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        """Start one simulation-gated placement-plan step."""
        if step_index < 0:
            raise ValueError("step_index must not be negative")
        if not safety_checks_confirmed:
            raise ValueError(
                "single placement requires explicit confirmation that feeders "
                "and the PCB are fixed, both nozzles are level, the envelope "
                "is clear, and power isolation is reachable"
            )
        return self._request(
            "POST",
            f"/jobs/{job_id}/placement-executions",
            {
                "lease_id": str(lease_id),
                "calibration_id": str(calibration_id),
                "step_index": step_index,
                "confirmed_external_safety_checks": True,
            },
        )

    def placement_execution(
        self, execution_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request("GET", f"/placement-executions/{execution_id}")

    def reanalyze_placement(
        self, execution_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        """Recheck retained board images without machine motion."""
        return self._request(
            "POST",
            f"/placement-executions/{execution_id}/reanalyze-placement",
        )

    def reanalyze_placement_source_pose(
        self, execution_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        """Recheck a retained source-pocket image without machine motion."""
        return self._request(
            "POST",
            f"/placement-executions/{execution_id}/reanalyze-source-pose",
        )

    def placement_outcomes(self) -> list[dict[str, Any]]:
        return self._request("GET", "/placement-outcomes")["outcomes"]

    def placement_outcome(
        self, execution_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self._request(
            "GET", f"/placement-executions/{execution_id}/outcome",
        )

    def review_placement_outcome(
        self,
        execution_id: str | uuid.UUID,
        disposition: str,
        *,
        note: str | None = None,
    ) -> dict[str, Any]:
        valid = {"good", "offset", "rotated", "tombstoned", "missing", "uncertain"}
        if disposition not in valid:
            raise ValueError(f"placement disposition must be one of {sorted(valid)}")
        return self._request(
            "POST",
            f"/placement-executions/{execution_id}/outcome",
            {"disposition": disposition, "note": note},
        )

    def placement_process(
        self,
        part_id: str | uuid.UUID,
        nozzle_id: str = "N1",
        surface_offset_mm: float = 0.0,
    ) -> dict[str, Any]:
        if not nozzle_id:
            raise ValueError("nozzle_id must not be empty")
        if not math.isfinite(surface_offset_mm) or not 0.0 <= surface_offset_mm <= 10.0:
            raise ValueError("surface_offset_mm must be between 0.0 and 10.0")
        return self._request(
            "GET",
            f"/parts/{part_id}/placement-process/{nozzle_id}/surfaces/{surface_offset_mm}",
        )

    def qualify_placement_sequence(
        self,
        job_id: str | uuid.UUID,
        lease_id: str | uuid.UUID,
        calibration_id: str | uuid.UUID,
        step_indices: list[int],
        *,
        safety_checks_confirmed: bool = False,
    ) -> dict[str, Any]:
        """Run one to five guarded placements and stop after the first failure."""
        if not safety_checks_confirmed:
            raise ValueError(
                "placement sequence requires explicit confirmation that feeders "
                "and the PCB are fixed, both nozzles are level, the envelope "
                "is clear, and power isolation is reachable"
            )
        if not 1 <= len(step_indices) <= 5:
            raise ValueError("placement sequence requires one to five steps")
        if any(step < 0 for step in step_indices) or len(set(step_indices)) != len(step_indices):
            raise ValueError("placement sequence steps must be distinct and nonnegative")
        plan = self.plan_job(job_id)
        try:
            selected = [plan["steps"][step] for step in step_indices]
        except IndexError as error:
            raise ValueError("placement sequence step is outside the job plan") from error
        keys = {
            (
                step["part_key"],
                step["nozzle_id"],
                tuple(
                    step.get(
                        "placement_release_seating_offsets_mm",
                        [step["placement_seating_offset_mm"]],
                    )
                ),
                step["placement_surface_offset_mm"],
            )
            for step in selected
        }
        if len(keys) != 1:
            raise ValueError(
                "placement sequence steps must use one part, nozzle, release schedule, "
                "and surface offset"
            )
        executions = []
        outcomes = []
        for step_index in step_indices:
            started = self.start_guarded_single_placement(
                job_id,
                lease_id,
                calibration_id,
                step_index=step_index,
                safety_checks_confirmed=True,
            )
            operation_failed = False
            try:
                self.wait(started["operation"])
            except OperationFailed:
                operation_failed = True
            execution_id = started["execution"]["id"]
            execution = self.placement_execution(execution_id)
            outcome = self.placement_outcome(execution_id)
            executions.append(execution)
            outcomes.append(outcome)
            if operation_failed or outcome["automatic_disposition"] != "good":
                break
        return {
            "job_id": str(job_id),
            "requested_steps": step_indices,
            "completed_count": len(executions),
            "passed": len(executions) == len(step_indices)
            and all(
                outcome["automatic_disposition"] == "good"
                for outcome in outcomes
            ),
            "executions": executions,
            "outcomes": outcomes,
        }

    def inspections(self) -> list[dict[str, Any]]:
        return self._request("GET", "/inspections")["inspections"]

    def put_inspection(self, report: dict[str, Any]) -> dict[str, Any]:
        return self._request("POST", "/inspections", report)

    def disposition_finding(
        self,
        report_id: str | uuid.UUID,
        finding_id: str | uuid.UUID,
        disposition: str,
    ) -> dict[str, Any]:
        return self._request(
            "POST",
            f"/inspections/{report_id}/findings/{finding_id}/disposition",
            {"disposition": disposition},
        )

    def wait(self, operation: dict[str, Any], poll_interval: float = 0.1) -> dict[str, Any]:
        deadline = time.monotonic() + self.timeout
        while operation["status"] in {"pending", "running"}:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"operation {operation['id']} did not finish within {self.timeout}s")
            time.sleep(poll_interval)
            operation = self._request("GET", f"/operations/{operation['id']}")
        if operation["status"] == "failed":
            raise OperationFailed(operation.get("message") or f"operation {operation['id']} failed")
        return operation

    def _request(self, method: str, path: str, body: dict[str, Any] | None = None) -> Any:
        data = None if body is None else json.dumps(body).encode("utf-8")
        return self._request_data(method, path, data, "application/json")

    def _request_data(
        self, method: str, path: str, data: bytes | None, content_type: str,
    ) -> Any:
        request = Request(
            f"{self.base_url}{path}", data=data, method=method,
            headers={"Accept": "application/json", **({"Content-Type": content_type} if data else {})},
        )
        try:
            with self._opener.open(request, timeout=self.timeout) as response:
                return json.load(response)
        except HTTPError as error:
            try:
                payload = json.load(error)
                message = payload.get("error", {}).get("message", str(error))
            except (ValueError, TypeError):
                message = str(error)
            raise ApiError(error.code, message) from error

    def _request_bytes(self, method: str, path: str, accept: str) -> bytes:
        request = Request(
            f"{self.base_url}{path}", method=method, headers={"Accept": accept},
        )
        try:
            with self._opener.open(request, timeout=self.timeout) as response:
                return response.read()
        except HTTPError as error:
            try:
                payload = json.load(error)
                message = payload.get("error", {}).get("message", str(error))
            except (ValueError, TypeError):
                message = str(error)
            raise ApiError(error.code, message) from error
