from __future__ import annotations

import json
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
        visual_homing_sample_count: int = 30,
    ) -> dict[str, Any]:
        if not safety_checks_confirmed:
            raise ValueError(
                "calibration requires explicit confirmation that both nozzles are level, "
                "the envelope is clear, and power isolation is reachable"
            )
        if not 30 <= visual_homing_sample_count <= 100:
            raise ValueError("visual_homing_sample_count must be between 30 and 100")
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
        visual_homing_sample_count: int = 30,
    ) -> dict[str, Any]:
        started = self.start_calibration_run(
            calibration_id,
            lease_id,
            safety_checks_confirmed=safety_checks_confirmed,
            visual_homing_sample_count=visual_homing_sample_count,
        )
        self.wait(started["operation"])
        return self.calibration_run(started["run"]["id"])

    def feeders(self) -> list[dict[str, Any]]:
        return self._request("GET", "/feeders")["feeders"]

    def put_feeder(self, feeder: dict[str, Any]) -> dict[str, Any]:
        return self._request("POST", "/feeders", feeder)

    def jobs(self) -> list[dict[str, Any]]:
        return self._request("GET", "/jobs")["jobs"]

    def put_job(self, job: dict[str, Any]) -> dict[str, Any]:
        return self._request("POST", "/jobs", job)

    def plan_job(self, job_id: str | uuid.UUID) -> dict[str, Any]:
        return self._request("GET", f"/jobs/{job_id}/plan")["plan"]

    def simulate_job(
        self, job_id: str | uuid.UUID, lease_id: str | uuid.UUID,
    ) -> dict[str, Any]:
        return self.wait(self._request("POST", f"/jobs/{job_id}/simulate", {
            "lease_id": str(lease_id),
        }))

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
