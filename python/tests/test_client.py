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


if __name__ == "__main__":
    unittest.main()
