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

    def test_home_requires_explicit_physical_confirmation(self):
        client = Client(timeout=1)
        with patch.object(client, "_request") as send:
            with self.assertRaisesRegex(ValueError, "explicit confirmation"):
                client.home("00000000-0000-0000-0000-000000000000")
        send.assert_not_called()


if __name__ == "__main__":
    unittest.main()
