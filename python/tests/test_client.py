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


if __name__ == "__main__":
    unittest.main()
