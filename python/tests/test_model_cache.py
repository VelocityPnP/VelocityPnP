from __future__ import annotations

import io
import json
from pathlib import Path
import shutil
import sys
import tarfile
from tempfile import TemporaryDirectory
import unittest


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from velocitypnp.model_cache import (  # noqa: E402
    activate_lock,
    current_selection,
    download_github_asset,
    load_lock,
    rollback_selection,
    sha256_bytes,
    sha256_file,
    sync_lock,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


class ModelCacheTests(unittest.TestCase):
    def make_package(
        self,
        root: Path,
        version: str,
        marker: bytes,
    ) -> tuple[Path, dict]:
        family = "test-model"
        model_id = f"{family}/{version}"
        package_root = f"{family}-{version}"
        checkpoint = b"checkpoint-" + marker
        onnx = b"onnx-" + marker
        dataset_checksum = sha256_bytes(b"dataset-" + marker)
        manifest = {
            "contract_version": "velocitypnp.test.v1",
            "dataset": {"content_sha256": dataset_checksum},
            "artifacts": {
                "checkpoint": {
                    "path": "best.pt",
                    "sha256": sha256_bytes(checkpoint),
                    "size_bytes": len(checkpoint),
                },
                "onnx": {
                    "path": "best.onnx",
                    "sha256": sha256_bytes(onnx),
                    "size_bytes": len(onnx),
                },
            },
        }
        manifest_bytes = (json.dumps(manifest, sort_keys=True) + "\n").encode()
        payloads = {
            "LICENSE": b"Apache-2.0 test license\n",
            "best.onnx": onnx,
            "best.pt": checkpoint,
            "model-card.md": b"# Test model\n",
            "model-manifest.json": manifest_bytes,
        }
        metadata = {
            "schema_version": 1,
            "id": model_id,
            "family": family,
            "version": version,
            "release_tag": "models-test",
            "contract_version": "velocitypnp.test.v1",
            "license": {"artifact": "Apache-2.0"},
            "manifest_sha256": sha256_bytes(manifest_bytes),
            "files": [
                {
                    "path": name,
                    "sha256": sha256_bytes(data),
                    "size_bytes": len(data),
                }
                for name, data in sorted(payloads.items())
            ],
        }
        payloads["package-metadata.json"] = (
            json.dumps(metadata, sort_keys=True) + "\n"
        ).encode()
        archive = root / f"{package_root}.tar.gz"
        with tarfile.open(archive, "w:gz") as output:
            for name, data in sorted(payloads.items()):
                info = tarfile.TarInfo(f"{package_root}/{name}")
                info.size = len(data)
                output.addfile(info, io.BytesIO(data))
        archive_checksum = sha256_file(archive)
        model = {
            "role": "test",
            "id": model_id,
            "status": "approved",
            "contract_version": "velocitypnp.test.v1",
            "package_root": package_root,
            "release_tag": "models-test",
            "dataset_content_sha256": dataset_checksum,
            "manifest_sha256": sha256_bytes(manifest_bytes),
            "onnx_sha256": sha256_bytes(onnx),
            "checkpoint_sha256": sha256_bytes(checkpoint),
            "asset": {
                "name": archive.name,
                "size_bytes": archive.stat().st_size,
                "sha256": archive_checksum,
                "api_url": (
                    "https://api.github.com/repos/test/models/releases/assets/1"
                ),
                "download_url": (
                    f"https://github.com/test/models/releases/download/"
                    f"models-test/{archive.name}"
                ),
            },
        }
        return archive, model

    def write_lock(
        self,
        path: Path,
        model: dict,
        policy: str,
    ) -> None:
        document = {
            "schema_version": 2,
            "selection_policy": policy,
            "catalog": {
                "repository": "test/models",
                "commit": "1" * 40,
            },
            "models": [model],
        }
        path.write_text(json.dumps(document, indent=2) + "\n")

    def fetcher(self, archives: dict[str, Path], calls: list[str]):
        def fetch(model: dict, destination: Path) -> None:
            calls.append(model["id"])
            shutil.copyfile(archives[model["id"]], destination)

        return fetch

    def test_repository_lock_selects_the_approved_runtime_set(self) -> None:
        document, _, digest = load_lock(REPOSITORY_ROOT / "models.lock.json")

        self.assertEqual(document["selection_policy"], "runtime")
        self.assertEqual(len(document["models"]), 3)
        self.assertEqual(len(digest), 64)
        self.assertTrue(
            all(item["status"] == "approved" for item in document["models"])
        )

    def test_public_asset_download_does_not_require_github_cli(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            archive, model = self.make_package(root, "1", b"one")
            model["asset"]["download_url"] = archive.as_uri()
            destination = root / "download.tar.gz"

            download_github_asset(model, destination)

            self.assertEqual(destination.read_bytes(), archive.read_bytes())

    def test_sync_is_content_addressed_and_does_not_activate(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            archive, model = self.make_package(root, "1", b"one")
            model["status"] = "candidate"
            lock = root / "models.lock.json"
            cache = root / "cache"
            self.write_lock(lock, model, "cache_only")
            calls: list[str] = []
            fetch = self.fetcher({model["id"]: archive}, calls)

            first = sync_lock(lock, cache, fetch)
            second = sync_lock(lock, cache, fetch)

            self.assertFalse(first["activated"])
            self.assertFalse((cache / "state.json").exists())
            self.assertEqual(calls, [model["id"]])
            self.assertTrue(second["models"]["test"]["archive_cache_hit"])
            self.assertTrue(second["models"]["test"]["package_cache_hit"])
            with self.assertRaisesRegex(ValueError, "does not permit"):
                activate_lock(lock, cache, fetch)

    def test_corrupt_download_does_not_enter_cache(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            _, model = self.make_package(root, "1", b"one")
            lock = root / "models.lock.json"
            cache = root / "cache"
            self.write_lock(lock, model, "runtime")

            def corrupt_fetch(_: dict, destination: Path) -> None:
                destination.write_bytes(b"corrupt")

            with self.assertRaisesRegex(ValueError, "archive size"):
                sync_lock(lock, cache, corrupt_fetch)
            self.assertFalse(
                (cache / "archives" / f"{model['asset']['sha256']}.tar.gz").exists()
            )

    def test_unsafe_archive_member_is_rejected(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            archive, model = self.make_package(root, "1", b"one")
            with tarfile.open(archive, "w:gz") as output:
                data = b"escape"
                info = tarfile.TarInfo(f"{model['package_root']}/../escape")
                info.size = len(data)
                output.addfile(info, io.BytesIO(data))
            model["asset"]["size_bytes"] = archive.stat().st_size
            model["asset"]["sha256"] = sha256_file(archive)
            lock = root / "models.lock.json"
            cache = root / "cache"
            self.write_lock(lock, model, "runtime")
            fetch = self.fetcher({model["id"]: archive}, [])

            with self.assertRaisesRegex(ValueError, "unsafe package member"):
                sync_lock(lock, cache, fetch)
            self.assertFalse((root / "escape").exists())

    def test_corrupt_cached_package_is_quarantined_and_rebuilt(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            archive, model = self.make_package(root, "1", b"one")
            lock = root / "models.lock.json"
            cache = root / "cache"
            self.write_lock(lock, model, "runtime")
            calls: list[str] = []
            fetch = self.fetcher({model["id"]: archive}, calls)
            first = sync_lock(lock, cache, fetch)
            package = Path(first["models"]["test"]["package_dir"])
            (package / "best.onnx").write_bytes(b"changed")

            second = sync_lock(lock, cache, fetch)

            self.assertEqual(calls, [model["id"]])
            self.assertTrue(second["models"]["test"]["archive_cache_hit"])
            self.assertFalse(second["models"]["test"]["package_cache_hit"])
            self.assertTrue(any((cache / "packages" / "quarantine").iterdir()))

    def test_runtime_selection_rolls_back_to_prior_lock(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            first_archive, first_model = self.make_package(root, "1", b"one")
            second_archive, second_model = self.make_package(root, "2", b"two")
            first_lock = root / "first.lock.json"
            second_lock = root / "second.lock.json"
            cache = root / "cache"
            self.write_lock(first_lock, first_model, "runtime")
            self.write_lock(second_lock, second_model, "runtime")
            calls: list[str] = []
            fetch = self.fetcher(
                {
                    first_model["id"]: first_archive,
                    second_model["id"]: second_archive,
                },
                calls,
            )

            first = activate_lock(first_lock, cache, fetch)
            second = activate_lock(second_lock, cache, fetch)
            current = current_selection(cache)
            rolled_back = rollback_selection(cache)

            self.assertTrue(first["activated"])
            self.assertEqual(second["history_depth"], 1)
            self.assertEqual(current["models"]["test"]["id"], second_model["id"])
            self.assertEqual(rolled_back["models"]["test"]["id"], first_model["id"])
            self.assertEqual(rolled_back["history_depth"], 0)
            self.assertEqual(calls, [first_model["id"], second_model["id"]])


if __name__ == "__main__":
    unittest.main()
