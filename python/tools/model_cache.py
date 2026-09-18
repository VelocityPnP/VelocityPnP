#!/usr/bin/env python3
"""Download, verify, cache, select, and roll back VelocityPnP models."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import tarfile
import tempfile
from typing import Any, Callable
import uuid


SCHEMA_VERSION = 1
PACKAGE_SCHEMA_VERSION = 1
SHA256 = re.compile(r"^[0-9a-f]{64}$")
GIT_COMMIT = re.compile(r"^[0-9a-f]{40}$")
MODEL_STATES = {"candidate", "validated", "approved", "retired"}
SELECTION_POLICIES = {"cache_only", "runtime"}
REQUIRED_PACKAGE_FILES = {
    "LICENSE",
    "best.onnx",
    "best.pt",
    "model-card.md",
    "model-manifest.json",
}
MAX_UNPACKED_BYTES = 512 * 1024 * 1024
Fetcher = Callable[[dict[str, Any], Path], None]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def default_cache_root() -> Path:
    xdg_cache = os.environ.get("XDG_CACHE_HOME")
    base = Path(xdg_cache) if xdg_cache else Path.home() / ".cache"
    return base / "velocitypnp" / "models"


def exact_keys(value: dict[str, Any], expected: set[str], context: str) -> None:
    actual = set(value)
    require(actual == expected, f"{context} fields differ: {sorted(actual ^ expected)}")


def validate_sha256(value: object, context: str) -> str:
    require(isinstance(value, str) and SHA256.fullmatch(value), f"{context} is invalid")
    return value


def validate_lock(document: object) -> dict[str, Any]:
    require(isinstance(document, dict), "lockfile must be an object")
    exact_keys(
        document,
        {"schema_version", "selection_policy", "catalog", "models"},
        "lockfile",
    )
    require(document["schema_version"] == SCHEMA_VERSION, "unsupported lockfile schema")
    policy = document["selection_policy"]
    require(policy in SELECTION_POLICIES, "selection policy is invalid")

    catalog = document["catalog"]
    require(isinstance(catalog, dict), "catalog must be an object")
    exact_keys(catalog, {"repository", "commit", "release_tag"}, "catalog")
    repository = catalog["repository"]
    require(
        isinstance(repository, str)
        and re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", repository),
        "catalog repository is invalid",
    )
    require(
        isinstance(catalog["commit"], str)
        and GIT_COMMIT.fullmatch(catalog["commit"]),
        "catalog commit is invalid",
    )
    release_tag = catalog["release_tag"]
    require(isinstance(release_tag, str) and release_tag, "release tag is invalid")

    models = document["models"]
    require(isinstance(models, list) and models, "models must be a non-empty list")
    roles: set[str] = set()
    ids: set[str] = set()
    for model in models:
        require(isinstance(model, dict), "model entry must be an object")
        exact_keys(
            model,
            {
                "role",
                "id",
                "status",
                "contract_version",
                "package_root",
                "dataset_content_sha256",
                "manifest_sha256",
                "onnx_sha256",
                "checkpoint_sha256",
                "asset",
            },
            "model entry",
        )
        role = model["role"]
        model_id = model["id"]
        require(isinstance(role, str) and role, "model role is invalid")
        require(role not in roles, f"duplicate model role: {role}")
        require(isinstance(model_id, str), "model ID is invalid")
        parts = model_id.split("/")
        require(len(parts) == 2 and all(parts), "model ID must be family/version")
        require(model_id not in ids, f"duplicate model ID: {model_id}")
        require(model["status"] in MODEL_STATES, f"{model_id} has invalid status")
        if policy == "runtime":
            require(model["status"] == "approved", f"{model_id} is not approved")
        require(
            isinstance(model["contract_version"], str)
            and model["contract_version"],
            f"{model_id} has no contract version",
        )
        expected_root = f"{parts[0]}-{parts[1]}"
        require(model["package_root"] == expected_root, f"{model_id} root is invalid")
        for field in (
            "dataset_content_sha256",
            "manifest_sha256",
            "onnx_sha256",
            "checkpoint_sha256",
        ):
            validate_sha256(model[field], f"{model_id} {field}")

        asset = model["asset"]
        require(isinstance(asset, dict), f"{model_id} asset must be an object")
        exact_keys(
            asset,
            {"name", "size_bytes", "sha256", "api_url", "download_url"},
            f"{model_id} asset",
        )
        expected_name = f"{expected_root}.tar.gz"
        require(asset["name"] == expected_name, f"{model_id} asset name is invalid")
        require(
            isinstance(asset["size_bytes"], int) and asset["size_bytes"] > 0,
            f"{model_id} asset size is invalid",
        )
        validate_sha256(asset["sha256"], f"{model_id} asset checksum")
        expected_api_prefix = (
            f"https://api.github.com/repos/{repository}/releases/assets/"
        )
        require(
            isinstance(asset["api_url"], str)
            and asset["api_url"].startswith(expected_api_prefix)
            and asset["api_url"][len(expected_api_prefix) :].isdigit(),
            f"{model_id} asset API URL is invalid",
        )
        expected_download = (
            f"https://github.com/{repository}/releases/download/"
            f"{release_tag}/{expected_name}"
        )
        require(
            asset["download_url"] == expected_download,
            f"{model_id} download URL is invalid",
        )
        roles.add(role)
        ids.add(model_id)
    return document


def load_lock(path: Path) -> tuple[dict[str, Any], bytes, str]:
    raw = path.read_bytes()
    document = validate_lock(json.loads(raw))
    return document, raw, sha256_bytes(raw)


def metadata_inventory(metadata: dict[str, Any]) -> dict[str, tuple[str, int]]:
    files = metadata.get("files")
    require(isinstance(files, list), "package file inventory is invalid")
    inventory: dict[str, tuple[str, int]] = {}
    for item in files:
        require(isinstance(item, dict), "package inventory item is invalid")
        exact_keys(item, {"path", "sha256", "size_bytes"}, "package inventory item")
        name = item["path"]
        require(
            isinstance(name, str) and PurePosixPath(name).parts == (name,),
            "package inventory path is unsafe",
        )
        require(name not in inventory, f"duplicate package inventory path: {name}")
        checksum = validate_sha256(item["sha256"], f"package file {name} checksum")
        size = item["size_bytes"]
        require(
            isinstance(size, int) and size >= 0,
            f"package file {name} size is invalid",
        )
        inventory[name] = (checksum, size)
    return inventory


def verify_payloads(
    root: str,
    payloads: dict[str, bytes],
    model: dict[str, Any],
) -> None:
    require(root == model["package_root"], f"{model['id']} package root differs")
    require("package-metadata.json" in payloads, "package metadata is absent")
    metadata = json.loads(payloads["package-metadata.json"])
    require(
        metadata.get("schema_version") == PACKAGE_SCHEMA_VERSION,
        "unsupported package metadata schema",
    )
    require(metadata.get("id") == model["id"], "package model ID differs")
    require(
        metadata.get("contract_version") == model["contract_version"],
        "package contract version differs",
    )
    require(
        metadata.get("manifest_sha256") == model["manifest_sha256"],
        "package manifest checksum differs",
    )
    inventory = metadata_inventory(metadata)
    packaged_names = set(payloads) - {"package-metadata.json"}
    require(set(inventory) == packaged_names, "package inventory differs from files")
    require(
        REQUIRED_PACKAGE_FILES <= packaged_names,
        f"package is missing: {sorted(REQUIRED_PACKAGE_FILES - packaged_names)}",
    )
    for name, (checksum, size) in inventory.items():
        data = payloads[name]
        require(len(data) == size, f"package file size differs: {name}")
        require(
            sha256_bytes(data) == checksum,
            f"package file checksum differs: {name}",
        )

    manifest_bytes = payloads["model-manifest.json"]
    require(
        sha256_bytes(manifest_bytes) == model["manifest_sha256"],
        "model manifest checksum differs",
    )
    manifest = json.loads(manifest_bytes)
    require(
        manifest.get("contract_version") == model["contract_version"],
        "model manifest contract differs",
    )
    require(
        manifest.get("dataset", {}).get("content_sha256")
        == model["dataset_content_sha256"],
        "model dataset checksum differs",
    )
    for kind, locked_field in (
        ("onnx", "onnx_sha256"),
        ("checkpoint", "checkpoint_sha256"),
    ):
        artifact = manifest.get("artifacts", {}).get(kind)
        require(isinstance(artifact, dict), f"manifest {kind} artifact is absent")
        name = artifact.get("path")
        require(
            isinstance(name, str) and name in payloads,
            f"package {kind} is absent",
        )
        data = payloads[name]
        require(
            len(data) == artifact.get("size_bytes"),
            f"package {kind} size differs",
        )
        require(
            sha256_bytes(data) == artifact.get("sha256") == model[locked_field],
            f"package {kind} checksum differs",
        )


def read_package(path: Path, model: dict[str, Any]) -> tuple[str, dict[str, bytes]]:
    payloads: dict[str, bytes] = {}
    roots: set[str] = set()
    total_size = 0
    with tarfile.open(path, "r:gz") as archive:
        for member in archive.getmembers():
            pure = PurePosixPath(member.name)
            require(
                member.isfile() and len(pure.parts) == 2,
                f"unsafe package member: {member.name}",
            )
            root, name = pure.parts
            require(name not in payloads, f"duplicate package member: {name}")
            total_size += member.size
            require(
                total_size <= MAX_UNPACKED_BYTES,
                "package expands beyond the limit",
            )
            stream = archive.extractfile(member)
            require(stream is not None, f"cannot read package member: {member.name}")
            payloads[name] = stream.read()
            roots.add(root)
    require(len(roots) == 1, "package must have one top-level directory")
    root = next(iter(roots))
    verify_payloads(root, payloads, model)
    return root, payloads


def verify_cached_package(package: Path, model: dict[str, Any]) -> None:
    require(package.is_dir() and not package.is_symlink(), "cached package is absent")
    payloads: dict[str, bytes] = {}
    for path in package.iterdir():
        require(
            path.is_file() and not path.is_symlink(),
            "cached package has unsafe files",
        )
        payloads[path.name] = path.read_bytes()
    verify_payloads(package.name, payloads, model)


def download_github_asset(model: dict[str, Any], destination: Path) -> None:
    executable = shutil.which("gh")
    if executable is None:
        raise RuntimeError("GitHub CLI is required to download private model assets")
    with destination.open("wb") as output:
        result = subprocess.run(
            [
                executable,
                "api",
                "--method",
                "GET",
                "--header",
                "Accept: application/octet-stream",
                model["asset"]["api_url"],
            ],
            stdout=output,
            stderr=subprocess.PIPE,
            text=False,
            check=False,
        )
    if result.returncode != 0:
        message = result.stderr.decode(errors="replace").strip()
        raise RuntimeError(f"GitHub model download failed: {message}")


def verify_archive(path: Path, model: dict[str, Any]) -> None:
    asset = model["asset"]
    require(
        path.stat().st_size == asset["size_bytes"],
        "model archive size differs",
    )
    require(
        sha256_file(path) == asset["sha256"],
        "model archive checksum differs",
    )


def ensure_archive(
    model: dict[str, Any],
    cache_root: Path,
    fetcher: Fetcher,
) -> tuple[Path, bool]:
    archives = cache_root / "archives"
    archives.mkdir(parents=True, exist_ok=True)
    checksum = model["asset"]["sha256"]
    destination = archives / f"{checksum}.tar.gz"
    if destination.is_file():
        try:
            verify_archive(destination, model)
            return destination, True
        except ValueError:
            quarantine = archives / "quarantine"
            quarantine.mkdir(parents=True, exist_ok=True)
            os.replace(
                destination,
                quarantine / f"{destination.name}.{uuid.uuid4().hex}.invalid",
            )

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{checksum[:12]}.",
        suffix=".download",
        dir=archives,
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        fetcher(model, temporary)
        verify_archive(temporary, model)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)
    return destination, False


def ensure_package(
    archive: Path,
    model: dict[str, Any],
    cache_root: Path,
) -> tuple[Path, bool]:
    packages = cache_root / "packages"
    packages.mkdir(parents=True, exist_ok=True)
    destination = packages / model["asset"]["sha256"]
    package = destination / model["package_root"]
    if destination.is_dir():
        try:
            verify_cached_package(package, model)
            return package, True
        except ValueError:
            quarantine = packages / "quarantine"
            quarantine.mkdir(parents=True, exist_ok=True)
            os.replace(
                destination,
                quarantine / f"{destination.name}.{uuid.uuid4().hex}.invalid",
            )

    root, payloads = read_package(archive, model)
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{destination.name[:12]}.", dir=packages)
    )
    try:
        extracted = temporary / root
        extracted.mkdir()
        for name, data in payloads.items():
            (extracted / name).write_bytes(data)
        verify_cached_package(extracted, model)
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    return package, False


def atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        dir=path.parent,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as output:
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, 0o644)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def write_lock_snapshot(cache_root: Path, lock_digest: str, raw: bytes) -> Path:
    destination = cache_root / "locks" / f"{lock_digest}.json"
    if destination.is_file():
        require(destination.read_bytes() == raw, "cached lock snapshot differs")
    else:
        atomic_write(destination, raw)
    return destination


def sync_lock(
    lock_path: Path,
    cache_root: Path,
    fetcher: Fetcher = download_github_asset,
) -> dict[str, Any]:
    document, raw, lock_digest = load_lock(lock_path)
    resolved: dict[str, Any] = {}
    for model in document["models"]:
        archive, archive_hit = ensure_archive(model, cache_root, fetcher)
        package, package_hit = ensure_package(archive, model, cache_root)
        resolved[model["role"]] = {
            "id": model["id"],
            "status": model["status"],
            "package_dir": str(package.resolve()),
            "archive_cache_hit": archive_hit,
            "package_cache_hit": package_hit,
        }
    write_lock_snapshot(cache_root, lock_digest, raw)
    return {
        "schema_version": SCHEMA_VERSION,
        "lock_sha256": lock_digest,
        "selection_policy": document["selection_policy"],
        "activated": False,
        "models": resolved,
    }


def state_path(cache_root: Path) -> Path:
    return cache_root / "state.json"


def load_state(cache_root: Path) -> dict[str, Any]:
    path = state_path(cache_root)
    if not path.is_file():
        return {
            "schema_version": SCHEMA_VERSION,
            "current_lock_sha256": None,
            "history": [],
        }
    state = json.loads(path.read_text())
    require(
        state.get("schema_version") == SCHEMA_VERSION,
        "cache state schema differs",
    )
    require(
        state.get("current_lock_sha256") is None
        or (
            isinstance(state["current_lock_sha256"], str)
            and SHA256.fullmatch(state["current_lock_sha256"])
        ),
        "cache current lock is invalid",
    )
    history = state.get("history")
    require(
        isinstance(history, list)
        and all(
            isinstance(item, str) and SHA256.fullmatch(item)
            for item in history
        ),
        "cache history is invalid",
    )
    return state


def write_state(cache_root: Path, state: dict[str, Any]) -> None:
    data = (json.dumps(state, indent=2, sort_keys=True) + "\n").encode()
    atomic_write(state_path(cache_root), data)


def resolve_snapshot(cache_root: Path, lock_digest: str) -> dict[str, Any]:
    lock_path = cache_root / "locks" / f"{lock_digest}.json"
    require(lock_path.is_file(), f"cached lock snapshot is absent: {lock_digest}")
    document, _, actual_digest = load_lock(lock_path)
    require(actual_digest == lock_digest, "cached lock snapshot checksum differs")
    resolved: dict[str, Any] = {}
    for model in document["models"]:
        package = (
            cache_root
            / "packages"
            / model["asset"]["sha256"]
            / model["package_root"]
        )
        verify_cached_package(package, model)
        resolved[model["role"]] = {
            "id": model["id"],
            "package_dir": str(package.resolve()),
        }
    return {
        "schema_version": SCHEMA_VERSION,
        "lock_sha256": lock_digest,
        "selection_policy": document["selection_policy"],
        "models": resolved,
    }


def activate_lock(
    lock_path: Path,
    cache_root: Path,
    fetcher: Fetcher = download_github_asset,
) -> dict[str, Any]:
    document, _, lock_digest = load_lock(lock_path)
    require(
        document["selection_policy"] == "runtime",
        "lockfile policy does not permit runtime selection",
    )
    require(
        all(model["status"] == "approved" for model in document["models"]),
        "all runtime models must be approved",
    )
    synced = sync_lock(lock_path, cache_root, fetcher)
    state = load_state(cache_root)
    current = state["current_lock_sha256"]
    if current != lock_digest:
        if current is not None:
            state["history"].append(current)
        state["current_lock_sha256"] = lock_digest
        write_state(cache_root, state)
    synced["activated"] = True
    synced["history_depth"] = len(state["history"])
    return synced


def current_selection(cache_root: Path) -> dict[str, Any]:
    state = load_state(cache_root)
    current = state["current_lock_sha256"]
    require(current is not None, "no runtime model selection is active")
    result = resolve_snapshot(cache_root, current)
    result["history_depth"] = len(state["history"])
    return result


def rollback_selection(cache_root: Path) -> dict[str, Any]:
    state = load_state(cache_root)
    require(state["history"], "no prior runtime model selection is available")
    target = state["history"][-1]
    result = resolve_snapshot(cache_root, target)
    state["history"].pop()
    state["current_lock_sha256"] = target
    write_state(cache_root, state)
    result["history_depth"] = len(state["history"])
    result["rolled_back"] = True
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache-dir", type=Path, default=default_cache_root())
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("validate", "sync", "activate"):
        child = commands.add_parser(command)
        child.add_argument("lockfile", type=Path)
    commands.add_parser("current")
    commands.add_parser("rollback")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        if args.command == "validate":
            document, _, digest = load_lock(args.lockfile)
            result = {
                "schema_version": document["schema_version"],
                "lock_sha256": digest,
                "selection_policy": document["selection_policy"],
                "model_count": len(document["models"]),
            }
        elif args.command == "sync":
            result = sync_lock(args.lockfile, args.cache_dir)
        elif args.command == "activate":
            result = activate_lock(args.lockfile, args.cache_dir)
        elif args.command == "current":
            result = current_selection(args.cache_dir)
        else:
            result = rollback_selection(args.cache_dir)
    except (
        OSError,
        ValueError,
        RuntimeError,
        tarfile.TarError,
        json.JSONDecodeError,
    ) as error:
        raise SystemExit(f"error: {error}") from error
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
