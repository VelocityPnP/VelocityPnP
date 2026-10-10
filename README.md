# VelocityPnP

VelocityPnP is a headless, API-first pick-and-place control system focused on
measurable calibration, safe motion, and automation. The initial hardware
target is a LumenPnP V3.0.2 with REV4 controller electronics.

> **Safety:** VelocityPnP controls machinery. Keep hands and tools clear,
> maintain access to physical power isolation, and validate all motion in the
> simulator before enabling hardware. Hardware access is disabled unless the
> daemon is started with `--allow-hardware`.

## Status

This repository contains the first working platform slice:

- Rust domain/state model, simulator and Marlin transport
- Linux USB/serial/camera discovery
- SQLite configuration and event journal
- KiCad `.kicad_pcb` import and optional BOM merge
- deterministic job plans, simulator placement cycles, and inspection review records
- staged, one-way OpenPnP XML migration that never bypasses recalibration
- versioned REST/SSE API, CLI and Python client
- React operator UI with discovery, control, calibration and board import
- persisted machine-executed calibration runs with progress and evidence
- automatic reciprocal camera, N1/N2 offset, and nozzle-runout calibration
- session-scoped fine XYZ homing from the visual datum and vacuum-contact pad
- installed-target planar camera calibration and height-aware, occlusion-tolerant plate scans
- calibrated top-camera workspace scanning with streaming survey mosaics,
  bounded OpenCV tile registration, explicit raster coordinates, measured
  plate-plane geometry, reconnaissance seam diagnostics, lineage-tracked
  selective refinement, prioritized refinement queues, preflight estimates,
  georeferenced refinement composites, explicit image-quality gates, and
  artifact quotas
- no-motion workspace board candidates with machine-space outlines and
  imported KiCad dimension matching
- automatic top-side PCB registration from scan seeds and local KiCad
  fiducials, with persisted visual evidence and residual-gated transforms
- strip-feeder drafts, center-refined live holdouts, and closed-loop pickup
  qualification
- hash-checked CPU ONNX source-component inference with guarded pickup-center
  and empty-pocket decisions, detector comparison, and retained evidence
- paired bottom-camera held-part dataset export with oriented-box, condition,
  and directed-end review
- hash-checked CPU ONNX held-part inference with retained deterministic and
  learned comparison evidence and a guarded bare-nozzle release decision
- hash-checked top-camera placement verification with guarded deterministic
  agreement and persistent before/after evidence
- bounded placement-transfer calibration with provisional corrections,
  independent check targets, and restart-safe evidence continuation
- persistent guarded job execution with pause, recovery, and restart-safe state
- per-nozzle reference-seal vacuum characterization with learned hysteresis
- live Marlin motion settings, per-move timing, and bounded XY speed sweeps
- V3.0.2 profile, calibration workflow definitions and test fixtures

Supported physical calibration stages are executed by the daemon; callers
cannot mark a step complete by entering measurements. Later calibration,
feeder, and production stages remain explicitly unavailable until their
machine runners pass hardware-in-loop acceptance. See
[docs/ROADMAP.md](docs/ROADMAP.md) and the current
[hardware validation log](docs/HARDWARE_VALIDATION.md).

## Quick start

The Rust vision crate requires the native OpenCV development library and its
Clang binding toolchain. See [Development prerequisites](docs/DEVELOPMENT.md)
for the Debian/Ubuntu installation command.

```bash
cd web && npm install && npm run build
cd .. && cargo run -p velocitypnpd -- --database velocitypnp.db
```

The daemon serves the production UI and API on all IPv4 interfaces at port
`4080` by default. It finds `web/dist` in a source checkout or the installed
`/usr/share/velocitypnp/web` directory. The service has no user authentication.
Use it only on an isolated, trusted network, and use the host firewall to limit
access. Pass `--web-dir` to select another build, or `--disable-web-ui` to serve
only the API. The optional Vite development server listens on all IPv4
interfaces at port `4081` and proxies `/api` to the loopback address.
The daemon also writes daily logs to `logs/velocitypnpd.YYYY-MM-DD.log` and
keeps at most 14 files. `logs/velocitypnpd.current.log` points to the active
file. Use `--log-dir`, `--log-retention-files`, or
`--disable-file-logging` to change this behavior.

```bash
cargo run -p velocitypnp -- status
cargo run -p velocitypnp -- devices
python3 -m pip install -e python
```

See [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) for prerequisites and commands.
See [docs/MODEL_DISTRIBUTION.md](docs/MODEL_DISTRIBUTION.md) for model package
validation, approval, selection, and rollback.

Model weights are not stored in this repository. Local working data and model
packages are below the ignored `datasets/` tree. Final reviewed datasets and
model packages are immutable release assets in the private model repository.
The daemon accepts explicit model package directories and verifies each
manifest, contract, license, file size, and checksum before inference.
