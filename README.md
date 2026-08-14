# VelocityPnP

VelocityPnP is a headless, API-first pick-and-place control system focused on
measurable calibration, safe motion, and automation. The initial hardware
target is a LumenPnP V3.0.4 with REV4 controller electronics.

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
- V3.0.4 profile, calibration workflow definitions and test fixtures

Physical calibration, camera CV, feeders and production placement are exposed
as gated workflow stages and must pass hardware-in-loop acceptance before being
marked production-ready. See [docs/ROADMAP.md](docs/ROADMAP.md) and the current
[hardware validation log](docs/HARDWARE_VALIDATION.md).

## Quick start

```bash
cargo run -p velocitypnpd -- --simulate --database velocitypnp.db
cd web && npm install && npm run dev
```

The API listens on `127.0.0.1:4080` by default. The development UI proxies
`/api` to it. To serve a built UI from the daemon, run `npm run build` and pass
`--web-dir web/dist`.

```bash
cargo run -p velocitypnp -- status
cargo run -p velocitypnp -- devices
python3 -m pip install -e python
```

See [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) for prerequisites and commands.
