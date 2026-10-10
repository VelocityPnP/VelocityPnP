# VelocityPnP

VelocityPnP is an API-first pick-and-place control system. It focuses on
measured calibration, guarded motion, and repeatable automation. The current
hardware target is a LumenPnP V3.0.2 with REV4 controller electronics.

> **Safety:** VelocityPnP controls machinery. Keep hands and tools clear. Keep
> physical power isolation in reach. Validate all motion in simulation before
> you enable hardware. Hardware access is disabled unless the daemon starts
> with both `--simulate=false` and `--allow-hardware`.

## Project status

VelocityPnP is under active development. It has run calibration, scanning,
feeder, pickup, placement, and inspection workflows on the target machine.
Some production gates are still open. Review the [roadmap](docs/ROADMAP.md)
and [hardware validation log](docs/HARDWARE_VALIDATION.md) before you use
physical hardware.

Current capabilities include:

- a Rust machine state model, simulator, Marlin transport, and SQLite journal;
- Linux controller and camera discovery;
- a versioned REST/SSE API, CLI, Python client, and React operator UI;
- direct KiCad board import with optional CSV BOM data;
- machine-run calibration with retained measurements and visual evidence;
- calibrated workspace scans, board registration, and inventory tracking;
- vision-assisted strip-feeder setup and guarded pickup qualification;
- persistent job execution with pause, recovery, and restart-safe state; and
- checksum-verified ONNX models with deterministic safety gates.

The daemon executes supported physical calibration stages. A caller cannot
complete a stage by entering a measurement. Unsupported stages stay locked.

## Quick start

The Rust vision crate needs the native OpenCV development library and Clang
bindings. See [development prerequisites](docs/DEVELOPMENT.md#prerequisites)
for the Debian and Ubuntu packages.

```bash
cd web
npm ci
npm run build
cd ..
cargo run -p velocitypnpd -- \
  --database velocitypnp.db \
  --listen 127.0.0.1:4080
```

Open `http://127.0.0.1:4080`. The default run uses the simulator.

The daemon has no user authentication. Its default listen address is
`0.0.0.0:4080`. Use that address only on an isolated, trusted network with a
host firewall. Use `--listen 127.0.0.1:4080` when remote access is not needed.

The daemon serves `web/dist` from a source checkout. Pass `--web-dir` to use a
different build, or pass `--disable-web-ui` to serve only the API. It writes
daily logs below `logs/` and keeps 14 files by default.

In another terminal, you can use the CLI or install the Python client:

```bash
cargo run -p velocitypnp -- status
cargo run -p velocitypnp -- devices
python3 -m pip install -e python
```

## Models

Model weights are not stored in this repository. The public
[VelocityPnP models repository](https://github.com/VelocityPnP/velocitypnp-models)
contains model and dataset catalogs. Binary packages are GitHub release
assets. The reviewed `models.lock.json` selects exact package versions and
checksums. See [model distribution](docs/MODEL_DISTRIBUTION.md) for validation,
activation, and rollback.

## Documentation

- [Development and test commands](docs/DEVELOPMENT.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Machine safety](docs/SAFETY.md)
- [Implementation status](docs/ROADMAP.md)
- [Hardware validation evidence](docs/HARDWARE_VALIDATION.md)
- [OpenAPI specification](api/openapi.yaml)
- [Contribution guide](CONTRIBUTING.md)

## Maintainer

[Tim Smith on GitHub](https://github.com/timbot) ·
[Tim Smith on X](https://x.com/timbot)

## License

VelocityPnP is licensed under the [Apache License 2.0](LICENSE).
