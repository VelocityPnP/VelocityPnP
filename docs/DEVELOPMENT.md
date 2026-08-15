# Development

## Prerequisites

- Rust 1.88 or newer (the repository pins 1.88.0)
- Node.js 20.19.x, or 22.12 or newer (the repository pins Node 22.23.2)
- Python 3.10 or newer
- Clang/libclang for the Linux V4L2 bindings
- Optional: OpenCV development libraries for `--features native-opencv`

The SQLite library is statically bundled, so a system SQLite development
package is not required.

## Build and test

```bash
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check

cd web
npm ci
npm run build
npm test -- --run
npm audit

cd ../python
PYTHONPATH=src python3 -m unittest discover -s tests
```

## Run in simulation

```bash
cargo run -p velocitypnpd -- --database velocitypnp.db
```

The daemon listens on `127.0.0.1:4080`. In another terminal:

```bash
cargo run -p velocitypnp -- status
cargo run -p velocitypnp -- connect
cargo run -p velocitypnp -- preflight
cargo run -p velocitypnp -- lease
```

Homing is intentionally not implicit. After physically leveling both nozzles,
clearing the complete travel envelope, and putting power isolation within
reach, pass `home --lease <UUID> --confirm-safety-checks`.

CLI jog commands preserve the controller-reported primary-nozzle angle when
`--rotation` is omitted. Physical rotation requests are rejected while the
active profile's `rotation_motion` capability is false.

Attached PnP camera roles are available without opening the motion controller:

```bash
wget http://127.0.0.1:4080/api/v1/cameras/top_camera/snapshot -O top.png
wget http://127.0.0.1:4080/api/v1/cameras/bottom_camera/snapshot -O bottom.png
wget -qO- http://127.0.0.1:4080/api/v1/cameras/top_camera/controls
wget -qO- http://127.0.0.1:4080/api/v1/cameras/top_camera/profile
```

Snapshots use the stable role assigned during discovery, discard five settling
frames, and return 1280x720 grayscale PNGs from native YUYV capture.
Camera control updates use `PUT` on the role-scoped controls endpoint. The
daemon validates IDs, types, ranges, steps, menu values, and writability before
applying values in request order, so automatic mode can be disabled before its
manual value is set.

Role-scoped profiles persist capture format, ordered controls, ring-light use,
and reproducibility thresholds in SQLite. A missing profile resolves to the
V3.0.4 seed: 1280x720 YUYV at 10 frames per second, five settling frames,
manual exposure 200, manual 4600 K white balance, and zero gain. Update a
profile with `PUT /api/v1/cameras/{role}/profile`.

After homing and acquiring a control lease, a bounded 30-frame check applies
the profile once, turns on the configured lights, and reports relative luma and
focus ranges, target detection rate, and p95 target-center jitter:

```bash
curl -sS -X POST \
  http://127.0.0.1:4080/api/v1/cameras/top_camera/reproducibility \
  -H 'content-type: application/json' \
  -d '{"lease_id":"LEASE_UUID","sample_count":30}'
```

The top-camera metric tracks the bright center island of the LumenPnP datum;
the bottom-camera metric tracks the dark nozzle bore. Thresholds are
provisional until both attached-camera runs pass and are recorded in the
hardware evidence log.

Build the UI with `npm run build`, then serve it from the daemon using
`--web-dir web/dist`. The Vite development server is loopback-only and proxies
API calls to the daemon.

## Physical hardware gate

Do not enable hardware until simulator tests pass and the machine is physically
clear. Starting the physical driver is intentionally verbose:

```bash
cargo run -p velocitypnpd -- \
  --simulate=false \
  --allow-hardware \
  --serial-port /dev/serial/by-id/usb-STMicroelectronics_MARLIN_OPULO_LUMEN_REV4_CDC_in_FS_Mode_3286336D3331-if00
```

Opening a USB CDC controller may reset it. Connect does only the bounded Marlin
identity query, unit/mode setup, and documented integrated-vacuum-sensor
initialization; it does not home, move, or enable an output automatically.

The V3.0.4 profile exposes `vacuum_1` and `vacuum_2` as both actuators and raw
pressure sensors. Reads require a lease and are allowed during preflight,
unhomed, or ready states:

```bash
velocitypnp read-sensor --lease "$VELOCITYPNP_LEASE" vacuum_1
velocitypnp actuate --lease "$VELOCITYPNP_LEASE" vacuum_1 true
velocitypnp actuate --lease "$VELOCITYPNP_LEASE" vacuum_1 false
```

Sensor readings are signed controller counts, not pressure units. Establish
the ambient and sealed-nozzle distributions on the attached machine before
using them for part detection. An actuator-on validation must always include
an explicit off command and a fail-safe disconnect.

Primary-datum visual homing defaults to observation mode. It moves to the
profile seed, applies the top-camera profile and lights, detects the datum, and
returns the proposed correction without changing the coordinate frame:

```bash
velocitypnp visual-home --lease "$VELOCITYPNP_LEASE"
```

`--current-pose` skips the seed move for controlled axis-sign probes.
`--apply-offset` is rejected while `axis_mapping_validated` is false. Once the
mapping has physical evidence, apply mode permits only a one-millimetre
correction, captures a verification frame with at most one pixel residual, and
then sends a profile-defined XY-only coordinate reset. Z and both nozzle angles
are never included in that reset.
