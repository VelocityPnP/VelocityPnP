# Development

## Prerequisites

- Rust 1.88 or newer (the repository pins 1.88.0)
- Node.js 20.19.x, or 22.12 or newer (the repository pins Node 22.23.2)
- Python 3.10 or newer
- OpenCV 4.6 or newer, `pkg-config`, and CMake
- Clang and libclang for the OpenCV and Linux V4L2 Rust bindings

On Debian or Ubuntu, install the native vision toolchain with:

```bash
sudo apt install clang cmake libclang-dev libopencv-dev pkg-config
```

The workspace pins Rust crate `opencv` 0.99.1. OpenCV is a required build and
runtime dependency, not a Python package or an optional Cargo feature.

The SQLite library is statically bundled, so a system SQLite development
package is not required.

## Vision implementation boundary

V4L2 remains the camera capture backend. Captured grayscale frames convert to
owned OpenCV matrices at the vision boundary. OpenCV supplies frame statistics,
Laplacian focus measurements, threshold masks, and four-connected component
labels. VelocityPnP retains the machine-specific physical-size checks,
concentricity rules, candidate scoring, calibration models, residual gates,
and independent holdouts.

Fixture conformance tests compare the OpenCV statistics with the earlier
validated calculation and keep datum, nozzle, camera-outline, and plate-hole
detections tied to attached V3.0.4 captures. This boundary also makes OpenCV
feature matching and geometric estimation available to the workspace mosaic
runner without routing camera capture through OpenCV.

## Build and test

```bash
cargo test --workspace --all-features
cargo clippy --workspace --all-targets --all-features -- -D warnings
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

## Simulated job motion

The job planner produces logical pick, place, and safe-Z poses. The separate
motion planner converts them to controller poses. The simulator endpoint uses
raw controller coordinates when `calibration_id` is absent. Supply a snapshot
ID to test its current, holdout-enabled local direction compensation:

```json
POST /api/v1/jobs/JOB_UUID/simulate
{
  "lease_id": "LEASE_UUID",
  "calibration_id": "CALIBRATION_UUID"
}
```

The planner retains logical and controller pose state independently. It keeps
the active X/Y correction during Z-only moves and checks bounds again after
correction. A stale, wrong-profile, failed, non-zero-mean, or unvalidated
artifact is rejected before the operation starts. Existing calibration runners
use the raw machine-actor path deliberately. Each active sequence binds the
snapshot UUID and artifact fingerprint. The physical job endpoint remains
disabled.

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
V3.0.4 seed: 1280x720 YUYV at 10 frames per second, 60 settling frames,
manual exposure 200, manual 4600 K white balance, and zero gain. Update a
profile with `PUT /api/v1/cameras/{role}/profile`.

After homing and acquiring a control lease, a bounded 30-frame check applies
the profile once, turns on the configured lights, opens one capture stream,
discards its configured settling frames once, and reports relative luma and
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
Before the identity query, connect sends `M999` and consumes its reply with a
bounded asynchronous read. This normalizes either a running or previously
stopped Marlin parser and clears stale queued commands. Connect still enters
preflight with no trusted pose or homing.

Before a physical daemon restart after source changes, run `cargo build
--workspace`. `cargo check`, `cargo clippy`, and `cargo test` validate source
and test harnesses, but they do not guarantee that the standalone
`target/debug/velocitypnpd` executable was relinked. Check the executable
timestamp and the expected hardware-path strings before connection.

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

The bounded datum-board feasibility probe uses the homing fiducial only for
visual registration. It seals on the separate 5 mm front-mask opening at
X=-22.5 mm and Y=-7.5 mm from the fiducial. These coordinates come from the
tagged Opulo V3.0.4 datum PCB. Before descent, the probe uses the bottom camera
to identify and center the current tip. It then uses the top camera to center
the 5 mm target instead of trusting the PCB offset alone. It moves X/Y while
both nozzles are level at controller Z=31.5, lowers the selected nozzle to the
clear search start, starts the pump, and descends at fixed X/Y. A stable vacuum
transition defines contact. The pump stops before the nozzle retracts.

The attached V3 profile currently bounds the search from nozzle Z=5.0 to
Z=3.0. N1 produced a stable vacuum transition at Z=3.4. The lower limit leaves
four 0.1 mm steps for detection variation but prevents deep travel if the
sensor does not respond. It is a safety bound, not a surface coordinate:

```bash
velocitypnp vacuum-probe f6acd39d-b509-4e42-b265-132c0efcf8d2 \
  --lease "$VELOCITYPNP_LEASE" --nozzle N1 --confirm-safety-checks
```

The command stores camera target correction, the directly measured current-tip
offset, applied height-model correction, raw off and open-nozzle values, and
descent samples in `values.vacuum_probe.<nozzle>` on the calibration snapshot. A
successful signal is feasibility evidence only. It does not complete the
Z-plane or vacuum-threshold workflow steps. It also records an independent
`vacuum_contact_n1` or `vacuum_contact_n2` artifact with the exact runtime-tip,
tooling, sensor, and vacuum-profile fingerprints that authorized the probe.

Reference-seal characterization is a separate step. It requires a current,
successful contact artifact and repeats camera registration and tip centering.
The stored contact Z narrows the new search to 0.3 mm above contact through the
same contact Z. The machine does not search below the validated value:

```bash
velocitypnp vacuum-characterize CALIBRATION_UUID \
  --lease "$VELOCITYPNP_LEASE" --nozzle N1 --confirm-safety-checks
```

The pump-off, pump-on open-nozzle, sealed-reference, and pump-off release
samples are stored in `values.vacuum_sensor_characterization.<nozzle>`. The
runner calculates robust distribution separation, signal polarity, and
separate acquire and release thresholds. Both nozzle/tool branches must pass
before the `vacuum_sensor_characterization` workflow step is complete.

These thresholds classify the fixed reference seal only. They do not claim
that a production part is held. Known-part distributions remain a later,
part-aware calibration after feeder pickup Z is available.

The automated head-geometry pass also holds each centered nozzle at fixed X/Y
and captures it at the profile Z heights. Bounded X/Y probes measure the local
image map at each height, and the runner fits XY shift per millimetre of nozzle
Z. It always retracts to the level controller Z before the next lateral move.
Reciprocal observations of the camera outlines, including the V3 top-camera
33 mm cutout, supply the shared working-plane transfer term automatically.

The observation command is retained as a diagnostic override:

```bash
velocitypnp observe-nozzle-height f6acd39d-b509-4e42-b265-132c0efcf8d2 \
  --nozzle N1 --nozzle-z-mm 3.5 \
  --error-x-mm 0.5 --error-y-mm -0.5 --uncertainty-mm 0.3
```

This command does not move the machine or complete an automated step. Normal
calibration and vacuum targeting do not require an operator measurement.

Primary-datum visual homing defaults to observation mode. It moves to the
profile seed, applies the top-camera profile and lights, detects the datum, and
returns the proposed correction without changing the coordinate frame:

```bash
velocitypnp visual-home --lease "$VELOCITYPNP_LEASE"
```

`--current-pose` skips the seed move for controlled axis-sign probes.
`--apply-offset` is rejected while `axis_mapping_validated` is false. Once the
mapping has physical evidence, apply mode permits up to three corrections while
keeping the total offset from the seed within one millimetre. Every correction
uses a fresh frame; the final correction distance must pass the profile limit
in millimetres before the profile-defined XY-only coordinate reset is sent.
Pixel error remains diagnostic evidence. Z and both nozzle angles are never
included in that reset.

Local X/Y direction response is a separate automatic capture. It reuses the
current top-camera working-plane map, performs one live visual home, and ends
every measurement at the same datum pose. Fixed datum location and local lens
distortion therefore cancel. The runner approaches from both sides of X and Y
with the profile distances and feeds:

```bash
velocitypnp calibrate-xy-direction-response CALIBRATION_UUID \
  --lease "$VELOCITYPNP_LEASE" --confirm-safety-checks
```

Sampling is adaptive. It starts with three pairs that cover multiple distances
and feeds. A resolved direction gap can stop when its 95% confidence interval
is small relative to the observed effect. An unresolved result gets five pairs.
An uncertain resolved result can continue to seven pairs. No fixed millimetre
accuracy rule decides whether backlash exists. The runner tests each measured
direction correction on an independent holdout pair. It stores zero applied
compensation when the effect is unresolved or the holdout does not reduce the
direction gap. The two direction corrections are the zero-mean half-gap. This
removes camera drift and other common offset from the backlash term.

This result is local to the homing datum. It does not establish full-bed scale,
skew, or lens distortion. A simulated job can select the snapshot explicitly
and apply its holdout-enabled vectors while retaining logical coordinates.
Manual jog and calibration motion remain raw, and physical job execution is
not implemented.

## Installed camera and plate geometry

The installed-geometry runner calibrates the fixed-height optical paths and
the plate geometry without a loose checkerboard or a cleared machine bed:

```bash
velocitypnp calibrate-installed-geometry CALIBRATION_UUID \
  --lease "$VELOCITYPNP_LEASE" \
  --confirm-safety-checks
```

The top-camera sweep moves the installed primary datum through a profile-bounded
image grid. The bottom-camera sweep moves the verified N1 tip through its image
grid. Each fit reserves spatial holdouts. The distortion-polynomial model is
selected only when it predicts those unseen positions better than the affine
model. The result is a planar pixel-to-centering model over the observed image
support. It is intentionally not an unconstrained intrinsic calibration for
arbitrary object depths.
Every camera and plate observation uses the same bounded negative-X/negative-Y
approach. Direction response is therefore a separate artifact, not a hidden
mathematical input to the optical fit.

The runner then scans documented staging- and build-plate hole positions. The
plate with the datum PCB is the bottom, or south, staging plate. The adjacent
north plate is the top build plate. The machine profile stores these roles and
their shared grid-edge relationship explicitly. It registers the staging plate
first and uses that measured offset as the initial estimate for the build
plate. It first observes one hole from three known camera poses. This gives each
plate a local pixel-to-machine map at its installed height. A second hole must
confirm the plate offset before the wider scan starts. Thus, the runner does
not assume that the raised datum PCB and the lower plates have the same image
scale. Plate drawings provide candidate coordinates, not an assertion that
each feature is visible. A target with no stable concentric hole and pad
observation is recorded as rejected and skipped. The detector searches only
the bounded target region. It takes extra confirmation frames only after the
first frame contains a plausible hole. This permits feeders, boards, brackets,
or other installed hardware to obscure part of a plate without making each
rejection expensive. A plate qualifies only when the remaining observations
span enough rows, columns, and millimetres. Independent holdouts select the
fitted affine map only when it predicts better than the documented nominal
grid. Otherwise, the runner retains the simpler nominal map. Holdouts are
distributed across the alternating valid-hole pattern, not assigned to whole
columns. Checkpoints report the training and holdout counts and retain partial
samples. The scan stops early after it has enough distributed evidence; it
does not need to inspect every hole.

The run requalifies and reuses compatible camera sweeps from an interrupted
run. It stores `top_camera_planar_model`, `bottom_camera_planar_model`, and
`installed_plate_geometry` artifacts with the raw samples, missing-target
reasons, holdout residuals, coverage results, and image evidence. Failure to
qualify one plate is a reported limitation. The complete run passes only when
both camera models and both configured plates qualify.

## Automated calibration runs

The supported calibration path completes automated steps only from machine
measurements. Create a snapshot, then start its initial machine runner from the
web UI or the CLI. A bounded working-plane residual can refine a completed
height model, but it does not complete a workflow step. Safety confirmation
covers conditions the V3 cannot observe before its
first motion: level nozzles, a clear travel envelope, and reachable physical
power isolation. The nozzle tubes must also be seated, routed normally, and
free of snags.

```bash
velocitypnp new-calibration --machine-serial 3286336D3331
velocitypnp calibrate CALIBRATION_UUID \
  --lease "$VELOCITYPNP_LEASE" \
  --confirm-safety-checks
```

The runner performs profile-scoped combined XYZ coarse homing, pulses and reads
back each configured vacuum actuator/sensor pair with explicit off cleanup,
moves to both camera working views, applies stored camera controls, measures
reproducibility, and presents N1 and N2 to the bottom camera. The N045 bore and
the N24 outer-tip circle distinguish the configured tools. The operator does
not declare the loaded tips. Camera open and capture failures get five bounded
attempts with 500 ms between attempts so a short USB re-enumeration does not
discard a long calibration run. A persistent camera fault still stops the
runner. Tooling identity uses dark thresholds 45, 60, and 90. It stores every
identity frame and does not treat an unsupported wrong-feature sample as a
confirmed different tool. It then repeats coarse home, bounded visual
refinement, a deterministic -10/-10 mm excursion, return, and observation. The
default five cycles are a commissioning check. Production qualification is
separate and still requires at least 30 stored visual-home samples. After the
configured cycles pass, the runner performs head geometry. The runner:

1. Uses bounded X/Y probes around the homing fiducial to measure the top-camera
   working-plane map.
2. Uses bounded local X/Y probes to measure the bottom-camera map.
3. Uses each camera to locate the opposing camera outline. Their
   reciprocal poses and apparent scale solve the working-plane ray transfer.
4. Locates each configured nozzle feature from the prior profile offset seed.
   Nozzle offsets use the top-camera reference and nozzle poses observed by the
   same fixed bottom camera.
5. Rotates N1/A and N2/B through 12 alternating offsets from ±15 to
   ±165 degrees. The initial tooling check supplies identity. During rotation,
   a bounded multi-threshold tracker follows the same verified feature and
   records the last frame from each attempt. This avoids treating a reflective
   edge change as an installed-tool change.
6. Restores each starting angle and fits the rotation-axis position, head
   offset, and first-harmonic nozzle-tip runout coefficients.

Only the calibration runner can request these profile-bounded rotations.
Normal physical jog rotation remains disabled by `rotation_motion: false`.
There is no unrestricted HTTP nozzle-rotation endpoint.

Every phase and sample is persisted under
`GET /api/v1/calibration-runs/{id}`. A completed measurement outside its gate is
`rejected`, while camera, storage, lease, or motion failures are `failed`.
Settled reciprocal-camera frames are stored in SQLite. Their metadata and
relative URLs are in `camera_reports.head_geometry_artifacts`; retrieve an
image with `GET /api/v1/calibration-runs/{id}/artifacts/{name}`.
Successful evidence advances the snapshot automatically. Calibration stages
whose runners are not implemented are reported as planned and cannot be
completed manually. Starting a run renews the caller's matching control lease
to the machine actor's one-hour maximum. The runner refreshes that same lease
before each long-running measurement checkpoint, so long runs keep control
without asking the operator to renew it manually.

## Selective calibration artifacts

The legacy ordered steps remain available to existing clients. Each measured
or calculated result is also stored as a fine-grained artifact. Artifacts store
the algorithm version, relevant machine/camera/tooling input fingerprints,
dependency fingerprints, source run, value, and a stable SHA-256 fingerprint.
The graph separates physical captures from mathematical calculations:

```text
top map + reciprocal cameras -> camera-ray transfer (calculated)
top map -> local X/Y direction response (captured and holdout-validated)
top map -> top planar camera model
bottom map + N1 tooling -> bottom planar camera model
top planar camera model -> installed plate geometry
bottom map + N1 tooling -> N1 runout -> N1 height model
bottom map + N2 tooling -> N2 runout -> N2 height model
camera-ray transfer + N1 branch -> N1 runtime tip -> N1 vacuum contact
  -> N1 reference-seal characterization
camera-ray transfer + N2 branch -> N2 runtime tip -> N2 vacuum contact
  -> N2 reference-seal characterization
```

Coarse homing and one live visual alignment are session motion preconditions.
They are not mathematical parents of every stored geometry result. A change to
one nozzle branch does not invalidate the other nozzle's runtime transform.

Preview reuse, zero-motion calculation, and required recapture:

```bash
velocitypnp calibration-plan CALIBRATION_UUID
```

Recalculate stale derived artifacts without commanding machine motion:

```bash
velocitypnp recompute-derived-calibration CALIBRATION_UUID
```

Capture only a selected physical branch. The run performs one live visual
alignment and verifies the installed tooling before measurement:

```bash
velocitypnp capture-calibration-artifacts CALIBRATION_UUID \
  --lease "$VELOCITYPNP_LEASE" \
  --target nozzle_axis_runout_n1 --target nozzle_height_n1 \
  --confirm-safety-checks
```

Use repeated `--target ARTIFACT_ID` arguments to limit either command. Legacy
snapshots migrate automatically when the planner or a dependent operation first
uses them. Camera-ray transfer, shared offset summaries, and per-nozzle runtime
tip transforms are calculated from retained source captures.
