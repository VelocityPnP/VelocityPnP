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

After homing, the bounded live vision smoke test captures three top-datum
frames and three frames for each configured nozzle. It returns to the datum
and switches the lights off without changing calibration data:

```bash
wget -qO- --method=POST \
  --header='Content-Type: application/json' \
  --body-data='{
    "lease_id":"LEASE_UUID",
    "confirmed_external_safety_checks":true,
    "sample_count":3
  }' \
  http://127.0.0.1:4080/api/v1/vision/smoke-test
```

The test passes only when the top-camera stability gates pass, each bottom
capture produces image metrics, and each configured nozzle tip is identified.

After a passing top-camera planar calibration, a caller can scan a bounded XY
region. Ask the daemon for a preflight estimate before motion. This request
estimates the path, tile and registration counts, duration, mosaic dimensions,
and retained bytes:

```bash
wget -qO- --method=POST \
  --header='Content-Type: application/json' \
  --body-data='{
    "calibration_id":"CALIBRATION_UUID",
    "mode":"survey",
    "artifact_retention":"mosaic_only",
    "region_preset":"installed_plates"
  }' \
  http://127.0.0.1:4080/api/v1/workspace-scans/estimate
```

`precision` mode keeps the original calibrated-resolution behavior. It stores
full source tiles by default and has a 64-tile limit. `survey` mode defaults to
0.10 mm/pixel, 30 percent overlap, the machine profile's validated default XY
feed, a 4,096-tile limit, and `mosaic_only` retention. Layered surveys with
`transient_sources` have a 12,000-tile limit. An installed-plate scan uses at
least 60 percent overlap so that each surface has enough shared image area for
reliable reconstruction. It discards four queued frames after each completed
move before it accepts a capture. It holds one V4L2 stream open across moves.
Each mosaic layer retains two full camera rows. Height matching can retain up
to three preceding rows for wider-baseline pairs. The system registers and
renders each requested height layer and measures parallax. The height analysis
also compares wider non-adjacent tile pairs that still have at least 20 percent
overlap. This keeps a useful parallax baseline when capture overlap is high.
Memory use is bounded by height layers and a small number of camera rows
instead of total tile count.
`feed_mm_per_min` can request a slower feed, but cannot exceed the profile
limit.

Artifact retention can be `full_sources`, `transient_sources`, `thumbnails`,
or `mosaic_only`. `transient_sources` is for installed-plate surveys. It keeps
encoded captures in one scan-specific temporary directory, renders the
height-corrected image in 512-row bands with a boundary halo, and deletes the
captures after reconstruction. The report and final products remain in the
database. The preflight warning reports a conservative temporary-disk bound;
temporary bytes do not count against the retained-artifact quota. Every
request has a retained-byte estimate and quota. A request that exceeds its
quota fails before motion. The runtime checks actual encoded bytes before each
artifact is stored. The default survey quota is 64 MiB.

After a transient-source scan builds its height map, the retained parallax
JSON keeps the full aggregate counters and a deterministic sample of at most
25,000 observations. The final height map already consumed every accepted
observation. This bound prevents dense board features from making diagnostic
JSON dominate retained storage. Full-source scans keep every observation for
offline reconstruction.

`reconnaissance` mode requires the `installed_plates` region preset. It makes
one installed-plate-plane mosaic at 0.20 mm/pixel with 10 percent overlap and
`mosaic_only` retention by default. It uses calibrated placement without tile
registration or parallax analysis. After each move completes, it accepts a
frame whose monotonic V4L2 timestamp is at least one frame period newer than
the motion boundary. A camera without monotonic timestamps uses the validated
four-frame fallback. The scan report records actual discarded frames in
`discarded_capture_frames` and timestamp-selected captures in
`timestamp_fresh_frames`. Its estimate includes the serialized motion-control
and capture-dispatch cost measured on the attached V3.0.4 machine.

Reconnaissance also measures every horizontal and vertical tile overlap. It
combines projected-source luma disagreement with a bounded cross-axis
correlation. This analysis does not change tile placement. It classifies the
scan as `accept`, `targeted_refinement`, `review_geometry`, or
`increase_overlap`. Local problem seams become merged machine-coordinate
regions in `seam-health.json`. A coherent, material displacement across much
of the scan requests geometry review. Textureless or ambiguous problem seams
request more overlap instead of inventing a correction.

The report field `seam_health` summarizes the result and names these retained
artifacts:

- `seam-health.json`: per-overlap measurements and target regions
- `mosaic-disagreement.png`: projected source-image disagreement
- `overlap-confidence.png`: distinct source-view coverage
- `seam-health-overlay.png`: transparent problem overlay
- `mosaic-seam-health.png`: overlay composited onto the mosaic

These products are diagnostics. A later refinement request can use the target
regions, but the reconnaissance scan does not automatically move again.

Tune the classification policy against retained scan products without moving
the machine or changing the original scan record:

```bash
cargo run -p velocitypnp-vision --example reanalyze_workspace_seams -- \
  SCAN_ARTIFACT_DIRECTORY OUTPUT_DIRECTORY
```

The source directory must contain `manifest.json`, `mosaic.png`,
`mosaic-disagreement.png`, `overlap-confidence.png`, and `seam-health.json`.
The command reuses the recorded overlap correlations and writes a new report
and inspection images to the output directory.

Query current selective-refinement targets before choosing an index:

```bash
wget -qO- \
  http://127.0.0.1:4080/api/v1/workspace-scans/PARENT_UUID/refinement-targets
```

This read-only endpoint replays the retained diagnostic rasters with the
installed classifier. It does not change the parent report or move the
machine. This keeps target selection current after classifier updates.

Start one bounded target patch with `mode` set to `refinement`:

```bash
wget -qO- --method=POST \
  --header='Content-Type: application/json' \
  --body-data='{
    "lease_id":"LEASE_UUID",
    "calibration_id":"CALIBRATION_UUID",
    "confirmed_external_safety_checks":true,
    "mode":"refinement",
    "parent_scan_id":"PARENT_UUID",
    "refinement_target_index":0
  }' \
  http://127.0.0.1:4080/api/v1/workspace-scans
```

Refinement requires a successful reconnaissance parent with the same current
profile and calibration. It rejects `accept` and `review_geometry` parents.
It selects the ROI from the current replay, intersects it with reachable plate
coverage, and defaults to 60 percent overlap, 0.10 mm/pixel, and transient
sources. The child report stores `parent_scan_id`,
`refinement_target_index`, and a machine-coordinate raster transform. Its
`height_map.corrected_mosaic_artifact_name` names the georeferenced refined
patch. Temporary captures are deleted after chunked height reconstruction.
Each request scans one target, so quota checks, failures, and any retry stay
local to that ROI.

The target response also contains a severity-ranked `targets` list. Each item
keeps its stable `target_index`, reports why it was ranked, and identifies the
latest and successful child scans. `remaining_targets` excludes targets with a
successful child under the current classifier version. The response and child
reports record `workspace_refinement_priority.v1` separately from the seam
classifier version so future ranking changes remain traceable.

Start all remaining targets as one sequential queue:

```bash
wget -qO- --method=POST \
  --header='Content-Type: application/json' \
  --body-data='{
    "lease_id":"LEASE_UUID",
    "calibration_id":"CALIBRATION_UUID",
    "confirmed_external_safety_checks":true
  }' \
  http://127.0.0.1:4080/api/v1/workspace-scans/PARENT_UUID/refinements
```

The daemon calculates every child plan and quota before it schedules motion.
It skips targets that already succeeded, then runs the remaining targets from
highest to lowest severity. `max_targets` can bound one batch to between 1 and
32 patches. One exclusive scan lock prevents a manual scan from interleaving
with the queue. Each child has its own operation, report, quota, lineage,
priority, and corrected mosaic. The batch renews its lease before each child.
The batch operation message reports the active target and completed count.
A failed child stops the queue, parks and disables lights through the normal
scan cleanup path, and marks unattempted children with the reason. A later
request automatically retries failed or unattempted targets and still skips
successful ones. Daemon restart recovery fails both interrupted children and
their batch operation instead of leaving work marked as running.

After the queue succeeds, build one refined workspace image without machine
motion:

```bash
wget -qO- --method=POST \
  --header='Content-Type: application/json' \
  --body-data='{}' \
  http://127.0.0.1:4080/api/v1/workspace-scans/PARENT_UUID/refinement-composite
```

The daemon replays the current seam classifier. It selects the latest
successful, matching, height-corrected child for each target. It resamples
each child into the parent machine-coordinate raster. A two-millimeter feather
joins each patch to the coarse image. A robust boundary median supplies a
brightness correction that is limited to 32 luma levels.

The hard gate requires every current target, at least 99 percent reported
coverage in every patch, and at least 0.60 retained texture energy when the
parent region has enough texture to measure. Boundary disagreement above 64
luma and tile-midline discontinuity above 3.0 are review observations. They do
not fail the gate because an object edge or a real height change can produce
the same signal after transient source frames are deleted.

The parent scan stores `workspace_refinement_composite.v1` under
`refinement_composite` with the source classifier version and action. A
geometry-review action fails the gate. The generated artifacts are:

- `mosaic-refined.png`: plain composite at the parent scale and transform.
- `refinement-quality.json`: thresholds, measurements, failures, and review
  observations for each child.
- `mosaic-refined-quality.png`: the composite with green accepted or red
  rejected target outlines.

Generation is idempotent. It replaces these artifacts and recalculates the
stored byte count and manifest instead of adding their sizes again. A
successful child scan is only evidence that capture and reconstruction
finished. The separate composite result says whether the image passed the
automatic gate.

Start a bounded datum-PCB survey with the tagged V3.0.4 PCB extent:

```bash
wget -qO- --method=POST \
  --header='Content-Type: application/json' \
  --body-data='{
    "lease_id":"LEASE_UUID",
    "calibration_id":"CALIBRATION_UUID",
    "confirmed_external_safety_checks":true,
    "mode":"survey",
    "artifact_retention":"mosaic_only",
    "region":{
      "x_min_mm":180.131,
      "x_max_mm":260.131,
      "y_min_mm":165.271,
      "y_max_mm":200.271
    }
  }' \
  http://127.0.0.1:4080/api/v1/workspace-scans
```

Poll `GET /api/v1/workspace-scans/SCAN_UUID`. Fetch the final image from
`GET /api/v1/workspace-scans/SCAN_UUID/artifacts/mosaic.png` and the complete
report from `artifacts/manifest.json`. The `mosaic_layers` entries name every
height-specific image made from the same captures. OpenCV overlap registration
can apply
only a small, validated translation. It first uses ORB features inside the
predicted overlap. A bounded correlation fallback divides the overlap into
three independent segments and requires two unique, consistent cross-axis
peaks. The motion axis remains at calibrated machine placement because the
narrow overlap does not make that axis observable. Registration does not fit
scale or rotation. Rejected or textureless overlaps retain calibrated machine
placement and add a warning. Streaming surveys require two consistent
neighboring ORB registrations. One guided-correlation registration can apply
because it already has independent segment support.
If an accepted correction moves an outer tile inward, only the newly exposed
edge pixels use nominal machine placement. The manifest reports this value as
`nominal_fallback_fraction`.

Retained full-source scans can be replayed without machine motion. Save the
calibration snapshot that the scan names, then run:

```bash
cargo run -p velocitypnp-vision --example replay_workspace_scan -- \
  SCAN_DIRECTORY CALIBRATION_JSON OUTPUT.png OUTPUT.json \
  [recorded|working_plane|installed_plate_planes|surface_fraction:F] \
  [registration|no_registration]
```

`SCAN_DIRECTORY` must contain `manifest.json` and the retained tile PNG files.
The replay report records the registration method, evidence, and applied tile
corrections.

To measure sparse local height evidence from retained source frames, run:

```bash
cargo run -p velocitypnp-vision --example analyze_workspace_parallax -- \
  SCAN_DIRECTORY CALIBRATION_JSON OUTPUT.json

cargo run -p velocitypnp-vision --example build_workspace_height_mosaic -- \
  SCAN_DIRECTORY CALIBRATION_JSON OUTPUT_DIRECTORY
```

Each accepted observation matches one feature from two camera positions and
solves its relative surface fraction. Fraction zero is the calibrated
top-camera working plane. Fraction one is the installed bare-plate plane.
Negative fractions are above the working plane. Every observation includes
its machine XY position, fit residual, parallax signal, and source tile pair.

Coarse rendering averages the source area represented by each output sample.
Tile weights feather across the full configured overlap on each axis. This
suppresses source-pixel aliasing and repeated camera shading at tile edges.

The mosaic is an overhead map. Image X points toward positive machine X, and
image Y points toward negative machine Y. Thus, the top row is the region's
maximum machine Y. The estimate and report make this convention explicit in
`raster_transform`.

For the complete configured staging and build plates, replace `region` with
`"region_preset":"installed_plates"`. The preset requires a current, passing
installed-plate geometry artifact. It uses the measured affine plate extents
and the source-derived plate outline. It derives a minimum tile overlap from
the footprint at the highest supported surface fraction. This keeps that
height range covered between tiles without removing reachable plate-only edge
areas. The same captures produce `mosaic.png` at the working plane and
`mosaic-installed-plate-planes.png` at the bare plate plane. The report records
both under `mosaic_layers`.

This separation is required because one projection cannot align a scene that
contains bare plates, PCBs, and feeders at different heights. The same pass
writes sparse, automatically measured evidence to
`surface-height-observations.json`; its summary is in the scan's `parallax`
field. Low-texture areas remain unmeasured.

For `full_sources` or `transient_sources` retention, the service groups that
evidence into 2 mm cells. It writes the quantitative map to
`surface-height-map.json`, a compact preview to `surface-height-map.png`, a
full-size confidence raster to
`surface-height-confidence.png`, and the selected-plane image to
`mosaic-height-corrected.png`. It also writes a transparent height-class layer
to `surface-height-overlay.png` and a ready-to-view composite to
`mosaic-height-classes.png`. Bare build and staging plate pixels have no color
overlay. Unknown pixels are also transparent. Measured non-plate cells use a
stronger tint than locally inferred cells.

The reconstruction tests candidate planes at one-eighth of the calibrated
working-plane-to-plate scale. It locally extends only consistent measurements.
Unknown cells stay unknown in the data. For display, the corrected mosaic
compares source-image disagreement at the calibrated working and installed
plate endpoints, then selects the locally supported projection with spatial
regularization. This does not turn display selection into measured height.
If at least three retained source frames agree on the bottom-camera aperture,
their detected center motion fits a local camera-depth projection. This avoids
using the plate-height pixel scale for the lower camera surface. All views are
aligned to one machine-space center and blended with more weight near the
optical center. A narrow guard band replaces residual ringing at the aperture
edge. The surrounding plate and light ring keep the normal height-corrected
reconstruction. The structured detection is stored in
`height_map.camera_aperture`.
The scan report's `height_map` field records the artifacts, map coverage, and
candidate planes. The height colors are geometric evidence, not semantic
object labels. Feeder, PCB, datum PCB, nozzle fixture, and bottom camera ring
labels will come from later object detectors and can use the same transparent
overlay format.

`full_sources` supports offline replay. `transient_sources` produces the same
height products without keeping thousands of source frames. `thumbnails` and
`mosaic_only` scans still produce the two endpoint mosaics and sparse parallax
observations, but do not produce the unified map or corrected mosaic.

The V3.0.4 plates are 600 by 120 mm, with 39 columns and seven rows; their
outline is one 15 mm pitch outside the outer feature centers. The region is
clipped to what the camera can observe from legal head poses, not to the
head-center travel rectangle. Thus, the V3 top camera can see column 5 beyond
the X=0 head-center limit, but it cannot see columns 1 through 4. The preflight
estimate and final report warn which source-outline edges were clipped by
optical reach. Supply both `region_preset` and `region` to crop a layered scan.
A daemon restart marks a pending or running scan and its operation failed. It
does not leave an interrupted scan reported as active.

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
