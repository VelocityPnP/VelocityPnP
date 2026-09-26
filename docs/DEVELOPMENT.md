# Development

## Prerequisites

- Rust 1.88 or newer (the repository pins 1.88.0)
- Node.js 20.19.x, or 22.12 or newer (the repository pins Node 22.23.2)
- Python 3.10 or newer
- OpenCV 4.6 or newer, `pkg-config`, and CMake
- Clang and libclang for the OpenCV and Linux V4L2 Rust bindings

On Debian or Ubuntu, install the native vision toolchain and operator tools
with:

```bash
sudo apt install clang cmake curl libclang-dev libopencv-dev pkg-config \
  sqlite3 v4l-utils
```

The workspace pins Rust crate `opencv` 0.99.1. OpenCV is a required build and
runtime dependency, not a Python package or an optional Cargo feature.

The SQLite library is statically bundled, so a system SQLite development
package is not required. The `sqlite3` command is useful for database checks
and recovery.

The physical daemon user must have access to the V4L2 camera devices. On a
standard Debian or Ubuntu system, add the user to the `video` group and then
start a new login session:

```bash
sudo usermod -aG video "$USER"
```

Use `v4l2-ctl --list-devices` to verify camera access before a physical run.

## Vision implementation boundary

V4L2 remains the camera capture backend. Captured grayscale frames convert to
owned OpenCV matrices at the vision boundary. OpenCV supplies frame statistics,
Laplacian focus measurements, threshold masks, and four-connected component
labels. VelocityPnP retains the machine-specific physical-size checks,
concentricity rules, candidate scoring, calibration models, residual gates,
and independent holdouts.

Fixture conformance tests compare the OpenCV statistics with the earlier
validated calculation and keep datum, nozzle, camera-outline, and plate-hole
detections tied to attached V3.0.2 captures. This boundary also makes OpenCV
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

The daemon listens on `0.0.0.0:4080`. It has no user authentication, so use an
isolated, trusted network or a host firewall. In another terminal:

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
0.10 mm/pixel, 30 percent overlap, the machine profile's validated clear-XY
feed, a 4,096-tile limit, and `mosaic_only` retention. Precision mode uses the
profile's default XY feed. Layered surveys with
`transient_sources` have a 12,000-tile limit. An installed-plate scan uses at
least 60 percent overlap so that each surface has enough shared image area for
reliable reconstruction. All scan modes hold one V4L2 stream open across
moves. After each move, they use the V4L2 timestamp source to require an
exposure that started after motion stopped. A start-of-exposure timestamp must
be newer than the motion boundary. An end-of-frame timestamp must be newer by
the configured manual exposure time. An unknown timestamp source keeps a full
frame-period guard. A camera without valid monotonic timestamps uses the
validated four-frame fallback.

Each mosaic layer retains two full camera rows. Height matching can retain up
to three preceding rows for wider-baseline pairs. The system registers and
renders each requested height layer and measures parallax. The height analysis
also compares wider non-adjacent tile pairs that still have at least 20 percent
overlap. This keeps a useful parallax baseline when capture overlap is high.
Memory use is bounded by height layers and a small number of camera rows
instead of total tile count.
`feed_mm_per_min` can request a slower feed, but cannot exceed the validated
clear-XY limit. A physical scan also requires direction-response holdout
evidence for its exact scan and datum-return feeds. Scan tiles and the final
datum return use the calibrated direction correction. They also reserve the
largest possible X/Y controller correction at the machine bounds. The
connected estimate uses the controller's X/Y and travel acceleration limits
plus measured command overhead. Thus, short scan moves are not estimated as
constant-speed travel.

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
registration or parallax analysis. The scan report records actual discarded
frames in `discarded_capture_frames` and timestamp-selected captures in
`timestamp_fresh_frames`. Its estimate includes the serialized motion-control
and capture-dispatch cost measured on the attached V3.0.2 machine.

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

Start a bounded datum-PCB survey with the tagged V3.0.2 PCB extent:

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
working-plane-to-plate scale. A measured or inferred 2 mm cell is a geometric
prior, not a hard image boundary. In cells next to a different height, the
renderer compares source-image disagreement in a 0.30 mm neighborhood at the
local candidate planes. It can move the display transition inside the cell
when two or more source tiles give a better-supported projection. Confidence
and measurement source add a change margin. Pixels without enough overlap keep
the cell prior. If ceil-rounded output dimensions put an outer raster pixel
just beyond the height-map region, that pixel keeps the working-plane image
and a transparent class overlay.

Transient reconstruction decodes each source frame once per output chunk and
shares it across candidate planes. Independent candidate planes use the
available CPU cores. A plane with no tile-placement correction omits the
identical nominal projection and its buffers. Candidate selection and chunk
merge order remain deterministic.

Unknown cells stay unknown in the data. For display, the corrected mosaic
compares source-image disagreement at the calibrated working and installed
plate endpoints, uses spatial regularization for the cell prior, and applies
the same overlap-supported pixel boundary localization. The transparent class
overlay follows the selected display boundary, but this does not turn display
selection into measured height.
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

### Workspace object proposals

A successful height-aware scan or accepted refinement composite can produce
initial object proposals without machine motion:

```bash
cargo run -p velocitypnp -- discover-objects SCAN_UUID
```

The equivalent API is
`POST /api/v1/workspace-scans/SCAN_UUID/object-proposals`. The former
`board-discovery` route remains an alias. The Python client provides
`discover_workspace_objects(scan_id)`. The operation uses an accepted refined
mosaic when present. Otherwise, it uses the height-corrected mosaic.

The `workspace_object_proposals.v6` policy selects known height-map cells above
the installed-plate plane. It joins nearby evidence and rejects small isolated
height noise, including regions below 100 square millimeters. Each proposal
rectangle is parallel and perpendicular to its
calibrated plate axes. Its enclosing machine-axis rectangle is the region that
the scan planner can refine.

For imported boards with usable Edge.Cuts dimensions, OpenCV line evidence
fits the design rectangle inside that coarse region. This removes clamps and
plate margin from the board measurement. A design match records its visual
outline, center, angle, edge support, and dimension error in machine
coordinates. A square or nearly square outline keeps
`orientation_ambiguous=true`; local fiducials must resolve its design
orientation before job setup.

The semantic classifier is a replaceable interface. The initial deterministic
classifier combines known-fixture overlap, plate role, raised-surface geometry,
KiCad dimensions, visual outline fits, and repeated parallel feeder tracks.
The staging-plate prior and feeder-track evidence combine only when both are
present. Weak or conflicting evidence produces `unknown`; the policy does not
force a feeder or board label. A future learned classifier will receive the
same normalized image crop and metadata. Each crop is retained as
`object-evidence-NNN.png` so operator corrections can become labeled training
data.

A staging-plate region with at least 0.65 confidence from repeated parallel
tracks is routed to the strip-feeder specialist as a review-required feeder
candidate. This lower routing threshold does not activate a feeder. Known
fixtures return before this rule, and the specialist must still find an
EIA-481 sprocket lattice.

The V3.0.2 profile supplies the source-derived datum-PCB outline and an
installed nozzle-bank envelope. The detected bottom-camera aperture supplies a
dynamic guard region. These objects remain semantic inventory evidence, but
their board likelihood is zero. A reconnaissance parent can merge height maps
from accepted refinement children. The report retains each source scan and
artifact as lineage.

The scan stores `board-discovery.json`, `mosaic-board-candidates.png`, and one
normalized evidence crop per proposal. Green outlines are boards, blue outlines
are feeders, purple outlines are known fixtures, and amber outlines need more
evidence. A visually matched board also projects enabled top-side placement
orientation ticks in cyan and KiCad fiducials in yellow. These marks use the
canonical outline orientation for diagnostic preview. They do not resolve a
square board's orientation ambiguity.

### Source-component learning dataset

Raw `source-pocket-before.png` artifacts can seed an oriented component
detector without more machine motion. The exporter rectifies each image to the
carrier-tape axes and applies the current deterministic detector as weak
supervision:

```bash
cargo run -p velocitypnpd --example export_source_component_dataset -- \
  velocitypnp.db datasets/source-components-v1 EVAL_REGISTRATION_UUID
```

A persisted workspace inventory can supply a balanced set of centered,
safe-Z camera frames. First, bind each inventory lane to an existing feeder
registration or create an inactive vision draft:

```bash
cargo run --release -p velocitypnp -- \
  promote-strip-feeder-inventory SCAN_UUID
```

This command reuses a registration only when its tape width, pocket pitch,
angle, and cross-lane position agree with the inventory. Thus, an old false
feeder proposal does not replace the measured inventory geometry. Then capture
known occupied and empty pockets after coarse and fine XYZ home:

```bash
cargo run --release -p velocitypnp -- \
  capture-strip-feeder-sources SCAN_UUID \
  --lease LEASE_UUID --occupied 12 --empty 12 \
  --lane 0 --lane 1 --lane 2 --lane 3 --lane 4 --lane 5 \
  --confirm-safety-checks
```

Omit `--lane` to use all inventory lanes. Selection is balanced across lanes
and takes samples from both ends of each known occupancy run. The camera stays
at safe Z. The operation visual-homes once, follows a short travel route, and
stores every raw frame and its inventory lineage in SQLite. It does not infer
tape width again or depend on a temporary mosaic file. Query a run with:

```bash
cargo run --release -p velocitypnp -- \
  strip-feeder-source-capture CAPTURE_UUID
```

Inventory occupancy is capture evidence, not final ground truth. Review it
when tape contents changed after the source scan.

One physical feeder registration must stay entirely in the evaluation split.
Do not split repeated views of one strip or pocket between training and
evaluation. The exporter writes raw rectified crops, a provenance manifest,
and `review.html`. Draft labels are not ground truth.
Open the review page and mark each sample as accepted, needs adjustment, empty,
on its edge, or rejected. Keep evaluation labels out of model fitting and
threshold tuning. An on-edge component is a labeled non-pickable pose. Reject
only an unusable image, such as a blurred or incorrect crop.

The OBB represents presence, center, extent, and an undirected package axis.
It cannot distinguish rotations that differ by 180 degrees. A later directed
package dataset must add a polarity keypoint or a separate orientation label.
Model confidence is not a ground-truth field. Calibrate it only against the
accepted evaluation set.

Copy the downloaded first review to `review-decisions.json` in the dataset
directory. Then make learned correction proposals:

```bash
cargo run -p velocitypnpd --example refine_source_component_dataset -- \
  datasets/source-components-v1
```

The refinement keeps accepted labels and empty images unchanged. For each
known part, it learns the median box extent from accepted examples and from
verified carried ground truth. It applies that extent at the detected center
of each flat part that needs adjustment. It does not treat this weak label as
final ground truth. Open
`review-refined.html` to accept or edit each proposal. Drag a box to move it,
drag a square handle to resize it, and drag the round handle to rotate it.
On-edge parts keep a separate class and require explicit review.

Save the second review, and then promote the reviewed labels:

```bash
cargo run -p velocitypnpd --example finalize_source_component_dataset -- \
  datasets/source-components-v1 review-refined-decisions.json
```

A relative review filename is resolved inside the dataset directory. An
absolute path can select a downloaded review file elsewhere.

The finalizer validates the review set, class names, image bounds, OBB
rectangularity, and point order. It writes the result and split assignments to
the provenance-bearing `manifest.ground-truth.json`. Empty images have no
outline. Rejected images stay in the manifest as excluded samples.

For an independent feeder check, export one new source-capture run by itself:

```bash
cargo run --release -p velocitypnpd \
  --example export_source_component_dataset -- \
  velocitypnp.db datasets/source-components-holdout \
  --capture-run CAPTURE_UUID
```

This mode puts every image in the evaluation split and gives the capture its
own group ID. It does not include older images, even when the feeder
registration was used in training. Review the images as above. Finish the
review with `--eval-only`:

```bash
cargo run --release -p velocitypnpd \
  --example finalize_source_component_dataset -- \
  datasets/source-components-holdout review-refined-decisions.json \
  --eval-only
```

Do not use an evaluation-only dataset for model fitting.

After the holdout has been scored, it can enter a later training set. Move a
different, reviewed feeder group to evaluation so marked flat, empty, and
on-edge pockets are checked. For the September 2026 retrain:

```bash
python3 python/tools/build_source_component_retrain_dataset.py \
  datasets/source-components-v3 \
  datasets/source-components-holdout-2026-09-20 \
  datasets/source-components-v4 \
  --eval-group 7caf2f45-0b4f-43eb-9dce-e2413ee99481
```

The builder records source hashes, keeps capture groups in one split, and
rejects exact image copies across splits. The former holdout is no longer an
independent test. Capture a new one before deployment.

### Source-component model training

VelocityPnP uses its own compact PyTorch model for pocket occupancy and
oriented component pose. It has no pretrained parameters. The source,
checkpoint, and exported model use Apache-2.0 terms. PyTorch uses BSD-3-Clause
terms. The model reads reviewed outlines directly from
`manifest.ground-truth.json`.

Create a CPU-only environment and validate the finalized dataset:

```bash
sudo apt-get install python3-venv
python3 -m venv .venv-training
.venv-training/bin/python -m pip install \
  -r python/training-requirements.txt
.venv-training/bin/python python/tools/train_source_component_pose.py \
  datasets/source-components-v2 --validate-only
```

Start the reproducible baseline run:

```bash
.venv-training/bin/python python/tools/train_source_component_pose.py \
  datasets/source-components-v2
```

The default run uses a 192-pixel input, batch size 16, seed 0, deterministic
training, and early stopping. Limited translation, scale, flip, brightness,
contrast, and noise augmentation reflect the installed camera. The best
checkpoint, ONNX model, report, histories, and evaluation images are stored
under the dataset's ignored `training/` directory. The report records dataset,
source, environment, parameter, metric, and artifact hashes. Classifier
calibration gives extra weight to empty pockets that look occupied. It selects
only from checkpoints with the fewest reviewed empty-pocket false detections.
It then uses the combined held-out and training classification error rate.
For a controlled comparison, `--classifier-selection eval-first` ranks
evaluation false positives, then evaluation class errors, then training false
positives. The model manifest records the policy and both false-positive
counts. Neither policy replaces a new independent test before deployment.

The laptop can validate data and run small smoke tests. Use the pinned NVIDIA
PyTorch container on an ARM64 DGX Spark for full runs:

```bash
python/training/run-container.sh \
  datasets/source-components-v2 source-component-pose-v2
```

The container tag is local. Its base image uses a fixed manifest digest.

The evaluation feeder must contain examples for each class that is used as a
production gate. A missing class is reported as an evaluation limitation. Do
not tune a production confidence threshold against this held-out split.

Verify that the ONNX export gives the same values as PyTorch:

```bash
.venv-training/bin/python \
  python/tools/verify_source_component_pose.py \
  datasets/source-components-v2 \
  datasets/source-components-v2/training/source-component-pose-v2/best.pt \
  datasets/source-components-v2/training/source-component-pose-v2/best.onnx
```

The verifier fails for a class change or an output difference above 0.0001.
Training reports empty-pocket false detections, recall, oriented intersection
over union, center error, size error, and axis error. It also writes one
annotated image per sample and an evaluation contact sheet. Versioned baseline
summaries are in `docs/baselines/`. Model weights and full training output stay
with the ignored dataset and are not source artifacts.

### Source-component model runtime

The daemon can load one source-component model package:

```bash
cargo run -p velocitypnpd -- \
  --source-component-model-dir \
  datasets/source-components-v2/training/source-component-pose-v2
```

`VELOCITYPNP_SOURCE_COMPONENT_MODEL_DIR` sets the same option. The package must
contain `model-manifest.json` and its declared ONNX file. Startup checks the v2
input and output contract, class order, ONNX opset, Apache-2.0 artifact terms,
file size, SHA-256 digest, and exact resize contract. Version 1 packages are
rejected because their Pillow resize behavior was not portable. An invalid or
absent package disables learned inference. It does not prevent the daemon from
starting. `/api/v1/health`
reports `disabled`, `invalid`, or `ready` and includes the loaded artifact
digest when ready.

The Rust runtime uses the pinned Apache-2.0/MIT `tract-onnx` 0.22 release. It
runs on CPU and does not require Python, PyTorch, ONNX Runtime, or a GPU on the
machine controller.

Replay the Rust preprocessing and runtime against the accepted evaluation
split:

```bash
cargo run --release -p velocitypnp-model \
  --example evaluate_source_component -- \
  datasets/source-components-v2/training/source-component-pose-v2 \
  datasets/source-components-v2 eval
```

The command exits with an error for each ground-truth class mismatch. The v2
shadow baseline has one low-confidence held-out miss. It is recorded in
`docs/baselines/source-component-pose-v2.json` and is not a production gate.
The command also reports mean decoded center error in source-image pixels.

New inventory source-capture samples run both the deterministic detector and
the learned model. Each sample keeps the decoded prediction, model digest,
detector agreement, center difference, and review decision. It also retains a
rectified crop, JSON report, and PNG comparison. Green marks show the
deterministic outline. Cyan marks show the learned outline. A magenta frame
marks a sample that requires review. A conflict with the inventory result that
selected the pocket always requires review. Existing retained captures can be
processed without motion:

```text
POST /api/v1/strip-feeder-source-captures/CAPTURE_UUID/reanalyze-model
```

The Python client method is
`reanalyze_strip_feeder_source_capture_model()`. Inventory analysis remains in
shadow mode and sets `control_authorized` to false. Model output cannot select
a pocket or change inventory.

The guarded pickup path can authorize one narrower result: the center of a
freshly observed, high-confidence `component_flat` prediction in a pocket that
the live holdout already marked occupied. The daemon maps the rectified crop
center to calibrated machine XY and bounds both its nominal-pocket offset and
its difference from the deterministic center. A high-confidence `empty` or
`component_on_edge` result stops before nozzle descent. An inconclusive model
uses the bounded deterministic center. The retained JSON report records
whether the learned center controlled that pickup. The model runs before the
legacy source-pose comparison. A confident flat-part result controls the
pickup while the legacy result remains audit evidence. A confident `empty`
result settles the reserved pocket as empty only when deterministic CV also
reports `empty`. A detector disagreement stops motion and keeps the pocket
under review. An agreed empty result selects the next available pocket and
synchronizes the active feeder count. Held-part pose results remain in shadow
mode outside the strict post-release bare-nozzle gate.

For a later capture session, carry reviewed labels into a new dataset without
showing them again:

```bash
cargo run -p velocitypnpd --example export_source_component_dataset -- \
  velocitypnp.db datasets/source-components-v2 EVAL_REGISTRATION_UUID \
  --carry-reviewed datasets/source-components-v1
```

The exporter carries a label only when the source ID, source SHA-256 digest,
and generated image bytes all match. It writes verified labels to
`manifest.carried-ground-truth.json`. The draft manifest and `review.html`
contain only new samples. The finalizer checks the carried image hashes and
combines the carried and newly reviewed labels in the final dataset.

New live-pocket holdouts retain one unannotated frame from each camera pose as
`source-pocket-frame-NNN.png`. The holdout report records the matching camera
poses. The four offset poses measure tape geometry and can show only part of a
neighboring pocket. The dataset exporter uses only the pose nearest the
requested pocket as a component sample. It also rejects captures without a
three-hole sprocket lattice. An explicit pocket index can therefore collect
occupied, empty, edge, and difficult examples without lowering a nozzle.

### Held-part learning dataset

Retained pickup and placement runs contain a bare-nozzle reference and one or
more bottom-camera observations. Export these pairs without machine motion:

```bash
cargo run --release -p velocitypnpd \
  --example export_held_part_dataset -- \
  velocitypnp.db profiles/lumenpnp-v3.0.2.json \
  datasets/held-parts-v1 EVAL_REGISTRATION_UUID
```

All observations from one feeder registration stay in one split. Older
records without a live registration stay grouped by their retained feeder
identity. Supply at least one registration for evaluation. The current-machine
evaluation split is provisional. A production gate also needs independent
machine sessions and more component families.

For a later capture session, carry unchanged reviewed labels into a new
dataset. The exporter checks both retained source hashes and both generated
crop hashes before it carries a label:

```bash
cargo run --release -p velocitypnpd \
  --example export_held_part_dataset -- \
  velocitypnp.db profiles/lumenpnp-v3.0.2.json \
  datasets/held-parts-v2 EVAL_REGISTRATION_UUID \
  --carry-reviewed datasets/held-parts-v1
```

The new draft manifest and `review.html` contain only new or changed pairs.
The finalizer validates `manifest.carried-ground-truth.json` and combines its
labels with the newly reviewed labels. It preserves the prior split and feeder
group for carried samples, even if later machine setup changed an active
feeder link.

The exporter stores 512-pixel bare and observed crops. It includes initial
held views, corrected held views, and every retained post-release view. The
current deterministic detector supplies a weak oriented-box proposal. Vacuum,
release, and stored vision outcomes are review evidence only.

Open `review.html`. It shows the bare reference, observed image, and an
amplified absolute difference. For each pair, select one result:

- `component_flat`
- `component_on_edge`
- `multiple_components`
- bare nozzle
- rejected image pair

Drag the green box, corner handles, or rotation handle as needed. The
multiple-component class uses one enclosing box. For a directed part, mark a
visible polarity end with the magenta endpoint. Use `Flip end` when the part
faces opposite the draft direction, such as a part that was returned to its
tape in reverse. Use `not visible` when the part is present but the image
cannot resolve its end. Do not guess.

Download `review-decisions.json`. The finalizer checks that every sample was
reviewed, both image hashes still match, all boxes are rectangular and inside
the image, and directed endpoints are valid:

```bash
cargo run -p velocitypnpd --example finalize_held_part_dataset -- \
  datasets/held-parts-v1 /path/to/review-decisions.json
```

The result is `manifest.ground-truth.json`. Empty pairs remain included
negative samples. Rejected pairs remain excluded provenance records. This
dataset does not authorize motion or replace vacuum and deterministic safety
evidence.

Validate the reviewed dataset before training:

```bash
python python/tools/train_held_part_pose.py \
  datasets/held-parts-v1 --validate-only
```

The held-part model takes the bare-nozzle and observed images as one six-channel
input. It predicts bare, flat, or on-edge state, one oriented package box, and
an undirected package axis. Its rotation repeats after 180 degrees. Reviewed
directed-end labels remain in the dataset for future work, but v1 does not use
or output them. This first contract cannot detect multiple components because
the current dataset has no examples.

Train on an NVIDIA system with the pinned container:

```bash
python/training/run-held-part-container.sh \
  datasets/held-parts-v1 held-part-pose-v1
```

The trainer writes `training-state.pt` after each epoch with an atomic rename.
If the host, container, or GPU stops before the model package is complete,
resume the same run:

```bash
python/training/run-held-part-container.sh \
  datasets/held-parts-v1 held-part-pose-v1 --resume
```

Resume checks the dataset digest, training source, and training parameters. A
successful run removes the temporary state after it writes the model manifest.

Training rotates only the reviewed component and its local shadow. The bare
nozzle image and fixed camera features do not rotate. This prevents the model
from learning an angle from the four bottom-camera mounting bolts. Inspect the
synthetic samples before each training run:

```bash
python python/tools/render_held_part_rotation_augmentation.py \
  datasets/held-parts-v1 \
  /tmp/velocitypnp-held-part-component-rotation.png
```

Checkpoint selection also uses fixed synthetic angles at 15-degree steps from
-75 through +90 degrees, with zero excluded. This check can find an
angle-collapsed model, but it is not an independent machine evaluation. Real
captures at commanded rotations are still required before the pose result can
control placement.

The run writes a checkpoint, ONNX export, metrics, provenance, and evaluation
contact sheets below `datasets/held-parts-v1/training/held-part-pose-v1`.
Keep the result in shadow mode. The current four-image evaluation split is not
a production gate.

### Held-part model runtime

The daemon can load one held-part model package:

```bash
cargo run -p velocitypnpd -- \
  --held-part-model-dir \
  datasets/held-parts-v1/training/held-part-pose-v1
```

The `VELOCITYPNP_HELD_PART_MODEL_DIR` environment variable sets the same
path. The package must contain `model-manifest.json` and its declared ONNX
file. Startup checks the v1 contract, class order, six-channel input, portable
resize rule, undirected angle, Apache-2.0 artifact terms, file size, and
SHA-256 digest. `/api/v1/health` reports the load state and model digest.

The model runs beside each deterministic bottom-camera held-part inspection.
It compares bare-nozzle and observed 512-pixel crops. Each run stores the
prediction, both detector results, confidence, model digest, disagreements,
and an annotated preview. Green is deterministic, cyan is learned, and a
magenta border requires review.

Normal held-part detection and pose results stay in shadow mode. Release
verification has one narrow exception. A learned `bare_nozzle` result can
confirm a clear nozzle when its class probability is at least 0.98, its class
margin is at least 0.95, and the vacuum sensor also confirms release. The
report keeps any disagreement with deterministic CV for later review.

Run the Rust runtime against all reviewed pairs and make one comparison sheet:

```bash
cargo run --release -p velocitypnpd \
  --example evaluate_held_part_shadow -- \
  datasets/held-parts-v1 \
  datasets/held-parts-v1/training/held-part-pose-v1 \
  /tmp/velocitypnp-held-part-cv-vs-ml.png \
  /tmp/velocitypnp-held-part-cv-vs-ml.json
```

Each sample has a deterministic panel on the left and a learned panel on the
right. Magenta is reviewed ground truth. Green is deterministic. Cyan is
learned. A red border marks an incorrect panel. The JSON file maps every sheet
index to its source ID and keeps all decoded values.

Classification is not accepted job registration: this no-motion stage does not
create a `BoardInstance`, move the camera, resolve a symmetric outline, or
accept a design-to-machine transform. Use the local fiducial workflow below.
Package-template verification remains a later gate.

### Strip-feeder vision drafts

Analyze a semantic feeder proposal from its retained normalized crop:

```bash
cargo run -p velocitypnp -- analyze-strip-feeder SCAN_UUID --candidate 0
```

The API route is
`POST /workspace-scans/SCAN_UUID/strip-feeder-vision`. The Python client uses
`analyze_strip_feeder()`. This operation causes no motion. It rejects known
fixtures. It accepts a feeder classification, or an unknown staging-plate
candidate that has feeder evidence. The specialist must still find a carrier
tape lattice before a usable draft can be stored.

OpenCV contour measurements find 1.5 mm sprocket-hole candidates. At least
five features must fit the 4 mm carrier-tape lattice during workspace
analysis. This rejects short plate-hole alignments. The live holdout uses its
separate, bounded camera span. The crop basis comes from the installed plate,
so tape axes stay parallel or perpendicular to the calibrated plate axes.
Neighboring sprocket rows describe feeder placement; they do not describe tape
width. The detector tests component evidence at the EIA-481-C `F` offsets for
standard 8, 12, 16, and 24 mm tape. It selects the common bank direction from
the strongest lanes.

Rectangular component contours estimate visible part size. A constrained
lattice fit selects a 2, 4, 8, 12, or 16 mm component pitch while penalizing
unsupported subharmonics. Lane-local brightness classes, anchored by measured
component contours, classify each observed feed position as occupied, empty,
or uncertain. Counts cover only the visible tape extent. Part size is a
visible-outline estimate, not a package or part-identity result.

Version 5 evaluates one phase-correct pocket beyond each terminal sprocket
range. It accepts the endpoint only when its image patch matches the measured
lane and remains far enough from the crop edge. This fixes the common
half-pitch undercount without changing the fitted pitch. A weak endpoint stays
outside the count.

Version 6 uses a stronger correlation gate for the first extra pocket. At the
lower image boundary, it can accept one more phase-correct pocket when the
feeder body hides the terminal sprocket but the component-pocket patch remains
visible. This extension is limited to one additional slot and requires direct
image evidence.

Occupancy uses three lane-local checks after contour detection. Strong luma
transitions can reject repeated embossed pocket outlines. A homogeneous lane
with repeated component evidence can recover weak component contours. Bright
tape with small dark parts uses a fine central edge profile. A weak transition
is `uncertain`; the detector does not force it to present or absent.

The physical eight-feeder labels are retained in
`crates/vision/tests/fixtures/lumenpnp-v3.0.2-strip-feeder-bank-ground-truth.json`.
The fixture README gives the no-motion replay command. The evaluator aligns a
consecutive detected range with each full labeled lane and fails if a
confident occupancy label disagrees.

Circular component pockets can also form a 4 mm lattice. The detector pairs
two candidate rows that are 3.5 mm apart. Both rows cannot be sprocket rows because
the separation is less than the width of standard carrier tape. The stronger
row stays as the sprocket row. The weaker row becomes rejected pocket evidence
and gives the exact pickup side for 8 mm tape. Unpaired or ambiguous rows stay
in review state.

The scan retains `strip-feeder-vision-NNN.json` and
`strip-feeder-vision-NNN.png`. Yellow circles show sprocket candidates. Cyan
lines show pickup-centerline hypotheses. Green boxes show detected components.
Red marks show absent feed positions and rejected component-pocket lattices.
The JSON report keeps component outlines, size, pitch, occupancy, counts, and
rejected-row evidence. The web workspace card exposes the same analysis.

Combine overlapping focused scans on one workspace mosaic:

```bash
cargo run -p velocitypnp -- build-strip-feeder-inventory WORKSPACE_SCAN \
  --source LEFT_SCAN:0 --source CENTER_SCAN:0 --source RIGHT_SCAN:0
```

The API route is
`POST /workspace-scans/SCAN_UUID/strip-feeder-inventory`. The Python client
uses `build_strip_feeder_inventory()`. Compatible source scans must use the
same profile and calibration. Each feeder report must trace to a
height-corrected mosaic. If the target scan has completed workspace discovery,
the operation starts with `mosaic-board-candidates.png`. Thus, detected
workspace elements, the fitted PCB outline, KiCad placement marks, and KiCad
fiducials remain in the full-workspace image. It replaces only the inside of
each strip with corrected feeder pixels, merges observations from overlapping
or adjacent scans, and stores `strip-feeder-inventory.json` and
`mosaic-strip-feeder-inventory.png` on the workspace scan. Cyan polygons mark
strip bounds. Green polygons mark present component outlines. Red marks show
absent feed positions; amber marks show uncertain positions. A lane that ends
within two sprocket pitches of a source-image edge is marked clipped. Its
counts are lower bounds, and the report gives a region for an extension scan.
This operation causes no motion.

A measured lane can be stored as an inactive draft:

```bash
cargo run -p velocitypnp -- save-strip-feeder-draft SCAN_UUID \
  --candidate 0 --proposal 0 \
  --part-key 'R_0603:10k' --part-rotation 90
```

The corresponding API is `POST /strip-feeder-registrations`. The Python
client uses `create_strip_feeder_draft()`. A draft records source lineage,
tape geometry, pocket hypotheses, optional part identity, and optional part
orientation. It does not create an active `Feeder`.

Run a fresh top-camera holdout after coarse and fine XYZ home:

```bash
cargo run --release -p velocitypnp -- live-strip-feeder-holdout \
  DRAFT_UUID --lease LEASE_UUID --confirm-safety-checks
```

The API route is
`POST /strip-feeder-registrations/DRAFT_UUID/live-holdouts`. The Python client
uses `start_strip_feeder_live_holdout()`. The machine visual-homes, then makes
a five-pose safe-Z sweep across the middle pocket and both possible sprocket
rows. The two views on each sprocket row are 4 mm before and after the predicted
pocket. It captures three locked top-camera frames at each pose. This gives
about 14 mm of combined tape coverage despite the narrow calibrated top-camera
field. The vision worker rectifies all captures into the draft tape basis with
the calibrated planar camera model. It then checks the 4 mm sprocket lattice,
local line angle, pocket phase, bounded XY correction, and occupied-component
image support.

Version 3 derives the stored pocket center and correction only from the
sprocket line, tape centerline, and periodic pocket phase. A loose component
can move inside its pocket, and image clutter can move its foreground
centroid. Neither effect is tape geometry. Foreground support still selects an
occupied pocket. If the requested pocket has strong occupied evidence, it is
kept even when a visible neighbor has a stronger response. A neighbor is used
only when the requested pocket is not occupied. The pickup operation measures
the component center again in a fresh top-camera image immediately before
nozzle motion. When the API request names a pocket index, a neighbor cannot
satisfy that request. A neighbor selection fails the explicit holdout and
cannot change the named pocket's persistent state.

The persistent report gives predicted and measured machine coordinates. Its
PNG marks expected sprocket lines in yellow, the expected pocket centerline in
cyan, the measured sprocket line in magenta, detected holes in yellow, and
geometry-derived pocket centers in green. A passed result selects a measured
occupied pocket in the draft. It does not lower a nozzle, index tape, measure
pickup Z, or create an active `Feeder`.

Qualify pickup Z, vacuum hold, bottom vision, and release after a passed live
holdout:

```bash
cargo run --release -p velocitypnp -- qualify-strip-feeder-pickup \
  DRAFT_UUID --lease LEASE_UUID --nozzle N1 \
  --drop-x FALLBACK_X --drop-y FALLBACK_Y --confirm-safety-checks
```

Add a bounded real-angle dataset sweep to the same safe pickup and return:

```bash
cargo run --release -p velocitypnp -- qualify-strip-feeder-pickup \
  DRAFT_UUID --lease LEASE_UUID --nozzle N1 \
  --rotation-capture-offset -75 -45 -15 15 45 75 \
  --confirm-safety-checks
```

The runner first uses a fresh top-camera frame to reject an empty or unsafe
source pocket. It then captures the bare nozzle at every requested angle.
After pickup, it rotates the held part through the same angles and captures
each matching view. Fixed camera features therefore stay fixed, while every
model input uses a bare-nozzle reference at the correct nozzle angle. The
runner checks vacuum before each held capture, permits no more than 12 unique
angles, restores the original nozzle angle, and uses the normal verified
pocket-return path. A lost vacuum hold stops the sweep and starts recovery.

The report records requested and actual angles, vacuum samples, deterministic
inspection, optional learned shadow output, and artifact names. The held-part
dataset exporter pairs each rotation view with its matching bare image and
keeps the angle metadata through final review.

The API route is
`POST /strip-feeder-registrations/DRAFT_UUID/pickup-qualifications`. The
Python client uses `start_strip_feeder_pickup_qualification()`. The machine
rechecks fine X/Y home and the installed nozzle tip. It captures a centered
bare-nozzle bottom-camera reference, moves to the measured occupied pocket at
level Z, and starts vacuum before it descends. The descent starts 2.0 mm above
the stored datum-board contact and stops 0.2 mm above it. Vacuum transition
and the calibrated acquire threshold stop the descent.

The runner lifts the part to level Z, transfers it to the exact bare-nozzle
bottom-camera pose, and checks both retained vacuum and a frame-difference
part outline. It applies at most two bounded X/Y and nozzle-angle corrections.
Each correction gets a new camera measurement. The final gate requires a
0.10 mm center residual and a 2 degree package-rotation residual. The runner
then restores the pickup nozzle angle, returns to the exact source head pose,
and releases at the detected pickup Z. A supplied fallback point is used only
when the source return target is invalid.

Every production part needs an orientation policy. `axis_symmetric` treats
angles 180 degrees apart as equivalent. This is suitable for non-polar
two-terminal parts. `feeder_referenced` keeps the full angle assigned during
feeder registration and resolves the measured package axis against that
source angle. `brighter_end` and `darker_end` measure both ends of the held
package. They require a configured luma separation and can apply a package
zero-angle offset. Weak end contrast stops before rotation or board motion.

For example, a package whose darker end defines its zero-degree direction can
use this part field:

```json
{
  "orientation": {
    "mode": "darker_end",
    "minimum_contrast_luma": 8.0,
    "rotation_offset_deg": 0.0
  }
}
```

The older `rectangular_passive` vision profile maps to `axis_symmetric` for
stored part compatibility. New part records must provide an explicit policy.
Job preflight rejects a part with no valid orientation policy. A change to the
part policy or feeder source angle also makes older pickup-angle evidence
ineligible for placement.

After release, the runner moves the nozzle to the same bottom-camera pose used
for its bare reference. Any detected component means that release is not
complete, even when the vacuum sensor reads its off state. The runner returns
to the same source pose and retries once. It then captures a second
bottom-camera view. Two failed attempts leave the disposition as
`not_released` and quarantine the pocket.

The top camera records the pocket before pickup and after release. Version 3
measures the package center in each view and compares a package-sized image
patch before and after pickup. The cyan cross is the nominal pickup coordinate.
The green circle is the measured package center. A failed appearance match is
red. A passed return requires occupied evidence in both views, less than one
quarter pitch of measured center shift, and at least 0.55 normalized package
appearance similarity. This rejects an exposed pocket floor that has strong
edges but no returned package.

The retained report includes pickup samples, held and release samples,
bottom-camera measurements, alignment observations, return evidence,
disposition, and PNG artifacts. Use `strip_feeder_return_preview()` to get the
before/after overlay from Python. Recheck retained evidence after a vision
update without machine motion:

```bash
cargo run --release -p velocitypnp -- reanalyze-strip-feeder-return \
  QUALIFICATION_UUID
```

The API route is
`POST /strip-feeder-pickup-qualifications/QUALIFICATION_UUID/reanalyze-return`.
The Python client uses `reanalyze_strip_feeder_return()`. A rejected result
marks the source pocket as uncertain and needing review. It also removes an
active feeder that depends on the rejected qualification.

Run a short qualification on three to five distinct occupied pockets:

```bash
cargo run --release -p velocitypnp -- qualify-strip-feeder-sequence \
  DRAFT_UUID --lease LEASE_UUID --cycles 3 --nozzle N1 \
  --confirm-safety-checks
```

The Python client uses `qualify_strip_feeder_sequence()`. Each cycle is a
normal persistent pickup qualification with its own operation and artifacts.
The client chooses distinct available occupied pockets and stops on the first
failure. The API also accepts `pocket_index` on an individual pickup request,
so another headless client can implement the same bounded policy. The daemon
applies the current live-holdout correction to the fitted pocket lattice.
For an active feeder, a passed holdout also moves the planned pick pose to the
measured occupied pocket. This keeps the planner away from a quarantined or
empty prior source.
Three passed returns at distinct pockets make the feeder ready. One passed
return creates a provisional feeder. One successful guarded placement with
top inspection also makes that feeder ready. This second path avoids repeated
returns for small parts that do not settle reliably in carrier tape.

Held-part inspection aligns the bare-nozzle reference within an 8 pixel bound
before subtraction. It selects the brighter or darker foreground polarity
from the nozzle-axis region. Closing joins a package body, and opening removes
narrow nozzle reflections. An axis-support trim removes a narrow reflection
that remains connected to a package edge. A minimum filled-area gate rejects
circular nozzle rings. The report uses the oriented foreground bounds for
width, length, rotation, and center offset. In the overlay, green follows the
measured pixel contour, cyan shows the oriented measurement box, and orange
marks the nozzle axis.

Replay two retained bottom-camera frames without machine motion:

```bash
cargo run --release -p velocitypnp-vision --example inspect_held_part -- \
  BARE.png HELD.png OUTPUT.png CENTER_X CENTER_Y XX XY YX YY
```

The last four values are the calibrated bottom-camera millimeters-per-pixel
matrix in row-major order. The command writes the inspection report to
standard output and writes the annotated image to `OUTPUT.png`. Append
`brighter_end`, `darker_end`, or `feeder_referenced`, followed by minimum luma
contrast and rotation offset, to replay directed orientation. The magenta line
points toward the detected package end.

Replay two retained top-camera pocket views without machine motion:

```bash
cargo run --release -p velocitypnp-vision \
  --example inspect_strip_feeder_return -- \
  BEFORE.png AFTER.png MODEL.json REGISTRATION.json OUTPUT.png \
  POCKET_X POCKET_Y POSE_X POSE_Y POSE_Z POSE_ROTATION
```

A draft stores each feed position as available, empty, reserved, consumed, or
in need of review. Reservation and settlement are atomic. Selection skips
known empty and consumed pockets. A returned part becomes available only after
top vision confirms it. Consumption requires vacuum acquisition, retained
vacuum, and passed bottom vision. A failed operation can retry twice. A third
failure or an uncertain release requires review. A daemon restart quarantines
its reserved pocket.

A passed qualification stores pickup Z and measured part width, length,
rotation, and confidence. It creates a provisional strip `Feeder` only when
the draft also has a part key and orientation. The provisional feeder can run
one guarded, inspected placement. Passive strip advancement is a logical
next-pocket operation; it does not move the tape.

### Guarded single-placement execution

Execute one job-plan step in the simulator or on gated hardware:

```bash
cargo run --release -p velocitypnp -- execute-single-placement JOB_UUID \
  --lease LEASE_UUID --calibration CALIBRATION_UUID --step 0 \
  --confirm-safety-checks
```

The API route is `POST /jobs/JOB_UUID/placement-executions`. The Python client
uses `start_guarded_single_placement()`. The executor requires a homed machine,
a current local direction-response calibration, a registered board transform
and measured surface Z, a part-library height, and an active strip feeder. A
provisional feeder needs one passed pickup-return cycle with the planned
nozzle. Its first passed top-inspected placement promotes it. A feeder can
also become ready through three distinct passed pickup-return cycles. Set a
job's `placement_references` list to run a deliberate subset of board parts.

The plan carries the feeder's directed part angle separately from the KiCad
board-placement angle. Bottom vision preserves or measures the 180-degree
phase, calculates a full-angle residual for directed parts, and rotates the
nozzle through the shorter path. The V3 profile permits the complete
180-degree correction range. Its periodic runout model was measured at
30-degree intervals from -165 through +165 degrees and remains defined at the
shared -180/+180 endpoint.

A physical placement binds its reservation to the available strip pocket at
the planned pick position. It then captures that pocket with the top camera
and uses the measured component center for the pickup move. Source-pose
version 3 first detects a package-sized rectangular edge. If glare breaks the
edge, it finds the dark package core within a region bounded by the surveyed
package size and measures the current core bounds. The report retains the four
machine-space outline corners.
The preview uses a cyan cross for the nominal pocket center, a green rectangle
for the component outline, a magenta circle for the component center, and a
yellow cross for the nozzle target. It also compares
the source package with all retained source images from the feeder's successful
pickup sequence. This legacy comparison remains audit evidence when a
confident flat-part model result controls pickup. Otherwise, its best match
must pass occupancy, center, and appearance checks. This rejects an
edge-standing package before nozzle descent without a fixed package-luma rule.
The report and
`source-pocket-pose-inspection.png` identify the selected successful reference.
A model-reported missing or edge-standing part, a rejected fallback source
pose, or an excessive center correction stops before nozzle descent.
Use
`POST /placement-executions/EXECUTION_UUID/reanalyze-source-pose` to apply the
current source-pose check to a retained attempt without machine motion. The
Python client method is `reanalyze_placement_source_pose()`.

The ordinary executor also rejects a board reference after an earlier physical
run reached its placement pose.
This prevents a failed release or inspection from causing a second component
to be stacked on the same target. Rework needs a separate explicit workflow.

Before pickup, guarded placement also registers the bare local two-terminal
pad pair. It stores `board-target-registration.png` and applies the bounded
local XY correction to the placement pose. Connected-component detection is
the primary path. If adhesive glare connects the pads to nearby artwork, a
fallback requires two bright, balanced windows with the expected package
axis, spacing, and local contrast. A missing or ambiguous pad pair stops the
operation before pickup. The retained report keeps the nominal board pose and
records the local correction separately.

If the nominal pickup center produces no vacuum transition, version 10 can make
one bounded XY retry. It requires at least two recent, passed top inspections
for the same job, nozzle, and part. Their residuals must agree within 0.20 mm.
The retry applies the opposite mean residual and rejects a correction below
0.05 mm or above 0.45 mm. A weak vacuum transition does not retry. Each descent,
sensor series, head position, and correction is stored in `pickup_attempts`.
This provides a narrow recovery for measured nozzle-position residuals without
turning pickup into an unbounded search.

Bottom alignment places from its last observed pose. Each commanded
translation is followed by another bottom-camera capture. The final measured
residual is accepted only inside the configured tolerance; it is not added as
an unverified move. The live tip offset includes the installed tip and its
current runout state. Rotation-induced runout is present in the verified
bottom alignment translation. The height transform separately applies the
calibrated nozzle-axis tilt and working-plane transfer at placement Z.

The persistent run records preflight, live pickup, bottom alignment, calibrated
placement motion, bounded release verification, and top inspection. Top
inspection registers the retained before and after board views. It then matches
the part-library body dimensions near the planned pose and angle. Package text
can provide change evidence, but it cannot define the detected body. Use
an end-marked orientation policy to compare the two visible package ends after
the geometry match. A directed placement fails when the marker is ambiguous or
is 180 degrees from the planned board direction. A feeder-referenced policy
preserves motion phase but cannot prove that phase in the final image. Use
`POST /placement-executions/EXECUTION_UUID/reanalyze-placement` or the Python
client's `reanalyze_placement()` method to apply the current detector to saved
evidence without machine motion.

Every physical run that reaches board contact retains
`board-placement-after.png`, including a failed or uncertain release. A failed
release does not claim a passed top inspection, but its raw top-camera image
remains available for offline analysis. The runner also keeps the raw
`held-part.png` bottom-camera image for every acquired part.

Physical success consumes the reserved feeder pocket only after release
evidence. A failure before confirmed release tries to return the held part to
its source pocket and quarantines the reservation. The simulator does not
consume a pocket and does not claim live camera evidence. Job preflight requires
part height, body width, and body length.

Guarded placement can use a bounded shallow-to-deep release schedule. Each
attempt turns off the pump, pulses blowoff, retracts, reads the vacuum sensor,
and captures the nozzle with the bottom camera. Confirmed transfer requires
both a released vacuum signal and a clear-nozzle image. Before a deeper
attempt, the runner must see the retained part, restore vacuum hold, and pass a
fresh bottom-camera alignment. Ambiguous evidence, failed hold, failed
alignment, or the end of the range stops the sequence. Every attempt keeps its
depth, sensor samples, alignment report, and image names.

Set a job-wide range with `placement_release`:

```json
{
  "placement_release": {
    "start_seating_offset_mm": -0.2,
    "minimum_seating_offset_mm": -0.4,
    "step_mm": 0.1
  }
}
```

A part-level schedule overrides the job schedule. A KiCad placement property
named `Placement Seating Offset` overrides the schedule start for that one
reference. Preflight expands no more than ten depths and checks all of them
against the calibrated height and controller Z models.

Retained-part recovery can set an explicit `placement_seating_offset_mm` from
-0.50 to +0.50 mm for a bounded diagnostic release. The override becomes the
first attempt and removes shallower configured attempts. The -0.50 mm lower
bound still applies.
An optional `target_placement_reference` can redirect the held part to a
confirmed empty job target only when the part, nozzle, feeder, and board are
the same. The recovery report records both overrides.

The planned nozzle-contact Z keeps four terms separate:

```text
measured board surface
+ board-instance placement_surface_offset_mm
+ nominal part height_mm
+ resolved placement_seating_offset_mm
```

Use the board-instance surface offset for a known layer above the measured
datum, such as tape or paste. Use the part seating offset only as a process
adjustment. A positive seating offset stops the nozzle higher and reduces
compression. The part default can be from -0.25 to +0.50 mm. A KiCad placement
property named `Placement Seating Offset` overrides that default for one
reference. A job-level `placement_seating_offset_mm` overrides the part default
when no release schedule is set. Preflight records every term and every release
depth in the plan. The physical report records the shallowest successful depth.

Each physical execution also creates a persistent placement outcome. The
automatic disposition is one of `good`, `offset`, `rotated`, `tombstoned`,
`stacked`, `missing`, `stuck_on_nozzle`, or `uncertain`. A failed release with
bottom-camera retained-part evidence is `stuck_on_nozzle`. An operator review
overrides the automatic class but does not remove the original result:

```bash
cargo run --release -p velocitypnp -- review-placement-outcome EXECUTION_UUID \
  good --note "operator confirmed"
```

Use `placement-process PART_UUID --nozzle N1 --surface-offset 0.0` to group
outcomes by part, nozzle, board-surface offset, and seating offset. A
recommendation requires at least three conclusive results at one offset and an
80 percent pass rate. One update is limited to 0.05 mm and remains a
recommendation; it does not silently change part data.

Run a bounded series through the same guarded executor:

```bash
cargo run --release -p velocitypnp -- qualify-placement-sequence JOB_UUID \
  --lease LEASE_UUID --calibration CALIBRATION_UUID \
  --step 0 1 2 --confirm-safety-checks
```

The series accepts one to five distinct steps for one part, nozzle, release
schedule, and board-surface offset. It records each execution and outcome and
stops after the first failed operation or non-good automatic disposition.

Run a complete job with daemon-owned, persistent state:

```bash
cargo run --release -p velocitypnp -- execute-job JOB_UUID \
  --lease LEASE_UUID --calibration CALIBRATION_UUID \
  --confirm-safety-checks
cargo run --release -p velocitypnp -- job-execution EXECUTION_UUID
```

Use `pause-job-execution` or `cancel-job-execution` for cooperative control.
These commands do not use emergency stop and take effect between placements.
After a failure, use `resolve-job-step EXECUTION_UUID STEP retry` only if the
step did not reach board contact. Use `skip --note "..."` to record an explicit
operator recovery. Then use `resume-job-execution` with a new lease and current
fine XYZ home. The daemon pauses active jobs after restart and retains all
placement attempt IDs and evidence.

Before nozzle descent, source-pose vision compares the fresh pocket image with
retained successful pickup-return images. Directed parts must match the known
face texture. Axis-symmetric parts can also pass from a package rectangle that
matches the registered part size. This permits a passive part with its unmarked
face up, but rejects the narrow geometry of a part that stands on its edge.

### Placement-verification learning dataset

Export a review snapshot from retained top-camera captures after the board is
registered. The capture manifest names the active board registration, known
placed references, camera machine poses, and source frames. The exporter treats
all other selected board placements as empty. Optional `references` or
`reference_prefixes` fields limit a capture to fully covered targets:

```bash
CAPTURE=datasets/placement-verification-v1/captures/REGISTRATION_UUID
python python/tools/export_placement_verification_snapshot.py \
  velocitypnp.db \
  "$CAPTURE/capture-manifest.json" \
  datasets/placement-verification-v1/review-current
```

The exporter uses the registered KiCad transform, calibrated top-camera map,
and configured part dimensions. It averages repeated source frames. Each
output crop covers a 4 mm square, centers the KiCad target, and puts the
expected package long axis on the image X axis. This makes board rotation and
camera pose independent of the training label.

Retained physical placement executions provide paired evidence without new
machine motion. Export all usable before-contact and after-contact images with:

```bash
python python/tools/export_placement_execution_dataset.py \
  velocitypnp.db \
  datasets/placement-verification-v2/review-executions
```

The exporter uses the same target-normalized crop as the daemon. It keeps each
before/after pair in one source group and records unusable executions in the
draft manifest. Outcome records provide weak label proposals only. Review all
states and outlines before finalization. A `stacked` outcome proposes one flat
part before contact and multiple parts after contact.

After review and finalization, merge the new execution data with the earlier
reviewed survey:

```bash
python python/tools/merge_placement_verification_datasets.py \
  datasets/placement-verification-v2/merged \
  datasets/placement-verification-v1/review-corrected \
  datasets/placement-verification-v2/review-executions
```

The merge keeps source capture groups together. It also keeps all images for
one board reference in one split, including images from different sessions.
Evaluation membership propagates through these links. This prevents a view of
the same physical target from becoming a near-copy across both splits.

Training augmentation uses small scale and crop-registration changes, target-
axis-preserving flips, bounded brightness and contrast, mild gamma and focus
changes, and sensor noise. Do not rotate the complete crop by an arbitrary
angle. The crop is already aligned to the expected package axis, and a complete
rotation also rotates board pads and text. Component-only pose augmentation can
be added after reviewed outlines provide reliable masks and the generated edge
transitions pass visual review.

Snapshots made before the placement-angle correction reflected the component
angle through the top-side board transform. Re-export the snapshot. Then move
the reviewed outlines into the corrected crop basis without changing their
operator labels:

```bash
python python/tools/migrate_placement_verification_review.py \
  datasets/placement-verification-v1/review-current/manifest.ground-truth.json \
  datasets/placement-verification-v1/review-corrected/manifest.draft.json \
  datasets/placement-verification-v1/review-corrected/review-decisions.json
```

Open the corrected review page and check the migrated outlines before
training. The migration only transforms existing operator labels. It does not
use a detector to create new ground truth.

`manifest.draft.json` contains operator-supplied presence as a draft only. It
does not make this data ground truth. Open `index.html` and accept, correct, or
reject every sample. A flat-component outline must cover the complete physical
package, including its end terminals. Use `component_on_edge` for a package
that is not flat and `multiple_components` when one target has a stack.
Download the result as `review-decisions.json`.

Finalize the first review while keeping complete source-camera captures in one
split. Repeat `--evaluation-capture` for each held-out capture:

```bash
python python/tools/finalize_placement_verification_dataset.py \
  datasets/placement-verification-v1/review-current \
  /path/to/review-decisions.json \
  --evaluation-capture ring-upper
```

The first model predicts `empty`, `component_flat`, `component_on_edge`, or
`multiple_components` and one undirected package outline. The outline gives
center, extent, and rotation relative to the normalized KiCad target. The
daemon policy can map these measurements to `missing`, `good`, `offset`,
`rotated`, `tombstoned`, or `stacked`. The current guarded policy uses the
`empty` and `component_flat` classes. A class with no reviewed examples remains
in the model contract but is a reported dataset limitation.

Validate the reviewed data without fitting parameters:

```bash
python python/tools/train_placement_verification.py \
  datasets/placement-verification-v1/review-current --validate-only
```

Run the full fit in the pinned NVIDIA container:

```bash
python/training/run-placement-container.sh \
  datasets/placement-verification-v1/review-current \
  placement-verification-v1
```

The model starts without pretrained parameters. Training code and weights use
Apache-2.0 terms. PyTorch uses BSD-3-Clause terms. The training report records
exact dataset, source, environment, parameter, and artifact hashes. It also
keeps annotated training and evaluation sheets. Broad model-only decisions
remain disabled until independent sessions cover all condition classes and
calibrate acceptance thresholds.

Verify that ONNX export matches the selected PyTorch checkpoint:

```bash
DATASET=datasets/placement-verification-v1/review-current
MODEL="$DATASET/training/placement-verification-v1"
python python/tools/verify_placement_verification.py \
  "$DATASET" \
  "$MODEL/best.pt" "$MODEL/best.onnx" \
  --report "$MODEL/onnx-parity.json"
```

The first reviewed run is recorded in
`docs/baselines/placement-verification-v1.json`. The retained-execution retrain
is recorded in `docs/baselines/placement-verification-v2.json`. These small,
single-machine evaluation sets are not acceptance thresholds.

Load the exported package in the daemon with:

```bash
cargo run -p velocitypnpd -- \
  --placement-verification-model-dir "$MODEL"
```

The environment variable
`VELOCITYPNP_PLACEMENT_VERIFICATION_MODEL_DIR` supplies the same setting. The
health response reports `placement_verification` as `ready`, `disabled`, or
`invalid`. The daemon checks the contract, license, tensor shape, ONNX hash,
file size, and one startup inference before it reports `ready`.

The first promoted policy is a guarded hybrid. The model can control an
axis-symmetric placement only when it confidently predicts `empty` or
`component_flat`. Deterministic CV must agree about occupancy and must register
the local target. A flat prediction must also match the configured package
size. A disagreement cannot pass. Low-confidence results and classes without
training examples keep the deterministic result and require review. Directed
parts continue to use deterministic orientation checks because this model has
no polarity output.

Every model run keeps `board-placement-model-crop.png`,
`board-placement-model-inspection.png`, and
`board-placement-model-comparison.json`. The existing
`board-placement-inspection.png` keeps the paired-frame deterministic result.
This evidence supports later gate calibration and direct comparisons.

Replay the package through the same Rust ONNX runtime used by the daemon:

```bash
cargo run -p velocitypnp-model \
  --example evaluate_placement_verification -- \
  "$MODEL" "$DATASET" eval
```

Raw captures and normalized samples are below `datasets/`, which Git ignores.
Keep them across restarts. Their image hashes and registration IDs provide the
source identity needed for later grouped train and evaluation splits.

### Automatic board registration

Register a visually matched board after the machine is coarse-homed and the
scan calibration is active as the session fine-home revision:

```bash
cargo run --release -p velocitypnp -- register-board SCAN_UUID \
  --candidate 0 \
  --board BOARD_UUID \
  --lease LEASE_UUID \
  --confirm-safety-checks
```

The API route is
`POST /workspace-scans/SCAN_UUID/board-registrations`. The Python client
provides `register_board()`. The web workspace candidate card provides the
same guided action.

Physical registration rejects a coarse-only home. The runner rechecks
visual-refined X/Y homing, then uses the visual PCB outline
as a bounded seed and tests four 90-degree hypotheses when the outline is
square. For each hypothesis, the top camera visits the predicted KiCad
fiducials from a consistent negative X/Y approach. At least three enabled
top-side fiducials are required. Each target must be stable in three to five
frames and remain within the local camera-model and seed-correction bounds.

The accepted fit maps reflected top-side KiCad coordinates into machine
coordinates. The maximum fiducial residual must be at most 0.15 mm and the
fitted angle must remain within 5 degrees of its seed hypothesis. The board
outline may overhang head travel; every commanded camera and nozzle target is
checked separately against the applicable motion bounds. An accepted run
contains a `BoardInstance` with the design-to-machine transform. Board surface
Z remains unset until a separate surface measurement passes.

Registration runs and their evidence survive daemon restarts. Fetch them from
`GET /board-registrations` and `GET /board-registrations/RUN_UUID`. Each
attempted fiducial stores an annotated PNG under
`GET /board-registrations/RUN_UUID/artifacts/ARTIFACT_NAME`; amber marks the
expected optical center and green marks the measured feature.

Measure the registered PCB surface without contact:

```bash
cargo run --release -p velocitypnp -- measure-board-surface \
  REGISTRATION_UUID --lease LEASE_UUID --nozzle N1 \
  --confirm-safety-checks
```

The runner first repeats fine visual X/Y home. It then observes three known
KiCad fiducials from three safe-Z camera positions. It derives the two offset
views from 65% of the calibrated camera aperture, so it keeps margin at the
model limits. Parallax between the working-plane and installed-plate camera
models gives the bare PCB surface.
The V3.0.2 profile records the 1.6 mm distance between these planes. The
stored N1 or N2 datum-contact result converts the optical fraction into that
nozzle's working Z coordinates. The nozzle does not touch the PCB.

The operation needs at least six valid view pairs. It rejects a fraction p95
spread above 0.10, a fit p95 above 0.15 mm, or total uncertainty above
0.20 mm. A passing result updates the registered `BoardInstance.surface_z_mm`.
A failed result keeps the old surface value. Runs, the JSON report, and each
annotated fiducial image survive daemon restarts. The Python client provides
`measure_board_surface()` and the related start, get, list, and artifact
methods.

### Persistent workspace inventory

Use a successful full scan and its accepted refinements as a stable inventory
baseline. First, build the object proposals. Then, activate the baseline:

```bash
cargo run -p velocitypnp -- discover-objects FULL_SCAN_UUID
cargo run -p velocitypnp -- set-workspace-baseline FULL_SCAN_UUID
```

For later board or feeder changes, use a fast reconnaissance scan and compare
it directly with the baseline:

```bash
cargo run -p velocitypnp -- workspace-changes FAST_SCAN_UUID
```

The API routes are `POST /workspace-scans/SCAN_UUID/inventory-baseline` and
`POST /workspace-scans/SCAN_UUID/change-set`. The Python client provides
`set_workspace_inventory_baseline()` and `compare_workspace_observation()`.

For two height-aware semantic scans, the comparison uses calibrated machine
regions, the semantic class, and the normalized image crop. A reconnaissance
scan also keeps its plate-projected raster score after it gains semantic
proposals. Thus, height refinement cannot hide a change that the fast scan
already measured. Before height refinement, baseline-guided mode uses the
baseline object regions, both plate-projected mosaics, and the scan's
seam-health regions. It removes global luma changes before it compares image
regions. A partial scan keeps baseline objects that are outside the observed
region. A covered object can be added, modified, or unchanged. An accepted
refinement that covers at least 90% of an object satisfies its refinement
request. The change remains in the report, but it does not make a repeat scan
target.

Changes to boards, feeders, unrecognized tooling, and unknown objects make
bounded scan refinement targets. Known tooling, calibration, and fixed-machine
fixtures do not make targets. The initial raw pass also ignores unmatched seam
regions below 400 square millimeters. This suppresses the retained plate-grid
noise, but it means that an isolated new small object needs a later detector.

A baseline is compatible only with the same machine profile, calibration
snapshot, and object-proposal algorithm. A geometry calibration change thus
requires a new baseline. Each comparison is stored as
`workspace-change-set.json`; the scan manifest contains its counts and target
count. The intended operation is one occasional full baseline scan, then fast
observations and refinement of only the detected changes.

The report records all match, overlap, appearance, and target-margin
thresholds. The first values are versioned implementation defaults, not
physical acceptance limits. Tune them with repeated unchanged scans and
deliberate board and feeder changes before production use.

`full_sources` supports offline replay. `transient_sources` produces the same
height products without keeping thousands of source frames. `thumbnails` and
`mosaic_only` scans still produce the two endpoint mosaics and sparse parallax
observations, but do not produce the unified map or corrected mosaic.

The V3.0.2 plates are 600 by 120 mm, with 39 columns and seven rows; their
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

Snapshots use the stable role assigned during discovery and return 1280x720
grayscale PNGs. They use the stored transport format, frame rate, and settling
count.
Camera control updates use `PUT` on the role-scoped controls endpoint. The
daemon validates IDs, types, ranges, steps, menu values, and writability before
applying values in request order, so automatic mode can be disabled before its
manual value is set.

Role-scoped profiles persist capture format, ordered controls, ring-light use,
and reproducibility thresholds in SQLite. A missing top-camera profile uses
1280x720 MJPEG at 30 frames per second with no initial discard. A missing
bottom-camera profile uses 1280x720 YUYV at 10 frames per second and discards
20 initial frames. Both seeds use manual exposure 200, manual 4600 K white
balance, and zero gain. Update a profile with
`PUT /api/v1/cameras/{role}/profile`.

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

The reproducibility request can set `pixel_format` and
`frames_per_second` for one diagnostic capture. These values do not change the
stored profile. The response reports the tested profile and whether it used a
temporary capture override.

Build the UI with `npm run build`, then serve it from the daemon using
`--web-dir web/dist`. The Vite development server listens on all IPv4
interfaces at port `4081` and proxies API calls to the daemon on loopback.

## Physical hardware gate

Do not enable hardware until simulator tests pass and the machine is physically
clear. Starting the physical driver is intentionally verbose:

```bash
cargo run --release -p velocitypnpd -- \
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
--release --workspace`. `cargo check`, `cargo clippy`, and `cargo test`
validate source and test harnesses, but they do not guarantee that the
standalone `target/release/velocitypnpd` executable was relinked. Check the
executable timestamp and the expected hardware-path strings before connection.
Use a release binary for height-aware scans; debug builds make the per-pixel
reconstruction path much slower.

The V3.0.2 profile exposes `vacuum_1` and `vacuum_2` as both actuators and raw
pressure sensors. Reads require a lease and are allowed during preflight,
unhomed, or ready states:

```bash
velocitypnp read-sensor --lease "$VELOCITYPNP_LEASE" vacuum_1
velocitypnp actuate --lease "$VELOCITYPNP_LEASE" vacuum_1 true
velocitypnp actuate --lease "$VELOCITYPNP_LEASE" vacuum_1 false
```

The same profile exposes `blowoff_1` and `blowoff_2` for independent MOS2
valve control. These actuators do not enable the vacuum pump. Use them only for
bounded release tests, and always send a final off command:

```bash
velocitypnp actuate --lease "$VELOCITYPNP_LEASE" blowoff_1 true
velocitypnp actuate --lease "$VELOCITYPNP_LEASE" blowoff_1 false
```

Sensor readings are signed controller counts, not pressure units. Establish
the ambient and sealed-nozzle distributions on the attached machine before
using them for part detection. An actuator-on validation must always include
an explicit off command and a fail-safe disconnect.

The bounded datum-board feasibility probe uses the homing fiducial only for
visual registration. It seals on the separate 5 mm front-mask opening at
X=-22.5 mm and Y=-7.5 mm from the fiducial. These coordinates come from the
tagged Opulo V3.0.2 datum PCB. Before descent, the probe uses the bottom camera
to identify and center the current tip. It then uses the top camera to center
the 5 mm target instead of trusting the PCB offset alone. It moves X/Y while
both nozzles are level at controller Z=31.5, lowers the selected nozzle to the
clear search start, starts the pump, and descends at fixed X/Y. A stable vacuum
transition defines contact. The pump stops before the nozzle retracts.

The contact gate uses both the median signal change and the robust separation
between the open and candidate-seal sample distributions. The attached V3
profile requires an 8-count median change and a 3-count distribution gap.
This accepts the repeated, non-overlapping 9-count N1 seal but rejects a
large median change if its raw samples overlap the open-nozzle distribution.

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

After a calibrated operation parks at the datum, check its return without an
additional move:

```bash
velocitypnp camera-return-check --lease "$VELOCITYPNP_LEASE"
```

This command captures at the current corrected pose and applies the 0.05 mm
return gate. Do not use a plain `visual-home` call for this check. Its default
behavior first moves to the nominal seed pose.

Visual X/Y alone does not authorize precision motion. After coarse `G28`, use
the validated datum-board contact artifact to establish the complete session
XYZ reference:

```bash
velocitypnp fine-home CALIBRATION_UUID \
  --lease "$VELOCITYPNP_LEASE" --confirm-safety-checks
```

Fine home visually refines X/Y, identifies and centers the installed N2 tip,
centers the separate sealing target, and reproduces vacuum contact within
0.3 mm of the stored safe contact. It then applies a Z-only coordinate reset;
the correction is limited to 0.5 mm. The active calibration UUID is recorded
in machine status. Coarse re-home, disconnect, fault, or emergency stop clears
it. Physical workspace scans, refinements, and board registration reject a
missing or different fine-home revision.

N2 is the V3.0.2 default because its stored contact has useful bounded travel
on both sides after non-repeatable stall homing. N1 remains selectable, but its
contact is close to the profile's established lower safety limit.

Local X/Y direction response is a separate automatic capture. It reuses the
current top-camera working-plane map and performs one live visual home. Visual
home always finishes at the datum from negative X and negative Y at the
profile datum feed. This approach defines the coordinate anchor. Fixed datum
location and local lens distortion therefore cancel. The runner approaches
from both sides of X and Y at each exact profile feed:

```bash
velocitypnp calibrate-xy-direction-response CALIBRATION_UUID \
  --lease "$VELOCITYPNP_LEASE" --confirm-safety-checks
```

Sampling is adaptive for each axis and feed. It starts with three pairs across
the configured distances. A resolved direction gap can stop when its 95%
confidence interval is small relative to the observed effect. An unresolved
result gets five pairs. No fixed millimetre rule decides whether backlash
exists. The runner tests each correction on an independent holdout pair. The
holdout must keep both its direction gap and each absolute residual at or below
0.05 mm. The correction is tested even when the two directions have the same
response. This permits correction of a speed-dependent common shift. The run
saves the failed condition and stops before it tests another speed.

The stored axis corrections are absolute camera residuals for each axis,
approach side, and exact feed. The axis model does not interpolate or
extrapolate.
It subtracts the negative-X, negative-Y datum-feed correction that was present
when fine home assigned the datum coordinates. Thus the fine-home approach has
zero later correction, while another approach gets only its measured
difference from that anchor.

The axis capture does not measure a simultaneous X/Y return. Use the separate
diagnostic to collect single-axis and four-quadrant path evidence without
changing the active calibration:

```bash
velocitypnp motion-model-diagnostic \
  --lease "$VELOCITYPNP_LEASE" \
  --feed 23000 \
  --offset 25,25 100,50 50,100 200,100 \
  --confirm-safety-checks
```

A qualified planar surface must contain each X distance, Y distance, and
quadrant exactly once. Every node must pass its held-out repeat. The surface
also needs at least 20 independent live validation samples, with a maximum
residual no greater than the profile's 0.05 mm limit. Validate interpolation
at distances that are not grid nodes by adding `--surface SURFACE.json` to the
diagnostic.

Attach a passed surface only after review of that independent result:

```bash
velocitypnp attach-xy-planar-response CALIBRATION_UUID SURFACE.json \
  --lease "$VELOCITYPNP_LEASE" --confirm-safety-checks
```

Test spatial invariance against known installed-plate holes after activation:

```bash
velocitypnp xy-spatial-holdout CALIBRATION_UUID \
  --lease "$VELOCITYPNP_LEASE" \
  --locations-per-plate 3 --samples 3 \
  --offset-x-mm 50 --offset-y-mm 50 \
  --confirm-safety-checks
```

The runner selects separated, previously detected holes on each plate. It
uses a 300 mm/min negative-X/negative-Y reference at each hole. Every fast
return is bracketed by slow references, so camera and detector drift do not
appear as motion error. The report is retained as an operation artifact.
The 0.05 mm qualification limit records a failed sample but does not stop the
run. A 0.25 mm response, invalid motion, or a machine fault stops the run.

The planar model applies only to two-axis moves at its exact feed and within
its measured X/Y distance range. It uses bilinear interpolation. Single-axis
moves, unmeasured feeds, and distances below the first grid node keep the prior
axis model. A two-axis move beyond the maximum measured X or Y distance is
rejected at a surface feed. The planner reserves the largest correction from
both models at controller bounds.

This result is local to the homing datum. It does not establish full-bed scale,
skew, or lens distortion. Calibrated job and strip-feeder transit use only
qualified exact feeds while retaining logical coordinates. Manual jog and
other calibration motion remain raw.

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
