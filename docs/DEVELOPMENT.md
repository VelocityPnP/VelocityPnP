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

The V3.0.4 profile supplies the source-derived datum-PCB outline and an
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
`crates/vision/tests/fixtures/lumenpnp-v3.0.4-strip-feeder-bank-ground-truth.json`.
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

The persistent report gives predicted and measured machine coordinates. Its
PNG marks expected sprocket lines in yellow, the expected pocket centerline in
cyan, the measured sprocket line in magenta, detected holes in yellow, and
measured pocket centers in green. A passed result selects a measured occupied
pocket in the draft. It does not lower a nozzle, index tape, measure pickup Z,
or create an active `Feeder`. Pickup-Z and indexing validation remain the next
hardware gates.

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

N2 is the V3.0.4 default because its stored contact has useful bounded travel
on both sides after non-repeatable stall homing. N1 remains selectable, but its
contact is close to the profile's established lower safety limit.

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
