# Implementation status and hardware gates

The project is organized by measurable gates rather than calendar estimates.

## Implemented foundation

- [x] Rust workspace, simulator, Linux discovery, REV4 Marlin transport
- [x] Single-owner state machine, control lease, bounds, preflight, homing, E-stop
- [x] SQLite snapshots, operation tracking, and append-only event journal
- [x] React UI, CLI, Python client, REST/SSE interface
- [x] Direct KiCad PCB import, BOM enrichment, top/bottom data preservation
- [x] Strip/drag/Photon configuration models and Photon packet codec
- [x] Deterministic job preflight/planning and simulator placement cycles
- [x] Calibration dependency graph, invalidation, residual gates, and rollback-ready snapshots
- [x] Fiducial, rigid-transform, residual, and placement-tolerance vision primitives

## Gate 1 — Camera and initial hardware validation

- [x] Detect the attached REV4 controller and assign both PnP camera roles
- [x] Validate bounded controller handshake, reconnect, and fail-safe disconnect
- [x] Record a sanitized physical-controller transcript fixture without motion
- [x] Capture stable frames from both attached UVC cameras through the native backend
- [x] Enumerate native camera formats, controls, and initial automatic settings
- [x] Define the V3 working-plane views: datum fiducial/top camera and N1 nozzle/bottom camera under ring lighting
- [x] Lock camera exposure, white balance, gain, working-plane focus, and lighting
- [x] Acquire the top datum and N1 bottom-camera views; measure initial geometry/exposure seeds
- [x] Validate the shared ring-light, vacuum valve/pump, and pressure sensor commands on V3.0.2
- [x] Implement profile-scoped actuator and raw pressure-sensor APIs
- [x] Persist role-scoped camera settings and reproducibility thresholds
- [x] Record actual top-datum and cleaned-N045 camera fixtures
- [x] Pass 30-frame luma, focus, detection, and center-jitter checks on both cameras

## Gate 2 — Motion and homing validation

- [x] Require explicit preflight/lease gates and verify combined XYZ home with
  bounded `M114` reports
- [x] Preserve reported nozzle rotation on jog and reject unvalidated physical angle changes
- [x] Complete physical homing, safe-Z XY probes, target acquisition, park, and
  disconnect
- [ ] Validate axis signs, safe-Z convention, feed limits, controlled stop, and reconnect
- [ ] Tune V3 sensorless homing with bounded per-axis procedures
- [x] Implement bounded primary-datum detection, correction, and XY reset
- [x] Validate both pixel-to-axis signs and measure primary-datum refinement
- [x] Add session-scoped fine XYZ homing from visual X/Y and bounded vacuum
  contact, and require it for physical precision scans and board registration
- [x] Persist a first-class initial calibration runner with API, CLI, Python,
  and guided web progress
- [x] Remove operator-entered calibration metrics from the supported workflow
- [x] Add bottom-camera detection of configured N045/N24 tooling assignments
- [x] Pass 30-cycle visual-refined homing p95 residual of 0.05 mm or better
- [x] Add adaptive local X/Y direction-response measurement with independent
  compensation holdouts at the homing datum

## Gate 3 — Full calibration

- [ ] Fabricate and publish the known-dimension multi-fiducial calibration PCB
- [x] Fit fixed-height top- and bottom-camera planar models from installed targets,
  with independent holdouts selecting affine or distortion-polynomial models
- [x] Add an occlusion-tolerant machine runner for installed staging/build-plate
  scale, skew, and affine geometry
- [x] Pass installed-camera and both installed-plate geometry calibrations on
  attached hardware
- [ ] Evaluate whether residual full-bed error justifies a dedicated
  multi-fiducial PCB or a higher-order spatial model
- [x] Measure the top-camera working-plane map from bounded homing-fiducial moves
- [x] Add automatic bottom-camera local mapping, reciprocal camera centering,
  and N1/N2 head-offset measurement
- [x] Fit and persist nozzle runout coefficients per detected nozzle/tip
- [x] Fit per-nozzle XY axis shift from safe, height-specific bottom-camera
  image-to-motion maps
- [x] Apply height and working-plane transfer corrections to vacuum targeting
- [x] Replace the bootstrap working-plane observation with reciprocal camera geometry
- [x] Pass the earlier attached-hardware head geometry and runout validation
- [x] Pass the revised N24 outer-tip and camera-ray calibration on attached hardware
- [x] Add camera-centered, vacuum-transition datum-board probing with a bounded Z interval
- [x] Pass the revised vacuum probe on both nozzles without visible tip compression
- [x] Add automatic reference-seal vacuum characterization per nozzle/tip
- [ ] Validate known-part vacuum classification per nozzle/tip and part class
  - [x] Validate N1/N045 with 0603 1 kOhm resistors from three successful
    placement-transfer picks
  - [ ] Validate N2 and other production part classes
- [ ] Set and pass empirically justified full-bed and nozzle accuracy and
  repeatability gates

## Gate 4 — Feeder and job production validation

- [x] Add logical-to-controller motion state, explicit raw mode, calibrated
  bounds checks, and simulator consumption of current local direction vectors
- [x] Add no-motion strip-feeder lane proposals and persistent setup drafts
  from retained workspace evidence
- [x] Measure visible strip population, part pitch, part outline size, and
  present/absent feed-position counts across compatible focused scans
- [x] Add persistent vision-assisted strip setup through live pocket,
  bounded pickup Z, vacuum hold, bottom inspection, and release
- [x] Verify release with bottom-camera nozzle-clear evidence, an adaptive
  bounded depth schedule, and fail-closed top-camera confirmation
- [x] Add persistent per-pocket reservation, evidence-gated consumption,
  empty-pocket skipping, retry limits, and restart recovery
- [x] Add three-to-five-cycle qualification across distinct occupied pockets
- [x] Add a persistent guarded single-placement executor with simulator and
  gated live pickup, alignment, release, and inspection paths
- [x] Add a persistent multi-placement job runner with cooperative pause,
  cancel, reboot-safe resume, and per-placement recovery state
- [x] Persist automatic and operator-reviewed placement outcomes and add a
  bounded, stop-on-failure placement qualification sequence
- [x] Estimate seating offsets by part, nozzle, and board surface from qualified
  outcome groups without changing part data automatically
- [x] Validate deterministic next-pocket advancement during physical job use
- [ ] Validate drag motion/actuator sequences with collision review
- [ ] Validate Photon discovery/feed/status/retry on arriving hardware
- [x] Add live vacuum and bottom-vision feedback to the physical job executor
- [x] Route the physical job executor through the validated motion planner
  without enabling compensation for calibration or manual jog
- [x] Apply local two-terminal pad registration before pickup and place only
  from a bottom-camera pose that follows the final alignment command
- [x] Register adhesive-covered pad pairs with a bounded expected-axis glare
  fallback when connected-component segmentation cannot separate both pads
- [x] Retry one missed pickup with a bounded correction learned from recent,
  agreeing placement residuals for the same job, nozzle, and part
- [x] Add first-class placement-transfer calibration with provisional XY
  adjustment, independent check placements, and persistent evidence
- [x] Separate systematic transfer correction from placement variation and add
  validation-only placements that cannot change the stored correction
- [x] Pass the N1 placement-transfer check and verify correction persistence
  on attached hardware
  - 2026-10-02: correction reduced the three-part check p95 from 0.226 to
    0.079 mm. A 0.010 mm decision resolution removed unjustified
    sub-resolution boundary failures. Reviewed run
    `8cc3a045-a8b1-4b20-90da-bca01fd8380c` committed X+0.029425/Y-0.173670
    mm. A daemon restart retained the correction and current artifact tree.
- [ ] Select or create a reference placement board and acquire compatible parts
  for a repeatable production qualification
- [ ] Pass a 100-placement run on that fixture with accuracy and reliability
  limits defined before motion

## Gate 5 — Inspection and newer machines

- [x] Adopt OpenCV frame statistics and connected components behind the
  validated calibration policy, with attached-fixture conformance tests
- [x] Build calibrated top-camera workspace mosaics with precision source-tile
  retention and bounded OpenCV overlap registration
- [x] Add persistent-capture, bounded-memory coarse surveys with preflight
  duration/storage estimates, artifact quotas, and mosaic-only retention
- [x] Add transient-source, chunked height reconstruction for full-workspace
  layered scans without persistent per-tile storage
- [x] Add and physically validate a low-overlap, timestamp-gated reconnaissance
  scan that produces a usable full-workspace inventory in under 20 minutes
- [x] Add diagnostic-only reconnaissance seam health, visual artifacts, and
  machine-coordinate target regions for selective refinement
- [x] Physically validate a lineage-tracked, height-corrected selective
  refinement patch from a reconnaissance target
- [x] Add severity-ranked, resumable, sequential refinement queues that skip
  successful targets and stop safely on the first failed patch
- [x] Merge successful height-corrected refinements into the parent raster and
  apply a separate, versioned image-quality gate
- [x] Localize coarse height-cell display boundaries with overlap-supported
  per-pixel candidate selection
- [x] Georeference survey rasters and use installed plate-plane geometry for
  full-workspace scan planning
- [x] Add no-motion raised-surface object proposals with calibrated plate-axis
  outlines, semantic fixture handling, ML-ready evidence crops, and KiCad
  dimension matching
- [x] Add persistent inventory baselines, partial-observation change sets, and
  change-driven refinement regions
- [x] Add baseline-guided raw reconnaissance comparison so changes can select
  refinements before a new height map exists
- [x] Add scan-seeded automatic top-side board registration with square-board
  orientation hypotheses, local KiCad fiducial measurements, evidence images,
  and a residual-gated design-to-machine transform
- [x] Physically validate persistent non-contact board-surface measurement
  from safe-Z parallax views of registered KiCad fiducials
- [ ] Tune inventory change thresholds with unchanged scans and deliberate
  board and feeder changes on attached hardware
- [ ] Add learned semantic segmentation with versioned datasets, calibrated
  confidence, and machine/configuration holdout tests
- [x] Add a provenance-bearing source-component draft exporter with
  registration-grouped train/evaluation splits and operator review
- [x] Add review-guided flat-box refinement and an oriented box editor for
  difficult and on-edge source components
- [x] Add persistent, inventory-driven safe-Z capture of balanced occupied and
  empty pocket views, with exact-hash label carry-forward and new-only review
- [ ] Extend balanced source capture across more component families and
  independent machine sessions
- [x] Fine-tune an oriented source-component detector and compare it with the
  deterministic detector on accepted evaluation labels
- [x] Select a license-compatible runtime model and export the oriented
  source-component detector for daemon integration
- [x] Load hash-checked source-component model packages in the Rust daemon and
  persist shadow predictions, deterministic comparisons, and review artifacts
- [x] Use a high-confidence flat source-component model center for guarded
  pickup, with occupied-pocket, machine-offset, and CV-disagreement bounds
- [x] Invalidate stale occupied-pocket state before nozzle descent only when
  the source model and deterministic CV agree that the pocket is empty
- [ ] Calibrate learned-model confidence gates on independent machines and
  component families before model output can make broader feeder decisions
- [x] Add paired bare/observed held-part dataset export with deterministic
  drafts, grouped splits, editable condition boxes, and directed-end review
- [x] Train, export, and integrate a held-part condition and pose model in
  shadow mode against retained pickup and release evidence
- [x] Add paired same-angle bare and held bottom-camera rotation sweeps to the
  guarded pickup-return operation and held-part dataset exporter
- [x] Add package-template pose comparison after live fiducial verification
- [x] Train, export, and integrate a top-camera placement condition and pose
  model with guarded CV agreement from registration-normalized reviews
- [x] Merge retained placement-execution evidence without target leakage,
  retrain with bounded camera augmentation, and validate the package in the
  Python, ONNX, Rust, and daemon runtimes
- [x] Record labeled-dataset accuracy and known false flags for the current
  models, and keep known errors behind deterministic agreement gates
- [x] Reproduce the source-component, held-part, and placement-verification
  models from clean, commit-identified source trees
- [x] Publish versioned Apache-2.0 runtime packages in a separate model-zoo
  repository with model cards, checksums, and evaluation records
- [x] Publish immutable reviewed-dataset packages beside the model catalog,
  with label records, images, audit data, checksums, and model links
- [x] Publish a reviewed catalog revision that approves the current three model
  packages for their existing guarded runtime roles
- [x] Add application-managed model download, cache, checksum, selection, and
  rollback, with strict daemon startup verification and health provenance
- [ ] Add V3.1/V3.2 and V4.x profiles with hardware-in-loop fixtures
- [ ] Package signed Debian releases and migration-tested backup/restore bundles

The physical job runner uses the guarded single-placement executor for each
step. Each run keeps persistent progress and stops for operator recovery after
a failed placement.

## Gate 6 — Motion performance and operator interface

- [ ] Measure command, travel, camera-settle, actuator, and inspection time for
  a guarded placement and a workspace scan row
- [x] Separate profile-scoped safe-Z transit, vision correction, Z approach,
  Z contact, and rotation speed limits
- [ ] Validate faster motion one class at a time against bounds, controller
  limits, controlled stop, and calibration residuals
- [ ] Audit the complete current web interface in simulation and record the
  normal operator workflow and recovery paths
- [x] Add a dedicated live Workspace view with georeferenced scan imagery,
  machine pose, scan progress, object, board, and active-placement overlays
- [x] Gate board and per-lane feeder display patches against registered
  fiducials or pocket lattices, with persisted transforms and residuals
- [x] Promote each feeder rescan as one atomic lane revision containing the
  image registration, tape geometry, pocket positions, occupancy, and active
  feeder plan
- [x] Add a dedicated Feeder setup view with stable bank/lane names, part
  assignment, qualification, revision history, and direct Workspace access
- [x] Add a live Workspace activity console with scan/job progress, ETA where
  measured, compact results, evidence links, and server-authorized recovery
- [x] Add simulated browser tests for machine bootstrap, Workspace interaction,
  and feeder setup workflows
- [ ] Continue splitting calibration, board, and job setup out of the main
  React application component

See `docs/HARDWARE_VALIDATION.md` for measured attached-hardware evidence.
Verify live machine state and get a new safety confirmation before motion.
