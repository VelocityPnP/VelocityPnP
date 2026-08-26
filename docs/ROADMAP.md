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
- [x] Validate the shared ring-light, vacuum valve/pump, and pressure sensor commands on V3.0.4
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
- [ ] Set and pass empirically justified full-bed and nozzle accuracy and
  repeatability gates

## Gate 4 — Feeder and job production validation

- [x] Add logical-to-controller motion state, explicit raw mode, calibrated
  bounds checks, and simulator consumption of current local direction vectors
- [x] Add no-motion strip-feeder lane proposals and persistent setup drafts
  from retained workspace evidence
- [x] Measure visible strip population, part pitch, part outline size, and
  present/absent feed-position counts across compatible focused scans
- [ ] Validate vision-assisted strip setup and indexing
- [ ] Validate drag motion/actuator sequences with collision review
- [ ] Validate Photon discovery/feed/status/retry on arriving hardware
- [ ] Add vacuum and bottom-vision feedback to the physical job executor
- [ ] Route the future physical job executor through the validated motion
  planner without enabling compensation for calibration or manual jog
- [ ] Pass 100 imperial-0402 placements with the documented accuracy/reliability thresholds

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
- [ ] Tune inventory change thresholds with unchanged scans and deliberate
  board and feeder changes on attached hardware
- [ ] Add learned semantic segmentation with versioned datasets, calibrated
  confidence, and machine/configuration holdout tests
- [ ] Add package-template pose comparison after live fiducial verification
- [ ] Meet labeled-dataset recall and false-flag targets
- [ ] Add V3.1/V3.2 and V4.x profiles with hardware-in-loop fixtures
- [ ] Package signed Debian releases and migration-tested backup/restore bundles

The physical job execution endpoint remains absent until Gates 1–4 pass. The
simulator endpoint exists specifically to exercise job state and path planning
without turning an unvalidated workflow into machine motion.
