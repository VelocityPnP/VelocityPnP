# Session handoff

This document records the current hardware, vision, and model state through
2026-09-26. Verify live state again before each new session.

## Live state

- The physical daemon is running on loopback port 4080. At the last check, the
  machine was connected, homed, ready, and fault-free at safe Z. It had a
  session lease. Recheck status and get a new operator safety confirmation
  before more motion.
- The database contains the motion, camera, pickup, board-registration, and
  full-workspace qualification evidence. The stored camera profiles use
  top-camera MJPEG at 30 FPS with zero initial discard and bottom-camera YUYV
  at 10 FPS with 20 initial discard frames.
- Coarse home `63e7428e-0480-4ffd-8cc2-d262e14e57be` and N2 fine home
  `2109e395-9064-431f-b681-23a0eb7541af` passed after the latest daemon
  restart. Fine home reset the datum contact from Z59.100 to Z59.200.
- Calibration `77f004b5-04d3-44b1-921b-9218df91e6a0` and board instance
  `763a44ae-2765-4839-8f46-930002525330` are active.
- The last recorded states for lights, both vacuum outputs, and blowoff are
  off.
- Do not infer that the machine is safe from this document. Get a new operator
  confirmation before motion.
- The daemon reports all three learned models as ready.

The active local model packages are:

| Purpose | Contract | ONNX SHA-256 |
| --- | --- | --- |
| Source component | `source-component-pose.v2` | `e97e723c68e31d7044220742562562d2d80e244f149c8ab0f0aa0177ae3ab072` |
| Held part | `held-part-pose.v1` | `6e3a50e742b5e6968dbf080a02fbb1d7cb24a5f16b23f4ab8879ffb27e3d08f4` |
| Placement verification | `placement-verification.v1` | `eb5302231998b3a0dba92c2746c76bfc3ebc82c45c347c5b0d49568eb63ba232` |

The active model directories are below the ignored local `datasets/` tree.
They persist across reboots on this computer, but Git does not contain them.

## 2026-09-26 placement-transfer and model work

The N1 placement-transfer runner completed baseline placements R25, R28, and
R31. Their signed physical offsets were X+0.091/Y+0.055,
X+0.110/Y+0.185, and X+0.026/Y+0.091 mm. The robust center was
X+0.091/Y+0.091 mm, and p95 radial error was 0.215 mm. The runner prepared
that value as a provisional correction but did not commit it.

The first check stopped before pickup at feeder 7 pocket 10. Continuation run
`9d1a443a-0a38-4032-a712-b9225968d2ac` stopped before pickup at pocket 14.
The retained image shows a resistor and a correct learned pose, but all local
source checkpoints classify it as empty. Deterministic vision selected the
tape edge. This is a source-model regression, not placement evidence.

The finalized all-history source dataset has 595 unique images. It includes
567 images: 336 flat components, 51 on-edge components, and 180 empty pockets.
The other 28 images are rejected. Of the 336 new labels, the review accepted
198 parts, marked 122 pockets empty, and rejected 16 images. The stored review
SHA-256 is
`1bb55a902dd9fd93732108883e26b0d42ac613d99ea0347e371935f6d5a03e7a`.
The final local dataset is `datasets/source-components-v8-model-seeded`.
The earlier 382-image page repeated 46 reviewed images because the combined v4
manifest did not contain raw-source metadata. Do not restart
placement-transfer motion with the current source model.

The private model repository release `datasets-2026-09-26.1` stores immutable
packages for the three model training datasets and the newer September 20
feeder dataset. Each package contains final labels, reviewed images, review
records, checksums, and a dataset card. Published model catalog entries now
name their exact dataset package.

Nine N1 pickups after the latest vacuum calibration have bottom-camera part
confirmation. Their open-nozzle median was 237 counts, their held-part median
was 228 counts, and their held range was 228 to 231 counts. None were false
negatives at the 231.092-count acquire threshold. This is useful retrospective
evidence, but the stored calibration still needs a controlled known-part
validation.

A clean training clone at commit `61b44e1` is on `svl-spark` at
`/home/tsmith/velocitypnp-training/clean-20260926`. Clean held-part and
placement reruns reproduced the existing checkpoints, ONNX files, histories,
and metrics byte for byte. Their new manifests record `dirty: false`. The
source retrain must wait for the new feeder decisions and a new independent
holdout. See `docs/baselines/model-reproduction-2026-09-26.md`.

The current board has physical contacts at R7, R9-R14, R17-R18, R25, R27-R28,
R30-R31, and R40. R30 is on its side. R17 has no detected result center. Do
not reuse any of these targets. R26, R29, and R32 remain the intended
180-degree continuation targets.

The database backup before this work is
`/tmp/velocitypnp-before-placement-transfer-20260926.db`. Its SHA-256 is
`ffae73385decc0e126fd02d54ad28c671915ea9c231c7369831d10d222161ccf`.

## Model distribution

The model zoo is `VelocityPnP/velocitypnp-models`. Immutable prerelease
`models-2026-09-14.1` contains the combined September 2026 source-component,
held-part, and placement-verification packages. The model-zoo catalog records
package, manifest, checkpoint, ONNX, dataset, and training-source checksums.
Raw training images remain in separate storage.

This repository pins those packages in `models.lock.json`. The lock is
cache-only because all three models remain candidates and all three training
manifests record a dirty source worktree. Run
`python3 python/tools/model_cache.py sync models.lock.json` to download and
verify them. The tool uses a content-addressed local cache and cannot activate
this candidate lock.

Runtime activation requires a separate lock with `selection_policy: runtime`
and approved status for every package. The cache keeps exact prior lock
snapshots for explicit rollback. Model selection still does not change daemon
acceptance policy. See `docs/MODEL_DISTRIBUTION.md`.

## 2026-09-07 speed closeout

The current LumenPnP V3.0.2 profile uses 23,000 mm/min for clear and carried
XY transit. The controller travel acceleration stays at 1,000 mm/s2. No
controller acceleration or persistent firmware setting changed.

The active calibration before disconnect was
`3b837434-a961-4e60-945a-18335b42d21f`. It contains an exact-feed planar XY
response surface for 23,000 mm/min. The surface covers 5-200 mm X travel and
5-180 mm Y travel in all four approach quadrants. The planner uses the prior
axis response for single-axis moves and other qualified feeds. It rejects a
23,000 mm/min diagonal move outside the surface range.

The independent datum interpolation holdout passed 108 of 108 returns. Mean
error was 0.0178 mm, p95 was 0.0369 mm, and maximum error was 0.04987 mm. The
fixed-hole spatial holdout then passed 72 of 72 returns at six plate locations.
Its RMS error was 0.01866 mm, p95 was 0.03377 mm, and maximum error was
0.04889 mm. This supports use of one surface across the tested workspace. The
narrow right edge beyond X340 mm has no fixed-hole endpoint test.

Three pickup-and-return cycles at 23,000 mm/min took 30.54, 25.90, and 25.88
seconds. The first cycle included camera start. Each later cycle used about
2.52 seconds for bottom-camera capture and 1.07 seconds for top-camera capture.
This is not a PCB placement benchmark. A physical multi-placement job remains
unqualified until suitable parts are available.

The measured full reconnaissance scan took 7 minutes 26.8 seconds before the
final timestamp-source improvement. The final 10-tile qualification projects
about 6 minutes 52 seconds for the same 1,023-tile scan. Short raster moves are
acceleration-bound, so 24,000 mm/min would give almost no scan improvement.
Do not increase acceleration without a separate mechanical-noise review.

Good later speed tasks, in priority order, are:

1. Measure a real multi-placement job when suitable parts are available.
2. Compare and qualify a faster bottom-camera mode. The current profile keeps
   YUYV at 10 FPS with 20 initial discard frames.
3. Qualify higher clear and carried Z rates if a one-to-two-second cycle gain
   is useful. The current Z rate is 1,200 mm/min.
4. Measure a complete feeder-bank population scan with the final camera and
   motion configuration.
5. Change scan acceleration or add continuous-motion capture only if a scan
   below approximately seven minutes becomes necessary.

The refined workspace mosaic copy is
`/tmp/velocitypnp-refined-workspace-4f007afa-cfa9-4984-b382-7c749304e071.png`.
Its SHA-256 is
`9504fb140b055bcb597adaf3d0ae70d5ca26c4e9629a4bd64a30e65605e2e61d`.

## 2026-09-06 scan closeout

The configured and direction-qualified clear-XY rate is 23,000 mm/min. The
controller reports a 30,000 mm/min X/Y limit and 1,000 mm/s2 travel
acceleration. X and Y use 64 microsteps with interpolation. These controller
values are not independent proof of the mechanical limit.

Full reconnaissance scan `42f9e894-b9c7-4ca0-ac81-f34f8709a341` captured
all 1,023 tiles in 7 minutes 26.8 seconds. It used 5 minutes 39.4 seconds for
tile motion and 1 minute 41.3 seconds for capture. The remaining work used
about 6.1 seconds. The mosaic had full coverage. Visual review found no
missing tiles. Its SHA-256 is
`e71a2064d2d58e16b50014e516e9bbce115d4cf94ff058a7e658ad2860acf090`.
The immediate no-move camera return error was 0.0201 mm.

The scan now reserves the maximum calibrated correction at each controller
edge. The attached calibration needs 0.060 mm in X and 0.070 mm in Y. Thus,
the standard `installed_plates` request no longer needs a manual 0.10 mm
region inset. Scan estimates also validate the calibrated path before motion.

The connected estimate now models the controller acceleration and measured
90 ms command cost. It estimates 432 seconds for the same full scan. This is a
conservative estimate because it does not assume a camera timestamp source.

The top camera reports start-of-exposure timestamps. Three 10-tile scans with
the source-aware freshness gate took 5.343, 5.344, and 5.343 seconds. Each
discarded exactly one frame per tile. Later captures averaged 64.1 to 66.1 ms,
down from 97.8 ms. The three immediate no-move return errors were 0.0185,
0.0174, and 0.0167 mm. All passed the 0.05 mm gate. The measured full-scan
projection is about 412 seconds, or 6 minutes 52 seconds.

The full scan found 612 unresolved and 46 elevated seams among 1,982 seams.
It recommends eight targeted refinement regions. This is image-content and
reconstruction work. It is not a coverage failure. Do not start all eight
refinements without a new time and value review.

## Motion-performance session (historical 2026-09-05)

The V3.0.2 profile keeps 1,200 mm/min as its general default. Configured clear
and carried safe-Z transit use 12,000 mm/min. Important calibration feeds are
300 mm/min, installed-plate scans use 600 mm/min, and vacuum contact uses 60
mm/min. The controller reports a 30,000 mm/min X/Y limit. This is not a
validated mechanical limit.

Connection now records the active `M503` and `M122` controller settings in
machine status. Each jog records requested feed, linear distance, ideal
constant-speed time, controller-wait time, and failure details in the event
journal. The `motion-sweep` CLI command runs clear-XY moves at safe Z. Its
default levels are 1,200, 3,000, 6,000, 9,000, and 12,000 mm/min. It compares
each return with a direction-conditioned camera baseline at the primary datum.
It stops speed escalation when a return differs from the baseline by more than
0.05 mm. The attached machine passed all levels. Its worst return error was
0.02076 mm.

The 2026-09-05 measured pass completed these items:

1. X and Y response from both directions passed at 300, 1,200, 3,000, 6,000,
   9,000, and 12,000 mm/min. Each condition used three training pairs and an
   independent holdout. The worst holdout direction gap was 0.01283 mm. The
   worst absolute holdout residual was 0.01505 mm.
2. Fine-home operation `b4e73459-9d7a-4c8c-a249-ef420a355101` passed. It used
   the fixed negative-X and negative-Y approach and removed the stored 300
   mm/min datum-approach response from the home correction.
3. Live holdout `ecd563e5-1733-4c9b-8c6f-6569ded11159` passed at pocket 10.
   Pickup-return qualification `c75ac060-3850-48c6-a9bb-838793fdd224` then
   passed with N1 at 12,000 mm/min clear and carried transit. The 1206 part was
   returned to pocket 10, which remains occupied and available.
4. One-row survey `c8277770-ca0f-4f45-a291-420371e224a7` captured all 10
   planned tiles in 27.37 seconds at 1,200 mm/min. It produced full coverage.
   Four of nine overlap registrations passed; calibrated placement supplied
   the other five.

The first pickup-return operation took 180.5 seconds. The optimized operation,
qualification `9e38a0d3-3255-42bb-a09f-1fba1e11484a`, took 44.80 seconds. This
is a 75.2 percent reduction. It passed vacuum hold, bottom vision, release, and
top-camera return verification. The 1206 part is in pocket 10, which is
occupied and available.

Safe-level XY uses 12,000 mm/min. Clear and carried Z use 1,200 mm/min. Pickup
contact stays at 60 mm/min, and the last 2 mm of each release approach stays
at 300 mm/min. The 300 mm/min time decreased from 87.34 to 3.84 seconds. Total
motion now uses 20.31 seconds.

One camera session per role stays open for a full physical operation. Ten of
ten top-camera 80-frame cold-start runs passed with no initial discard. Five
bottom-camera runs needed at most 12 discarded frames before the remaining
series passed. The stored and source defaults now discard 0 top-camera frames
and 20 bottom-camera frames.

Pickup benchmark `089f2c58-36b2-4448-b15f-3e657be71245` passed five cycles
with the old 60-frame values. Its first cycle took 44.90 seconds and later
cycles took 29.69 to 30.18 seconds. Validation benchmark
`204fc507-ace9-4f15-894e-bff640d062df` passed five cycles with the new values.
Its first cycle took 34.85 seconds and later cycles took 29.77 to 30.08
seconds. The reduced settling removed 100 cold frames and 10.05 seconds. A
continuous job also shares camera sessions across placement steps. The PCB
placement and recovery paths use the same split Z approach. These production
paths have automated coverage but no physical multi-part placement test.

The scan used 5.46 seconds for tile motion and 11.06 seconds for its final
300 mm/min park. The first capture waited 7.51 seconds after motion. Later
tile cycles used about 0.30 seconds outside motion. The camera profile discards
60 frames for its first capture and four frames after each later move.

Source instrumentation writes one `operation.workspace_scan_timing` event
per scan. It separates camera open, first and later captures, tile motion,
artifact storage, mosaic work, progress storage, finalization, parking, and
light control. Actuator and sensor events include elapsed time. Pickup runs add
`operation.strip_feeder_pickup_timing`. Physical continuous job runs add
`operation.job_execution_timing` with aggregate camera timing.

Defer a normal guarded placement until a board-compatible component is
available. The next camera work is to reduce repeated capture count or sample
count with gate-specific evidence. Do not infer PCB placement timing from the
1206 pickup-return benchmark. Use emergency stop only for an actual emergency.

The main code locations are:

- `profiles/lumenpnp-v3.0.2.json`
- `crates/domain/src/machine.rs`
- `crates/driver/src/actor.rs`
- `crates/driver/src/marlin.rs`
- `crates/planner/src/motion.rs`
- the scan and placement runners in `apps/daemon`

Do not tune camera correction or vacuum-contact feeds as if they were transit
moves. Preserve the motion planner, bounds, lease, homing, and calibration
gates.

## Web-interface session

The UI is a React and TypeScript application in `web/`. It currently has four
tabs: Machine, Calibration, Boards, and Jobs. The Boards tab also contains the
workspace scan, object, feeder, inventory, and board-registration workflows.
Most UI behavior is in one large `web/src/App.tsx` file.

The closeout baseline passed with Node 22.23.2 and npm 10.9.8:

```bash
cd web
npm test -- --run
npm run typecheck
npm run build
```

Start UI work against a separate simulated database:

```bash
cargo run -p velocitypnpd -- \
  --simulate true --database /tmp/velocitypnp-ui.db
cd web
npm run dev
```

First inspect the complete interface in a browser. Record its main workflows,
missing states, unsafe or confusing actions, and narrow-screen behavior before
large visual changes. Good first implementation slices are:

1. Split the monolithic application into route or feature components.
2. Give Workspace and Feeders clear navigation instead of nesting all work in
   Boards.
3. Add model-health, operation-progress, evidence, and recovery views.
4. Make the normal sequence clear: connect, home, calibrate, scan, configure,
   plan, execute, and inspect.
5. Add browser-level workflow tests. The current suite has one UI test.

Use simulation for the initial UI audit. A visible button must not be treated
as proof that its physical workflow is ready.

## Repository state

The main worktree is expected to be clean after this handoff commit. The local
branch will be 45 commits ahead of `origin/main`. Push these commits before
work moves to another computer.

There is no project `SKILL.md`. `AGENTS.md` contains the current repository
rules, so no skill was created or changed during closeout.

## Planar XY motion model

The active calibration now contains one four-quadrant response surface at
23,000 mm/min. It covers 5-200 mm X travel and 5-180 mm Y travel. The planner
uses bilinear interpolation for two-axis moves inside that range. It keeps the
prior axis-direction model for single-axis moves, other feeds, and distances
below the grid. It rejects a 23,000 mm/min diagonal move beyond the maximum X
or Y distance.

The independent surface holdout passed 108 of 108 samples. Mean residual was
0.0178 mm and maximum residual was 0.04987 mm. Repeated raw nodes differed by
up to 0.0323 mm between runs, mainly in Y. Do not tighten the 0.05 mm gate from
this evidence.

Active FTP board registration passed with 0.0314 mm maximum residual. Its
long return to the datum passed at 0.0351 mm, down from 0.0634 mm. A ten-tile
survey passed in 5.343 seconds and returned at 0.00525 mm. The machine ended
that run ready, homed, and fault-free at X220.091 Y190.278 Z31.500.

After the final range guard was added, the new release restarted cleanly.
Connect, preflight, coarse home, and N2 fine home passed. The current pose is
X220.123 Y190.234 Z31.500. At that point, the machine was ready, homed, and
fault-free with the same calibration active. A no-move camera return then
passed at 0.02347 mm.

The fixed-hole spatial holdout is complete. Its bracketed-reference procedure
passed six of six locations and 72 of 72 returns. RMS was 0.01866 mm, p95 was
0.03377 mm, and maximum error was 0.04889 mm. It took 386.45 seconds. The test
kept acceleration at 1,000 mm/s2 and did not change the motion surface.

The test directly covered 50 mm by 50 mm paths at fixed-hole centers from
X54.902-X340.386 and Y99.929-Y310.308. The datum matrix supplies distance
coverage. The narrow right edge beyond X340 mm still lacks a fixed-hole
endpoint test. After loading the final binary, coarse and fine home passed
again. A no-move datum check passed at 0.02719 mm. At that point, the machine
was ready, homed, and fault-free at X220.124 Y190.234 Z31.500. Qualification
data is in
`datasets/motion-model-qualification-2026-09-06`. Normal disconnect
`d52d27ff-8c1a-469a-a45e-605aac6f1f4f` then succeeded, and the physical
daemon was stopped.
