# Session handoff

This document records the state at the end of the 2026-09-06 hardware and
vision session. Verify live state again before each new session.

## Live state

- The physical daemon is stopped. The machine is normally disconnected,
  unhomed, and fault-free. Start a new release daemon with `velocitypnp.db`.
- The database contains the motion, camera, pickup, board-registration, and
  full-workspace qualification evidence. The stored camera profiles use
  top-camera MJPEG at 30 FPS with zero initial discard and bottom-camera YUYV
  at 10 FPS with 20 initial discard frames.
- The last physical operation was normal disconnect
  `7bab1274-9eba-432e-9927-830c3f50ce4c`. It cleared the calibration revision
  and control lease. Acquire a new lease and home before more motion.
- Do not infer that the machine is safe from this document. Get a new operator
  confirmation before motion.
- The daemon reports all three learned models as ready.

The active local model packages are:

| Purpose | Contract | ONNX SHA-256 |
| --- | --- | --- |
| Source component | `source-component-pose.v2` | `4a89e8abb0a2c5a513ef43ddddb9a936ee5b6656f34047b671530c155a94d204` |
| Held part | `held-part-pose.v1` | `a34f48bdbd9e275dc136dd2eaafb733e691ebd952ece9b30f7f0d15b520699bf` |
| Placement verification | `placement-verification.v1` | `10e07560dfc78c53a86d9da4b98e59fc663af58adebde83ddc45875a2247670e` |

The active model directories are below the ignored local `datasets/` tree.
They persist across reboots on this computer, but Git does not contain them.

## Model distribution

Create a separate model-zoo repository before a public model release. Keep
model contracts, training code, and evaluation baselines in this repository.
Put versioned runtime packages, checksums, licenses, and model cards in the
model-zoo repository. Keep raw training images in separate storage.

The three current artifacts declare Apache-2.0 terms. Their training code
declares Apache-2.0 terms, and the training framework declares BSD-3-Clause
terms. The source-component manifest has no Git commit provenance. The
held-part manifest records a dirty source worktree. Retrain or reproduce those
two packages from clean commits before a final model-zoo release. The current
placement-verification package records clean source provenance.

The application still needs a versioned model download, cache, checksum, and
selection mechanism. Do not put changing binary model history in the main
source repository.

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
branch will be 42 commits ahead of `origin/main`. Push these commits before
work moves to another computer.

There is no project `SKILL.md`. `AGENTS.md` contains the current repository
rules, so no skill was created or changed during closeout.
