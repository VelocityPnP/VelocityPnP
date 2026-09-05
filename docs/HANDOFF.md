# Session handoff

This document records the state at the end of the 2026-09-05 hardware and
vision session. Verify live state again before each new session.

## Live state

- The release physical daemon listens on `127.0.0.1:4080` and uses
  `velocitypnp.db`.
- At 22:23 UTC, the API reported the machine ready, homed, fault-free, and at
  X220.131, Y190.271, Z31.500. Its calibration revision was
  `3b837434-a961-4e60-945a-18335b42d21f`.
- Control lease `fdc48cb3-f3b5-4c1a-be80-6def86ca898b` expires at 23:22 UTC.
  Verify that it is still active before more motion.
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

## Motion-performance session

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

Continue the performance work with measurement:

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

The pickup-return operation took 180.5 seconds. Its six fast moves covered
839.6 mm in 6.02 seconds. Precision moves at 300 mm/min took 87.33 seconds.
Camera, vacuum, light, and analysis work used about 85 seconds. More transit
speed is not the first target for this qualification path.

The scan used 5.46 seconds for tile motion and 11.06 seconds for its final
300 mm/min park. The first capture waited 7.51 seconds after motion. Later
tile cycles used about 0.30 seconds outside motion. The camera profile discards
60 frames for its first capture and four frames after each later move.

New source instrumentation writes one `operation.workspace_scan_timing` event
per scan. It separates camera open, first and later captures, tile motion,
artifact storage, mosaic work, progress storage, finalization, parking, and
light control. Actuator and sensor events now include elapsed time. The active
physical daemon does not contain this uncommitted build and must not be treated
as proof of the new timing event.

Defer a normal guarded placement until a board-compatible component is
available. Next, use the new timing event in simulation or after a controlled
daemon restart. Then validate a controlled stop and reconnect. Use emergency
stop only for an actual emergency.

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

The main worktree was clean before this handoff update. The local branch was
29 commits ahead of `origin/main`; this handoff commit adds one more commit.
Push these commits before work moves to another computer.

There is no project `SKILL.md`. `AGENTS.md` contains the current repository
rules, so no skill was created or changed during closeout.
