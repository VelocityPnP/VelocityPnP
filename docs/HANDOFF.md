# Session handoff

This document records the state at the end of the 2026-09-05 hardware and
vision session. Verify live state again before each new session.

## Live state

- The physical daemon listens on `0.0.0.0:4080` and uses `velocitypnp.db`.
- The API reports the machine as disconnected, not homed, with no control
  lease and no active calibration revision.
- A motion session must connect, run preflight, acquire a lease, run standard
  XYZ home, and run fine XYZ home before precision work.
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

1. Capture X and Y response from both directions at every qualified transit
   feed. Each exact condition must pass an independent 0.05 mm holdout.
2. Run fine home with its fixed negative-X and negative-Y datum approach.
3. Run a 1206 pickup-return qualification at 12,000 mm/min clear and carried
   transit.
4. Measure camera-settle time and actuator wait time for one guarded placement
   and one scan row.
5. Validate controlled stop and reconnect. Use emergency stop only for an
   actual emergency.

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
