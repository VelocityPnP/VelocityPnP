# Architecture

VelocityPnP separates policy from hardware so the same workflows run against a
deterministic simulator, recorded sessions, or a physical LumenPnP.

```text
React UI / CLI / Python
          │ HTTP + SSE
          ▼
    velocitypnpd API
       ├── logical job plan ── calibrated motion planner ─┐
       └── raw calibration and manual motion ─────────────┤
                                                         ▼
                                              single machine actor
                                                   │       └── event journal
                                             MachineDriver trait
                                               ├── simulator
                                               └── Marlin serial ── Photon

KiCad importer ── board/part model ── deterministic planner
Vision datasets ─ calibration graph ─ SQLite snapshots/artifacts
```

## Safety boundary

Only the machine actor owns a controller connection. It serializes commands and
enforces state, lease, homing, finite coordinates, configured bounds, feed-rate
limits, and fault transitions. HTTP handlers cannot emit G-code. Emergency stop
does not require a lease and revokes any active lease.

Physical access has two startup gates: `--simulate=false` selects the Marlin
driver and `--allow-hardware` acknowledges that physical access is intended.
The daemon defaults to simulation and loopback networking.

## Coordinate frames

The domain uses millimetres and degrees. `Pose` values are always labeled by the
owning resource; `Transform2d` maps explicitly from a source frame to a target
frame. Board instances hold a fiducial-derived design-to-machine transform and
a separately measured surface Z. Bottom-side execution gets a new transform
after the guided board flip; it does not reuse a mirrored top-side guess.

Visual homing first centers the profile-defined machine datum in the top
camera, then resets the controller's current X/Y working coordinates to the
datum's calibrated machine location. The reset is applied only after a second
image passes the configured residual gate; Z and rotation coordinates remain
controller-reported and untouched.

Job planning and motion planning are separate. The job planner produces
logical feeder, board, and safe-Z poses. The motion planner owns the conversion
from those logical poses to controller targets. Its state retains both poses
and the last X/Y approach side. A Z-only move therefore keeps the active X/Y
correction. A move that changes X or Y selects the correction measured for the
side that the axis approaches from. Corrections from simultaneous X and Y
motion are additive. Both the logical target and final controller target must
be inside the machine bounds.

A simulated job can select one calibration snapshot explicitly. The daemon
accepts its local direction response only when the artifact and all of its
dependencies are current, both axis vectors are zero-mean, and every enabled
vector has resolved-effect and improved-holdout evidence. Calibration runners
continue to send raw controller poses, so they cannot consume the result that
they are measuring. Active motion state binds both the snapshot UUID and
artifact fingerprint, because a recapture can update an existing snapshot.
Physical job execution remains disabled.

## Persistence and reproducibility

SQLite stores versioned JSON documents for boards, board registrations,
feeders, jobs, operations, inspection reports, calibration snapshots, and
calibration runs. Board-registration evidence images use a dedicated artifact
table. An
append-only event journal records machine state changes and operation results.
Calibration runs persist their phase, automatic checks, camera evidence,
per-cycle samples, rejection reason, and report after every meaningful stage.
An interrupted run is marked failed on daemon restart rather than appearing to
continue. Calibration snapshots retain parentage, dependencies, residuals, and
production-readiness status. Changing an upstream measurement invalidates all
dependent results and values.

Calibration results cross the trust boundary in one direction: a machine
runner may complete a step from controller or vision evidence, but API clients
cannot submit measured residuals or mark a step complete. Operator input is
limited to external conditions the attached machine cannot observe safely.
Installed tooling is a vision check: the runner presents each configured
nozzle to the bottom camera and compares its bore measurement with the
profile-scoped expected tip.

Head geometry uses the bottom camera as a common comparator. Small X/Y probes
measure its local image-to-motion Jacobian. At the shared camera pose, the
bottom camera locates the top-camera lens and the top camera locates the bottom
camera lens. Their corrections are combined and verified. Each detected nozzle
then rotates through bounded profile offsets. A harmonic fit separates the
stationary rotation axis and head offset from periodic nozzle-tip runout. The
raw samples, coefficients, and physical residuals are stored in the calibration
snapshot.

Local direction response uses the same top-camera datum and working-plane
Jacobian. Each sample returns to one commanded pose from a known side, so fixed
optical and datum offsets cancel. Adaptive paired statistics decide when enough
evidence exists. Direction-specific corrections remain disabled unless a
separate compensated holdout reduces the observed endpoint gap. This artifact
stores the zero-mean half-gap, so common camera drift is not encoded as
backlash. It is not a substitute for later multi-fiducial full-bed geometry.

Installed-geometry calibration uses the machine's existing fixed-height
targets. It moves the primary datum through the top-camera image and N1 through
the bottom-camera image. Independent holdouts select an affine inverse camera
map unless the distortion-polynomial map predicts unseen positions better.
The top-camera map then finds candidate mounting holes from the documented
staging- and build-plate grids. The profile identifies the datum-PCB plate as
the bottom, or south, staging plate. It identifies the adjacent north plate as
the top build plate. Their shared grid-edge relationship seeds build-plate
registration from the measured staging-plate offset. A three-pose probe
measures the local camera map at each plate height, and a second hole confirms
plate registration. Each candidate can be absent, occupied, clipped, or
ambiguous. The bounded detector rejects an implausible first frame immediately
and confirms plausible holes with the configured extra frames. Plate fits use
only accepted concentric hole/pad observations and must cover enough distinct
rows, columns, and physical span. Spatially distributed holdouts select an
affine correction only when it predicts better than the nominal grid;
otherwise the nominal map is retained. Each checkpoint stores partial samples
and separate training/holdout counts. Both configured plates must qualify.
This produces an empirical planar map for the installed working geometry. It
does not claim an unconstrained camera matrix for arbitrary depths or angles.

The vision crate owns the OpenCV boundary. It converts V4L2 grayscale frames
to owned OpenCV matrices and uses OpenCV for frame statistics, Laplacian focus
measurements, threshold masks, and connected-component labels. Domain-specific
detectors still own physical dimensions, concentricity, ambiguity rejection,
candidate scoring, and calibration acceptance. Attached-camera fixtures check
that this boundary preserves the validated detector outputs. Workspace scans
use the calibrated inverse planar model to plan a serpentine path and render
pixels directly into machine coordinates. OpenCV ORB feature matching and
RANSAC similarity estimation refine overlapping tile placement only inside
strict translation, rotation, scale, and residual limits. Machine geometry
remains the fallback when an overlap is blank, occupied, or rejected.
Streaming surveys also require two consistent neighboring registrations before
they apply a correction. This prevents an isolated match on a repetitive plate
grid from moving one tile away from its calibrated pose.

Large survey scans keep one V4L2 stream open across the motion path. The
streaming mosaic builder keeps only the current and prior full-resolution tile
rows. It samples them directly into lower-resolution corrected and nominal
mosaic accumulators, then releases the old row. Survey requests default to
mosaic-only retention and enforce estimated and actual artifact-byte quotas.
This avoids total-tile memory and storage growth while retaining calibrated
machine placement for low-texture overlaps.

Height-aware transient reconstruction decodes each source once per output
chunk. Candidate camera planes share those immutable frames and render in
parallel on a bounded number of CPU workers. Chunks remain sequential to keep
peak memory bounded. A zero-correction projection omits the duplicate nominal
accumulator; corrected scans retain it for edge fallback.

Coarse samples use a source-area mean instead of one source pixel. Blending
uses separate X and Y feather weights across the full overlap. This reduces
aliasing and periodic camera-shading bands without retaining more rows.

Workspace rasters use an overhead machine-coordinate convention. Pixel X
increases with machine X. Pixel Y increases as machine Y decreases, so the
upper-left pixel maps to `(x_min, y_max)`. Each scan report contains this
signed pixel-to-machine transform. Installed-plate presets also replace the
datum working-plane scale with the measured local plate-plane mappings. The
fitted grid and the source-defined 39-by-7 feature count define each V3 plate's
600 by 120 mm outline. The preset intersects that outline with the union of
camera coverage from legal head poses. The image can therefore extend past a
head-center travel limit when the camera can safely see that area. It cannot
include a physical plate edge that lies outside that optical union.

## Crates and applications

- `velocitypnp-domain`: geometry, machine state, manufacturing entities, and calibration graph
- `velocitypnp-driver`: simulator, Linux discovery, Marlin transport, Photon framing, and machine actor
- `velocitypnp-kicad`: direct `.kicad_pcb` parser and optional BOM merge
- `velocitypnp-openpnp`: one-way migration of user-owned OpenPnP XML into untrusted calibration/configuration seeds
- `velocitypnp-planner`: job validation plus logical-to-controller motion planning
- `velocitypnp-vision`: OpenCV image primitives, deterministic calibration policy, and native Linux V4L2 capture
- `velocitypnp-store`: SQLite migrations, entities, operations, and event journal
- `velocitypnpd`: REST/SSE API and static web hosting
- `velocitypnp`: command-line API client
- `web`: React/TypeScript operator interface
- `python`: standard-library Python API client
