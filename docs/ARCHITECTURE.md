# Architecture

VelocityPnP separates policy from hardware so the same workflows run against a
deterministic simulator, recorded sessions, or a physical LumenPnP.

```text
React UI / CLI / Python
          │ HTTP + SSE
          ▼
    velocitypnpd API
          │ typed operations
          ▼
   single machine actor ───── event journal
          │
    MachineDriver trait
       ├── simulator
       └── Marlin serial ─── Photon RS-485 bridge

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

## Persistence and reproducibility

SQLite stores versioned JSON documents for boards, feeders, jobs, operations,
inspection reports, and calibration snapshots. An append-only event journal
records machine state changes and operation results. Calibration snapshots
retain parentage, dependencies, residuals, and production-readiness status.
Changing an upstream measurement invalidates all dependent results.

## Crates and applications

- `velocitypnp-domain`: geometry, machine state, manufacturing entities, and calibration graph
- `velocitypnp-driver`: simulator, Linux discovery, Marlin transport, Photon framing, and machine actor
- `velocitypnp-kicad`: direct `.kicad_pcb` parser and optional BOM merge
- `velocitypnp-openpnp`: one-way migration of user-owned OpenPnP XML into untrusted calibration/configuration seeds
- `velocitypnp-planner`: job validation and conservative placement planning
- `velocitypnp-vision`: deterministic image/geometry primitives, native Linux V4L2 capture, and optional OpenCV calibration
- `velocitypnp-store`: SQLite migrations, entities, operations, and event journal
- `velocitypnpd`: REST/SSE API and static web hosting
- `velocitypnp`: command-line API client
- `web`: React/TypeScript operator interface
- `python`: standard-library Python API client
