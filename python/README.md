# VelocityPnP Python client

The client uses only the Python standard library and calls the daemon's
versioned HTTP API. It intentionally exposes typed machine operations rather
than raw G-code.

The package also installs `velocitypnp-models`. It downloads, verifies,
selects, and rolls back immutable model packages:

```bash
velocitypnp-models activate models.lock.json
velocitypnp-models current
velocitypnp-models rollback
```

Activation is atomic. The daemon verifies the active cache again at startup
and reports the selected lock and package checksums through `/api/v1/health`.

```python
from pathlib import Path
from velocitypnp import Client

with Client("http://127.0.0.1:4080/api/v1") as pnp:
    pnp.set_camera_controls("top_camera", {
        10094849: 1,    # manual exposure mode
        10094851: False,
        10094850: 200,  # 20 ms in V4L2 100 us units
    })
    Path("top.png").write_bytes(pnp.camera_snapshot("top_camera"))
    board = pnp.import_board("controller.kicad_pcb", bom="controller-bom.csv")
    pnp.connect()
    pnp.complete_preflight()
    lease = pnp.acquire_lease("calibration-script", duration_seconds=3600)
    ambient = pnp.read_sensor(lease["id"], "vacuum_1")
    # Set this only after physically checking the nozzles and travel envelope.
    pnp.home(lease["id"], safety_checks_confirmed=True)
    datum = pnp.visual_home(lease["id"])  # observation only by default
    snapshot = pnp.create_calibration("YOUR_MACHINE_SERIAL")
    report = pnp.calibrate(
        snapshot["id"],
        lease["id"],
        safety_checks_confirmed=True,
    )
    estimate = pnp.estimate_workspace_scan(
        snapshot["id"], region_preset="installed_plates",
    )
    survey = pnp.start_workspace_scan(
        snapshot["id"],
        lease["id"],
        region_preset="installed_plates",
        safety_checks_confirmed=True,
    )
    vacuum = pnp.vacuum_probe(
        snapshot["id"],
        lease["id"],
        "N1",
        safety_checks_confirmed=True,
    )
    reference_seal = pnp.characterize_vacuum(
        snapshot["id"], lease["id"], "N1", safety_checks_confirmed=True,
    )
    snapshot = pnp.observe_nozzle_height(
        snapshot["id"], "N1", 0.5, -0.5, 0.3, nozzle_z_mm=3.5,
    )
```

`calibrate` waits for the persisted machine runner; it never sends residuals
or marks workflow steps complete. Use `start_calibration_run` plus
`calibration_run` when an application wants to display progress asynchronously.
`observe_nozzle_height` records a bounded working-plane residual. It does not
move the machine or complete a workflow step. Pass
`post_correction_residual=True` when the inspected pose already used the stored
correction; the service then updates only that nozzle's remaining constant.

Workspace scans are asynchronous because a full survey can take close to an
hour. Use `workspace_scan(survey["scan"]["id"])` to poll progress, then use
`workspace_scan_mosaic` to retrieve the final PNG. The survey defaults retain
only the coarse mosaic and manifest, not every full-resolution camera frame.
They use 30 percent overlap and the active profile's validated default XY feed.
Installed-plate layered scans raise overlap to at least 60 percent for reliable
multi-height reconstruction.
Pass `feed_mm_per_min` only to request a slower scan.

Use `mode="reconnaissance"` with `region_preset="installed_plates"` for a
fast plate-plane inventory image. It defaults to 20 percent overlap,
0.20 mm/pixel output, and mosaic-only retention. It uses post-motion camera
timestamps instead of a fixed four-frame wait when the camera driver supports
monotonic timestamps. Raised objects can have local seams; use a focused
layered scan for precise height reconstruction after object discovery.

An installed-plate scan with `artifact_retention="full_sources"` or
`artifact_retention="transient_sources"` also measures a confidence-aware
surface map and makes a height-corrected mosaic. Transient retention deletes
the source frames after chunked reconstruction and keeps the final products.
Retrieve them with `workspace_scan_height_map`,
`workspace_scan_height_map_preview`, `workspace_scan_height_confidence`, and
`workspace_scan_height_corrected_mosaic`. Use `workspace_scan_height_overlay`
for the transparent non-plate height classes or
`workspace_scan_height_class_preview` for those classes composited onto the
mosaic. Bare plate and unknown pixels have no overlay. Unknown map cells remain
explicit; their display pixels use the endpoint projection with lower local
source disagreement. When retained sources contain the bottom camera, repeated
aperture detections fit its local depth projection. The agreeing views are
self-aligned and center-weighted to remove camera-well ringing. The scan report
records the result under `height_map.camera_aperture`.

After a reconnaissance refinement queue completes, call
`build_workspace_refinement_composite(parent_scan_id)`. This causes no motion.
It returns the separate image-quality result and stores it on the parent scan.
Use `workspace_scan_refined_mosaic` for the plain georeferenced PNG,
`workspace_scan_refinement_quality` for the measurements, and
`workspace_scan_refinement_quality_preview` for the outlined review image.
