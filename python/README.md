# VelocityPnP Python client

The client uses only the Python standard library and calls the daemon's
versioned HTTP API. It intentionally exposes typed machine operations rather
than raw G-code.

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
    snapshot = pnp.create_calibration("3286336D3331")
    report = pnp.calibrate(
        snapshot["id"],
        lease["id"],
        safety_checks_confirmed=True,
    )
```

`calibrate` waits for the persisted machine runner; it never sends residuals
or marks workflow steps complete. Use `start_calibration_run` plus
`calibration_run` when an application wants to display progress asynchronously.
