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
    lease = pnp.acquire_lease("calibration-script")
    ambient = pnp.read_sensor(lease["id"], "vacuum_1")
    # Set this only after physically checking the nozzles and travel envelope.
    pnp.home(lease["id"], safety_checks_confirmed=True)
```
