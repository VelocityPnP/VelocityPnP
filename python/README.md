# VelocityPnP Python client

The client uses only the Python standard library and calls the daemon's
versioned HTTP API. It intentionally exposes typed machine operations rather
than raw G-code.

```python
from velocitypnp import Client

with Client("http://127.0.0.1:4080/api/v1") as pnp:
    board = pnp.import_board("controller.kicad_pcb", bom="controller-bom.csv")
    pnp.connect()
    pnp.complete_preflight()
    lease = pnp.acquire_lease("calibration-script")
    pnp.home(lease["id"])
```
