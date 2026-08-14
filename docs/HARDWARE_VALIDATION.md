# LumenPnP V3.0.4 hardware validation

This is the evidence log for the first attached-machine validation on
2026-08-14. It is deliberately split into facts observed by software and
checks that still require an operator or further implementation.

No homing, jogging, or other motion command was sent during this session.

## Attached hardware

| Device | Stable identity | Result |
| --- | --- | --- |
| REV4 controller | STMicroelectronics `0483:5740`, `/dev/serial/by-id/...MARLIN_OPULO_LUMEN_REV4...` | Detected and opened |
| Top camera | PnP Top Camera `32e4:9310`, capture node `video-index0` | Detected; capture pending |
| Bottom camera | PnP Bottom Camera `32e4:9310`, capture node `video-index0` | Detected; capture pending |

The two cameras report the same manufacturer serial, so role assignment must
also use their distinct USB product names and stable by-id paths. Metadata-only
`video-index1` nodes are excluded from capture candidates.

## Passed

- Physical mode requires the explicit pair `--simulate=false --allow-hardware`.
- Linux discovery assigned the REV4 controller and both PnP camera roles while
  leaving the laptop camera unassigned.
- Three bounded connect/disconnect cycles identified Marlin `bugfix-2.1.x`,
  build date 2022-10-03, `MACHINE_TYPE:LumenPnP`, zero extruders, and six axes.
- Connection sent only `M115`, `G21`, `G90`, and `M82`. The machine remained in
  preflight with no known pose and was never marked homed.
- The controller acknowledged bounded on/off pulses for the shared ring lights,
  VAC1, and VAC2. Software cannot confirm the physical light or pump response;
  that operator observation remains pending.
- API disconnect acknowledged explicit off commands for VAC1, VAC2, and the
  lights, then cleared pose, homing, and the control lease.
- SIGINT while physically connected ran the graceful disconnect path, exited
  successfully, and released the serial device.
- The sanitized M115 response is retained as a driver regression fixture at
  `crates/driver/tests/fixtures/lumenpnp-v3.0.4-rev4-m115.txt`.

## Camera blocker

Both cameras enumerated their expected formats. MJPEG supports 1280x720 at 30
frames per second; uncompressed YUY2 supports 1280x720 at 10 frames per second.
Attempts to open either capture node returned `EBUSY` even after PipeWire and
WirePlumber were excluded. Kernel diagnostics reported USB suspend/disable
timeouts. Targeted device and UVC resets did not recover the cameras, and the
shared camera hub did not re-enumerate after a logical reset.

The next step is a physical reconnect of the USB cable or hub carrying the Top
and Bottom cameras. After both stable by-id nodes return, capture one MJPEG frame
from each camera, inspect role/orientation/focus, and record controlled exposure,
white-balance, focus, and lighting settings. This is a host USB recovery issue,
not evidence of a VelocityPnP vision-pipeline failure.

## Still prohibited

Homing and motion remain out of scope until the camera gate is complete and an
operator explicitly confirms the actuator observations and physical preflight.
Production job execution remains unavailable by design.
