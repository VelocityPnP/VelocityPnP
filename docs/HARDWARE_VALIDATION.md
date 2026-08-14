# LumenPnP V3.0.4 hardware validation

This is the evidence log for the first attached-machine validation on
2026-08-14. It is deliberately split into facts observed by software and
checks that still require an operator or further implementation.

No homing, jogging, or other motion command has been sent as of this revision.

## Attached hardware

| Device | Stable identity | Result |
| --- | --- | --- |
| REV4 controller | STMicroelectronics `0483:5740`, `/dev/serial/by-id/...MARLIN_OPULO_LUMEN_REV4...` | Detected and opened |
| Top camera | PnP Top Camera `32e4:9310`, capture node `video-index0` | Native capture passed |
| Bottom camera | PnP Bottom Camera `32e4:9310`, capture node `video-index0` | Native capture passed |

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
- The non-moving unhomed M114 response is retained at
  `crates/driver/tests/fixtures/lumenpnp-v3.0.4-rev4-m114-unhomed.txt`.
- Each camera delivered 10 of 10 full-resolution YUYV frames at 10 frames per
  second after the USB connection was physically reconnected.
- The native VelocityPnP V4L2 backend resolved each stable role, discarded five
  settling frames, and returned valid 1280x720 grayscale PNG snapshots through
  the API. Both snapshots reported capture sequence 6.
- Native control enumeration succeeded for both cameras. Their control ranges
  and current values are identical.
- A non-moving `M114` probe against the attached REV4 firmware returned all six
  coordinates in the expected format. Before homing it reported X/Y/Z at zero,
  A/B at 720 degrees, and C at 10000. Coarse-home verification therefore checks
  the G28-established X/Y/Z reference while preserving the finite reported A
  rotation; nozzle rotation reference/limits remain a separate calibration.

## Camera recovery and pending calibration

Both cameras enumerated their expected formats. MJPEG supports 1280x720 at 30
frames per second; uncompressed YUY2 supports 1280x720 at 10 frames per second.
Initial attempts to open either capture node returned `EBUSY`. Kernel
diagnostics reported USB suspend/disable timeouts, and targeted software resets
did not recover the devices. A physical reconnect of the Lenovo dock/camera USB
path restored both devices. Their `/dev/videoN` numbers swapped, while discovery
continued assigning the correct roles from stable by-id paths.

Both cameras currently use automatic white balance, aperture-priority exposure,
and dynamic frame rate. Manual white balance is 4600 K and exposure is 625 in
the devices' 100 microsecond units, but both controls are inactive while the
automatic modes are enabled. Neither camera exposes a focus control.

The captured Top scene is a nearly uniform plane; the Bottom scene contains an
out-of-focus obstruction or edge. These are not the correct calibration views.
After coarse homing, the top camera must be positioned over the machine's 1 mm
datum fiducial at the exact center of the Opulo logo. Nozzle N1 with the N045
tip must be positioned over the bottom camera at the V3 camera Z of 31.5 mm.
The ring lights remain on for both views. Camera defaults are retained while
automatic exposure is toggled on for settling and then off; exposure is the
first control to tune. Those machine-native targets are used to assess
focus/orientation and lock repeatable capture settings.

## Still prohibited

Homing is now the prerequisite for working-plane camera setup. The V3 profile
homes the coupled nozzle Z axis first with `G28 Z` and verifies its 31.5 mm
level/safe coordinate before allowing `G28 X Y`. The second stage verifies X,
Y, and the preserved Z coordinate. Both stages require an operator to confirm
that both nozzles are initially level, the full travel envelope is clear, and
physical power isolation is within reach. Jogging remains prohibited until
both stages are verified from controller `M114` position reports.
Production job execution remains unavailable by design.

## Procedure sources

- [Opulo V3 connect and bottom-camera calibration](https://docs.opulo.io/misc/calibration/2-connect-to-machine/)
- [Opulo V3 homing-fiducial calibration](https://docs.opulo.io/misc/calibration/4-homing-fiducial/)
- [Opulo LumenPnP V3.0.4 OpenPnP machine profile](https://github.com/opulo-inc/lumenpnp/blob/v3.0.4/openpnp/machine.xml)
