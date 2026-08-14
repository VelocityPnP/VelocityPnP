# LumenPnP V3.0.4 hardware validation

This is the evidence log for the first attached-machine validation on
2026-08-14. It is deliberately split into facts observed by software and
checks that still require an operator or further implementation. The first
bounded physical homing and translation session completed on the same date.

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
- With the operator-confirmed nozzles initially level and the envelope clear,
  staged physical homing completed in 14.6 seconds. `G28 Z` established and
  verified Z=31.500 before `G28 X Y`; the final report was X=0.000, Y=0.000,
  Z=31.500, A=720.000 with no fault.
- Independent +5 mm X and +5 mm Y translations completed at 300 mm/min while
  holding Z=31.5 and A=720. Subsequent calibration moves completed at 600
  mm/min. The head returned to X=0/Y=0/Z=31.5 before a successful fail-safe
  disconnect cleared its pose and control lease.
- The role-scoped camera-control API restored bottom-camera manual exposure
  mode, exposure 200, and disabled dynamic frame rate after reconnect; all
  three values read back correctly before capture.
- A post-cleaning bottom-camera capture resolved the N045 bore and concentric
  tip edges substantially better. In the same 320 by 320 center crop, variance
  of the discrete Laplacian increased from 54.24 to 72.23 (about 33 percent)
  relative to the pre-cleaning exposure-200 frame.

## First working-plane observations

The top camera acquired the correct 1 mm datum target from the V3.0.4 nominal
seed. A +1 mm X probe measured approximately +75 raw pixels, consistent with
the 0.01351 mm/pixel seed. The manually centered coarse-home position was
X=220.131, Y=190.271, with the fiducial center within roughly one pixel of the
1280x720 image center. The image was optically sharp.

Using the V3.0.4 N1 head offset, the bottom camera acquired N1 at safe
Z=31.5. Separate +1 mm probes measured approximately +30 pixels in X and +32
pixels in Y. The manually centered head position was X=242.335, Y=215.353,
which implies a provisional bottom-camera location of X=218.574, Y=148.734
after applying the N1 offset. The operator confirmed N045 was installed on N1
and N24 on N2. The N045 center hole was resolved, but the image remained
optically soft. After the bottom camera was cleaned with alcohol, a new capture
at the same exposure materially improved local contrast and edge definition.
The nozzle bore is now usable as a vision target; repeat captures are still
needed before deciding whether physical focus adjustment is worthwhile.

For both cameras, automatic exposure 625 (62.5 ms) washed out useful detail.
Manual trials at 200, 100, and 50 selected 200 (20 ms) as the initial setting;
dynamic frame rate was disabled and all other controls remained at defaults.
These V4L2 values and geometry observations are provisional calibration seeds.
They are not yet persisted by the daemon or production-approved.

## Camera recovery and pending calibration

Both cameras enumerated their expected formats. MJPEG supports 1280x720 at 30
frames per second; uncompressed YUY2 supports 1280x720 at 10 frames per second.
Initial attempts to open either capture node returned `EBUSY`. Kernel
diagnostics reported USB suspend/disable timeouts, and targeted software resets
did not recover the devices. A physical reconnect of the Lenovo dock/camera USB
path restored both devices. Their `/dev/videoN` numbers swapped, while discovery
continued assigning the correct roles from stable by-id paths.

At initial enumeration both cameras used automatic white balance,
aperture-priority exposure, and dynamic frame rate. Manual white balance was
4600 K and exposure was 625 in the devices' 100 microsecond units, but both
controls were inactive while the automatic modes were enabled. Neither camera
exposes a focus control.

The initial captured Top scene was a nearly uniform plane; the Bottom scene
contained an out-of-focus obstruction or edge. Those were not the correct
calibration views.
After coarse homing, the top camera must be positioned over the machine's 1 mm
datum fiducial at the exact center of the Opulo logo. Nozzle N1 with the N045
tip must be positioned over the bottom camera at the V3 camera Z of 31.5 mm.
The ring lights remain on for both views. Camera defaults are retained while
automatic exposure is toggled on for settling and then off; exposure is the
first control to tune. Those machine-native targets are used to assess
focus/orientation and lock repeatable capture settings.

## Remaining prohibitions

Every fresh connection still requires the operator leveling heuristic because
the V3 has no absolute nozzle-height feedback. Jogging remains prohibited until
that session's Z-first and XY stages pass. Rotation motion, higher feed limits,
controlled-stop behavior, repeatability claims, and automated visual-home
coordinate correction remain unvalidated.
Production job execution remains unavailable by design.

One conservative X/Y traverse exposed a completion-timeout defect. The
324 mm move from home to the N1 bottom-camera target needs about 65 seconds at
300 mm/min, but the original driver allowed only 60 seconds for `M400`. The
software entered a recoverable fault, revoked its lease, invalidated homing,
and then completed a fail-safe disconnect. A non-moving capture after reconnect
confirmed N045 at the intended target. The driver now reads `M114` before a
move and derives a bounded 60-to-900-second completion timeout from the
reported start pose, target, and feed. That correction has unit coverage but
still requires a fresh physical traverse before it is hardware-validated.

## Procedure sources

- [Opulo V3 connect and bottom-camera calibration](https://docs.opulo.io/misc/calibration/2-connect-to-machine/)
- [Opulo V3 homing-fiducial calibration](https://docs.opulo.io/misc/calibration/4-homing-fiducial/)
- [Opulo LumenPnP V3.0.4 OpenPnP machine profile](https://github.com/opulo-inc/lumenpnp/blob/v3.0.4/openpnp/machine.xml)
