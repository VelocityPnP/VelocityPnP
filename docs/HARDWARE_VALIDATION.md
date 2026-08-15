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
- During the recorded session, connection sent only `M115`, `G21`, `G90`, and
  `M82`. The machine remained in preflight with no known pose and was never
  marked homed.
- The controller acknowledged bounded on/off pulses for the shared ring lights,
  VAC1, and VAC2. On 2026-08-15 the operator observed both ring lights and both
  vacuum channels working during separate five-second pulses. Each output was
  explicitly switched off afterward.
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
- A fresh Z-before-XY home followed by the exact 324 mm bottom-camera traverse
  at 300 mm/min validated the distance-derived completion timeout. The move
  completed in 65.0 seconds without a fault. A repeat exposure-200 frame was
  stable at mean 44.71 versus 44.07 and Laplacian variance 74.07 versus 72.23.
  The head then returned to X=0/Y=0/Z=31.5 at 600 mm/min and disconnected
  fail-safe with all outputs off.
- The exposure-200 top-datum and post-cleaning N045 frames are retained as
  byte-stable vision regression fixtures. Offline tests detect the datum's
  bright center island at approximately (646, 360) and the nozzle's dark bore
  at approximately (638, 355). This validates target polarity and fixture
  usability, not live multi-frame repeatability.
- The current REV04 pressure-sensor initialization and read commands completed
  on the attached controller. Both channels returned a stable ambient value of
  255 raw counts. With vacuum enabled and the nozzle opening sealed by the
  operator, N1/VAC1 returned 226 in all 10 samples and N2/VAC2 returned 235 in
  8 of 10 samples, with the other samples at 237 and 243. These are clear
  29-count and typical 20-count sealed-nozzle deltas; they are not calibrated
  pressure units or part-present thresholds.
- A fresh operator-confirmed Z-before-XY home completed in 6.2 seconds on
  2026-08-15. It reported X=0.000, Y=0.000, Z=31.500, and A=720.000 with no
  fault before the camera validation traverses.

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
These V4L2 values and geometry observations remain provisional calibration
seeds. The daemon now supplies and persists a role-scoped fully manual profile,
but its white-balance, gain, lighting, and reproducibility thresholds are not
production-approved until the live 30-frame checks pass on both cameras.

The first live 30-sample runs on 2026-08-15 had no capture failures and detected
the intended target in every frame. The top camera passed luma and focus range
but reported 0.990 px p95 center jitter against the provisional 0.5 px limit;
the first two detections formed a settling transient. The bottom camera passed
luma and center jitter at 0.120 px p95, but its 16.90 percent focus range
narrowly exceeded the provisional 15 percent limit. Both overall checks
therefore remain failed pending a settling/capture diagnosis; their thresholds
have not been relaxed to manufacture a pass. Those runs also exposed that the
endpoint reopened the device and discarded five frames for every sample. It now
collects the series from one stream after a single settling interval; attached
hardware reruns passed. The top camera detected 30 of 30 targets with 0.050 px
p95 center jitter, 1.09 percent focus range, and 1.06 percent luma range. The
bottom camera initially showed a monotonic lighting transient from mean luma
61.97 to 57.42. Increasing only its settling interval from 5 to 60 frames made
the subsequent 30-frame run pass with 0.156 px p95 center jitter, 2.54 percent
focus range, 0.576 percent luma range, and 30 of 30 detections. Exposure, gain,
white balance, lighting, and every acceptance threshold remained unchanged.

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
the V3 has no absolute nozzle-height feedback. At this stage, jogging remained
prohibited until that session's Z-first and XY stages passed. Rotation motion,
higher feed limits, cooperative cancellation, and the 30-cycle visual-refined
homing acceptance claim were unvalidated. `M112` is reserved for the explicit
emergency-stop operation. An ordinary actuator cleanup failure now faults the
machine, revokes its lease, and invalidates homing without sending `M112`.
Production job execution remains unavailable by design.

The driver has the current Opulo REV04 initialization and VAC1/VAC2 read
sequences implemented behind leased, profile-scoped raw sensor reads. The
connect path writes command register 48 with value 27 on both multiplexer
channels, then reads register 6 with `M261`. Those commands now have both unit
coverage and the attached-controller ambient/sealed evidence above. Pressure
conversion and useful per-tip part-detection thresholds remain uncalibrated.

The primary-datum fixture produces a bounded visual-home observation. A prior
+1 mm X probe moved the target approximately +75 raw pixels. On 2026-08-15 a
+1 mm Y probe moved it from raw Y=368.33 to Y=297.17, a -71.17 px shift. Thus
target error converts to machine correction with a negative X coefficient and
a positive Y coefficient. The probe exposed and corrected the provisional Y
sign copied from the tagged profile. The attached V3.0.4 profile now marks the
mapping validated. At this stage, apply mode still required physical
verification of its bounded correction and a post-move residual no larger than
one pixel before an XY-only `G92` reset. Later attached-machine evidence below
showed that this provisional pixel gate was not a valid machine-scale limit.

The first apply-mode run with the corrected signs completed successfully. Its
initial 2.238 px error requested a 0.0303 mm correction to X=220.1598 and
Y=190.2617. The verification target was 0.6985 px from center, so it passed the
1 px residual gate before the driver issued an XY-only `G92`; reported Z=31.5
and A=720 remained unchanged. The machine stayed homed, ready, and fault-free.

An exploratory repeatability run was stopped after seven complete coarse-home
observations because applying a precision p95 target to raw sensorless stall
homing is the wrong acceptance criterion. All seven coarse homes completed and
returned the datum within the visual capture and one-millimetre correction
bounds. Their proposed X corrections ranged from 0.7851 to 0.8157 mm; Y ranged
from -0.0237 to 0.3856 mm. The eighth already-submitted home completed before
the client loop stopped, leaving the machine ready at X=0, Y=0, and Z=31.5.
This is coarse-envelope evidence, not a failed precision claim. The remaining
30-cycle criterion measures the verified residual after visual refinement.

The first end-to-end repeatability cycle stopped before its excursion when the
single correction left a 1.306 px verification error against the unchanged
1 px gate. No `G92` was sent, the lights were disabled, and the machine stayed
homed, ready, and fault-free at the bounded corrected pose. Apply mode now
allows up to three fresh-frame correction iterations while constraining the
total offset from the configured seed to one millimetre. Attached validation of
that convergence change initially stopped after three corrections at 1.295 px,
again without `G92`. Five independent observation-only captures at the fixed
pose showed that increasing top-camera settling from 5 to 60 frames reduced
reopened-capture spread to about 0.11 px. A fresh end-to-end pilot then passed
with one correction, a 0.0114 mm/0.843 px verification residual, and a 0.0168 mm
independent residual after the fixed excursion and return. Both role defaults
now use 60 settling frames. Two subsequent ad hoc cycles passed with independent
return residuals of 0.0137 mm and 0.0106 mm. A fourth cycle stopped safely when
three correction attempts left 1.589 px of image residual. That is about
0.0215 mm at the current scale, below the overall 0.05 mm repeatability target,
but above the separate one-pixel apply gate; no `G92` was issued.

Further ad hoc cycling was stopped because a shell loop is not a usable or
auditable calibration product. The daemon now owns a persisted initial-machine
calibration run. After one external-safety confirmation, it performs combined
XYZ coarse homing; pulses and reads both vacuum actuator/sensor paths with
explicit off cleanup; moves to the top and bottom working views; captures camera
reproducibility; presents N1 and N2 to the bottom camera; compares their bore
diameters with the profile's N045 and N24 assignments; and executes the agreed
coarse-home, visual-refine, fixed-excursion, return, and independent-observation
sequence. Progress, per-cycle samples, rejection reasons, and the final report
are stored in SQLite and exposed through the API, CLI, Python client, and web
UI. The operator UI no longer accepts calibration metrics. The next 30-cycle
hardware gate used this first-class operation.

The first persisted physical run on 2026-08-15 exposed two useful gates before
the 30-cycle phase. Its top-camera check passed all 30 captures with 0.0391 px
p95 center jitter, zero capture failures, and stable focus and luma. N1 tooling
classification then rejected: all five frames produced small, spatially
inconsistent candidates averaging 0.163 mm rather than the expected N045 bore.
The run preserved those detections and stopped before presenting N2.

A revised detector rejected small components outside every known tip range and
required the selected feature to stay spatially stable. The next persisted run
passed the top camera at 0.0483 px p95 jitter and identified N1 as N045 in all
five frames. Its mean measured bore was 0.3922 mm, its diameter range was
0.0314 mm, and its p95 center jitter was 0.1183 px. N2 then produced no valid
feature because the low grayscale threshold isolated only a 43-to-44 px dark
core, not the reflective edge of its nominal 2.40 mm bore.

The first two-threshold hardware rerun again passed combined XYZ homing, both
vacuum command/read paths, the top camera, and N1. The top camera detected all
30 targets with 0.0404 px p95 jitter. N1 was classified as N045 in all five
frames with a 0.4047 mm mean diameter, 0.0157 mm diameter range, and 0.0962 px
p95 center jitter. N2 still rejected because its bore edge shifted above the
initial second threshold. Diagnostic captures at the rejected pose showed the
same 75-to-75.5 px edge at grayscale thresholds from 90 through 120. The
detector's locked thresholds are now 45 and 90: the first isolates N045 and the
second resolves the reflective N24 edge. The attached N24 frame measures
2.3687 mm at the current camera scale, inside the profile's nominal 2.40 plus
or minus 0.60 mm interval. A further persisted runner validation is required
before this is a passed N2 claim.

During that run the operator observed that the axis-scoped `G28 Z` command
moved the gantry toward X/Y zero before homing Z. Because the required operator
check already levels both nozzles, the staged sequence did not provide its
intended safety property. The run was already rejected at N1 when software
emergency stop was requested. The V3.0.4 profile now uses one standard `G28`
command and verifies its reported X/Y/Z result together.

The same emergency stop exposed a host-recovery defect: shutdown tried ordinary
actuator-off commands after `M112` had halted the Marlin parser, and a normal
reconnect could not complete its `M115` probe. The driver now records when it
has sent `M112`, releases that stopped serial session without redundant cleanup,
and sends the documented Marlin `M999` stop restart before identification on
the next explicit connect. Hardware validation of that recovery path follows
this software change; it is not claimed by the unit suite alone.

The first recovery attempts identified lower-level causes for the apparent
unbounded probe: serial `flush()` and input-buffer `clear()` entered Linux
`tty_wait_until_sent` because the stopped USB CDC controller was not draining
output. Complete short G-code lines are already queued by `write_all()`, so the
driver no longer requests an unbounded kernel drain. Stale `M999` replies are
consumed with a bounded asynchronous read-until-quiet.

After a hard controller reset cleared the wedged USB endpoint, the revised
connect path sent `M999` before `M115`, identified the attached firmware, ran
the non-motion setup, and completed preflight in 4.2 seconds. Machine state was
unhomed with no trusted pose, lease, or fault. This validates proactive `M999`
normalization on a responsive controller; it does not claim that firmware-level
recovery can succeed after this REV4 build stops servicing its USB endpoint.

The V3.0.4 bottom-vision seed uses the attached machine's observed camera
center and the tagged Opulo N1/N2 head offsets. The retained N045 fixture
measures a 0.422 mm bore and is automatically classified as N045. N24 uses a
higher threshold for its reflective bore edge while keeping the physical
2.40 mm model. At this stage, live N2/N24 classification remained part of the
next runner validation and was not yet passed attached-hardware evidence.

One conservative X/Y traverse exposed a completion-timeout defect. The
324 mm move from home to the N1 bottom-camera target needs about 65 seconds at
300 mm/min, but the original driver allowed only 60 seconds for `M400`. The
software entered a recoverable fault, revoked its lease, invalidated homing,
and then completed a fail-safe disconnect. A non-moving capture after reconnect
confirmed N045 at the intended target. The driver now reads `M114` before a
move and derives a bounded 60-to-900-second completion timeout from the
reported start pose, target, and feed. That correction has unit coverage but
was also validated by repeating the same physical traverse in 65.0 seconds,
past the old fixed limit, followed by a successful 32.6-second return move.

The first run with both grayscale thresholds found the real N24 bore in three
of five frames but selected small, unrelated features in two frames. The mixed
set had a 2.0706 mm diameter range and 50.7 px p95 center jitter, so the runner
rejected it. The detector now groups candidates across all frames and requires
a stable center and diameter. It also prefers features with a concentric edge.
A following run selected N24 in four of five frames at a stable 2.3530 mm and
passed both tooling checks and the bottom-camera check. This removed the
single-frame noise path without hiding missing detections.

That run completed five visual-home cycles, then the provisional one-pixel
verification rule rejected a 1.047 px residual. At the calibrated image scale,
the same residual was about 0.014 mm and was well inside the declared 0.05 mm
repeatability goal. Repository history showed that the pixel rule came from
the provisional visual-home implementation, not from an Opulo requirement or
attached-machine evidence. Visual-home acceptance now uses correction distance
in millimetres. Pixel residuals remain stored as vision diagnostics.

One further run showed that a stable N045 outer annulus can also resemble N24.
The multi-frame selector now ranks equal-support stable groups by the smaller
diameter before range and jitter. Synthetic tests cover this case, reflective
N24 detection, intermittent small noise, out-of-range components, and a stable
wrong tip. Live saved N1 and N2 frames also select their expected 12-to-12.5 px
and 75-to-75.5 px bore groups.

The complete physical runner validation then succeeded on 2026-08-15. Run
`e31a11fa-3ec7-4e81-b0b5-e7db5c7c2c80` belongs to calibration
`44f3476c-430a-4887-a247-7b008e1a9797`. Combined XYZ homing, both vacuum
command/read/off paths, both cameras, both tooling checks, and all 30 visual
homing cycles passed. The top camera detected 30 of 30 targets with 0.0481 px
p95 center jitter. The bottom camera detected 30 of 30 targets with 0.2766 px
p95 center jitter and zero capture failures.

N1 was identified as N045 in five of five frames. Its mean bore was 0.3828 mm,
its diameter range was 0.0157 mm, and its p95 center jitter was 0.0651 px. N2
was identified as N24 in five of five frames. Its mean bore was 2.3593 mm, its
diameter range was 0.0157 mm, and its p95 center jitter was 0.4018 px. The N1
and N2 p95 errors from the configured camera center were 29.14 px and 82.18 px.
Both are inside the current 100 px acquisition bound, but automatic head-offset
centering remains useful future geometry-calibration work.

All 30 visual-home cycles converged with one correction. Visual verification
had 0.0198 mm p95, 0.0226 mm maximum, and 0.0145 mm RMS residual. Independent
return observation had 0.0148 mm p95, 0.0156 mm maximum, and 0.0105 mm RMS
residual. Initial coarse-home corrections ranged from 0.0457 to 0.8296 mm,
which confirms that sensorless homing is a capture-range operation, not the
precision datum. The attached machine ended homed, ready, and fault-free. This
completes the persisted initial-machine runner and its visual-homing workflow
step. It does not make the calibration production-ready: motion geometry,
nozzle runout, and final placement validation still require their own evidence.

## Head geometry implementation pending physical validation

On 2026-08-15 the initial-machine runner was extended beyond the validated
visual-home stage. The new software first identifies N1/N045 and N2/N24 without
operator tip entry. It uses central ±0.5 mm X/Y probes to measure the local
bottom-camera image-to-motion map. It then has both cameras locate the opposing
lens and applies at most three reciprocal corrections, each bounded to 4 mm.
This phase runs after the first successful visual-home cycle, but its snapshot
steps are not committed unless the complete 30-cycle repeatability gate passes.

For each detected nozzle, the runner reads its current controller angle and
uses 12 alternating relative offsets from ±15 to ±165 degrees at 600
degrees/minute. This makes the first rotation small while retaining a broad
fit. N1 is mapped to controller axis A and N2 to B from the tagged V3.0.4
profile. The runner returns each axis to its captured start angle and fits the
stationary axis, top-camera head offset, and periodic tip-runout coefficients.
The runout fit must have a p95 physical residual no larger than 0.05 mm. Normal
physical jog rotation remains disabled, and no unrestricted rotation endpoint
exists.

Synthetic vision and fit tests, simulator actor tests, and command-generation
tests pass. This section is implementation evidence only. The opposing-lens
detector, A/B motion, measured offsets, and runout residuals do not yet have
attached-machine evidence.

## Procedure sources

- [Opulo V3 connect and bottom-camera calibration](https://docs.opulo.io/misc/calibration/2-connect-to-machine/)
- [Opulo V3 homing-fiducial calibration](https://docs.opulo.io/misc/calibration/4-homing-fiducial/)
- [Opulo V2/V3 G-code macro update guide](https://docs.opulo.io/guides/update-gcode-macros/)
- [Opulo actuator check](https://docs.opulo.io/byop/testing/actuator-check/)
- [Opulo LumenPnP V3.0.4 OpenPnP machine profile](https://github.com/opulo-inc/lumenpnp/blob/v3.0.4/openpnp/machine.xml)
