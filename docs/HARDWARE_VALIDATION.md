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
step. At that point, motion geometry, nozzle runout, and final placement
validation still required their own evidence.

## Head geometry and runout validation

The results in this section are valid historical evidence for the earlier
detector and runner. The current camera-only transfer revision changes the N24
calibration feature and adds a live top-camera working-plane map. It requires a
new attached-hardware pass before these results apply to the revised path.

The complete head-geometry run passed on the attached machine on 2026-08-15.
Run `aead8ce5-2bbe-4491-8461-4797b12449a0` belongs to calibration
`f6acd39d-b509-4e42-b265-132c0efcf8d2` for machine serial `3286336D3331`.
All 14 runner checks passed. The run stored seven settled reciprocal-camera
frames in SQLite for later diagnosis.

The bottom camera used central ±0.5 mm probes. Its measured image motion per
machine millimetre was `[[27.8256, -0.3190], [0.5374, 28.0278]]`. Independent
bottom-camera corrections put the visible top-camera lens within 0.0403 mm of
the image centre. The top camera then measured a local map at the raised lens
plane. Its image motion was `[[39.3652, -0.0741], [0.0821, -42.0470]]` pixels
per machine millimetre. Independent top-camera corrections put the visible
bottom-camera lens within 0.0234 mm of the image centre.

Early runs exposed three bootstrap defects before rotation started. An
open-loop return error was incorrectly used as a mapping gate. The two camera
centres were incorrectly forced to share one controller pose and one movement
bound. A five-frame lens batch was rejected when one frame had no detection.
The final runner treats open-loop return as a diagnostic, centres each camera
from its own bounded seed, measures the local top-camera map, and requires a
stable three-of-five lens cluster. Saved hardware frames cover the large
cropped lens and the smaller concentric features near centre.

The runner identified N1 as N045 and N2 as N24 before rotation. Each nozzle
then used 12 alternating offsets from ±15 to ±165 degrees at 600
degrees/minute. Both controller axes started at 720 degrees and were restored
to 720 degrees after measurement. N1 runout radius was 0.0881 mm. Its fit had
0.00795 mm p95 residual. N2 runout radius was 0.1433 mm. Its fit had 0.00861 mm
p95 residual. Both are below the 0.05 mm fit-residual limit.

The measured N1 head offset was X=-22.1268 mm and Y=-70.7246 mm. The measured
N2 head offset was X=25.4152 mm and Y=-70.9452 mm. The N1-to-N2 rotation-axis
offset was X=47.5420 mm and Y=-0.2205 mm. The combined 24-sample runout fit had
0.00861 mm p95 and maximum residual, and 0.00421 mm RMS residual.

The same run completed all 30 visual-home cycles. Closed-loop verification had
0.0106 mm p95, 0.0129 mm maximum, and 0.00594 mm RMS residual. Independent
return had 0.0143 mm p95, 0.0153 mm maximum, and 0.00791 mm RMS residual. The
machine ended homed, ready, and fault-free. The snapshot now contains coarse
homing, camera reproducibility, visual homing, bottom-camera geometry, nozzle
offsets, and nozzle runout. It remains not production-ready because camera
intrinsics, motion geometry, Z planes, vacuum thresholds, and known-board
validation are still incomplete.

## Datum-board vacuum probe validation

The first N1 vacuum probe ran on 2026-08-16. It safely returned to the visual
home pose with no fault, but it did not lower N1. The initial profile had
incorrectly treated controller Z=31.5 as the datum surface. The operator saw
that the nozzle stayed level. The recorded open-nozzle median was 239 counts.
All samples from controller Z=32.9 through Z=31.5 settled at 234 or 235 counts.
This flat 5-count change was below the 10-count limit and was not contact
evidence.

An N2 probe started, but the operator reported the Z error before it lowered.
A normal disconnect stopped the run and commanded both vacuum outputs off. A
late cleanup request then changed the disconnected state to a recoverable
actuator fault. The cleanup path now treats output disable after disconnect as
an idempotent operation. No emergency stop was used.

Inspection of the tagged Opulo V3.0.4 files confirmed that controller Z=31.5
is the level home coordinate. Working Z=0 maps to controller Z=0 for N1 and
Z=63 for N2. The datum PCB puts the visual fiducial at (40, -25) and a 5 mm
front-mask opening at (17.5, -17.5). After the machine Y-axis inversion, the
nominal sealing target is X=-22.5 mm and Y=-7.5 mm from the visual fiducial.

The next N1 run used those mappings and did lower the nozzle. It ended homed,
ready, and fault-free with both vacuum outputs off. The open-nozzle median was
226 counts. Every five-sample median from nozzle Z=0.9 through Z=0.0 was also
226, so no vacuum transition occurred. The operator observed that the tip was
not centered on the pad, was already pressing into it before the pump started
at the nominal Z=1.0 approach, and compressed it further during the descent.
The pump also remained on during retract and return. This run disproved the
assumption that nozzle working Z=0 is the installed-tip surface coordinate.
The machine profile mapping alone cannot provide contact Z.

With both nozzles level, the top camera then observed the 5 mm target at the
nominal PCB coordinate. The new detector measured its center at X=653.223 px
and Y=359.084 px with a 377 px diameter. That is a machine correction of about
X=-0.179 mm and Y=-0.012 mm from the nominal target.

The revised operation re-identifies and centers the installed tip over the
bottom camera at its current angle. It visually centers the 5 mm target, moves
the measured tip over that center, and approaches only to nozzle Z=5.0 with
the pump off. It then starts the pump, descends slowly, and uses the vacuum
transition as contact. The pump stops before retract. The current conservative
search ends at nozzle Z=3.0, above the position that visibly compressed N1.
This lower bound must not be extended without new physical clearance evidence.

An attempted validation at 16:47 on 2026-08-16 accidentally launched the
previously linked daemon executable. `cargo test` had compiled the new test
harnesses but had not relinked `target/debug/velocitypnpd`. The stale daemon
repeated the old Z=1.0-to-0.0 descent and recorded the same flat 226-count
signal. An explicit VAC1-off request was queued during retract and completed at
16:50:19. The machine returned to visual home homed, ready, and fault-free;
VAC1 then read 0. It disconnected normally. No emergency stop was used. The
workspace was then explicitly built. The linked CLI and daemon timestamps are
09:50:42 and 09:50:51 local time, and the daemon binary contains the revised
safe-interval and camera-centering paths. A physical run of that linked
sequence was still pending at that point.

The explicitly built N1 sequence then passed its vacuum-transition gate at
16:58 on 2026-08-16. Before motion, the running daemon inode and SHA-256 were
verified against the linked file. Combined XYZ homing completed at X=0, Y=0,
Z=31.5. Bottom-camera vision identified N045 and centered the current tip at
head X=242.3331 and Y=215.4924.

Top-camera vision corrected the nominal 5 mm target by X=-0.1453 and Y=-0.0622
mm. Its verified residual was 0.0119 mm, and the final target observation was
X=640.065 px, Y=359.123 px with a 376.5 px diameter. The corrected target was
X=197.4857 and Y=182.7088 mm. The resulting head target was X=219.5365 and
Y=253.5248 mm.

VAC1 started at nozzle Z=5.0. Its open-nozzle median was 236 counts. Step
medians remained 234 from Z=4.9 through Z=3.7, then changed monotonically to
230 at Z=3.6, 228 at Z=3.5, and 226 at Z=3.4. The 10-count transition at Z=3.4
stopped descent. VAC1 switched off before retract. The machine returned to
visual home at Z=31.5 homed, ready, and fault-free. VAC1 then read 0 and VAC2
read 255.

Because the physical transition occurred at Z=3.4, the fallback search floor
was raised from Z=1.5 to Z=3.0. This leaves 0.4 mm for detection variation and
limits compression if the vacuum sensor does not respond. N2 had not yet been
validated at this stage; the height-model validation below later completed it.

The operator then reported that N1 was approximately X=-2 mm and Y=+2 mm from
the pad center. Thus the run passed contact detection but failed XY placement.
The two reciprocal camera poses explain the error. The pose where the top
camera saw the bottom-camera lens was X=220.2824 and Y=144.6764. The pose where
the bottom camera saw the top-camera lens was X=218.8359 and Y=146.8758. Their
difference is X=-1.4464 and Y=+2.1993 mm, which agrees with the observed nozzle
error.

The head-offset code had used the first pose. That pose came from a soft,
almost frame-filling image of the bottom-camera lens. A cross-camera bridge is
not mathematically invalid if both observations identify the same optical
axis. However, this image did not provide a reliable axis landmark. The new
bridge uses the second pose. Thus, the sharper top-camera lens and the nozzle
tip are measured in the same fixed bottom-camera image. With the observed N1
centering pose, the new current tip offset is X=-23.4972 and Y=-68.6166 mm. For
the prior corrected pad target, the predicted head command changes from
X=219.5365 and Y=253.5248 to about X=220.9829 and Y=251.3254.

The common-camera N1 verification ran from 17:09 through 17:14 UTC on
2026-08-16. The running daemon was verified against the new linked binary
before motion. Combined XYZ homing completed at X=0, Y=0, Z=31.5. The measured
current N1 tip offset was X=-23.5988 and Y=-68.6359 mm. Top-camera vision
corrected the nominal pad target by X=-0.1489 and Y=-0.0718 mm. Target
verification had a 0.0033 mm residual. The final head target was X=221.0809 and
Y=251.3351 mm. This is X=+1.5444 and Y=-2.1897 mm from the incorrect prior
command, in agreement with the required correction.

VAC1 started at nozzle Z=5.0. Its open-nozzle median was 236.5 counts. The
signal delta was 6.5 counts at Z=3.6, 8.5 counts at Z=3.5, and 10.5 counts at
Z=3.4. The transition stopped descent at Z=3.4, above the Z=3.0 safety floor.
VAC1 switched off before retract. The machine returned to visual home at
Z=31.5 homed, ready, and fault-free. VAC1 then read 0 and VAC2 read 255.
The operator reported that N1 was close to the pad center but was slightly at
X=positive and Y=negative. Thus, the common-camera change removed the large
error and slightly passed the center in the correction direction. The residual
magnitude was not measured. N2 remains prohibited until this small systematic
error is understood.

### N1 Z-axis tilt experiment

A one-off experiment on 2026-08-16 held head X=242.4347 and Y=215.5116 mm
over the bottom camera. It lowered N1 from controller Z=31.5 to Z=3.5 with the
pump off. The operator confirmed that the camera was below the workspace plane
before the full descent. The operation used vertical motion only while N1 was
lowered. It then retracted to Z=31.5 before any lateral motion.

The standard bore detector produced ten of ten stable measurements at Z=31.5,
28.0, 24.5, 21.0, 17.5, and 14.0. A local multi-threshold bore detector gave
five stable measurements at Z=10.5. At Z=7.0, four of five frames gave a
low-confidence bore center. At Z=3.5, the raw frame showed the complete nozzle
face, but the bore was too blurred for a valid numerical detection. The
machine reached Z=3.5 without contact with the camera.

The high-confidence fit from Z=31.5 through Z=10.5 predicts a Z=3.5 pixel
change of X=-10.61 and Y=+5.95 from the level measurement. Including the
low-confidence Z=7.0 point predicts X=-11.43 and Y=+6.52 px. The calibrated
bottom-camera map converts this range to a physical tip shift of approximately
X=-0.39 and Y=+0.23 mm. The direct Z=7.0 point already gives approximately the
same physical shift. Thus, N1 or bottom-camera tilt is measurable, but its
direction is opposite the operator-observed X=positive, Y=negative pad
residual. It does not explain that residual and must not be applied as a pad
centering correction without a separate contact-plane calibration.

After the tilt experiment, N1 was parked at the prior pad command X=221.0809,
Y=251.3351, and Z=3.5 with the pump off. The operator estimated that the nozzle
OD was 2 mm from the pad's top and left edges and 1 mm from its bottom and right
edges. This gives an approximate contact residual of X=+0.5 and Y=-0.5 mm.
The measurement is coarse because the edge gaps could not be measured more
precisely.

The measured Z-dependent tip shift is X=-0.39 and Y=+0.23 mm. It acts opposite
the contact residual. Removing that shift from the contact observation gives
an approximate level-plane camera-bridge bias of X=+0.89 and Y=-0.73 mm. Thus,
adding tilt compensation alone would make the observed pad error larger. The
calibration needs a 3D camera-ray, feature-height, and nozzle-contact-plane
model. A contact-specific manual correction can validate the next N1 run, but
it must not replace that general model.

### Height-model implementation status

The calibration runner now measures each centered nozzle at Z=31.5, 28.0,
24.5, 21.0, 17.5, 14.0, and 10.5. At each height, it makes four bounded 0.5 mm
X/Y probes and measures a local image-to-motion map. It then returns to the
fixed head position before it records the feature center. This directly
measures the projective scale instead of estimating it from a focus-dependent
feature diameter. An error path first retracts at the current X/Y position and
only then returns laterally at the level Z pose. The model fits XY shift per
millimetre of nozzle Z. It stores nozzle-axis shift separately from the
constant top-camera working-plane transfer correction.

The first attached-hardware N1 sweep completed all seven Z-only captures. Its
centroid path was smooth, but the detected diameter changed from 14.0 px to
7.6 px as focus changed. The diameter-scaled fit therefore produced a false
0.153 mm p95 residual and was rejected. The slope magnitude remained within
the physical bound. This result caused the change to measured local maps.

The revised attached run `a9ddb819-f74f-40be-bf61-799b0d3496be` passed both
seven-height local-map fits. N1 had a 0.00514 mm p95 fit residual with slopes
X=+0.00948 and Y=-0.00655 mm/mm. N2 had a 0.02838 mm p95 fit residual with
slopes X=+0.01056 and Y=-0.00383 mm/mm. N1's measured image scale increased
from approximately 27.7 px/mm at Z=31.5 to 40.0 px/mm at Z=10.5. This smooth
change confirms that the non-monotonic detected bore diameter was not a valid
perspective scale.

That run later rejected the separate 30-cycle independent-return gate at
0.05685 mm p95 against 0.05000 mm. All 21 one-correction cycles returned
between 0.00533 and 0.01482 mm. All nine two-correction cycles returned between
0.05108 and 0.06307 mm, with approximately four pixels of positive Y error.
This is direction/history behavior, not height-fit noise. The runner now makes
up to two bounded visual refinements from the standardized excursion-return
direction when a return exceeds the limit. It repeats the excursion and still
rejects persistent error.

Run `5aa1951b-4e06-42d4-89b9-12867e8671de` then completed the full attached
workflow. Eleven of 30 visual cycles used one direction-conditioned return
refinement; none used the second allowed refinement. Visual verification had
0.04936 mm p95 and independent return had 0.01243 mm p95. The final N1 height
model had slopes X=+0.009795 and Y=-0.005003 mm/mm with a 0.01213 mm p95 fit
residual. N2 had slopes X=+0.009461 and Y=-0.003357 mm/mm with a 0.02291 mm p95
fit residual. Snapshot `f6acd39d-b509-4e42-b265-132c0efcf8d2` persists the
accepted visual, reciprocal-camera, nozzle offset, runout, and height models.

A bounded data-only observation can solve the transfer correction from an
observed nozzle-to-target error and its Z. For the current coarse N1 evidence,
the input is X=+0.5, Y=-0.5 at Z=3.5 with approximately 0.3 mm uncertainty.
With the final accepted N1 slope, this produces a common top-camera
working-plane transfer correction of X=+0.77427 and Y=-0.64007 mm. The N1
axis term at Z=3.5 is X=-0.27427 and Y=+0.14007 mm, so vacuum targeting applies
the observed combined correction X=+0.5 and Y=-0.5 mm.

The corrected N1 probe applied X=+0.49902 and Y=-0.49950 mm at its prior
Z=3.4 alignment estimate. Top-camera pad centering had a 0.00426 mm residual,
and the live bottom-camera tip measurement differed from the fitted model by
0.0773 mm. The sensor change reached 9 counts from Z=3.2 through Z=3.0, one
count below the configured transition, so the operation retracted without
claiming contact. N1 was then parked at the same corrected X/Y pose and Z=3.5
with the pump off. The operator reported that the tip appeared centered. This
validates the XY correction but leaves N1 vacuum-threshold fitting open.

The working-plane transfer belongs to the fixed camera pair and board plane,
not to one nozzle. The observation API now records its source and propagates
the transfer to every independently fitted nozzle-axis model. N2 therefore
combined the N1-sourced transfer with its own axis slope. Its attached probe
applied X=+0.51409 and Y=-0.54775 mm at the Z=4.0 alignment estimate. The live
N2 tip differed from its runout model by 0.01983 mm, and top-camera pad
centering had a 0.04211 mm residual. VAC2 detected an 11-count transition at
nozzle Z=3.8, controller Z=59.2, then switched off and retracted before the
lateral return. The operation ended homed, ready, and fault-free. This passes
N2 contact detection.

N2 was then parked at the detected contact pose with VAC2 off. The operator
reported that X appeared centered but Y remained approximately -0.25 to
-0.5 mm from the pad center. Thus, X supports the shared-camera,
per-nozzle-axis decomposition, but a purely shared Y transfer is insufficient.
The model now retains the shared camera term and adds a per-nozzle constant for
post-correction residuals. The N2 observation is represented by the midpoint
Y=-0.375 mm with 0.125 mm uncertainty. A corrected N2 rerun remains required.

### Camera-only transfer revision

The prior N2 detector centered a roughly 75 px inner feature. The tagged Opulo
V3.0.4 machine profile instead uses a 140-to-170 px N24 outer-tip feature. The
revised profile therefore identifies N045 from its 0.45 mm bore and N24 from
its 4.86 mm outer tip. The detector carries this configured feature identity
through tooling checks, runout, and height tracking.

The revised runner measures the top-camera working-plane map with bounded
motions around the homing fiducial. It also measures the bottom-camera map,
centers the top-camera reference in the bottom camera, and centers the
bottom-camera reference in the top camera. The camera CAD supplies the 33 mm
bottom-camera cutout diameter. Its observed size, the live working-plane scale, and
the reciprocal controller poses produce the camera-ray transfer term. Each
nozzle still gets an independent runout fit and height-dependent axis model.

Vacuum magnitude does not influence XY calibration. The pressure signal is
used only for bounded contact-Z detection and later pickup/seal validation.
The earlier N1 shared-transfer and N2 residual observations remain historical
diagnostic data. A new automated run creates fresh camera-derived models and
does not require those operator measurements.

This implementation uses a pinhole distance-ratio model and the configured
outer camera silhouette. Offline replay finds the top-camera outer reference
at 523.3 px in the saved bottom-camera frame and the N24 outer tip at 169.5 px
in the saved level-height frame. The retained top-camera fixture finds the
bottom-camera reference at about 899 px. These offline detections provided
pre-motion checks. The attached results below now validate the live path.
Lens distortion and a full intrinsic calibration remain separate work.

The first revised attached run, `fb5d4cd2-ae5d-4c26-ad61-050a4a67cec0`,
passed coarse home, actuator checks, top-camera reproducibility, and N045 bore
identification. N045 measured 0.433 mm across five stable captures. N24 then
rejected with zero in-range detections. A diagnostic frame at the unchanged
safe pose showed that threshold 45 isolated the correct outer ring at 171 px
with a stable circle-fit centre. The original physical interval ended at about
170.2 px. The connected-component box includes both boundary pixel centres,
which can make the official 170 px Hough bound report as 171 px. The N24
physical tolerance is now 0.55 mm instead of 0.48 mm; no centre or identity
measurement was entered manually. A rerun is required.

Run `adc50a38-741a-40f4-940f-a18d9add9ac2` then accepted N045 and N24 and
passed all 30 visual-home cycles. Independent return had 0.01867 mm p95 and
0.01869 mm maximum residual. The top working-plane and bottom-camera maps
passed. Reciprocal 15 mm camera references produced a +1.158 mm X and
-1.965 mm Y working-plane transfer, 2.281 mm total. N1 runout was 0.0849 mm
with a 0.0052 mm maximum fit residual. Its seven-height model had slopes
X=+0.00992 and Y=-0.00537 mm/mm with a 0.00986 mm p95 fit residual. N2
runout was 0.1358 mm with a 0.0075 mm maximum fit residual.

The run reached the sixth N2 height map after 105 minutes. Its one-hour lease
had been renewed once during observation, but it expired before the next
machine command. The runner failed without a controller fault and could not
make its normal retract with the expired lease. A fresh lease then moved only
Z from controller 49.0 to the level 31.5 pose. The machine ended homed, ready,
and fault-free. The runner now refreshes its existing lease at every long-run
measurement checkpoint. This is an operation keepalive, not a replacement
lease or an expansion of machine authority.

Three following attempts exposed reliability defects without producing an
accepted calibration. Run `3edebe56-4fac-440b-b0bf-f8976168494a` reached N2
rotation but let the full tooling classifier select a reflective feature at
-75 degrees. Run `7ffd7558-68bd-4b44-bb58-a7b1a00ceda3` stopped when the top
camera capture node re-enumerated during visual homing. Run
`cff2967e-4cb8-48e7-8b3b-557e8349f8c6` showed that the live N24 outer edge
needed grayscale threshold 60. The runner now tracks the already-verified tip
through rotation with bounded thresholds, retries transient capture failures,
and retains every initial tooling frame. Three live N24 frames are regression
fixtures; threshold 60 finds a stable 170-to-171 px outer edge in all three.

The terminal revised run
`d100539f-1901-4724-80af-6b7b9efc0ea9` succeeded on 2026-08-17. It belongs to
calibration `b6716644-422b-4cda-b48c-0ca9816c250e`. All 18 checks passed,
including combined XYZ home, both actuator/sensor paths, both camera checks,
tool identity, 30 visual-home cycles, reciprocal camera geometry, both runout
fits, and both seven-height fits. N24 was identified from its outer tip in all
five initial frames. The detections were 169.5-to-171.0 px with 0.0863 px p95
centre jitter. Both cameras captured 30 of 30 samples with no failures. The
top and bottom p95 centre jitter values were 0.0395 px and 0.1169 px.

All 30 visual-home cycles passed. Independent return had 0.01614 mm p95,
0.04793 mm maximum, and 0.01057 mm RMS residual. Closed-loop verification had
0.01058 mm p95, 0.01350 mm maximum, and 0.00745 mm RMS residual. This result
stays within the machine-scale 0.05 mm gate without a provisional pixel rule.

The reciprocal camera observations had 0.03989 mm and 0.04413 mm residuals.
The camera-ray fit measured a working-plane scale ratio of 0.84047 and produced
a common transfer of X=+1.16442 mm and Y=-1.99436 mm. N1 and N2 head offsets
were X=-23.38259/Y=-68.36371 mm and X=+24.16884/Y=-68.69924 mm. Thus, the
measured N1-to-N2 axis offset was X=+47.55143 mm and Y=-0.33553 mm.

N1 runout radius was 0.08095 mm with a 0.00639 mm p95 fit residual. N2 runout
radius was 0.13457 mm with a 0.00720 mm p95 fit residual. All 12 angles passed
for each nozzle, and the run stored exactly 24 angle artifacts. N1's height
slopes were X=+0.009911 and Y=-0.006522 mm/mm with a 0.01116 mm p95 residual.
N2's slopes were X=+0.011013 and Y=-0.002927 mm/mm with a 0.01342 mm p95
residual. Both seven-height models retain the camera-derived X=+1.16442 mm and
Y=-1.99436 mm transfer. They contain no vacuum or operator-entered XY term.

The same control lease remained active for the complete 112-minute run. No
external lease renewal was sent; checkpoint refreshes advanced its expiry
through both height sweeps. The runner stored 10 tooling, 6 reciprocal-camera,
24 runout, and 14 height artifacts. It ended at the level Z=31.5 pose, homed,
ready, and fault-free after explicit actuator cleanup. The snapshot contains
every current machine-geometry step. It is correctly not production-ready
because the remaining motion, vacuum, feeder, and job gates are not complete.

## Selective artifact migration and vacuum probes

Scoped head-geometry run `7341de88-577b-423a-9b44-5d43d1215102` succeeded on
2026-08-17 and produced snapshot `fce63a82-a847-4550-97bc-aa6d03dd3da9`.
It reused the validated parent homing evidence, performed one live visual
alignment, and captured the camera maps, reciprocal geometry, both runout
fits, and both height-axis models. The corrected 33 mm camera cutout produced
a camera-ray transfer of X=+0.530461 and Y=-0.908262 mm. N1 and N2 runout
radii were 0.07852 mm and 0.12899 mm.

The snapshot then migrated to 16 provenance-bearing artifacts. The migration
retained physical source captures and recalculated the camera-ray transfer,
head-offset summary, and both runtime tip transforms without motion. The
planner classified all geometry as reusable and requested only independent N1
and N2 vacuum-contact captures.

Both physical vacuum probes passed on 2026-08-17. N1/N045 contacted at nozzle
Z=3.2 with a 10-count signal delta and 0.02308 mm target-centering residual.
N2/N24 contacted at nozzle Z=3.8 with a 10-count signal delta and 0.06725 mm
target-centering residual. Their open-nozzle medians were 236 and 252 raw
counts. Each probe started the pump at the clear search height, moved only Z
during descent, stopped the pump before retract, and returned to X=220.131,
Y=190.271, Z=31.5. The controller ended homed, ready, and fault-free.

The first N1 persistence exposed a digest timing defect: one JSON
floating-point representation changed during its first storage round trip.
The semantic profile and dependency fingerprints were unchanged, but the
planner correctly refused to reuse the mismatched digest. Artifacts now
normalize through their storage representation before SHA-256 calculation.
A regression test covers the contact value, and the stored N1 report migrated
to algorithm `vacuum_contact_probe.v3` without another physical probe.

The initial post-probe contact plan reported no non-reusable artifacts, no
required motion, and a zero-second estimate. Those initial N1 and N2 contact
artifact fingerprints started with `df707f490307` and `6aca004909b6`.

## Reference-seal vacuum characterization

Automatic reference-seal characterization passed for both installed tips on
2026-08-17. The runner reused the current contact artifacts, performed a live
visual home, re-centered each tip with the bottom camera, and re-centered the
5 mm sealing target with the top camera.

The first N1 attempt sampled the supposed open state at nozzle Z=3.5. The
signal fell from 232 to 228 counts before descent because this height was
already in the seal transition. The runner did not meet its 10-count relative
contact criterion, stopped the pump, retracted, and returned to the visual-home
pose without moving below the stored Z=3.2 contact bound.

The corrected sequence samples the pump-on open state at the known-clear
nozzle Z=5.0. It then approaches at 300 mm/min to 0.3 mm above the stored
contact and samples only that final interval at 60 mm/min. The search never
moves below the previously validated contact Z.

N1/N045 measured an open median of 236.5 and a sealed median of 226.0 counts.
Its robust distribution gap was 9.45 counts. The reference-seal acquire and
release thresholds are 229.675 and 232.825 counts. N2/N24 measured an open
median of 252.0 and a sealed median of 240.0 counts. Its robust gap was 12.0
counts, with acquire and release thresholds of 244.0 and 248.0 counts. Both
sensors report lower values for more vacuum.

The pump was disabled while each nozzle remained stationary on the target.
Ten release samples recorded the pressure decay before vertical retract. N1
and N2 characterization artifact fingerprints start with `81dd78fc51d7` and
`e0c326ecd9f6`, respectively. The final planner reports both branches reusable,
zero required motion, and a zero-second estimate. The controller ended homed,
ready, and fault-free at X=220.131, Y=190.271, Z=31.5. Both pump-off sensor
values matched their established baselines: N1=0 and N2=255.

These thresholds validate only the fixed reference seal. They do not validate
that a production part is held; known-part distributions remain pending.

## Local X/Y direction response

The adaptive local direction-response runner passed on the attached V3.0.4 on
2026-08-17. It reused the current visual-home qualification and top-camera
working-plane map. It performed one live visual home, then approached the same
datum pose from both sides of X and Y at 1 mm and 5 mm distances and 300 and
900 mm/min feeds. Both axes met the relative uncertainty rule after the minimum
three pairs. No fixed millimetre acceptance rule was used.

The first physical report exposed a model error before commit. Its holdouts
improved, but each stored direction vector included a common image offset as
well as the direction difference. Algorithm v2 removes the common term and
uses opposite half-gap vectors whose mean is exactly zero. The artifact planner
invalidated only the v1 local direction artifact; all camera, homing, nozzle,
and vacuum artifacts remained reusable. A second physical run validated v2.

X measured a 0.06486 mm primary direction gap with a 0.01320 mm 95% confidence
interval. Its 0.01046 mm cross-axis gap is retained as diagnostic evidence. The
zero-mean X holdout used `(+0.03243, -0.00523) mm` and
`(-0.03243, +0.00523) mm`; the direction-vector gap fell from 0.06570 mm to
0.01865 mm.

Y measured a 0.04607 mm primary direction gap with a 0.01421 mm 95% confidence
interval and a 0.00009 mm cross-axis gap. Its zero-mean holdout used
`(+0.00004, +0.02304) mm` and `(-0.00004, -0.02304) mm`; the direction-vector
gap fell from 0.04607 mm to 0.02647 mm. Both axis corrections are stored as
holdout-enabled. The current artifact uses
`local_xy_direction_response.v2` and fingerprint prefix `c1c6e2114541`.

The final default artifact plan reports every target reusable, no required
motion, and a zero-second estimate. Both vacuum outputs remained off at their
known baselines, N1=0 and N2=255. The machine ended homed, ready, and fault-free
at X=220.13, Y=190.27, Z=31.5. Manual jog and calibration moves do not yet use
the stored correction; consumption belongs to the future physical motion
planner.

## Installed camera and plate geometry

Initial installed-geometry run `8fc53288-e0d9-46ca-b342-37a0d0ef6798`
qualified the staging plate on the attached V3.0.4 on 2026-08-18. It
requalified 25-position top- and bottom-camera sweeps from run
`f0cfaf51-c278-40ff-89d5-e7d02643f1cd`. The top-camera distortion model had a
0.04544 mm holdout RMS. The bottom-camera model had a 0.03601 mm holdout RMS.
Both models used eight spatial holdouts.

The staging plate registered from two holes. Its controller-to-grid offset was
(-0.20247, -4.33066) mm. A three-pose probe measured this local
pixel-error-to-machine map at the plate height:

```text
[[-0.0180044,  0.0000284],
 [ 0.0002130,  0.0180416]] mm/pixel
```

The occlusion-tolerant scan retained 12 empty holes and rejected eight covered
or ambiguous positions. The retained holes covered seven columns, all seven
rows, 420 mm in X, and 90 mm in Y. Three spatial holdouts had a 0.13538 mm RMS
and a 0.22953 mm maximum residual. The selected affine plate model was:

```text
[[1.0014176, -0.0001498],
 [0.0047962,  0.9999571]]
translation = [-0.21763, -4.38178] mm
```

Live tuning exposed three runner defects. A half-frame camera coverage rule
rejected a valid bottom-camera sweep by 0.245 pixels. The rule now requires
45 percent coverage while the independent holdout must still pass. The first
plate scan used datum-plane scale for the lower plate. A local three-pose map
now supplies the plate-height scale. The initial holdout expression assigned
whole alternating-hole columns to the test set. The current expression
distributes holdouts across valid holes and rows.

That run resumed 17 staged samples from interrupted run
`8b425ffd-6b26-4d1b-beb7-23ede58f3470`. It captured only three missing staging
samples. No build-plate grid feature was visible at five predicted poses. The
runner classified the plate as unavailable and retained this as a limitation.
It did not search the remaining offset hypotheses. The complete run passed
under the earlier one-plate acceptance rule. It was not a full two-plate
validation.

The plate identities are now explicit. The south, or bottom, plate carries the
datum PCB and has the staging role. The adjacent north, or top, plate has the
build role. The build plate references the staging plate's shared grid edge,
and its registration starts with the measured staging offset. Compatible
progress is reused per plate, so a build-profile correction does not require a
new staging scan. A run passes only when both configured plates qualify.

Run `e8ea8048-15a9-4160-8fc0-4760ab8f6f0e` first qualified both plates. Its
build correction differed from the staging correction by 14.87329 mm in Y.
This appeared to expose one extra 15 mm row pitch in the nominal build origin.
At that time, the profile placed the adjacent edge rows at the same Y and
changed the build origin from 108 mm to 93 mm relative to the visual datum.

Corrected run `58d959ee-33a4-4b62-b9fb-2c6722c392dc` passed on 2026-08-18.
It reused both camera sweeps and all 20 staging samples. The build plate
registered on the zero-offset hypothesis from two holes. Its measured
registration was X=-0.28536 and Y=-3.88047 mm, compared with staging at
X=-0.20247 and Y=-4.33066 mm. No grid-pitch correction remained.

The final scan retained 12 holes on each plate and rejected 16 occupied or
ambiguous targets. Staging covered seven columns, seven rows, 420 mm in X, and
90 mm in Y. Its three holdouts retained the prior 0.13538 mm RMS and 0.22953 mm
maximum residual. Build covered nine columns, six rows, 330 mm in X, and 90 mm
in Y. Its four holdouts had a 0.05384 mm RMS and a 0.06689 mm maximum residual.
Both affine models improved their nominal holdouts. There were no plate
limitations.

The later workspace survey showed that this conclusion selected the wrong
alias of the repetitive 15 mm grid. The tagged V3.0.4 KiCad source defines a
600 by 120 mm plate with 39 numbered columns, seven lettered rows, and one
pitch from each outer feature center to the PCB outline. The old profile
declared 40 columns and made the two plate grids overlap by 30 mm. It therefore
cut the two north rows from the build-plate survey while its observed holes
still fit the periodic grid. The corrected profile uses 39 columns, a two-row
center gap at the touching plate edges, and a build Y origin of 123 mm. This
profile correction made the previous installed-geometry artifact stale.

Run `74d82ef6-74cb-43aa-859d-0edcab891e37` requalified the corrected geometry
on 2026-08-18 local time. It reused both current planar camera sweeps and the
compatible staging measurements. The build plate registered from two holes at
X=-0.26437 and Y=-3.80493 mm. Twelve build samples covered eight columns, four
rows, 270 mm in X, and 90 mm in Y. Its three holdouts had a 0.06088 mm RMS and
a 0.09275 mm maximum residual. Both plates passed with no limitation. The
machine returned homed, ready, and fault-free to the visual datum.

Two corrected-profile attempts were stopped during diagnosis because the
initial 290 mm safe-Z move from coarse home to the visual datum was mistaken
for a stalled capture. The event journal confirmed the expected motion. A
direct one-frame stream test also passed for both cameras. No emergency stop
or controller reset was used. The successful run returned homed, ready, and
fault-free at X=220.131, Y=190.271, Z=31.5.

## OpenCV adoption smoke test

The test-only legacy oracle and the OpenCV detector produced equal candidate
counts, diameters, and confidence values on all seven attached-camera
fixtures. Candidate centers differed by no more than `1e-6` pixel. Raw
four-connected components also matched on 64 deterministic generated frames
at dark and bright threshold boundaries. OpenCV luma, deviation, and focus
statistics matched the prior calculation on all seven fixtures.

The physical smoke test ran on 2026-08-18 after one standard combined XYZ
home. The new daemon captured three frames at each target. The top datum, N045
on N1, and N24 on N2 were detected in all nine frames. Top-camera mean target
position was (641.822, 355.903) pixels. N1 center jitter p95 was 0.042 pixels;
N2 center jitter p95 was 0.111 pixels. Both tooling reports passed. The machine
returned ready and fault-free to X=220.131, Y=190.271, Z=31.5, and the lights
were switched off.

## Workspace scan and stitching

The final attached-machine scan ran on 2026-08-18 as scan
`7f55f58a-ac6f-47a0-9f3a-e289ddf959e0`. It covered X=211–229 mm and
Y=186–194 mm around the datum PCB with four tiles in a 2-by-2 serpentine path.
The calibrated output scale was 0.013248 mm/pixel, which produced a
1360-by-605 grayscale mosaic.

All four horizontal and vertical overlap registrations passed. Their inlier
ratios were 0.969–0.995. The p95 residual range was 0.0493–0.0554 mm. Scale
estimates were within 0.42 percent of unity, and absolute rotation estimates
were at most 0.207 degrees. The renderer covered 99.996 percent of the region.
Only 0.313 percent of pixels used nominal machine placement to fill outer-edge
space exposed by accepted registration corrections. The mosaic SHA-256 was
`75d0f5f1447a765038e5872bead4d6f2a32459d4db533bce80104e0ef145afb4`.

The first live plan used the distortion model's outer bounding box and covered
97.6 percent. Visual inspection found narrow black edge wedges. Planning now
uses a sampled, inscribed camera footprint with a two-pixel margin. A second
scan covered 99.5 percent before nominal edge fallback was added. The final
mosaic had no visible seam or edge gap. The acceptance gate is now 99 percent.

For that precision scan, each raw tile, the mosaic, and the JSON manifest
remain in SQLite. The scan does not change calibration data. It parks at the
visual datum and switches the lights off after report finalization. The final
machine state was homed, ready, and fault-free at X=220.131, Y=190.271,
Z=31.5.

### Streaming datum-PCB survey

Scan `57efc86e-1b2f-4052-9ea6-4f4ff6dae09e` exercised the streaming survey
path over the complete V3.0.4 datum-PCB bounding box. The tagged drawing puts
the PCB at X=0–80 mm and Y=-35–0 mm, with the visual datum at (40, -25).
After the installed Y-axis inversion, the requested machine region was
X=180.131–260.131 mm and Y=165.271–200.271 mm.

The preflight estimate specified 72 tiles in an 8-by-9 serpentine grid, 149
seconds, and 411,282 retained bytes. The attached scan completed in 125.0
seconds. It produced an 801-by-351 grayscale mosaic at 0.10 mm/pixel with 100
percent region coverage and 0.202 percent nominal edge fallback. Fifty-seven
of 127 overlap registrations passed. Textureless or rejected overlaps used the
calibrated machine placement.

Mosaic-only retention stored 166,004 image bytes and an 89,810-byte manifest,
or 255,814 bytes total. It stored no full-resolution source tiles. The mosaic
SHA-256 was
`86efd4921d91870b787b9edd3596365d2ff2f829779347da5bbb3e8c89e0a4c3`.
Visual inspection showed the complete PCB outline, mounting holes, fiducials,
sealing targets, and central grid without a visible broken seam. The machine
returned ready and fault-free to X=220.131, Y=190.271, Z=31.5 with the lights
off.

The first survey raster put `y_min` at its top edge and was vertically
inverted relative to an overhead machine view. It was superseded. The renderer
now puts `y_max` at the top edge. The report gives the signed transform:
pixel X advances with machine X, and pixel Y advances against machine Y.

### Streaming installed-plate survey

Scan `6ecb42ce-c310-4b5d-9c6a-8f252c3dced7` covered the complete configured
staging and build plates. It used the current installed-plate geometry
artifact, not the datum working-plane scale. The measured and machine-clamped
region was X=0–433 mm and Y=91.038–288.440 mm. The preflight specified 1,054
tiles in a 31-by-34 serpentine grid, 2,260 seconds, and 10,754,405 retained
bytes.

The attached scan completed in 2,073.2 seconds. It produced a
4,331-by-1,976 grayscale mosaic at 0.10 mm/pixel with 100 percent coverage and
no nominal edge fallback. Mosaic-only retention stored a 4,420,970-byte PNG
and a 1,293,122-byte manifest, or 5,714,092 bytes total. The mosaic SHA-256 was
`8b20e88ce2f5814c20ea7710072ba38722156e5663375ad4176524bacb10a711`.

Nine of 2,043 ORB registrations passed the strict geometry gates. Of the
rejected overlaps, 1,964 had too few plausible matches and 70 exceeded a
transform gate. This is expected on the repetitive, low-texture plate grid.
Those overlaps used calibrated machine placement. Visual inspection showed
the north build plate and south staging plate in the correct orientation, with
aligned grid features and no broken seam. The machine returned homed, ready,
and fault-free to X=220.131, Y=190.271, Z=31.5 with the lights off.

An earlier full scan was stopped normally after 322 tiles. Its plan used the
datum-plane camera scale for the lower installed plates and would have
distorted the workspace. The installed-plate preset now rescales the complete
top-camera distortion model to the mean measured plate-plane mapping and uses
the fitted plate affine models for its region. This reduced the live plan from
1,886 to 1,054 tiles. No emergency stop was used.

Tuned edge pilot `cf8cce92-eefc-4b13-9062-af8d0ed618c2` used 30 percent
overlap, 1,200 mm/min motion, four queued-frame discards, source-area coarse
sampling, and full-overlap feathering. It captured 296 tiles in 356.2 seconds,
or 1.20 seconds per tile, and retained a 1,019,517-byte PNG. The prior full
survey used 20 percent overlap at 600 mm/min and took 1.97 seconds per tile.
The pilot had 100 percent coverage and no nominal fallback. Visual inspection
showed materially smoother seams. It also made the old profile's 30 mm north
crop visible; this led to the source-geometry correction above.

Corrected full-height pilot `cbe7f0b8-c66f-4886-ada5-85713f3f8fa0` then
covered X=-8.021 to 80.000 mm and Y=83.502 to 326.067 mm. It captured 336
tiles in 412.3 seconds, produced an 882-by-2,427 image, and retained 1,159,261
bytes. Coverage was 100 percent with no nominal fallback. Visual inspection
showed all seven rows on both plates, both north and south PCB edges, and no
broken tile seam. Only two of 622 repetitive-grid registrations passed; the
other tiles retained calibrated machine placement. The report explicitly
warns that X-min and X-max are clipped by top-camera reach. The source-defined
plate extends to about X=-80 and X=520 mm, while the calibrated camera union is
only X=-8.021 to X=441.020 mm. The scan therefore starts at visible column 5;
columns 1 through 4 cannot be captured on this machine without more X travel.

Full corrected survey `dc69e4ea-ec38-472a-88f9-a38063805285` completed on
2026-08-19. It captured all 1,680 tiles in a 40-by-42 serpentine raster with
30 percent overlap, 1,200 mm/min motion, and four queued-frame discards. It
finished in 1,997.9 seconds, compared with the 2,205-second estimate. This was
75.3 seconds faster than the earlier cropped full scan despite using 59 percent
more tiles, more overlap, and 45.2 mm more Y coverage.

The final 4,492-by-2,427 mosaic covered 100 percent of the requested optical
region with no nominal fallback. Its PNG is 4,452,522 bytes. The PNG and
manifest retain 6,534,412 bytes in total. Twenty-nine of 3,278 registrations
passed their local gates, but no tile received a correction because no result
had the required two-neighbor support. Visual inspection showed both complete
seven-row plates, both Y edges, and no black gap or broken seam. The mosaic
SHA-256 is
`7bd203407e9e1439899c0f5062f53799cd3c0d3a3543bf48b5a3ef01830d8745`.
The machine returned homed, ready, and fault-free to X=220.131, Y=190.271,
Z=31.5, with the lights off.

### Datum-PCB alignment tuning

Source-retaining scan `38ec929d-1676-45ae-a6d7-f63b493ae781` isolated a
90-by-45 mm datum-PCB region in 64 tiles. All 112 old whole-frame ORB
registrations were rejected, so its mosaic used only controller placement.
The scan retained 36,280,401 bytes. Offline replay reproduced the stored
mosaic byte for byte before tuning. Its SHA-256 was
`2e205220cc909bd4a3621d6bf7567fe6955b5b20502b30e4a940b56e5aef13a2`.

Inspection of the retained overlaps showed that the visible row seams were
0.04-to-0.13 mm X shifts, not Y shifts. Registration now limits ORB features
to the predicted overlap and fits translation only. If ORB is not sufficient,
a guided local correlation searches the cross-axis direction in three
independent overlap segments. The motion axis stays at calibrated placement
because its narrow overlap did not give a unique peak. At least two unique
segment peaks must agree within 0.15 mm.

Physical repeat `8ae2c25e-7b79-4a5c-ad1a-e3e024ba7694` used the same region,
30 percent overlap, 1,200 mm/min feed, and four queued-frame discards. Ten of
112 registrations passed: seven guided correlations and three ORB
translations. The seven internally corroborated results applied corrections
to seven tiles. The largest applied correction was 0.109 mm. These results
agreed with an offline replay of the first capture within 0.03 mm.

The repeat retained 36,290,686 bytes and produced 100 percent coverage with no
nominal fallback. The mosaic SHA-256 was
`b0625f41ee14aeaa2fd4af4b0a162ec0f885758ed07d1cc6c9b68fd8436d9843`.
Visual inspection showed straighter and sharper vertical grid strokes through
the corrected row overlaps. Some horizontal tone banding remains; this pass
only tuned geometric alignment. The machine returned homed, ready, and
fault-free to X=220.131, Y=190.271, Z=31.5, with the lights off.

### Workspace height-model ablation

Visual review rejected the installed-plane datum mosaics despite their seam
registration statistics. Their raised PCB features were doubled and blurred,
while the earlier working-plane datum survey was clear. Retained scan
`38ec929d-1676-45ae-a6d7-f63b493ae781` was replayed with one variable changed
at a time. Registration was also disabled independently.

The installed-plate projection remained badly ghosted with registration both
enabled and disabled. The working-plane projection was clear in both cases.
The installed plate maps measured about 0.0180 mm/pixel, while the raised datum
PCB working plane measured 0.01325 mm/pixel. Applying the lower plate scale to
the raised PCB caused the large reconstruction error. The prior sub-0.13 mm
registration corrections were not its primary cause.

Control scan `aabfb704-5a39-4c93-ac08-776c0d8e0cfe` restored the earlier datum
settings: the explicit 80-by-35 mm region, working-plane projection, 20 percent
overlap, 0.10 mm/pixel output, and 600 mm/min feed. It captured all 72 tiles,
retained 42,663,315 bytes, and covered 100 percent of the region. Its mosaic
SHA-256 was
`09e44aebde660209820dc8d07a5c579402ead88f70b296a9aa13f63a2cb049f0`.
Visual inspection showed the datum outline, grid, fiducial, holes, and scale
without the prior doubling. Offline replay with registration disabled was also
clear. The machine returned homed, ready, and fault-free to X=220.131,
Y=190.271, Z=31.5, with the lights off.

Layered scan `f62d094b-da99-4a81-a284-a9e84b763f6c` then used one 20-frame
capture of the datum-board edge and staging plate. It reconstructed both the
top-camera working plane and installed-plate plane. Both layers covered 100
percent of the requested region without nominal fallback. The installed-plane
layer kept the staging-plate numbers single, while the working-plane layer
kept the raised datum-board edge aligned. Each layer blurred features at the
other height. This confirms that the source frames contain useful parallax and
that separate calibrated projections preserve it. The machine returned homed,
ready, and fault-free to X=220.131, Y=190.271, Z=31.5, with the lights off.

Source-retaining scan `2c744a92-b430-47e9-8bb6-f96d8c67cd73` repeated the same
mixed-height region for parallax validation. All 20 frames and both mosaics
completed, with 100 percent coverage and no nominal fallback. Feature matches
from all 31 adjacent tile pairs produced 206 accepted local height
observations. The raised datum-board edge had median surface fraction 0.045.
The engraved staging-plate numbers had median 0.964. The calibrated endpoints
are zero for the working plane and one for the installed plate plane. The
plate-number interquartile range was 0.929 to 0.993. This confirms that camera
translation can separate the two observed heights without vacuum contact or
manual labeling. The machine again returned homed, ready, and fault-free to
X=220.131, Y=190.271, Z=31.5, with the lights off.

Production-path survey `2d014024-3fe9-4f07-9a2a-b3b1fb9ff9b5` then ran with
`mosaic_only` retention. The streaming implementation measured 186
observations without saved source frames. The raised-region median was 0.039,
and the plate-number median was 0.961. Both mosaics again had 100 percent
coverage without nominal fallback. The machine returned homed, ready, and
fault-free to X=220.131, Y=190.271, Z=31.5, with the lights off.

### Mixed-height local projection selection

Layered ROI scan `5658f2d0-ed74-4ce6-9d8a-bbf3902c155c` targeted X=175 to
265 mm and Y=125 to 225 mm with 41.3 percent overlap. A motion timeout stopped
the scan after tile 326 of 364. No emergency stop was used. The first 25
complete rows gave 325 retained frames through Y=214.5 mm. Software reconnect
and standard XYZ homing returned the machine to a ready, homed, fault-free
state.

Offline reconstruction accepted 7,376 parallax observations. The 45-by-45
height map had 280 measured cells, 576 inferred cells, and 1,169 unknown
cells. A working-plane fallback kept the datum PCB clear but visibly doubled
plate holes, edge numbers, the camera ring, and the camera body. A global
installed-plate fallback aligned those plate features but broke unmeasured
parts of the datum PCB. Normal ORB tile registration also made the mixed-height
image worse because it tried to fit one translation across different planes.

The accepted reconstruction measures source-image disagreement at both
calibrated endpoint projections. It keeps unknown cells unknown in the height
data, but selects the locally supported endpoint for their display pixels and
regularizes neighboring selections. Replay kept the datum PCB coherent and
removed most ringing from the staging plate, holes, edge numbers, and outer
camera ring.

The bottom-camera aperture detector then found six agreeing views in tiles 58,
59, 71, 72, 84, and 85. Their center motion fitted the local camera-depth
projection instead of reusing a plate-height scale. The fitted aperture center
was X=221.067 mm and Y=144.652 mm. Its scale was 0.02304 mm/pixel, its fit
residual P95 was 0.052 mm, and its radius was 10.056 mm. The compositor aligned
all six views to that center, used a center-weighted blend, and replaced a
narrow guard band around the aperture. Visual inspection showed a coherent
camera lens and a clean circular boundary without the former rectangular depth
blocks. The surrounding plate, datum PCB, and light ring were unchanged. The
replay still accepted 7,376 observations and kept the same 280 measured, 576
inferred, and 1,169 unknown height cells. The corrected PNG SHA-256 is
`d0f43f140e9e2e7196ac755e2a36217ce0b9cd0191059af60c7b8d12a02b9dfe`.
This replay used retained images only and caused no machine motion.

Future installed-plate layered scans now use at least 60 percent overlap. Their
height analysis keeps a useful baseline by also comparing non-adjacent tiles
that retain at least 20 percent overlap. At this stage, a physical 60 percent
repeat was still required to validate the full path with a new capture.

Physical 60 percent scan `0d08896d-4d34-4f1b-8e98-a133bba4c061` then covered
X=193–249 mm and Y=117–173 mm around the bottom camera. It captured all 242
tiles in a 22-by-11 serpentine raster. Both calibrated endpoint mosaics had 100
percent coverage and no nominal fallback. The full-source scan retained
74,535,751 bytes and completed in 635.6 seconds in a debug daemon. The machine
returned homed, ready, and fault-free to X=220.131, Y=190.271, Z=31.5.

All 869 parallax pairs had overlap and produced 1,373 accepted observations.
The 28-by-28 height map had 101 measured cells, 264 inferred cells, and 419
unknown cells. Twelve source frames agreed on the bottom-camera aperture. The
self-fitted projection measured 0.02297 mm/pixel with a 0.070 mm P95 residual.
It located the center at X=220.241 mm and Y=144.827 mm and measured a 10.060 mm
radius. This independently repeated the earlier six-view retained-data fit.

The physical scan's first corrected image had a narrow plate-plane strip left
of the bright camera ring. Offline replay increased the aperture artifact guard
from 1.5 to 3.0 mm. Visual inspection showed that this removed the strip while
keeping the coherent lens, circular aperture boundary, surrounding plate, and
outer light-ring geometry. The final replay PNG SHA-256 is
`5f639371664ec790d570cb676b8a46946e8ae16c3ffc180aecd5fa93a1986ec8`.

### Full-workspace chunked reconstruction

Full scan `d41979f7-c559-46f2-8878-6bea539d653e` validated transient source
retention and chunked height reconstruction on 2026-08-20. It scanned the
reachable installed-plate area from X=-5.947 to 438.950 mm and Y=83.502 to
326.067 mm. The plan used 60 percent overlap, 0.10 mm/pixel output, and
1,200 mm/min motion. It captured all 9,292 tiles in a 101-by-92 serpentine
raster and completed in 8,872.6 seconds.

Both endpoint mosaics covered 100 percent of the scan area without nominal
fallback. The primary layer accepted 5,836 of 18,391 overlap registrations
and applied 2,026 supported corrections. Parallax analysis accepted 846,635
observations from 36,589 overlapping tile pairs. The 223-by-122 height map had
7,763 measured cells, 5,224 inferred cells, and 14,219 unknown cells. It used
25 candidate surface fractions from -1.0 through 2.0.

Height-corrected reconstruction used five output-row chunks. It retained no
source frames in SQLite. The transient source directory reached 4.72 GiB
during reconstruction and was removed after the final artifacts were stored.
The complete retained artifact set is 284,066,859 bytes. The corrected mosaic
is a 4,450-by-2,427 grayscale PNG. Its SHA-256 is
`d920437c4043fb6129e65d44550247146d7e78d60aa4010d8dda4ecb058fa4d4`.

Visual inspection showed coherent plate holes, datum-PCB features, feeders,
the FTP PCB, and the bottom-camera aperture. No chunk boundary was visible.
The mean absolute intensity changes at output rows 512, 1,024, 1,536, and
2,048 were 0.640, 0.696, 2.966, and 1.326. All four were below the local
95th-percentile row change. The first, second, and fourth were also below the
local median.

Twelve source views fitted the bottom-camera aperture at X=220.250 mm and
Y=144.859 mm with a 10.049 mm radius. The fitted projection was
0.02287 mm/pixel with a 0.154 mm P95 residual. After artifact publication, the
machine returned homed, ready, and fault-free to X=220.131, Y=190.271,
Z=31.5.

### Full-workspace reconnaissance

Reconnaissance baseline `ed0f676c-64e9-4f7f-89ae-8aecaf0a5c60` completed on
2026-08-20. It used 20 percent overlap, 0.20 mm/pixel output, 1,200 mm/min
motion, calibrated installed-plate placement, and no tile registration or
parallax analysis. It captured all 1,295 tiles in a 37-by-35 raster. The scan
took 1,349.4 seconds, so it missed the 20-minute target by 149.4 seconds.

The camera supplied monotonic timestamps for all 1,294 captures after the
initial frame. The capture path discarded four queued frames after every
move, with no fixed-settle fallback. Coverage was 100 percent with no nominal
fallback. The 2,247-by-1,214 mosaic and manifest retained 1,618,636 bytes.
The mosaic SHA-256 is
`fab06c429d6d9e7f3172ab6f7d20d43f77c4832e2336c3f7c6303389796fb481`.

Visual inspection found clear plate holes and numbering. The datum PCB, FTP
PCB, feeder lanes, pocket tape, rails, and bottom-camera area were all
identifiable. This gave enough image-quality margin to reduce reconnaissance
overlap without changing resolution, feed, projection, or capture freshness.

Optimized scan `c17f976a-f25a-419c-8bab-e73dfec12540` used 10 percent overlap.
It captured all 1,023 tiles in a 33-by-31 raster and completed in 1,177.3
seconds, or 19 minutes 37.3 seconds. The hardware-adjusted estimate was 1,158
seconds, 19.3 seconds below the measured result. All 1,022 post-first captures
used monotonic timestamp selection. Coverage was again 100 percent with no
nominal fallback, registration, or failure.

The optimized 2,247-by-1,214 mosaic and manifest retained 1,513,489 bytes.
Its SHA-256 is
`e47ed6b6c371c1918928c3ddc4619da031bce41ed49f8d154c122d7a238299ce`.
Visual comparison found no material seam or identifiability regression at 10
percent overlap. The machine ended homed, ready, and fault-free at X=220.131,
Y=190.271, Z=31.5. This passes the under-20-minute workspace inventory target.

### Reconnaissance seam diagnostics

The first diagnostic scan,
`7b9fb2e7-61fb-4b5f-93d0-9244f050085f`, exposed an invalid disagreement
signal. The accumulator measured variation between source pixels from the
same tile. It therefore treated normal edges, such as plate holes, as overlap
error. This classification was rejected and was not used for machine motion.

The corrected compositor first reduces each source tile to one projected mean
at each output pixel. It then measures variance between distinct source tiles.
Physical scan `8e5db6e2-c51c-4d4b-a62a-29c390126918` captured all 1,023 tiles
in 1,180.9 seconds, or 19 minutes 40.9 seconds. Coverage was 100 percent. All
1,022 post-first captures used monotonic timestamp selection. The capture path
discarded 4,148 queued frames. The mosaic SHA-256 is
`4b4a74a0a82d6c94b359fb6804c3a6b718665acc1b29df5af2555c1472a94ac0`.

Retained-data replay applied the final component-support policy without
machine motion. It classified 1,344 of 1,982 overlaps as healthy, 53 as
elevated, and 585 as unresolved. It recommended targeted refinement and
produced eight machine-coordinate regions. Visual inspection matched these
regions to the feeder banks, FTP PCB, datum and bottom-camera area, and raised
fixtures. Normal plate holes outside these regions did not receive a problem
overlay. The report SHA-256 is
`8c2890557ffec394594e11b6c390a97f7c4502cd558ae6d3c41f88ddf288479e`.
The composited diagnostic image SHA-256 is
`5e15abd3e44706d393caaf31d77628380af078c3e083c3ea7a50cd2377906151`.

Seam analysis did not alter calibrated tile placement and did not start an
automatic second pass. The machine returned homed, ready, and fault-free to
X=220.131, Y=190.271, Z=31.5, with the lights off. Two repeat measurements of
the new diagnostic path were 1,180.6 and 1,180.9 seconds. The preflight model
now uses 0.20 seconds per post-first tile for measured control and dispatch
overhead.

### Selective FTP-PCB refinement

The current `workspace_seam_health.v2` policy replayed the retained products
from reconnaissance scan `8e5db6e2-c51c-4d4b-a62a-29c390126918` without
motion. It returned the same eight tuned target regions. Target 6 enclosed the
FTP PCB at X=346.779 to 441.020 mm and Y=210.867 to 315.067 mm. The parent
still retains its original four-region report; refinement planning deliberately
uses current-policy replay instead of changing that historical record.

Child scan `71e574d5-6975-45ae-bf25-190c88eeb7be` selected target 6 through
`mode=refinement`. Reachable camera coverage clipped only its X maximum to
438.950 mm. The child report retained the parent UUID, target index, and raster
transform. Its plan used 60 percent overlap, 0.10 mm/pixel output, 1,200
mm/min motion, transient sources, and three output-row reconstruction chunks.
The preflight estimate was 774 tiles, 1,546 seconds, 20,006,548 retained bytes,
and at most 784,650,240 temporary source bytes.

All 774 tiles completed. The scan took 848.1 seconds, or 14 minutes 8.1
seconds. The temporary source directory reached 372 MiB and was removed after
reconstruction. Final retained artifacts used 51,881,038 bytes, below the 64
MiB quota. Both endpoint mosaics covered 100 percent of the patch with no
nominal fallback.

Parallax analysis accepted 168,706 observations from all 2,913 attempted
pairs. The 47-by-53 height map measured 1,406 cells, inferred 387, and left
698 unknown. Visual inspection showed the expected ablation: the working-plane
layer kept the FTP PCB sharp but doubled plate holes, while the plate-plane
layer kept plate holes single but ghosted the PCB. The corrected patch kept
the PCB outline, traces, labels, fiducials, and components coherent while also
keeping the surrounding plate holes single. Tall white fixtures retained some
local block artifacts, but board identification was not impaired. The
height-corrected PNG SHA-256 is
`9926acc73b26cd0dbc814c23071fe010be108700d2d71bf442ed670ca6255726`.

This run also exposed a retained-storage estimate error. Its 168,706 dense
board observations made `surface-height-observations.json` 47,256,714 bytes,
while preflight estimated only 20,006,548 bytes for all artifacts. Runtime
quota enforcement still passed, but this left too little margin. Subsequent
transient-source scans now use every observation to build the height map, then
retain a deterministic sample of at most 25,000 observations with the full
aggregate counters. Preflight reserves 300 bytes for every retained sample.
Full-source scans remain unchanged because their purpose includes offline
reconstruction. This correction was validated without another machine scan.

The scan returned homed, ready, and fault-free to X=220.131, Y=190.271,
Z=31.5, with the lights off. This validates one bounded, lineage-tracked,
height-corrected refinement patch. The batch validation below completes the
automatic scheduling work.

### Prioritized refinement batch

On 2026-08-21, the current classifier replay again returned eight targets for
parent scan `8e5db6e2-c51c-4d4b-a62a-29c390126918`. It ranked them by local
elevated and unresolved seams, peak disagreement, material correction, and
area. The read-only target response identified the successful FTP-PCB child
for target 6 and reported seven remaining targets. The queue selected targets
0, 1, 5, 2, 3, 4, and 7 in descending severity order. The skipped target kept
priority 2, so the queued child priorities were 1 and 3 through 8.

After external safety confirmation, combined XYZ homing completed at X=0,
Y=0, and Z=31.5. Batch operation
`9d67063b-5875-4275-b364-755f9a62222b` then ran the seven children strictly
sequentially under the exclusive scan-motion lock. All 2,073 planned tiles
completed. The batch succeeded in 2,272.1 seconds, or 37 minutes 52.1 seconds,
against a conservative 4,181-second estimate. It retained 34,729,276 bytes
against a 90,971,496-byte estimate.

The child results were:

| Target | Tiles | Elapsed s | Retained bytes | Accepted registrations |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 1,007 | 1,118.1 | 13,714,571 | 1,885 / 1,942 |
| 1 | 510 | 513.8 | 10,509,466 | 722 / 959 |
| 5 | 306 | 308.2 | 8,686,017 | 430 / 577 |
| 2 | 180 | 210.3 | 1,281,309 | 17 / 319 |
| 3 | 50 | 56.3 | 321,645 | 0 / 85 |
| 4 | 8 | 26.4 | 102,179 | 9 / 10 |
| 7 | 12 | 39.1 | 114,089 | 0 / 16 |

Every child retained its parent UUID, target index, classifier version,
severity score, batch operation UUID, and priority. The subsequent
`workspace_refinement_priority.v1` lineage field and live batch progress
message are state-reporting additions; they did not require repeat motion.
Each child also produced a height map and `mosaic-height-corrected.png`. A
post-run target replay reported zero remaining targets. This validates
successful-target suppression and queue completion against the stored parent.

Visual inspection found feeder strips and tapes identifiable in target 0, the
datum tool clear in target 5, and the local objects in targets 2, 3, 4, and 7
coherent. Target 1 remained identifiable but showed block boundaries in its
mostly plate-height area. Targets 3 and 7 accepted no feature registrations;
their small corrected patches remained coherent through calibrated nominal
placement and endpoint selection.

### Refined workspace composite and quality gate

On the same date, the no-motion refinement-composite operation replayed the
current target set and found all eight successful height-corrected children.
It resampled them into the 2,247-by-1,214 parent raster at 0.20 mm/pixel. It
used the stored machine-coordinate transforms, a two-millimeter edge feather,
and a robust brightness correction of 0 to 9 luma levels across the patches.
The release build completed the idempotent operation in 0.75 seconds.

All eight patches had 100 percent reported coverage. Their texture-retention
ratios were 0.680 to 1.107, above the 0.60 gate. The result therefore passed
`workspace_refinement_composite.v1` with eight of eight targets accepted and
no missing target. The parent now contains `mosaic-refined.png`,
`refinement-quality.json`, and `mosaic-refined-quality.png`. These products
increased the complete retained parent artifact set to 12,222,422 bytes.
The plain composite SHA-256 is
`8afc34b68a99afa9b96e7753cc5e9964f1aeabd2db8a062d294f204541acde49`.
The quality-preview SHA-256 is
`ed6b3634995bc1fdfb9be3bd0caebe109ac581aca070ce02b1d4cf1c6cb1c372`.

Visual review found a coherent full workspace. The plate grid stays stable.
The feeder tapes, datum tool, bottom-camera area, and FTP PCB are identifiable
in one image. The quality report retains high boundary or midpoint signals as
review observations for targets 0, 1, 5, and 6. These signals did not fail the
gate because real feeder and PCB edges caused them. The green-outline preview
confirmed that every current target passed the hard completeness, coverage,
and texture checks. This separates successful scan execution from image
acceptance without requiring another machine run.

### Edge-localized height selection

Visual review of the refined composite found regular 2 mm blocks at the right
edges of the FTP PCB and feeder banks. The pattern matched the surface-map cell
size. The renderer had used one candidate projection for every pixel in a
measured or inferred cell. Its per-pixel source-disagreement signal affected
only the fallback choice for unknown cells.

The renderer now treats the cell height as a prior. Near a different local
height, it selects an alternate candidate only when a 0.30 mm image
neighborhood has coverage from at least two source tiles and its disagreement
beats a source- and confidence-dependent margin. Unknown cells use the same
pixel-local evidence between the working and installed-plate endpoints. A
single-source pixel cannot override the prior.

The completed refinement children used transient source retention, so their
deleted source frames cannot be reconstructed with the new selector. A
no-motion replay used the retained 242-frame full-source scan
`0d08896d-4d34-4f1b-8e98-a133bba4c061`. It rebuilt all 14 candidate planes
from 1,373 accepted parallax observations. Mean luma discontinuity on the
2 mm cell grid, relative to nearby non-grid lines, decreased from 0.2007 to
0.1495. This is a 25.5 percent decrease. Only 8.03 percent of output pixels
changed, with a median change of zero and a 95th-percentile change of one luma
level. The 561-by-561 output SHA-256 is
`06859bcf25236fd6d4cd98a5f523d9aa13eb03f848502078818e3cae822ab6e5`.

Focused tests also put a true projection boundary one millimeter inside a
2 mm measured cell. Two-source disagreement moved the display edge to the true
location. The same case with one source kept the measured-cell boundary.
Chunked and single-pass reconstruction remained byte-identical. A new scan of
the affected refinement regions was still required to update the current full
workspace composite.

After the queue, the machine was at the visual datum X=220.131, Y=190.271,
Z=31.5, homed, ready, and fault-free. Normal disconnect operation
`e85d478b-c406-44f9-b18c-791fcc21e720` switched outputs off, released control,
and cleared pose and homing. The daemon then stopped normally. No emergency
stop was used.

### Edge-localized refinement rescan

On 2026-08-21, physical rescans exercised the pixel-local height selector in
the four regions where review had found the clearest cell-boundary artifacts.
The first 180-frame target 2 attempt captured every frame, then stopped during
reconstruction because a ceil-rounded outer raster pixel lay just outside the
height-map region. Scan `3b81fa27-2ef6-4146-9474-220fe0d1bdba` and operation
`3404cb66-9062-4704-b1f7-37f873857353` failed without a controller fault. The
machine stayed homed and ready.

The renderer now leaves such edge pixels at the working-plane fallback with a
transparent class overlay. A regression test uses a 4-by-2 mm region at
0.30 mm/pixel, whose 15-by-8 output includes that rounding case. Targeted
height-map, strict Clippy, and chunked-versus-single reconstruction checks
passed before the release daemon was rebuilt and the machine was re-homed.

The successful physical results were:

| Target | Scan ID | Tiles | Time | Registrations | Retained bytes |
| --- | --- | ---: | ---: | ---: | ---: |
| 0, feeder bank | `f83e633e-e290-4729-bb38-0a2d0e399957` | 1,007 | 18m24s | 1,885/1,942 | 13,791,993 |
| 1, feeder | `c6554102-fa44-4a6e-889b-3bfdeb3e36cb` | 510 | 8m41s | 754/959 | 10,636,632 |
| 2, small feeder | `ff6ce636-8b2a-48b5-841f-9ce70c6ca5b3` | 180 | 3m21s | 17/319 | 1,323,327 |
| 6, FTP PCB | `aed03fc9-7259-456d-994d-817ce91bc39f` | 774 | 13m50s | 1,097/1,487 | 11,666,097 |

All four scans reported 100 percent coverage. Their combined successful scan
time was 44m16s. A diagnostic compared mean absolute luma gradients on the
2 mm height-map grid with neighboring gradients two pixels to either side.
The grid-relative seam excess changed as follows:

| Target | Before | After | Decrease |
| --- | ---: | ---: | ---: |
| 0 | 1.0343 | 0.5667 | 45.2% |
| 1 | 6.8775 | 4.1285 | 40.0% |
| 2 | 1.5086 | 0.6749 | 55.3% |
| 6 | 0.5717 | 0.3214 | 43.8% |

Visual review agreed with the diagnostic. The regular 2 mm protrusions largely
disappeared, long feeder rails and holes became more coherent, and the FTP PCB
became much sharper. A few marks remain at real vertical object transitions.
These are local projection-boundary cases, rather than the former regular grid
pattern.

The no-motion composite replay found and accepted all eight current refinement
children with complete coverage. It produced a 2,247-by-1,214 image. The
refined composite SHA-256 is
`3de638842c72b49acef13c10bcbd89ccda939080bc6f34b9edf556469834147e`.
The quality-preview SHA-256 is
`9d9c1913b7bdace75bd2e0e9d8590f1959e947ce06b425546008c551818d6cd1`.
The plate grid stayed stable while the changed feeder and PCB regions became
cleaner.

After reconstruction, the machine was homed, ready, fault-free, and parked at
X=220.131, Y=190.271, Z=31.5. Normal disconnect operation
`4901fd28-e1d5-49d0-b1c4-fbc0ac6e05b1` switched outputs off and released the
machine. The daemon then stopped normally. No emergency stop was used.

### No-motion board-region discovery

On 2026-08-22, a simulated daemon replayed physical reconnaissance parent
`8e5db6e2-c51c-4d4b-a62a-29c390126918`. The parent has an accepted refined
mosaic but no single full-workspace height map. Board discovery merged the
surface maps from all eight accepted refinement children into the parent's
machine-coordinate grid. Each child scan and map artifact remains in the
result as explicit lineage.

The `workspace_board_discovery.v1` replay found eight raised-surface regions.
Visual inspection found that the outlines covered the FTP PCB, the main feeder
bank, two smaller feeder groups, the datum tool, and local camera-area
features. The outlines are coarse regions from 2 mm surface evidence. Some
include clamps or plate margin, so they are not yet board registration results.

The stored board library was empty. Thus, the expected design-match count was
zero and the report warned that no imported board dimensions were available.
The report is 10,253 bytes. The RGB preview is 3,264,422 bytes. Its SHA-256 is
`c3c2e31ccaf980c0c73939c19fe0a4d510b598a969c1bf3439796ced55076ad5`.
The complete parent artifact set is now 15,487,829 bytes.

This replay opened no controller or camera and caused no machine motion. The
simulated daemon stopped normally after the artifacts and updated manifest
were stored. Package-template comparison and local fiducial verification are
still required before a candidate can become a board instance.

### Plate-aligned semantic proposal replay

On 2026-08-22, the same retained physical scan was replayed through
`workspace_object_proposals.v2`. The daemon used simulation mode and opened no
controller or camera. All eight raised components remained in the report as
inventory evidence.

The calibrated staging-plate proposal angle was 0.274 degrees. The calibrated
build-plate angle was 0.184 degrees. All proposal edges now follow these plate
axes; the two nozzle-bank fragments no longer have arbitrary PCA rotations.
The fixed-fixture policy classified the datum PCB once, the two nozzle-bank
fragments as tooling, and both bottom-camera-area fragments as fixed machine
features. Their board likelihood is zero. The remaining three regions are the
two feeder groups and the FTP PCB. With no imported board and no learned model,
they correctly remain `unknown` and require review.

Each proposal retained a plate-normalized image crop. Visual review found that
the main feeder crop cleanly contains its parallel lanes and tape features. The
FTP crop cleanly contains the PCB, mounting holes, and component field. These
are suitable inputs for later labels and learned segmentation. The proposal
preview is 3,261,911 bytes with SHA-256
`b6f34f64fb62e68ccc8e3ad3cea6bf226fb81f2af818d52ef78cb07d1faa4792`.
The report is 16,267 bytes with SHA-256
`bd37acfbff67edc0d8797ff89acb02bf0d681edb0893cd97f1cfb253f4df3428`.

### Persistent inventory change detection

On 2026-08-22, physical scans validated the persistent inventory workflow
against active baseline scan `8e5db6e2-c51c-4d4b-a62a-29c390126918`.
Unchanged control scan `d6b84845-2e7d-4780-b650-3e83383bc1d0`
captured all 1,023 reconnaissance tiles in 21 minutes 1 second. All eight
baseline objects were unchanged. Their normalized raster differences were
0.0092 to 0.0259 against the 0.0400 change threshold. No refinement target
was made. Two unmatched seam regions were below the 400 square millimeter
new-object review area.

The operator then rotated the Opulo FTP PCB clockwise and made a smaller
translation. Cropped reconnaissance scan
`7d50b7e5-f0b9-428e-a09b-f228115f56fd` covered X=335 to 441 mm and Y=200 to
326 mm. It captured all 136 tiles in 3 minutes 49 seconds. Baseline-guided
raster comparison marked only the FTP PCB as modified. Its difference was
0.0672. The other seven baseline objects were outside the partial scan and
were reused. The bounded refinement target was X=340.2 to 441.0 mm and
Y=205.98 to 319.0 mm.

Refinement scan `65b9e462-0c87-477c-82e9-acafc0185e67` captured all 817
tiles at 60 percent overlap and 0.1 mm/pixel. Capture took 15 minutes 49
seconds. Single-threaded height reconstruction took a further 37 minutes 55
seconds. It accepted 179,116 parallax observations and retained a distributed
25,000-observation sample. The transient source directory was removed after
reconstruction.

The refinement composite accepted the patch with complete coverage, a 1.13
texture-retention ratio, and an 11-luma boundary-disagreement p95. It recorded
an internal seam-discontinuity review ratio of 3.67 against the 3.00 review
level. Object proposals found the main 94.15-by-104.18 mm region and one false
6-by-6 mm component at the refinement boundary.

The first semantic comparison then called the rotated main region unchanged.
Its normalized proposal-crop difference was only 0.0334 against the 0.1000
semantic threshold. This lost the earlier raw raster evidence. It also called
the 6-by-6 mm boundary component an added object.

`workspace_change_set.v3` now retains a separate plate-projected raster score
when a reconnaissance scan later gains semantic proposals.
`workspace_object_proposals.v3` also rejects regions below 100 square
millimeters. No-motion replay produced seven baseline objects instead of
eight because it removed a 10-by-6 mm camera-area fragment. Baseline
`c142332c-a903-4464-9583-180917bf9851` superseded the v2 baseline. Replay of
the changed physical data then produced one modified PCB, no added or removed
objects, and six reused baseline objects. It retained both the 0.0334 semantic
score and the 0.0672 raster score. The false boundary object was absent.
Because the accepted child covered more than 90 percent of the changed object,
the final report made no repeat refinement target.

This test detects the board change but does not estimate the board's physical
rotation. Proposal outlines remain aligned to the calibrated plate. Image or
KiCad registration must measure the board pose before job setup.

Normal disconnect operation `6f0cfe6d-e4d2-45f7-93c7-692d0d8b703c`
released the controller before the daemon was rebuilt for no-motion replay.
The final daemon state is disconnected, unhomed, and fault-free. No emergency
stop was used.

### No-motion PCB and feeder recognition

On 2026-08-22, the official LumenPnP V3.0.4 FTP KiCad board was imported as
`Opulo FTP PCB REV00`. The importer read a 75-by-75 mm Edge.Cuts outline and
90 placements without warnings. The attached machine stayed disconnected.

`workspace_object_proposals.v4` kept the coarse proposal rectangles parallel
to each calibrated plate. It then used OpenCV line evidence to fit imported
board dimensions inside each crop. This separates a PCB from clamps and plate
margin. The refined full-workspace replay found the FTP PCB inside its former
92.17-by-104.03 mm raised region. The fitted result was:

| Field | Value |
| --- | ---: |
| Center X | 394.879 mm |
| Center Y | 259.003 mm |
| Canonical angle | 0.455 degrees |
| Dimension error | 0.041 mm |
| Edge support | 95.8 percent |
| Match confidence | 0.9984 |

The proposal stores the 75 mm visual outline separately from the coarse scan
region. The board is square, so the result correctly marks its design
orientation as ambiguous. It is not yet an accepted job transform.

The deterministic classifier also found repeated long parallel tracks in the
two unknown staging-plate regions. With the independent staging-plate prior,
it classified both as feeders at 0.9624 and 0.8870 confidence. The datum PCB,
nozzle bank, and bottom camera guard kept their fixture classes. No raised
region remained unknown.

The cropped changed-board scan independently fit the same 75 mm design at
X=395.469, Y=258.930 mm and 1.980 degrees. Its dimension error was 0.009 mm,
edge support was 94.1 percent, and confidence was 0.9997. Relative to the
baseline fit, this measures a 1.525-degree rotation and a 0.595 mm center
translation. This agrees with the operator report that the board was rotated
clockwise with a smaller shift.

The final preview draws the coarse build-region envelope in muted green and
the fitted PCB outline in bright green. Replay used retained artifacts only.
It opened no controller or camera and caused no motion.

### Height reconstruction throughput

The rotated-PCB refinement exposed a post-capture bottleneck. Scan
`65b9e462-0c87-477c-82e9-acafc0185e67` captured 817 frames in 15 minutes 49
seconds, then spent 37 minutes 55 seconds in height reconstruction. The debug
daemon used one busy reconstruction worker.

Retained full-source scan `0d08896d-4d34-4f1b-8e98-a133bba4c061` provided a
no-motion benchmark with 242 frames, 60 percent overlap, 0.1 mm/pixel output,
and 14 candidate height planes. The original debug replay took 390.34 seconds
of wall time, 406.96 seconds of user CPU time, and 280,952 KiB peak RSS.

The transient renderer now decodes each chunk source once and shares immutable
frames across candidate planes. Four bounded workers render independent planes.
When tile-placement corrections are zero, the stream omits an identical second
nominal projection. Registration-enabled or pre-corrected streams retain that
fallback.

The parallel shared-frame debug replay first took 234.55 seconds. Removing the
duplicate nominal projection reduced it to 141.28 seconds, a 2.76-fold speedup
over the original debug path. Final peak RSS was 320,424 KiB. The optimized
release replay took 19.41 seconds, 46.64 seconds of user CPU time, and 318,136
KiB peak RSS. This is 20.1 times faster than the debug path used by the physical
refinement.

All six output artifacts were byte-identical across the original debug,
parallel debug, final debug, and release replays. The corrected mosaic retained
SHA-256
`06859bcf25236fd6d4cd98a5f523d9aa13eb03f848502078818e3cae822ab6e5`.

Scaling by the measured frame and candidate-plane workload predicts about 1
minute 48 seconds of reconstruction for the 817-frame PCB refinement. With the
measured capture time, the same scan should finish in about 17 minutes 37
seconds. A later physical scan must confirm this estimate. Hardware scans now
use the release daemon.

### Fine XYZ home and automatic FTP PCB registration

On 2026-08-23, the first registration trial exposed that the new workflow had
performed coarse XYZ homing and visual X/Y refinement, but had not established
a session Z datum. The daemon was stopped normally at safe Z before the trial
continued. Interrupted registration `bc7d0580-b80a-49c8-89ba-26a8bc18b58b`
was retained as failed. No emergency stop was used.

Session fine home now combines visual X/Y refinement with a vacuum-contact Z
reference. It identifies and centers the installed tip, centers the separate
datum-board sealing target, searches 0.3 mm on either side of the stored
contact without exceeding the profile safety interval, and limits the Z-only
coordinate reset to 0.5 mm. Machine status records the calibration UUID.
Coarse re-home, disconnect, fault, or emergency stop clears it. Physical
workspace scans and board registration reject a missing or different UUID.

N1 reached its established Z=3.0 lower bound without a qualifying transition.
Its stored contact is Z=3.2, so non-repeatable stall homing leaves too little
search margin on that side. The runner switched the pump off, retracted, and
returned fault-free without applying a reset. N2/N24 has useful safe margin on
both sides and is now the V3.0.4 default. It succeeded once with a +0.100 mm
reset from controller Z59.100 to Z59.200. Two later runs reproduced contact at
Z59.200 and applied zero correction. Final fine-home operation
`c43af07a-d29f-46e1-a97f-c3be617935ef` succeeded.

The first local-fiducial fit correctly rejected a 1.212 mm residual because the
bright-component detector selected the printed `4` in `R40` instead of FID1.
Its annotated frame showed the true concentric feature at the predicted
center. The detector now limits bright candidates to one copper-pad diameter
from the visual seed and falls back to an OpenCV circle fit of the footprint's
3 mm mask opening. Replay of the retained frame found the true center near
(627.5, 346.5) pixels. A generated regression frame covers textured copper and
nearby bright print.

The next fit measured all three fiducials consistently but rejected the board
because its physical outline slightly overhangs the head travel limit. This
mixed workspace occupancy with head motion. Registration now checks every
camera target during acquisition and leaves later nozzle targets to their own
planner bounds; it does not require an uncommanded PCB edge to be reachable by
the head.

Registration `56a5cb24-9b1f-415f-b308-f22473fb5069` then accepted the first
orientation hypothesis. It measured all three KiCad fiducials, fit a board
angle of -1.53398 degrees, and produced 0.04773 mm RMS and 0.06200 mm maximum
residual. The accepted board instance is
`88781f07-a28f-42cd-be01-f9bcb150cb6b`; its surface Z remains deliberately
unset. The three annotated evidence PNGs are retained in SQLite and were also
copied to `/tmp/velocitypnp-registration-final-fid1.png` through `fid3.png`.

The machine returned to X=220.131, Y=190.271, Z=31.5, homed, ready, and
fault-free before normal disconnect operation
`8ff705ef-a418-41b8-90c0-fa4d5d50ba2a`. The final daemon state is disconnected,
unhomed, and fault-free. No emergency stop was used.

### Strip-feeder vision software gate after host restart

On 2026-08-23, the host restarted after it became unresponsive. The previous
workspace database and image artifacts were in temporary storage and are not
available for feeder replay. The persistent project database contains no scan
entities. No result from the synthetic feeder tests is recorded as physical
validation.

After the restart, Linux found the REV4 controller at `ttyACM0`, the top PnP
camera at `video2`, and the bottom PnP camera at `video0`. The VelocityPnP
daemon was not running, so the first API device request correctly failed with
connection refused. No controller was opened and no motion occurred.

The no-motion software gate passes synthetic 8 mm tape-bank coverage. It finds
three independent 4 mm sprocket lattices, uses the EIA-481-C 3.5 mm pocket
centerline offset, resolves a 2 mm component pitch, and persists an inactive
draft. Isolated round features do not form a lane. A new focused scan and live
camera holdout remain required before the roadmap validation item can pass.

### Physical strip-feeder vision holdout

On 2026-08-24, a host reboot cleared the stuck top-camera USB stream. Direct
V4L2 captures and daemon captures then succeeded for both machine cameras. A
new persistent calibration completed five visual-home cycles, head geometry,
local direction response, both vacuum contacts and characterizations, both
planar camera models, and installed plate geometry. N1 characterization needed
one bounded contact reprobe; N2 and the N1 retry passed. Fine XYZ home with
N2 reproduced the stored contact with a zero Z correction.

Focused survey `b501d9fb-3d92-4cf7-881a-98c94d57d9b3` covered staging-plate
X=90 to 135 mm and Y=105 to 195 mm. It used 60 percent overlap, 0.10
mm/pixel output, 1,200 mm/min motion, and transient source retention. It
captured all 296 tiles and completed in 315.7 seconds. Reconstruction retained
9,221,572 bytes and produced a 23-by-45 height map with 869 measured, 96
inferred, and 70 unknown cells. The corrected mosaic clearly shows four
carrier-tape lanes.

The first physical replay found four parallel tracks but left the region
unknown because its 0.662 feeder score did not meet the generic 0.75 class
threshold. Policy `workspace_object_proposals.v6` now lets a staging-plate
parallel-track score of at least 0.65 route a review candidate to the
sprocket-lattice specialist. Known fixtures still return first, and the
specialist still rejects a crop without an EIA-481 lattice. The same retained
scan then classified candidate 0 as a review-required feeder.

The first specialist replay exposed an eager `then_some(candidates[0])`
evaluation when adjacent track spacing matched no standard tape width. It
panicked before producing a report. A safe single-item check and regression
test now keep unresolved widths in review. No controller was connected during
either replay.

The fixed physical replay found five sprocket lattices. Proposal 0 passed the
image draft gate with 18 centers, a 0.111 mm sprocket-pitch residual, 12 mm
tape width, 2 mm pocket pitch, and 0.931 confidence. Its preview places the
detected holes on the left tape and the calculated centerline on its pockets.
The four weaker proposals remain review-required. Inactive draft
`4eefa2fd-11e1-45a8-8ec6-5c2140920922` stores proposal 0 without guessing a
part identity or orientation. Pickup Z, occupied-pocket selection, and a live
camera holdout remain unset, so the roadmap validation item remains open.

The machine returned to the visual datum after the scan. Normal disconnect
operation `0521c42e-8b3c-44bd-99d4-3bb3903194d3` switched outputs off,
released control, and cleared runtime homing before the daemon rebuild. No
emergency stop was used.

The operator later clarified that the focused region held six populated
component strips, not five. Thus, the five image proposals were candidate
lattices and did not prove five distinct populated strips. Two more strips
were then installed.

On 2026-08-25, the operator located all eight strip feeders between staging
plate columns 11 and 16. Survey
`7c7fedcf-d4ca-4da2-a697-41f6cb05264e` repeated X=90 to 135 mm and showed
only four real lanes. The 15 mm plate pitch showed that the region ended near
column 14; this was a scan-region error, not a detector recall result.

Fast reconnaissance `85255974-f066-42b0-80c5-050404fa84ee` covered X=70 to
275 mm and Y=105 to 195 mm. It captured 195 tiles in 235.5 seconds at 0.20
mm/pixel. Its plate-plane mosaic showed all eight tapes. Reconnaissance has no
height map, so it did not run the height-based object proposal stage.

The existing X=90 to 135 mm scan was retained. Right extension
`9e705141-e5fd-4512-89ab-476f91dc984d` covered X=130 to 175 mm with 296
tiles in 320.9 seconds. Left extension
`ab8edbf4-8645-4c2b-a68b-0a0226d38223` covered X=75 to 100 mm with 148
tiles in 201.2 seconds. Both used 60 percent overlap, 0.10 mm/pixel, and
transient source retention. This avoided a 666-tile repeat of the full bank.

The general detector left the narrow left extension unknown, even though its
staging-plate prior suggested a feeder. The no-motion strip specialist now
accepts this uncertain candidate for analysis. It still rejects known
fixtures, and draft creation still requires measured tape geometry.

After overlap deduplication, the specialist found the eight real sprocket rows
at machine X=82.24, 93.35, 104.41, 115.58, 126.70, 137.69, 148.77, and
159.83 mm. Thus, candidate recall was 8 of 8. It also proposed three component
pocket rows at X=130.24, 152.22, and 163.29 mm. Candidate precision was 8 of
11, and five real lanes still needed review because the image-only tape-width
or pocket-side choice was unresolved. This is not yet an eight-lane automatic
setup pass.

The recovered X=82.24 mm lane passed the image gate with 17 sprocket centers,
a 0.142 mm pitch residual, 12 mm tape width, 2 mm pocket pitch, and 0.924
confidence. Draft `6392fddd-11d4-4007-917e-dfe5e6eed0fb` stored this lane
without a part identity or orientation.

Live holdout `330b7ebe-5cba-4071-a07e-23cf5f58cc50` first rejected the
draft. Its evidence showed both sprocket rows aligned with their expected
lines, but the plus-or-minus 2 mm camera sweep exposed only two holes per row.
The three-hole lattice gate therefore had insufficient coverage. The camera
sweep was increased to plus-or-minus 4 mm along the tape. The 1.25 mm track
tolerance was not changed. Failure reporting now distinguishes insufficient
lattice coverage from a track-position error.

After normal controller disconnect, daemon restart, coarse XYZ home, and fine
XYZ home, holdout `46b0638e-55b4-4710-98fc-189c9cd6956e` passed. It measured
four sprocket centers with a 0.0431 mm pitch residual. The measured correction
was X=-0.0342 mm and Y=+0.5599 mm, with 0.5609 mm magnitude and -0.4516 degree
rotation. It selected occupied pocket X=87.7757 mm and Y=157.4994 mm with
93.6 percent occupancy confidence. Total holdout confidence was 0.753.

Both fine-home runs reproduced datum contact Z=59.200 mm and reset it to
Z=59.100 mm. Each holdout returned to the visual datum. No nozzle was lowered,
no vacuum output was used, no tape was indexed, and no emergency stop was
used. Final normal disconnect
`463b9f5e-cb3f-4668-9367-5546b2729810` succeeded. Pickup-Z and indexing
validation keep the combined roadmap item open.

### Strip-feeder pocket-row rejection replay

On 2026-08-25, version 2 of the strip-feeder specialist was replayed against
the three retained physical feeder scans. The controller stayed disconnected.
The replay caused no machine motion.

The detector now pairs 4 mm lattices that are 3.5 plus or minus 0.45 mm apart.
This separation is less than the width of standard carrier tape, so both rows
cannot be sprocket rows. The stronger row stays as the sprocket row. The weaker
row is kept as rejected component-pocket evidence. It also resolves the pickup
side and 8 mm tape width for the pair.

The left scan rejected the pocket row paired with the X=126.70 mm sprocket
row. Its local fitted offset was 3.595 mm. The right scan rejected the rows
paired with the X=148.77 and X=159.83 mm sprocket rows. Their fitted offsets
were 3.476 and 3.226 mm. Preview artifacts
`/tmp/velocitypnp-left-strip-v2.png` and
`/tmp/velocitypnp-right-strip-v2.png` show rejected rows in red.

After scan-overlap deduplication, all eight real rows remain at X=82.24,
93.35, 104.41, 115.58, 126.70, 137.69, 148.77, and 159.83 mm. All three prior
false proposals are rejected. Candidate recall is 8 of 8 and candidate
precision is 8 of 8 for this retained physical data. The three paired lanes
also resolve as 8 mm tape with a 4 mm component pitch.

The X=104.41 and X=115.58 mm rows still require review. Both resolve as 12 mm
tape, but their two possible pocket sides have similar image support. This
result does not complete an automatic eight-lane setup. Pickup-Z and indexing
validation remain open.

### Strip-feeder population inventory replay

On 2026-08-25, version 3 was replayed against the same three focused scans.
The controller stayed disconnected. The replay caused no machine motion.

Version 3 removed tape-width inference from neighboring sprocket rows. Those
distances describe feeder placement, not tape width. Component evidence at
the standard pocket offsets now selects the width and common feeder-bank
direction. Rectangular contour lattices measure part pitch and visible part
size. Lane-local brightness classes, anchored by detected part contours,
classify visible feed positions.

The combined report deduplicated the overlapping X=93 mm lane. It found all
eight installed strips. Each resolved as 8 mm tape with 4 mm part pitch. Seven
lanes passed automatically. The X=115.8 mm lane retained a review flag because
of weaker image confidence.

Across the visible tape extents, the report counted at least 84 present and 48
absent positions, with no uncertain positions. Per-lane present/absent counts were
3/13, 4/13, 12/3, 9/7, 11/3, 13/1, 17/2, and 15/6. Measured visible part
outlines ranged from 1.8 by 0.9 mm to 3.2 by 2.0 mm. These sizes do not claim
part identity or standardized package size.

The result is stored on reconnaissance scan
`85255974-f066-42b0-80c5-050404fa84ee`. Artifact
`mosaic-strip-feeder-inventory.png` overlays cyan strip boundaries, green
present-part outlines, and red absent-position marks on the workspace mosaic.
The related JSON artifact is `strip-feeder-inventory.json`.

Review on 2026-08-26 found that the first combined preview used the 0.20
mm/pixel installed-plate projection as its visible feeder background. The
detector itself used the focused scans' height-corrected crops, but the preview
made the annotations appear to use the wrong surface. Version 2 now requires
height-corrected source mosaics and inserts their 0.10 mm/pixel feeder pixels
into the combined preview. It also preserves an RGB workspace-discovery preview
when the target scan has one. Thus, one output can retain detected workspace
objects, KiCad board details, and feeder inventory.

The two focused scans that overlap the X=93.4 mm strip disagree on two pocket
states. Version 2 keeps those states uncertain instead of selecting one scan.
The merged result is therefore at least 83 present, 47 absent, and 2 uncertain
positions. Two lanes require review. The original 84/48 total above is the sum
selected by the version 3 lane reports before this conservative merge.

Coverage analysis also found that all eight north tape ends were within two
sprocket pitches of the Y=195 mm scan edge. The eighth lane also reached the
south edge. The merged counts are therefore lower bounds, not complete feeder
counts. The automatically recommended extension region is X=75 to 175 mm and
Y=92.0 to 209.5 mm. No motion was used for this replay.

The retained target scan covers only X=70 to 275 mm and Y=105 to 195 mm. It has
no workspace-discovery preview and cannot reproduce the earlier full-workspace
PCB overlay after the host restart removed that temporary image. A new
full-workspace reconnaissance scan and focused edge scans remain required for
the final combined physical artifact.

### Full semantic workspace and feeder inventory overlay

On 2026-08-26, the operator confirmed the full travel envelope. Coarse home
`38d86e6f-b51b-4417-8fa3-fff4683b8662` established X0, Y0, and Z31.5. Fine
home `94c8f228-588e-49a9-9264-db2ce76a3a02` repeated the N2 datum contact at
Z59.1 and applied zero correction.

Full reconnaissance scan `badf767f-967f-4ef0-91d7-2401dc1292ac` captured all
1,023 planned tiles. It covered X=-8.026 to 441.032 mm and Y=84.054 to 325.991
mm at 0.2 mm/pixel. Capture, mosaic generation, and seam analysis took 20
minutes 47 seconds. Seam health selected seven focused regions.

Refinement batch `a86dee55-dc9a-456e-a083-36b6dea2db58` captured all 2,855
planned height-aware tiles in about 87 minutes. Its child scans were:

| Target | Region | Tiles | Scan |
| ---: | --- | ---: | --- |
| 0 | main feeder bank | 1,007 | `cd0fe50f-61bf-4023-897d-8970dfa2c2d9` |
| 1 | smaller feeder area | 530 | `4a57e86f-fa56-4f27-b18d-0e12b92cfd8a` |
| 2 | nozzle bank | 180 | `8bd5a682-1ad6-414d-a7e3-5c891e00b202` |
| 3 | fixed camera-area feature | 50 | `7e6bfaed-fb10-4b79-ade0-7b75a3eed13d` |
| 4 | fixed camera-area feature | 8 | `791130e8-9490-4e2c-9120-3eacb7fbe40f` |
| 5 | datum and bottom-camera area | 306 | `bcf151c6-d831-41a1-a226-c7baf2e71bce` |
| 6 | FTP PCB | 774 | `fe31d7a6-d76d-428b-9e07-1662b889d99c` |

All seven patches had complete coverage and passed the automatic composition
gate. Object proposals then found seven workspace elements: the FTP PCB, two
feeder regions, the datum PCB, two nozzle-bank regions, and the bottom-camera
guard. The FTP board matched `Opulo FTP PCB REV00` at 0.9905 confidence. Its
fitted center was X395.457, Y258.778 mm, and its angle was -1.510 degrees. The
preview retained the fitted board outline, 80 enabled top-side component
placements, and three KiCad fiducials.

The focused feeder crop also fit a 75 mm square inside the tape bank. Strong
staging-plate parallel-track evidence differed from that false board match by
only 0.012 confidence. The classifier now routes that close conflict to the
feeder specialist and retains review state. The first specialist replay found
one false three-hole lattice between two tapes. Version 4 requires five holes
for offline workspace analysis and keeps the shorter live-holdout rule
separate.

The final 0.1 mm/pixel replay found exactly eight populated component strips.
All eight are 8 mm tape with 4 mm part pitch. It measured 110 present, 45
absent, and no uncertain visible feed positions. Seven tape extents are
complete in the refined image. The eighth leaves the south image boundary, so
its 27 present and 2 absent positions remain lower bounds.

The parent scan stores the combined RGB artifact as
`mosaic-strip-feeder-inventory.png`. It is 4,493 by 2,421 pixels and 8.7 MiB.
Its SHA-256 is
`7fb0bb5a814e039c79b763896e8991521f8cb3ef1d5259ee718570a89f017002`.
The image keeps all semantic workspace outlines, the PCB design overlay, the
height-corrected feeder pixels, strip outlines, and present or absent marks.

Normal disconnect operation `80b4775e-b066-47a0-a59f-9b989fcc218c` disabled
the machine and released control. No emergency stop was used. Final state was
disconnected, unhomed, and fault-free.

### Initial operator-labeled feeder inventory replay

On 2026-08-27, the operator gave initial labels for all eight strips in the
retained feeder crop. These labels contained 170 component pockets: 117
occupied and 53 empty. A later visual review corrected four lower endpoints;
see the next section. The source remains
`object-evidence-000.png` on scan
`cd0fe50f-61bf-4023-897d-8970dfa2c2d9`. Its SHA-256 is
`b08a2ddbe513b79de702cbe057cf7b879a35bc23633a5cbb5f86288dd1d129bf`.

Version 5 preserves the measured 4 mm pitch and checks adjacent phase-correct
pocket centers against the retained image. It also separates strong luma
transitions, homogeneous populated lanes, and small dark parts on bright tape.
Weak occupancy transitions remain uncertain.

The no-motion replay found 168 of the 170 labeled pocket centers. It omitted
the final bottom pocket on feeder 3 and feeder 5. This is 98.8% pocket-center
coverage. Of the 168 detected centers, 165 confident labels matched the
operator labels, none disagreed, and three were uncertain. The uncertain
labels are one empty transition pocket on feeder 1, feeder 2, and feeder 5.
This one scene is a regression fixture, not general feeder validation.

The rebuilt inventory contains 115 present, 50 absent, and 3 uncertain visible
positions. Feeder 7 and feeder 8 are correctly all populated. Feeder 8 still
reaches the source-image boundary, so the inventory keeps incomplete coverage
state. The regenerated 4,493 by 2,421 full-workspace overlay has SHA-256
`057e163ce5c00fac547835acca1e72c342dc363510d1fd35676ad4465bc77338`.

Baseline `4db1e1da-1f58-488a-a2f4-cca2ec1b0a5b` activated parent scan
`badf767f-967f-4ef0-91d7-2401dc1292ac`. It uses calibration
`3b837434-a961-4e60-945a-18335b42d21f` and object proposal version 6. The scan,
refinements, feeder report, overlay, and active baseline are in the persistent
database. The replay used a simulated daemon and caused no machine motion.

### Corrected feeder endpoint replay

On 2026-08-27, a follow-up review rejected one lower pocket on feeder 2 and
added one lower pocket on feeder 4, feeder 6, and feeder 7. It also superseded
the earlier lower-end estimates for feeder 3 and feeder 5. The corrected
fixture contains 170 pockets: 118 occupied and 52 empty.

Version 6 raises the first endpoint patch-correlation gate from 0.35 to 0.43.
The rejected feeder 2 patch measured 0.412. It expands the positive range gate
from 2.08 mm to 2.12 mm for a 4 mm pitch. Feeder 7 measured 2.099 mm from the
last sprocket and had 0.974 patch correlation. One further phase-correct slot
can pass when direct image evidence remains. The added feeder 4 and feeder 6
patches measured 0.911 and 0.434. This is a general image gate, not a stored
count for a specific feeder.

The corrected no-motion replay found all 170 labeled pocket centers. It had
167 correct confident occupancy labels, no incorrect confident labels, and
three uncertain labels. The uncertain labels are the empty transition pocket
on feeder 1 and feeder 2, and an occupied transition pocket on feeder 5.

The rebuilt inventory has 117 present, 50 absent, and 3 uncertain positions.
Feeder 8 still reaches the source-image boundary, so the report remains
clipped. The regenerated 4,493 by 2,421 full-workspace overlay has SHA-256
`0e03e134a6fe2c210e4821b94ea73d1e4e810664cb740b775e921d76f670daf7`.
The simulated daemon updated the persistent parent scan and caused no machine
motion.

### Physical strip-feeder pickup qualification

On 2026-08-27, after a host restart, the camera devices rejected the physical
daemon user. The user was added to the `video` group. The release daemon was
then started in that group. Both cameras opened and captured normally.

Draft `4447a5fb-3fe2-468a-8c3c-bcf10910a7cb` represents feeder 1 from focused
scan `cd0fe50f-61bf-4023-897d-8970dfa2c2d9`. The operator removed its cover
tape and confirmed the travel envelope. Coarse home
`c6b62e2b-41da-4f64-bd00-048f196e9ef7` passed. Fine home
`f596c719-685f-453b-8665-c34ed5cadfea` detected N2 with its N24 tip and reset
the datum contact from Z59.4 to Z59.1.

The first local holdout showed only four sprocket holes in its camera field.
The local rule now accepts a three-hole 4 mm lattice. The retained workspace
rule still requires five holes. A dark-disc detector now adds live sprocket
evidence when a rectified image seam hides a circle edge. Holdout
`0f620f1a-8aeb-4d68-818c-54c0a3c3a21f` then passed with four sprocket centers,
0.047 mm pitch residual, 0.845 mm position correction, and 0.722 occupancy
confidence. It selected pocket 12 at measured X85.892, Y155.137 mm.

The first physical pickup attempt measured a median vacuum value of 230 at
Z3.7 and 228 at Z3.6. The learned acquire threshold was 230.5, but the descent
used only the larger coarse delta gate. It therefore did not report contact
and did not transfer the part. The stop rule now accepts either the calibrated
threshold or the coarse delta gate. A second run at that point found no vacuum
transition. The fresh holdout above selected a different occupied pocket.

Qualification `afc75a54-17f2-440a-a2a4-465d02e1b4e7` passed at Z3.6. Its open
median was 236. The pickup median was 230, and the post-transfer hold median
was 228. Bottom-camera frame difference found the held part with 0.365 mm
center offset and 0.703 confidence. The runner returned to the exact source
head pose, lowered to the detected pickup Z, disabled vacuum, and confirmed a
released value of zero. Its disposition is `returned_to_pocket`.

The retained pickup artifacts and downloaded SHA-256 values are:

- `bare-nozzle.png`:
  `3a0014198827568ceb9309c1e069f3741b105f5336844523c50c3a5844473cc5`
- `held-part.png`:
  `73bd4b7e82bdac7f0a8b1561cc8639b7beaf70b955af6d1948f3700ae934ccf6`
- `held-part-inspection.png`:
  `ea341a47387daf8ed2ca0b4c5f6db0d3149e415291438c581d93c6608df9f922`

The live detector selected pocket 12, but the first stored registration used
the requested anchor index 10. The measured pickup point was correct. The
registration update now derives the index from the selected visual pocket.
The retained record was repaired to index 12, and the SQLite integrity check
passed.

Normal disconnect `2c411232-e2c1-4bff-9400-9002ffca012a` left the controller
disconnected, unhomed, and fault-free. Vacuum and lights were off. No emergency
stop was used.

On 2026-08-27, the retained bare and held frames were replayed after review
showed that version 1 measured the inner nozzle as a 4.94 by 5.20 mm square.
Version 2 first aligned the bare reference by X+0.50 and Y+1.50 pixels. Signed
foreground selection and a bounded opening removed the nozzle ring. The same
capture then measured the component as 3.36 by 1.57 mm at 2.33 degrees, with a
0.471 mm nozzle-axis offset and 0.835 confidence. The green contour follows the
component, and the cyan oriented box no longer contains the nozzle ring. The
replay caused no machine motion. Its downloaded overlay SHA-256 is
`3b4f1756796986a89f332f532c9f4f89d2a5705b608befb836e1563cd32e23e3`.

### Persistent pocket state and verified return

On 2026-08-27, release build validation used the same exposed feeder 1.
Coarse home `23a7d292-6542-4c4e-9b5d-b93a0ad4a7bc` passed at X0, Y0, and
Z31.5. Fine home `95cfc469-2101-4e84-b7fd-89aad5baf4dc` detected N2 with
its N24 tip. It reset the datum contact from Z59.4 to Z59.1.

Holdout `46bc9b97-9242-4957-afe1-b15243449ab7` passed with four sprocket
centers. It measured 0.071 mm pitch residual, 0.884 mm total correction, and
0.683 occupied-pocket confidence. It selected pocket 12 at X85.891 and
Y155.177 mm. The store marked only that occupied pocket available.

Pickup qualification `9758c19b-ef6b-4257-81b5-369243016371` reserved pocket
12 before motion. N1 reached its vacuum threshold at Z3.6. The held median was
228. The released samples were all zero. Bottom inspection version 3 measured
the part as 1.530 by 3.144 mm at 0.281 degrees. Its center offset was 0.452 mm,
and its confidence was 0.842.

Top vision found the source pocket occupied before pickup at 0.473 confidence
and after release at 0.533 confidence. The return check passed. The persistent
pocket state records one attempt, zero consecutive failures, a `returned`
outcome, and no active reservation. The pocket remains available. The feeder
registration stores the measured part size, rotation, and confidence.

Version 3 removes a narrow nozzle reflection that remains connected to a
package edge. Its retained held-part overlay SHA-256 is
`baeef45dc1502075fb7df710bc07829eb331883cbf9ec3ba551f19a89dbe6ec5`.
The retained before-and-after return overlay SHA-256 is
`8c09dab4389f2966b5cfbc815d72bf44edd80a0d45e1169345e290ab8fb484fc`.

No-motion replay later showed that the green version 1 circle was not the
pocket center. Its center was at the top package edge. The live phase search
had selected the strongest package edge instead of the package center. The
original workspace estimate, Y154.297 mm, was close to the physical center.

Version 2 keeps periodic phase as a tentative locator and measures local
foreground support for the package center. It measured the before center at
X85.926, Y154.257 mm and the after center at X85.922, Y154.307 mm. The
before-to-after shift was 0.050 mm. In the corrected replay, cyan marks the old
nominal location and green marks the measured centers. The replay caused no
machine motion. Its overlay SHA-256 is
`4a1a015ac6b386653d8d281d05e04c0b87e33cf3a39978cc019df136bbc0e3e9`.

Normal disconnect `31d1d944-4edc-4f22-a22b-58f6d54c95bb` left the
controller disconnected, unhomed, and fault-free. Vacuum and lights were off.
No emergency stop was used.

### Closed-loop pickup alignment and failed return detection

On 2026-08-28, coarse home
`f172a921-c9cd-439f-abdc-ba288a5990ce` and fine home
`f475db46-44ad-4690-b0a2-b5ac03d8cd00` passed. Fine home detected N2 with
its N24 tip and retained the Z59.100 mm datum contact.

Live holdout `26abc741-e5c9-46c8-9949-04ec4583101f` selected feeder 1 pocket
12. Its predicted center was X85.807, Y154.297 mm. Local vision measured
X85.945, Y154.228 mm, a 0.154 mm correction. It found four sprocket holes
with 0.035 mm pitch residual and 0.820 occupied confidence.

Pickup qualification `d31c0c2e-0f8e-43b1-98de-f0f796e12149` acquired
vacuum at Z4.3 and retained a raw value of 227 after transfer. Bottom vision
measured an initial 0.519 mm center offset and -4.637 degree package angle.
Closed-loop correction reduced the center residual to 0.051 mm and the axis
residual to 0.282 degrees.

The original return gate incorrectly passed. Direct image review showed a
component in the before image and an empty pocket floor in the after image.
The vacuum sensor was zero after release, so the component was not held on the
nozzle. Its final location was not observed.

Return algorithm version 3 adds a package-sized, normalized before-and-after
appearance check. No-motion reanalysis measured 0.347 similarity, below the
0.55 gate. It changed the qualification and operation to `failed`, changed
pocket 12 to `uncertain` and `needs_review`, and recorded
`release_uncertain`. It did not add a physical pickup attempt. The selected
safe pocket reverted to pocket 10. SQLite integrity validation returned `ok`.

The corrected return overlay marks the rejected after image in red. Its
SHA-256 is
`278e6ef57614da15a5afea4fd5ee9df8638b18bb7c005603abe697c117670c31`.
The physical daemon disconnected normally as operation
`f0501c35-f7e2-40b8-83fc-361f03268573`. The reanalysis daemon used simulation
mode and did not open the controller.

### FTP-board relocation and resistor pickup release

On 2026-08-28, height-aware scan
`fe80f3b4-84ae-49a1-b6a0-7d5747eef55e` found 22 feeder-7 pockets and all
29 feeder-8 pockets after the operator moved both tapes. Fast scan
`badf767f-967f-4ef0-91d7-2401dc1292ac` relocated the FTP PCB after the
operator removed and replaced it to add double-sided tape.

Coarse home `639b78cc-13b9-478c-be11-f3a59ab7bf78` and fine home
`616ddbe3-38e5-44b2-a47b-141e95bfc0b5` passed. Fine home detected the
installed N24 tip on N2 and refined the datum contact from controller Z59.2
to Z59.1. Board registration
`7a84d68f-1377-4808-936f-50fec50b7d92` measured three FTP fiducials. It
passed with a 0.047 mm RMS residual, a 0.065 mm maximum residual, and a
0.076 degree board angle. A fallback mask-circle detector needed the full
configured 2.5 mm search radius because the scan seed was more than one
fiducial diameter from the local measurement.

Feeder-8 draft `e719d20a-2bfc-40d8-9c04-1eae273bf7eb` describes the FTP
0603 resistors. Holdout `cf8cab4a-b32e-4fdf-adde-a533c3cf6583` passed with
four sprocket centers, 0.060 mm pitch residual, and 0.954 occupied-pocket
confidence. It measured a +0.464 mm X and -0.038 mm Y correction.

Pickup `d43f4e0d-4aea-422d-b6f0-dee625b4979f` returned its part, but the
original nozzle-clear gate treated a small nozzle reflection as a retained
component. Release verification version 2 now compares post-release size,
changed-pixel support, and contrast with the held component. This keeps a
real retained component as a failure while it rejects the smaller, faint
reflection.

Pickup `ea354d22-0c58-4d39-b550-4d43e7cb99fa` then found a real retained
resistor after both bounded release attempts. The vacuum sensor was at its
off value, but bottom vision still showed a 1.86 by 0.99 mm part. The operator
removed the part, returned it to the tape, and cleaned the N045 tip with
alcohol.

Pickup `90773d84-c748-416d-bd1e-5247cdf849d2` passed after cleaning. Bottom
vision measured the resistor as 2.262 by 1.263 mm. Closed-loop correction
ended with a 0.043 mm center residual and a 0.024 degree axis residual. The
part released on the first attempt. Top vision measured a 0.036 mm return
shift and confirmed the same pocket occupied. The machine returned to level
Z with vacuum off.

Pocket-3 pickup `4a33564d-0e9e-4ee2-ab95-5cb6f866a2c2` also released on
the first attempt and left the nozzle clear. Top vision found the pocket
occupied with 0.982 confidence and measured a 0.123 mm center shift. The
fixed-pixel appearance check still rejected the return at 0.404 similarity.
Direct inspection showed the resistor in the correct pocket with a small pose
change.

Return version 4 searches only a component-interior patch over at most
0.16 mm of local translation and 15 degrees of rotation. No-motion replay
scored the returned resistor at 0.591. The exact earlier empty-pocket failure
from qualification `d31c0c2e-0f8e-43b1-98de-f0f796e12149` scored 0.405 and
remained rejected by the unchanged 0.55 gate. Reanalysis then passed pocket 3
without adding a physical pickup attempt.

Release-daemon restart invalidated homing as designed. Coarse home
`a997c457-6770-4cd7-9f52-24cd77baaff8` and fine home
`4498d4f8-0cd1-442f-8368-c5c818e83a70` passed. Fine home detected N2 with
its N24 tip and reset the datum contact from controller Z59.2 to Z59.1.

Pocket-4 pickup `34cab48d-ebfb-4f93-b74d-10e35f1e23df` passed at nozzle
Z4.0. Bottom vision ended at a 0.068 mm center residual. The part released on
the first attempt. Top vision measured a 0.046 mm return shift and 0.843
aligned appearance similarity. Pockets 2, 3, and 4 are distinct passed
returns. Feeder 8 is ready as active feeder
`15080231-ec84-4b6b-a341-11a24ca6bcfb`.

### Non-contact FTP board-surface measurement

On 2026-08-28, coarse home
`8a78b3a1-85af-4880-a390-5c72680607ff` and fine home
`d33da4c7-10b1-4579-86d9-83728cfd1a53` passed after the surface-measurement
daemon restart. Fine home detected N2 with its N24 tip and reset the datum
contact from controller Z59.3 to Z59.1.

The first surface attempt
`f44eb7e6-4aeb-4b91-9c9d-2dcd514336b5` stopped before its third camera move.
A fixed +3 mm Y view was outside the top-camera model support. No PCB contact
occurred. Surface algorithm version 2 derives its two offset views from 65%
of the current model aperture instead of using fixed offsets.

Surface run `54576534-fdeb-4d3e-9264-f405bc5e7ede` then observed FTP fiducials
FID1, FID2, and FID3 from three camera poses each. All nine views used
controller Z31.5. No nozzle touched the PCB or its double-sided tape. Nine
view pairs measured a median surface fraction of -0.0355. The p95 fraction
spread was 0.0476. Fit residual was 0.015 mm RMS, 0.034 mm p95, and 0.034 mm
maximum.

Using the N1 datum-contact reference of nozzle Z3.4 and the installed 1.6 mm
plane separation, the run measured N1 board-surface Z3.457 with 0.126 mm
uncertainty. It passed and updated board registration
`7a84d68f-1377-4808-936f-50fec50b7d92`. The persistent report SHA-256 is
`cdc2caafed686a3b310ef4a0ed84da077c942c8bff897381c9c43b3932770dd6`.

### First guarded FTP resistor placements

On 2026-08-28, coarse home
`06a7af51-4da4-413f-b871-b83ac58c3596` and fine home
`f332563a-fce8-4c28-8978-9f909fc2087e` passed. The first `R2` execution,
`6cb1a18f-cb19-47d0-9498-b63b62acbd34`, picked feeder pocket 2 and detected
the resistor. It stopped before board motion because the required 48.334
degree turn exceeded the former 30 degree alignment limit. Recovery returned
the part to its source pocket.

Execution `20f7e97d-b173-4192-b4c6-681407cd04be` selected pocket 3 for `R3`.
Vacuum contact occurred at nozzle Z4.1. Bottom vision aligned the part to the
planned 0.076 degree board angle with a 0.052 mm center residual. The machine
reached component-center Z4.007 and turned vacuum off. The later bottom-camera
move incorrectly requested the pre-alignment nozzle angle, so the normal-jog
rotation interlock stopped the run. A separate top capture confirmed the
resistor on the `R3` pads. Its SHA-256 is
`56f77b7370490e0319f9c3c87e8d1cd42959a1bd9929cc0636b4b2f7d0e6dac0`.
The feeder count was corrected from 27 to 26. Pocket 3 remains conservatively
marked `needs_review`.

Release inspection now uses translation-only motion and preserves the nozzle
angle. The package-axis bound is 90 degrees, which covers all orientations of
a 180-degree-symmetric package while remaining below the driver's validated
165 degree rotation envelope.

After the daemon restart, coarse home
`94fa9af4-47a8-40d2-a33b-ad0c69950190` and fine home
`5d165c22-0a89-478b-9c46-187b02a60995` passed. Execution
`fd503af2-43c8-49b5-a955-a5e873a6df61` then completed the guarded `R2`
placement from pocket 4. Vacuum contact occurred at nozzle Z3.9. Bottom vision
aligned the resistor to 45.076 degrees with a 0.066 mm center residual. Three
job moves used the validated local direction compensation.

Vacuum release passed on the first attempt, and bottom vision confirmed a
clear nozzle. Top vision detected the placed resistor. The executor marked
pocket 4 consumed, set its occupancy to empty, and reduced feeder inventory
from 26 to 25. It retained ten PNG artifacts. The top-inspection overlay
SHA-256 is
`ac6e450e1dd27655ea083cbcf1ccd246c546d1771bada56222d1b8721d1f0699`.

The top difference detector passed, but its 0.525 mm center result and -68.6
degree angle do not agree with the visually coherent 45 degree placement.
Board inspection therefore needs a board-specific package-template pose fit
before it can enforce placement accuracy. SQLite integrity validation returned
`ok`. The machine ended ready and homed at safe Z with vacuum off.

### Placement-outcome qualification and R12 stop test

On 2026-08-28, the operator reviewed the first three seating-offset trials.
R9 was recorded as `tombstoned`, R10 as `good`, and R11 as `rotated`. The
stored records retain both the automatic result and the operator override.
The trials used three different seating offsets, so no offset has the minimum
three observations needed for a process recommendation.

Coarse home `93f634a8-1024-493a-b69e-72a7219abb46` and fine home
`34b86685-2f40-44dc-bc4e-03d9e3b768c7` passed. Fine home detected the N24
tip on N2 and reset the datum contact from controller Z59.4 to Z59.1.

Job `00000000-0000-4000-8000-000000000012` defined a bounded R12, R13,
and R14 qualification at a +0.10 mm seating offset. Execution
`254cdaf7-ff27-4e9e-a971-af6fdc9d8687` picked R12 from feeder-8 pocket 8.
Bottom vision ended with a 0.083 mm center residual and a 0.491 degree package
axis residual. The tool reached nozzle Z4.107 and turned vacuum off. All
release samples were zero, which confirmed pneumatic release.

The old visual release gate then treated the empty N1 nozzle bore as a retained
part. The held resistor was 1.318 by 2.268 mm, used 1877 foreground pixels,
and was brighter than its reference. The two post-release bore detections were
at most 1.210 by 1.655 mm, used at most 1012 pixels, and were darker than their
reference. The bounded sequence stopped after R12. It did not run R13 or R14.
The automatic R12 outcome remains `uncertain` pending operator review.

Release matching now requires at least 75% of each held-part dimension and the
same foreground polarity. The executor also records pneumatic release apart
from vision proof. If vacuum confirms release but vision is uncertain, it no
longer claims that recovery returned the part to the source pocket.

A separate safe-Z top-camera capture shows R12 on its pads. An approximate
offline registration measured a +0.24 mm X and +0.14 mm Y error and a +5.0
degree angle error. This result is provisional because it used one manual
frame and an approximate retained camera mapping. It did not change the stored
operator outcome.

R10 also showed a known orientation limitation. Its top marking proves that it
was not physically face-down, but it was rotated 180 degrees in the board
plane. Its part record identifies a resistor and stores the
`rectangular_passive` profile. However, the placement executor does not yet use
that profile. Its current bottom-alignment angle function treats 0 and 180
degrees as equal for every part. This is valid for R10, but it is not valid for
a polarized part. Polarized parts require a separate full-angle profile that
combines KiCad pad direction, feeder direction, and package-mark detection.
Preflight must reject polarized placement until that direction is known.

### Orientation-aware pickup and R13 release stop

On 2026-08-28, the resistor part received the explicit `axis_symmetric`
orientation policy. Retained R12 bottom-camera frames also exercised the
`brighter_end` and `darker_end` policies without motion. They resolved opposite
package directions with 30.50 luma of end contrast. This validates the directed
detector on retained evidence, but it is not a physical LED polarity test.

The first v5 pickup qualification,
`0fabec62-c507-4a43-a6ae-498aa1a61c46`, returned its part but stopped at a
1.337 degree final angle residual. The 1 degree gate was smaller than the
0.10 mm translation effect allowed for this 0603 package. The profile now uses
a 2 degree residual limit without weakening the separate 180 degree phase
check.

Coarse home `34fc0ada-1ae1-4af5-a618-d58f821e9f00` and fine home
`ab0dac5c-308c-4478-82bf-206f55eb01ab` passed. Pickup qualifications
`661e7319-fae0-43df-ba7d-4e6bcc4b6e51`,
`ee6c8a98-eac4-483a-bf5f-7cdd76a1df8a`, and
`03ba7ba8-7c3b-44f2-9e2e-591191416ad1` then passed at distinct pockets. Their
final center residuals were 0.085, 0.073, and 0.072 mm. Their angle residuals
were -0.759, +0.394, and -0.564 degrees.

R13 execution `1ff38bf5-d4ff-49be-8908-d529578ca051` stopped before transfer
because pocket 9 had no vacuum transition. It exposed a difference between the
planned strip position and the first available runtime reservation. Guarded
placement v3 now reserves the available pocket at the planned position. It
also captures that pocket immediately before pickup and uses the detected
component center for the nozzle move.

After deployment, coarse home `09b5413c-8172-498d-a849-cd7a6bc86276` and fine
home `30193f42-4879-4b52-9186-53c68ee5ace8` passed. R13 execution
`e59defa0-83e0-4647-a92b-c431e0286e14` reserved pocket 12. Top vision applied
a +0.312 mm X and +0.119 mm Y pickup correction. Vacuum contact occurred at
nozzle Z3.8. Bottom vision aligned the resistor to the +0.076 degree board
target with a 0.087 mm center residual and a -1.637 degree angle residual.

The tool reached nozzle Z4.107 at R13 and pneumatic release passed. Bottom
vision found the same resistor on N1 after both bounded release attempts. The
operation correctly stopped without claiming placement success or consuming
the feeder pocket. Pocket 12 is `needs_review`. The machine ended ready and
homed at safe Z over the solid datum-board area, with vacuum off and no
controller fault. The resistor remains mechanically attached to N1 and needs
removal or a validated stuck-part release process.

### R13 release-cycle experiments

On 2026-08-29, N1 returned to the recorded pocket 12 pickup pose and lowered
to the prior successful pickup height at nozzle Z3.8. It ran three cycles that
enabled the pump and MOS2 valve for 250 ms, then disabled both for 500 ms. A
final off command succeeded before the tool retracted to Z31.5. A bottom-camera
capture still showed the resistor moving with N1. The combined cycles did not
release the part.

The V3.0.4 profile then gained an independent `blowoff_1` actuator. Driver tests
verified that it sends only `M106 P1 S255` and `M107 P1`. After daemon restart,
N1 remained physically at safe Z over pocket 12. The controller was connected
but not re-homed. Three 250 ms valve-only pulses succeeded with the pump off,
followed by explicit valve and combined-output off commands. Release remained
pending bottom-camera confirmation. Standard XYZ home
`04ae94a7-58fc-4862-8682-ec2aae242b51` and fine XYZ home
`4c2d3fb8-f092-422e-a498-b1124258fb64` then passed. The bottom-camera frame was
unchanged from the pre-pulse frame and still showed the resistor moving with
N1. Valve-only cycling did not release the part and must not become an
automatic retry sequence.

### R13 nominal-depth board recovery

The retained R13 part was recovered on 2026-08-29. The prior bottom-camera
observation supplied a +0.507 mm X and -0.153 mm Y replacement correction. A
live frame at head X242.914 and Y215.479 showed the resistor centered on N1.
A fresh top-camera frame showed R13 empty before motion to the board.

The failed placement had used controller X386.787, Y336.150, and Z4.107 with a
+0.10 mm seating offset. Replacing the old alignment correction produced head
X386.745 and Y336.143. The recovery used controller Z4.007, which is the
nominal 0.00 mm seating target and 0.10 mm deeper than the failed attempt. At
contact, the pump remained off and `blowoff_1` stayed open for 500 ms.

The tool retracted straight to Z31.5. Bottom vision then showed a clear N1
nozzle, and the top-camera after frame showed the resistor across the R13 pads.
The machine ended ready, homed, and fault-free at the datum pose with all
pneumatic outputs off and a vacuum sensor value of zero. The retained images
are `/tmp/velocitypnp-r13-recovery-aligned-bottom.png`,
`/tmp/velocitypnp-r13-recovery-post-release-bottom.png`,
`/tmp/velocitypnp-r13-recovery-board-before.png`, and
`/tmp/velocitypnp-r13-recovery-board-after.png`.

Guarded placement version 4 now uses this bounded recovery after pneumatic
release with a visually retained part. It replaces the old alignment with the
new bottom-camera correction, rejects excessive rotation, seats by the
profile's 0.10 mm recovery depth, pulses the matching nozzle valve for the
profile's 500 ms duration, and requires a clear-nozzle verification.

### First directed LED pickup stop

On 2026-08-29, coarse home `d227e622-24c9-40ae-9db9-cfaf4a2a12d2` and fine
home `e95fd72f-efaf-42bd-836b-b09c0ba89c59` passed. Live holdout
`3014548e-0803-4fbf-aa31-534f1c4146a9` measured feeder 7 with a 0.042 mm
pitch residual. The requested pocket 14 was empty, so the holdout selected
the strongest occupied pocket in its bounded span, pocket 15.

Pickup qualification `3d5ef3c7-36f3-45d3-9bd5-ec69e62ba6bf` acquired an
LED at nozzle Z4.0. Bottom vision measured 1.786 by 0.533 mm at -10.136
degrees. It measured 5.61 luma of brighter-end separation. The provisional
6.0 luma floor stopped alignment. Retained-image analysis with a 4.0 luma
floor passed without changing the measured pose. The stored LED part now uses
the 4.0 luma floor. This value is provisional until several physical LED
captures show repeatable end direction.

Vacuum pressure returned to its off state, but bottom vision found the LED on
N1 after both release attempts. Manual recovery returned to the recorded
pocket and then used the configured recovery area. Single 500 and 700 ms
valve pulses, three 250 ms valve pulses, and a repeat at nozzle Z4.0 did not
clear the nozzle. The LED moved onto its edge. Generic jog commands rejected
the proposed rotation cycle because production rotation motion is not yet
hardware-validated. No rotation occurred.

The machine stopped ready and homed at safe Z over the bottom camera. The
pump and valve are off. The edge-standing LED remains mechanically attached
to N1 and needs operator removal. This failed experiment does not permit an
automatic source-pocket valve retry.

### Directed LED geometry and alignment correction

On 2026-08-30, pickup qualification
`96acfe40-07c2-4109-b9da-f2f9902b9238` acquired another LED at nozzle Z4.2.
The initial 0.549 by 1.857 mm observation was much narrower than the feeder
survey's 0.900 by 1.699 mm body. The raw image showed an edge-held LED. The
alignment loop also compared the rotated nozzle with its zero-angle reference.
This inflated later measurements and caused an invalid second correction. The
part was released, but top vision did not confirm an accurate pocket return.

Pickup now compares the initial bottom-camera dimensions with the feeder
survey. Each dimension must be between 70% and 150% of the survey value. This
rejects the 0.549 mm edge pickup but permits bounded lighting and segmentation
growth around a flat package. Each later alignment image must also match the
initial dimensions.

The alignment translation now rotates with the commanded nozzle angle. If the
initial part-to-nozzle correction is `C` and the nozzle turns by `a`, the move
uses `R(a)C`. This accounts for the part center rotating around the nozzle
axis. The initial directed phase plus the calibrated nozzle command supplies
the corrected phase. Later bottom-camera images still verify part retention,
dimensions, and center position. They do not replace that phase with nozzle
reflection artifacts.

Coarse home `ace08d9f-7077-4dcd-af81-a9534ef1a099` and fine home
`9f51747e-5f8d-48e7-bbd4-2692dcae8a43` passed. Holdout
`e5632eb8-a082-4c0d-b65f-8bacd6f01512` passed. Qualification
`3b1bdde1-fd0e-4fda-8fc3-82a897817d8b` then acquired a flat 0.982 by
1.773 mm LED. Its initial center error was 0.444 mm. One combined -0.721
degree rotation and XY correction reduced the center residual to 0.049 mm.
The final image had only 3.5 luma of end contrast, so this run stopped and
returned the LED. Release and top-camera pocket return passed.

The directed policy now uses the registered feeder phase when the visible end
contrast is ambiguous but package geometry passes. This fallback uses the
measured package axis. It cannot override an edge-size failure. Qualification
`45435485-0434-403f-939a-45e7b1306907` stopped without pickup when another
pocket produced no vacuum transition. Qualification
`bb52615b-a45c-4a69-a665-b561e84fbc5d` acquired and returned a flat LED, but
ran before the registered-phase fallback was deployed.

The final build passed coarse home `14e42b16-0eaa-41ab-a23e-adc48dc0725e`
and fine home `8352a49a-6283-40ca-9948-4b40907dd35d`. Qualification
`dd007d04-fb22-4ee6-9688-077c4a0b9bd2` acquired a 0.958 by 1.828 mm LED at
nozzle Z4.2. Its initial angle was +12.482 degrees and its center error was
0.144 mm. One combined correction produced a 0.008 mm center residual. The
rotation model resolved 0.000 degrees, while the final image independently
measured +0.908 degrees. Vacuum release, clear-nozzle inspection, and top
pocket inspection passed.

The final return moved the LED by 0.325 mm in its pocket. Earlier returns also
changed small LED angles. Repeated return qualification is therefore not the
preferred validation path for this irregular 0603 package. Future LED checks
should place a good, aligned part on the prepared board and use top-camera
inspection instead of repeatedly disturbing the carrier tape.

### First guarded LED placement attempts

On 2026-08-30, standard home `9bbcc13f-c77f-48ec-8886-c07fee178a3b` and
fine home `18b7fd70-f826-45ee-9e32-44953b1e0850` passed. Retained
qualification `dd007d04-fb22-4ee6-9688-077c4a0b9bd2` created a provisional
LED feeder. This feeder path needs one passed guarded placement instead of
two more carrier-tape returns.

Live holdout `4e976f74-aa40-483f-852a-21f3e654733d` found an occupied part
near pocket 12. Placement `0d214776-e440-4bc5-8282-20cf0bc4e042` found no
vacuum transition through its bounded Z3.6 search. Top vision confirmed that
the LED remained in the pocket. The pocket was quarantined and D1 was not
touched.

Live holdout `ebaf6b86-fdd9-4ca2-b564-96172119f4c0` corrected the inventory:
pocket 6 was empty and pocket 7 was the first occupied position. Placement
`af7df812-6a73-48ab-9889-8ad14edeeb12` acquired that LED and measured it as
1.024 by 1.619 mm with 17.7 luma of end contrast. Its initial direction was
174.0 degrees, so alignment commanded a large directed turn.

The zero-angle bare-nozzle reference did not match the nozzle appearance
after that turn. It enlarged the final difference outline to 1.711 by
2.235 mm even though the LED remained visible. Recovery returned the LED to
its source pocket. The strict 70% to 150% size gate remains in use for the
initial identity check. Post-rotation retention now permits up to 200% outline
growth, while it keeps the 70% lower bound, appearance, vacuum, centering, and
iteration gates. This change still needs a physical rerun.

A passed live holdout now updates the active feeder's planned pick pose to the
measured occupied pocket. The two attempts required a manual feeder API update
because that behavior was not yet deployed.

### Repeated resistor placement and occupied-target failure

On 2026-08-30, live holdouts
`eedee114-bd0b-4c08-9f1f-d2489f4f9208` and
`a7a29dd4-11a7-4f5b-b996-b0fa6dadbcc8` passed with 0.447 mm and
0.270 mm pickup corrections. Both measured 96% strip occupancy.

Placement `bac0f763-00e1-4b64-b3c1-57dccde3a487` reported R12 as good.
Its bottom-camera center residual was 0.088 mm. Top inspection measured
-0.020 mm X, -0.240 mm Y, and -0.033 degrees. Placement
`914ae61c-7a80-4976-aff2-b0f13be4e6cc` also reported R13 as good. Its
bottom-camera center residual was 0.091 mm. Top inspection measured 0.000 mm
X, -0.400 mm Y, and -2.870 degrees.

The operator found two resistors stacked at both R12 and R13. Retained
pre-placement images show that each target already had one resistor before
these executions. Earlier failed release or inspection runs had reached both
board targets. The executor captured the pre-placement images but used them
only as change references. It did not reject an already contacted target.

The ordinary physical executor now stores the exact board instance in each
report. It rejects a reference when any earlier physical execution reached the
placement pose on that board instance. Old reports use their stored
single-board job for this check. This conservative one-contact rule also
protects targets where release evidence is uncertain. A future explicit
rework workflow must own any intentional reuse.

Placement `b6b9a8d8-a035-43f2-8c15-b1585af075c3` put one resistor on the
previously empty R14 target. Top inspection measured -0.020 mm X, -0.100 mm Y,
and -4.474 degrees. The automatic four-degree limit rejected it. The operator
reported that it was a little angled but would reflow correctly. Operator
reviews can now use `stacked` to distinguish duplicate placement from other
bad results.

The saved operator reviews mark R12 and R13 as `stacked` and R14 as `good`.
The revised +0.10 mm summary has two accepted and two rejected conclusive
results, for a 50% pass rate. It does not qualify the process.

No more physical placements ran after the operator found the stacks. The
machine remained homed with the nozzle at safe Z31.5.

### Bounded pickup retry and live feeder correction

On 2026-09-01, guarded placement v8 added one bounded pickup XY retry. It
uses the opposite mean of two or three agreeing, passed top-placement
residuals for the same job, nozzle, and part. The current R4 and R1 evidence
produced X-0.260 mm and Y+0.250 mm. Each attempt stores its command, head pose,
sensor series, and contact result.

Placement `fb6ed874-7688-4560-9399-4cc274462303` tested the retry at pocket
0. Both the nominal and corrected attempts remained near the open N1 value
and found no contact through nozzle Z3.6. The executor retracted, disabled
vacuum, stopped before board contact, and retained both attempt reports.

Initial live holdouts at pockets 13 and 22 reported false 1.687 mm and
1.647 mm corrections. They used component foreground centroids as tape
geometry. Holdout v3 now derives pocket centers from the sprocket line, tape
centerline, and periodic phase. It also keeps a requested pocket when that
pocket has strong occupied evidence. Holdout
`10ebadce-f0be-479b-aaff-bf8e9382791e` then kept pocket 13 and passed with
four sprockets, a 0.258 mm correction, and 77% occupancy confidence.

Placement `06372c2a-3751-4249-91b3-4a3739d06b8e` repeated the nominal and
bounded corrected pickup at pocket 13. Both attempts found no contact. Its
source image showed that the resistor was standing on its edge, so it was not
a valid seal target. The existing source-pose inspection did not reject this
case.

N1 reference probe `4b5fa236-c0c9-45e6-882b-cab6eeaa42fc` then completed
with a failed seal report. Its open median was 236 counts. The best observed
value was 227 counts at nozzle Z3.0, for a 9-count signal against the validated
10-count minimum. Target-centering residual was 0.053 mm. No contact was
accepted. The failed report invalidated the N1 contact and vacuum
characterization branch, so later physical placements are gated.

The machine finished homed, fault-free, and at safe Z31.5. Before more N1
placements, inspect and reseat N045 and check its pneumatic path. Then rerun
N1 vacuum characterization. No additional board release reached the adaptive
depth schedule during this session because component acquisition did not
pass.

After a restart, standard XYZ home and fine XYZ home passed. A repeated N1
probe had an open median of 237 counts and a sealed median of 228 counts at
nozzle Z3.1 and Z3.0. The 9-count change reproduced the prior result. The
open values ranged from 236 to 238, and the sealed values ranged from 227 to
228. Their distributions did not overlap, but the fixed 10-count rule again
rejected the seal.

The probe gate now requires both an 8-count median change and the existing
3-count robust distribution gap. This bound is below the repeated 9-count N1
signal, but it still rejects overlapping noisy samples. Physical validation
must repeat the probe and characterization with this gate before placement
continues.

### Source-package pose gate

Guarded placement v9 now compares each fresh source-pocket image with the
retained source images from successful pickup-return qualifications. It uses
the best valid match across the sequence. This avoids a false rejection when
one reference differs because a flat part moved or rotated inside its pocket.
The comparison checks occupancy, relative center, and centered package
appearance before nozzle descent.

The retained flat pocket-22 image scored 0.522, 0.711, and 0.757 against the
three successful references. The retained edge-standing pocket-13 image
scored 0.042, 0.143, and 0.238. The 0.55 minimum therefore accepts the known
flat pose and rejects the known edge pose. The replay used only retained
images and did not move the machine.

Each run stores `source-pocket-pose-inspection.png` and a structured report
that names the selected successful qualification. The source-pose reanalysis
route can replay old placement evidence without changing the placement
status. Physical validation still needs a restored N1 vacuum calibration.

The robust vacuum gate then passed N1 at nozzle Z3.1 with a 9-count median
change and an 8-count robust gap. N2 characterization passed with a 12-count
median change and an 11-count robust gap. Standard home, fine XYZ home, and
both nozzle characterization branches passed.

The next R21 attempt stopped before descent because its flat resistor had the
unmarked face up. Its appearance similarity was only 0.156 against marked-face
references. Source-pose v2 now permits an alternate geometry gate only for
axis-symmetric parts. The retained flat case measured 2.006 by 1.232 mm and
passed against the registered package size. The retained edge-standing case
measured only 0.400 by 0.480 mm and remained rejected. Both checks replayed
retained images without motion.

The job evidence audit found that successful placements retained both the raw
bottom-camera held-part image and the raw top-camera post-placement image.
Some older failed release and recovery attempts reached board contact but did
not retain the top-camera after image. Guarded placement v10 now captures
`board-placement-after.png` after every physical board contact, before it
returns an uncertain retained part or reports release failure. This path still
needs a physical release-failure or ordinary placement check after N1 vacuum
calibration is restored.

### Feeder 5 pickup and failed-hold evidence

On 2026-09-02, coarse home `be0feefe-a0da-4f38-8769-c4b11ead0ba9` and fine
XYZ home `fd988552-fcd2-48eb-99b4-df5ce13820fc` passed. The fine-home Z
correction was +0.100 mm. The first N1 characterization did not reproduce its
borderline probe contact. A second probe and characterization passed with a
9.5-count median seal change and an 8.5-count robust distribution gap.

Feeder 5 scan `395cd0ae-1eb4-486c-a7a7-c7d1bbe2b5e4` retained 82 native
frames. It had complete coverage and accepted all 121 overlap registrations.
The survey measured an approximately 1.45 by 1.00 mm package. Registration
`afaf9bce-ac9b-4773-8f26-1e1152dc13ce` assigns these parts to the 0603
axis-symmetric test-resistor policy. This assignment is for process testing;
the electrical identity is not visually verified.

Pickup qualification `1fd2cd4f-80a8-4aa2-b32c-47fb55fe4666` passed at
nozzle Z4.300. Bottom vision measured a 1.787 by 1.088 mm part and a 0.027 mm
final center residual. One release attempt cleared N045. Top vision measured
a 0.007 mm return shift and 0.824 appearance similarity, so the return passed.

Job execution `060a759f-0a69-45db-9b79-9ed984584c5f` tried R24 twice with
two different feeder pockets. Both pickups crossed the calibrated acquire
threshold, but neither retained calibrated vacuum hold after transfer to the
bottom-camera pose. Both parts were returned to their source pockets. No board
contact occurred. The job remains in `needs_attention` state.

These failures exposed an evidence gap. The executor collected post-transfer
vacuum samples but returned before it persisted them or captured the bottom
camera. It now persists the samples, median, hold result, raw bottom image,
and initial held-part inspection before it recovers a failed hold.

A deployed diagnostic retry reproduced the strict hold failure at 232 counts,
0.66 counts above the calibrated acquire threshold. Bottom vision found a
1.777 by 1.148 mm held part with 96% confidence after the full source-to-camera
move. A fused gate now accepts this case only when bottom vision passes and
the held vacuum distribution has the characterized polarity, at least half
the reference-seal median change, and the full robust distribution gap. An
open-nozzle distribution or a failed camera result cannot bypass the strict
gate. Physical placement validation of the fused gate remains.

After deployment, coarse home `ecadbd15-93a4-4ab3-8480-02eea53c84f7`
and fine XYZ home `e2338f1d-e3e5-423c-a2cd-8898b9f3a587` passed. Feeder 5
pocket 1 then passed live holdout with a 0.243 mm correction and 96% occupancy
confidence. Placement retry `30b10e6b-f0d4-4c5f-921d-0dbd73db804d`
registered the R24 pad pair with a 0.144 mm correction and found the source
part with top vision. Its pickup search produced only a 3-count change through
nozzle Z3.700, below the 8-count acquire gate. It stopped before bottom-camera
or board motion. A second live holdout found the part still in pocket 1 with
93% occupancy confidence. This retry did not reach the fused transfer gate.

An offline replay of its retained source image exposed two different centers.
The old broad contrast centroid was X130.403/Y186.133. Source-pose version 3
detected the component core and measured a 1.040 by 0.680 mm outline. The new
component and nozzle target is X130.504/Y186.010. This changes the old target
by X+0.101/Y-0.123 mm. The replay artifact separates the nominal pocket
center, component outline, component center, and nozzle target. No machine
motion was used for this replay. A later pickup must validate the corrected
center.

### Live placement-model and release-gate check

On 2026-09-04, coarse home `db7f28f4-1523-4528-8025-fb319b8654a0`
and fine XYZ home `e61833ad-4061-47ea-bb2f-ef9333bf29f5` passed. The
fine-home operation reset the measured N2 datum contact from Z59.200 to
Z59.100. The machine used calibration
`3b837434-a961-4e60-945a-18335b42d21f`.

Placement `d53a5171-bc1a-4c65-8a3d-a3c2def45682` picked a feeder 5
resistor and placed it at R6. Local pad registration applied X+0.057 and
Y+0.091 mm. Bottom vision aligned the held part with a 0.031 mm center
residual. Vacuum confirmed release at the first -0.200 mm seating offset.
Deterministic bottom CV then marked a small nozzle reflection as a retained
part and caused a false release failure. The held-part model classified the
same image as `bare_nozzle` with 0.99999 probability and a 0.99999 margin.

Top CV found the R6 component at X+0.200 and Y+0.020 mm from target, with a
-4.59 degree rotation error. The placement model incorrectly classified its
correct, centered crop as empty with 0.811 probability. The conservative
CV/model disagreement gate kept CV active and marked the result for review.

Held-part release control now permits only a high-confidence `bare_nozzle`
result to override a deterministic false-positive retained-part result. The
minimum class probability is 0.98 and the minimum margin is 0.95. Vacuum must
also confirm release. Other learned held-part results remain non-controlling,
and detector disagreement remains in the retained report.

Placement `22373813-e20d-4770-a2ec-4a62c4aa610e` then tried R7. Its source
image showed a flat resistor, but the vacuum signal stayed at the open-nozzle
value through the bounded Z search. The runner stopped before board motion.
A later source image showed that the resistor was no longer in its pocket,
while a new bottom-camera bare-nozzle image showed a clear N045 tip. The part
was therefore displaced and released during failure recovery without a
detected vacuum hold.

Live holdouts `cb2f408c-dbc4-4d91-a8ad-2f41d55a942f` and
`1908e9e1-2674-4744-845f-bec0b11bb738` accepted two adjacent covered empty
pockets as occupied. The source-component model correctly rejected both
fresh placement images as empty before pickup. No additional board contact
occurred. This exposes a follow-up gap: a high-confidence source-model empty
result must invalidate the stale live-holdout pocket state. The machine ended
ready, homed, fault-free, and at safe Z.

### Source-state and adhesive-target validation

On 2026-09-04, coarse home `cd69c0bc-760c-48fa-ad72-7812aa3b42f1`
and fine XYZ home `af9266c3-7042-40c2-b065-d7332f02e848` passed. Placement
`fda9b288-3d9c-4cd5-a8f4-25d477652707` checked feeder 5 pocket 10. The
source model reported `empty` with 0.742 probability, below its 0.750 control
gate. One bounded pickup attempt found no vacuum transition. The pocket moved
to `needs_review`, so it was not reused. Placement
`1ef437e1-13b1-4f62-a645-986fe7b5a470` then classified pocket 11 as empty
above the control gate. It stopped before descent, persisted the pocket as
`empty`, and reduced the active feeder count to zero.

The first D1 and D2 attempts exposed local pad-registration failures under
the clear adhesive glare. The nominal fiducial transform was visibly centered
on both pad pairs. Target registration v2 adds a fallback that searches two
bounded bright windows at the known package axis. Retained-image replay found
D1 at X-0.038/Y-0.003 mm and D2 at X+0.030/Y+0.105 mm from nominal.

After deployment, coarse home `eb32967a-2dd1-4b3d-91a9-360d4f7633d4`
and fine XYZ home `f4c08577-67ba-45ef-814a-53812cac21fc` passed. Placement
`32b7b55c-6192-464a-a668-96102ff328bb` registered D2 with a 0.074 mm
correction and 0.936 confidence. It then classified feeder 7 pocket 6 as
empty, stopped before descent, and persisted the empty state.

Live holdouts at feeder 7 pockets 13, 16, and 20 found no carrier-tape
sprocket lattice. Their retained images show the open feeder slot. Two loose
LEDs are visible near pocket 16, but they are not in pickable carrier pockets.
Feeder 5 has no available parts, feeder 8 was previously exhausted, and no
further board-compatible placement source is available without operator
loading.

### Feeder 5 exact-pocket recovery

On 2026-09-05, standard home `f49b9b33-a15c-4dbe-872d-284f2c68f069`
and fine XYZ home `9aa090a5-5a2e-48fd-a451-08c19e1b5015` passed. Fine
home reset the measured N2 datum contact from Z59.000 to Z59.100.

Five-view holdouts checked the component-bearing end of feeder 5. Pocket 10
was selected at 57% occupancy confidence, but it remains under review because
the earlier source model and vacuum check both rejected it. Pocket 12 passed
at 94% confidence. Pocket 13 passed at 96% confidence.

The pocket 11 request selected a component two tape pitches away at pocket 9.
Its 80% confidence therefore described pocket 9, not pocket 11. The retained
pocket 14 center frame clearly shows a resistor, but its holdout failed because
the tape end did not provide the required three-hole sprocket lattice.

The recovered inventory records pockets 9, 12, 13, and 14 as occupied. Pockets
10 and 11 remain under review. The active feeder count is four. This run also
showed that an explicit pocket request must not accept a visible neighbor as
its result.

### Retained placement evidence and augmented model

On 2026-09-05, 32 retained before/after placement images received operator
review. The merged dataset has 72 images in source- and board-target-grouped
splits: 53 training images and 19 evaluation images. Both splits contain empty,
flat, and multiple-component examples. Neither split contains an on-edge
placement example.

The placement training pipeline added bounded gamma and mild focus changes to
the existing crop, brightness, contrast, flip, and sensor-noise augmentation.
It did not rotate complete crops because that would also rotate pads, text, and
other board context. The clean Spark run stopped after 181 epochs and selected
epoch 131. Training took 79.5 seconds on the NVIDIA GB10.

The new model classified 17 of 19 evaluation images correctly. Occupancy had
10 true positives, one false positive, no false negatives, 0.909 precision,
and 1.0 recall. Mean component-center error was 0.037 mm, mean undirected angle
error was 3.99 degrees, and mean oriented IoU was 0.863. It detected the one
reviewed stacked placement. The old model had the same aggregate evaluation
accuracy but treated that stack as one flat part. The new model also corrected
the R17, R19, and R22 live false negatives.

The new model produced more empty-to-flat errors on the training split. It is
therefore promoted only behind the existing confidence, class-margin, local
target-registration, package-size, and deterministic occupancy-agreement
gates. Both evaluation mismatches failed the model confidence gate. Python and
ONNX output parity passed. The Rust tract replay matched the expected model
behavior, and a simulated daemon loaded the hash-checked package successfully.

The physical daemon was restarted with placement-verification ONNX SHA-256
`10e07560dfc78c53a86d9da4b98e59fc663af58adebde83ddc45875a2247670e`.
No machine motion occurred. After restart, the API reported the machine as
disconnected, not homed, with no lease and no active calibration revision.
See `docs/baselines/placement-verification-v2.json` for the complete baseline.

### Session closeout

The web test, TypeScript check, and production build passed with Node 22.23.2
and npm 10.9.8. The model packages remain below the Git-ignored `datasets/`
tree. The source-component model lacks Git provenance in its manifest, and the
held-part model records a dirty training worktree. Reproduce these two models
from clean commits before publishing them in a model-zoo repository.

The next planned work is a measured motion-performance pass and an independent
web-interface audit. `docs/HANDOFF.md` records the entry state, commands, code
locations, and safety boundaries for both sessions.

## Procedure sources

- [Opulo V3 connect and bottom-camera calibration](https://docs.opulo.io/misc/calibration/2-connect-to-machine/)
- [Opulo V3 homing-fiducial calibration](https://docs.opulo.io/misc/calibration/4-homing-fiducial/)
- [Opulo V2/V3 G-code macro update guide](https://docs.opulo.io/guides/update-gcode-macros/)
- [Opulo actuator check](https://docs.opulo.io/byop/testing/actuator-check/)
- [Opulo LumenPnP V3.0.4 OpenPnP machine profile](https://github.com/opulo-inc/lumenpnp/blob/v3.0.4/openpnp/machine.xml)
- [Opulo LumenPnP V3.0.4 datum PCB](https://github.com/opulo-inc/lumenpnp/blob/v3.0.4/pnp/pcb/datum/datum.kicad_pcb)
- [Opulo LumenPnP V3.0.4 staging plate](https://github.com/opulo-inc/lumenpnp/blob/v3.0.4/pnp/pcb/staging-plate/staging-plate.kicad_pcb)
- [Opulo LumenPnP V3.0.4 FTP PCB](https://github.com/opulo-inc/lumenpnp/blob/v3.0.4/pnp/pcb/ftp/ftp.kicad_pcb)
- [ANSI/EIA-481-C carrier-tape dimensions](https://community.nxp.com/pwmxy87654/attachments/pwmxy87654/kinetis/37830/1/EIA-481-2003.pdf)
