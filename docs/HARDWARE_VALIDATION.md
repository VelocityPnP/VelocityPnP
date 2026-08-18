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

## Procedure sources

- [Opulo V3 connect and bottom-camera calibration](https://docs.opulo.io/misc/calibration/2-connect-to-machine/)
- [Opulo V3 homing-fiducial calibration](https://docs.opulo.io/misc/calibration/4-homing-fiducial/)
- [Opulo V2/V3 G-code macro update guide](https://docs.opulo.io/guides/update-gcode-macros/)
- [Opulo actuator check](https://docs.opulo.io/byop/testing/actuator-check/)
- [Opulo LumenPnP V3.0.4 OpenPnP machine profile](https://github.com/opulo-inc/lumenpnp/blob/v3.0.4/openpnp/machine.xml)
- [Opulo LumenPnP V3.0.4 datum PCB](https://github.com/opulo-inc/lumenpnp/blob/v3.0.4/pnp/pcb/datum/datum.kicad_pcb)
