# Machine safety

Software is not a substitute for physical guarding, power isolation, or an
emergency-stop circuit that removes actuator power.

Before homing or motion:

1. Level both nozzles and raise them clear of the staging plate.
2. Remove hands, tools, cables, loose parts, and obstructions from the envelope.
3. Keep physical power isolation within immediate reach.
4. Verify the selected mechanical profile and travel bounds.
5. Start with conservative feed and acceleration settings.
6. Before runout calibration, make sure each nozzle tube is fully seated,
   follows its normal routing, and is not kinked or caught on another cable.

Precision scanning and board registration require a session fine XYZ home for
the selected calibration. Fine home uses the fixed datum PCB: the top camera
refines X/Y, then N2 reproduces the validated vacuum contact on the separate
sealing target. The Z search stays within 0.3 mm of the stored contact and the
coordinate correction is limited to 0.5 mm. Vacuum switches off before
retraction. Coarse re-home, disconnect, or fault invalidates the fine home.

Automatic board registration also requires the PCB to be fixed. The runner
rechecks visual-refined X/Y before it visits any PCB feature. All board
moves stay at safe Z, omit nozzle rotation, use the profile feed limit, and
return to the visual-home seed. A failed fiducial hypothesis does not create a
board instance.

Automatic board-surface measurement is non-contact. The operator must confirm
that the PCB is fixed and that at least three fiducials are visible and clear
of adhesive or other cover. The top camera views each fiducial only at safe Z.
The runner does not lower a nozzle onto the PCB. It fails closed if the camera
models, datum-contact reference, fiducial detections, spread, residual, or
uncertainty are not valid.

Retained-image strip-feeder analysis causes no machine motion and does not
require homing. It can store a setup draft, but the draft has no pickup Z,
selected occupied pocket, or active feeder ID. The job planner cannot consume
it as a production feeder. The live holdout causes only safe-Z camera motion.
The pickup qualification requires coarse and fine XYZ home, a current nozzle
tip transform, a passed live holdout, and calibrated reference contact and
vacuum thresholds. Its Z search stops above the datum-board contact. Vacuum is
on before descent. Any detected contact returns to the exact source pose. A
fallback point is used only if that source pose is invalid. A failed cleanup
disables vacuum and faults the operation; it does not use emergency stop.
The source pocket is reserved before motion. It is not marked consumed without
vacuum and bottom-camera evidence. A normal return also needs a top-camera
occupied-pocket check and a direct before-and-after package appearance match.
Restart recovery marks an unresolved reservation as needing review. Retained
return reanalysis causes no machine motion. A rejected reanalysis marks the
pocket uncertain and removes any active feeder that depended on that pickup
qualification.

Vacuum-off state alone does not prove that a component left the nozzle. The
pickup runner compares a post-release bottom-camera frame with the bare-nozzle
reference. It retries the same release once if the sensor is not off or a
component remains visible. A clear nozzle with no part in the source pocket is
a lost-part failure. The pocket becomes uncertain and needs review.

Held-part alignment moves only after vacuum hold and initial bottom-camera
evidence pass. The profile limits it to two X/Y and angle corrections. The
runner restores the nozzle angle before it returns to the exact pickup pose.
It fails closed if the requested angle exceeds the configured bound.

The guarded single-placement endpoint requires one explicit plan index,
calibrated motion, a measured board surface, and complete part geometry. A
provisional feeder needs one current pickup-return cycle with the planned
nozzle. That evidence allows one guarded placement with top inspection. A
passed placement promotes the feeder. A feeder can also become ready after
three current pickup-return cycles at distinct pockets. Each return cycle needs
nozzle-clear and pocket-return evidence. Physical execution reserves one
known occupied pocket, uses live vacuum and bottom vision, verifies release in
at most two attempts, and then uses top vision. A failure before confirmed
release tries to return the part to the source pocket. Simulation records the
intended stages but does not claim live evidence or consume feeder inventory.

Placement Z keeps the measured board datum, a nonnegative board surface
offset, nominal part height, and a bounded signed seating offset as separate
terms. A positive seating offset stops higher. This prevents tape thickness or
nozzle compression tuning from changing calibration evidence.

The V3.0.4 profile uses the firmware's standard combined `G28` command after
the operator levels both nozzles. Axis-scoped `G28 Z` was retired after the
attached firmware was observed moving X/Y toward zero before homing Z, which
defeated the intended Z-before-X/Y safety property.

VelocityPnP invalidates homing after disconnect, controller fault, emergency
stop, or uncertain execution. It never resumes a physical job solely from the
last recorded coordinate. Emergency stop uses an out-of-band flag to interrupt
a blocking serial operation, sends Marlin `M112`, revokes control, and requires
controller recovery and a new homing cycle. Every later explicit connect first
sends `M999` to normalize Marlin into its running state and clear stale queued
commands, then identifies the firmware. Recovery enters preflight and never
restores the prior pose, lease, or homing. The polling latency is bounded to
roughly 20 ms in the driver, but USB, firmware, and electrical failures can
still defeat a software stop.

Physical jog requests preserve the controller-reported primary-nozzle angle
when rotation is omitted. The V3.0.4 profile disables rotation motion until it
is separately hardware-validated, and the machine actor rejects an explicit
angle change before any command reaches the driver. This interlock applies to
all API, CLI, UI, Python, and internal job callers.

The automated calibration runner has a narrower rotation capability. It reads
the current A or B angle, permits only the profile offsets up to ±165 degrees,
uses a conservative 600 degrees/minute feed, verifies the reported result, and
returns the nozzle to its starting angle. The mapping from N1/N2 to A/B comes
from the active machine profile. A driver error faults the machine and revokes
control. This capability does not enable normal jog or job rotation.

The V3.0.4 Z mechanism also has profile-scoped nozzle mappings. Controller Z
equals nozzle working Z for N1 and `63 - nozzle working Z` for N2. Both nozzles
are level and retracted at controller Z 31.5. Vacuum probing keeps them level
for X/Y motion. It identifies and centers the current tip over the bottom
camera. The nozzle offset uses the top-camera lens pose measured by that same
bottom camera. It then centers the separate sealing pad with the top camera.
Initial head calibration also measures each nozzle at a bounded list of Z
heights over the recessed bottom camera. X/Y stays fixed while the selected
nozzle is below the level pose. The machine retracts to controller Z 31.5
before it moves to another nozzle or camera target.
It lowers at fixed X/Y to a clear search start. The pump starts before the
bounded slow descent. A stable vacuum transition defines contact; nominal
working Z does not. The pump stops before retract, and the nozzle retracts to
level before the return X/Y move. Every normal or failed path commands the
vacuum output off.
Reference-seal characterization starts from a current successful contact
artifact. It searches only from 0.3 mm above the stored contact Z through that
contact Z. After sealed sampling, it switches the pump off while the nozzle is
stationary, measures the release signal, and then retracts. It never searches
below the previously validated contact Z.
The target is the 5 mm front-mask opening near X=-22.5 mm and Y=-7.5 mm from
the visual fiducial. The profile protects a 5.5 mm radius around that
fiducial. The operator must confirm that the datum board is fixed and that the
sealing target and nozzle tip are clean.

The working-plane height observation is data-only. It records a bounded XY
residual and uncertainty after the operator or a future inspection runner has
observed the result. It cannot command motion.

Visual homing is observation-only until both raw-camera axis signs have
physical evidence. Apply mode bounds the initial target and XY correction,
verifies the target again after moving, and changes only the controller's X/Y
working coordinates. A failed detection or verification never applies the
coordinate reset. Any disconnect, fault, or emergency stop still invalidates
the resulting homed state.

Local X/Y direction-response calibration first performs visual homing. It
keeps both nozzles level at controller Z 31.5 and uses only profile-bounded
lateral moves around the fixed datum. The maximum path is checked against
machine bounds before the operation starts. Measured correction vectors must
also remain inside the visual-homing correction safety bound before a holdout
move is allowed. A motion fault does not trigger an emergency stop or an
uncertain return move; the normal fault state invalidates homing.

Job motion has separate logical and controller poses. The motion planner checks
the requested logical pose, applies only current holdout-enabled compensation,
and checks the corrected controller pose again. It rejects a calibration change
during an active sequence. Raw mode bypasses and clears direction state.
Calibration runners and manual jog remain on the raw machine-actor path. Only
the simulator job endpoint uses the calibrated path; physical job execution is
still disabled.

Physical disconnect and graceful daemon shutdown command both vacuum channels
and the shared lights off before releasing serial. SIGINT and SIGTERM use this
path. A process crash, `SIGKILL`, USB failure, controller failure, or loss of
host power can bypass it, so physical power isolation remains mandatory.

The v1 trusted-LAN model has no user authentication. Keep the daemon bound to
loopback unless the network is isolated and trusted. The control lease prevents
competing writers but is not an authentication mechanism.

Starting an automated calibration authorizes that operation to refresh the
same control lease to the actor's one-hour maximum at measurement checkpoints.
It does not acquire a replacement lease, and it stops refreshing when the
operation ends. A revoked, replaced, or invalid lease stops the run.

## Hardware-command provenance

The V3.0.4 home position, travel bounds, axis letters, light commands, and
vacuum commands are derived from Opulo's tagged V3.0.4 `machine.xml`, not from
values measured on this particular machine. The Photon framing and CRC test
vectors are derived from Opulo's published Photon protocol. See these upstream
primary sources before changing a hardware profile:

- https://github.com/opulo-inc/lumenpnp/blob/v3.0.4/openpnp/machine.xml
- https://docs.opulo.io/byop/testing/actuator-check/
- https://docs.opulo.io/misc/calibration/4-homing-fiducial/
- https://docs.opulo.io/misc/photon/
- https://community.nxp.com/pwmxy87654/attachments/pwmxy87654/kinetis/37830/1/EIA-481-2003.pdf

Even source-backed values require non-moving actuator tests and conservative
motion validation on each physical machine before production use.
