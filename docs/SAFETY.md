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

Visual homing is observation-only until both raw-camera axis signs have
physical evidence. Apply mode bounds the initial target and XY correction,
verifies the target again after moving, and changes only the controller's X/Y
working coordinates. A failed detection or verification never applies the
coordinate reset. Any disconnect, fault, or emergency stop still invalidates
the resulting homed state.

Physical disconnect and graceful daemon shutdown command both vacuum channels
and the shared lights off before releasing serial. SIGINT and SIGTERM use this
path. A process crash, `SIGKILL`, USB failure, controller failure, or loss of
host power can bypass it, so physical power isolation remains mandatory.

The v1 trusted-LAN model has no user authentication. Keep the daemon bound to
loopback unless the network is isolated and trusted. The control lease prevents
competing writers but is not an authentication mechanism.

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

Even source-backed values require non-moving actuator tests and conservative
motion validation on each physical machine before production use.
