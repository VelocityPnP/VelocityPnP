# Machine safety

Software is not a substitute for physical guarding, power isolation, or an
emergency-stop circuit that removes actuator power.

Before homing or motion:

1. Level both nozzles and raise them clear of the staging plate.
2. Remove hands, tools, cables, loose parts, and obstructions from the envelope.
3. Keep physical power isolation within immediate reach.
4. Verify the selected mechanical profile and travel bounds.
5. Start with conservative feed and acceleration settings.

VelocityPnP invalidates homing after disconnect, controller fault, emergency
stop, or uncertain execution. It never resumes a physical job solely from the
last recorded coordinate. Emergency stop uses an out-of-band flag to interrupt
a blocking serial operation, sends Marlin `M112`, revokes control, and requires
controller recovery and a new homing cycle. The polling latency is bounded to
roughly 20 ms in the driver, but USB, firmware, and electrical failures can
still defeat a software stop.

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
- https://docs.opulo.io/misc/photon/

Even source-backed values require non-moving actuator tests and conservative
motion validation on each physical machine before production use.
