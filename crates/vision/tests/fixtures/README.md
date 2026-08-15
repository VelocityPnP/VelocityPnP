# Attached LumenPnP V3.0.4 camera fixtures

These are unmodified 1280x720 grayscale PNG frames captured from the attached
machine on 2026-08-14. They keep the initial target detectors tied to actual
optics instead of synthetic circles.

Both captures used YUYV at 10 frames per second, five settling frames, manual
exposure value 200, and disabled dynamic frame rate. The ring lights were on.
The remaining controls were at their then-current device values, so these
fixtures do not by themselves validate the subsequently defined fully manual
camera profile.

- `lumenpnp-v3.0.4-top-datum-exp200.png` shows the machine calibration datum
  under the top camera. SHA-256:
  `d0f5c62ffaa7ccc03ce4b2370099934b6709532846a0b4e7582e848d9deb1051`.
- `lumenpnp-v3.0.4-bottom-n045-exp200.png` shows the cleaned N045 nozzle tip
  under the bottom camera. SHA-256:
  `71453823aeee2ed50d68e3bcd78dd5c669b9c00863edf3247633215b9bdecf1a`.

The fixtures may be replaced only by a documented capture with equal or better
target visibility. Update the hashes and detector expectations in the same
change.
