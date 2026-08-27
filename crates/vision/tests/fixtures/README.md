# Attached LumenPnP V3.0.4 camera fixtures

These are unmodified 1280x720 grayscale PNG frames captured from the attached
machine on 2026-08-14, 2026-08-15, and 2026-08-17. They keep the initial target
detectors tied to actual optics instead of synthetic circles.

The same fixtures qualify the OpenCV boundary. Tests compare OpenCV frame
statistics with the earlier validated calculation and require OpenCV-backed
component extraction to retain the expected datum, nozzle, and camera-outline
detections.

The initial datum and nozzle captures used YUYV at 10 frames per second, five
settling frames, manual exposure value 200, and disabled dynamic frame rate.
The ring lights were on. The remaining controls were at their then-current
device values, so those fixtures do not by themselves validate the fully
manual camera profile. The reciprocal-camera captures used the persisted
manual profile and 60 settling frames during attached head-geometry runs.

- `lumenpnp-v3.0.4-top-datum-exp200.png` shows the machine calibration datum
  under the top camera. SHA-256:
  `d0f5c62ffaa7ccc03ce4b2370099934b6709532846a0b4e7582e848d9deb1051`.
- `lumenpnp-v3.0.4-bottom-n045-exp200.png` shows the cleaned N045 nozzle tip
  under the bottom camera. SHA-256:
  `71453823aeee2ed50d68e3bcd78dd5c669b9c00863edf3247633215b9bdecf1a`.
- `lumenpnp-v3.0.4-bottom-n24-exp200-{1,2,3}.png` show N24 under the bottom
  camera after an intermittent identity failure. The stable 170-to-171 px
  outer edge appears at grayscale threshold 60. SHA-256, in order:
  `26a261d365ffda91846b5d5a1a581509f2385989b0fe1cd507c8526de222da42`,
  `ba6961792a87a5e1d5ef0973499eaf6ddf49c511fdaac11ca50e72447c6c909c`,
  and `978051a569bc813a2029d650df21ee4473745bfaa12a443d797bf97558fd37ec`.
- `lumenpnp-v3.0.4-top-over-bottom-camera.png` shows the bottom-camera lens
  through the top camera at the reciprocal-centering seed. The lens crosses
  the frame boundary. SHA-256:
  `4f345bb7719a5346578eee8c4f1e64d5ee2088e28c49b7399632d5f2618a2084`.
- `lumenpnp-v3.0.4-top-over-bottom-camera-near-centered.png` shows the same
  lens after independent reciprocal correction. It also contains smaller
  concentric rings. SHA-256:
  `6c673f9807e849efe6effbc49540e278ecc7e3f0b51cabb5e663b08d742e9b0c`.

The fixtures may be replaced only by a documented capture with equal or better
target visibility. Update the hashes and detector expectations in the same
change.

## Physical strip-feeder ground truth

`lumenpnp-v3.0.4-strip-feeder-bank-ground-truth.json` records operator labels
for the eight physical strip feeders captured on 2026-08-26. Feeder numbers run
left to right. Pocket occupancy runs from the image top to the image bottom.
The source is candidate 0 from workspace scan
`cd0fe50f-61bf-4023-897d-8970dfa2c2d9`. Its retained evidence artifact is
`object-evidence-000.png`; SHA-256:
`b08a2ddbe513b79de702cbe057cf7b879a35bc23633a5cbb5f86288dd1d129bf`.

The image remains in the persistent workspace database, which is the
authoritative production artifact store. Export the image and discovery report
from a daemon that uses that database, then run the evaluator:

```bash
curl -o /tmp/feeder-evidence.png \
  http://127.0.0.1:4080/api/v1/workspace-scans/cd0fe50f-61bf-4023-897d-8970dfa2c2d9/artifacts/object-evidence-000.png
curl -o /tmp/feeder-discovery.json \
  http://127.0.0.1:4080/api/v1/workspace-scans/cd0fe50f-61bf-4023-897d-8970dfa2c2d9/artifacts/board-discovery.json
cargo run -p velocitypnp-vision \
  --example evaluate_strip_feeder_ground_truth -- \
  /tmp/feeder-evidence.png /tmp/feeder-discovery.json \
  crates/vision/tests/fixtures/lumenpnp-v3.0.4-strip-feeder-bank-ground-truth.json
```

The fixture contains 170 component pockets: 118 occupied and 52 empty.
