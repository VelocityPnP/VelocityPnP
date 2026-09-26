# Offline model validation, 2026-09-20

No machine movement or placement run was part of this check. The model
packages were not activated or released.

## Feeder v4 spot check

The Rust runtime called all 46 recent feeder images flat and placed its
center 1.293 source-image pixels from the earlier reviewed center on
average. These images were part of v4 training, so this is a runtime and
visual check, not an independent accuracy result. The local visual review
page is `datasets/source-components-v4/feeder-spot-check.html`.
The operator spot-checked its cyan model boxes and reported that they look
good. The number of images checked was not recorded. No box corrections
were requested.

## Held part v3

The Rust `tract-onnx` runtime loaded `held-part-pose-v3` and classified all
33 reviewed evaluation pairs correctly: nine bare nozzles and 24 flat parts.
Its mean center error on the 24 flat parts was 0.979 px at the 512 px image
size. The PyTorch manifest reports 0.367 px at its 192 px input size; these
values agree after scaling. The split has no on-edge held parts. The model
can stay in shadow mode; this check does not validate on-edge control.

The runtime comparison and image sheet are at
`/tmp/velocitypnp-held-v3-check.json` and
`/tmp/velocitypnp-held-v3-check.png`.

## Placement verification v3

The Rust runtime classified 17 of 21 reviewed evaluation images correctly.
It called four of ten empty targets `component_flat`. One of those four
passed the model's class-confidence gate. All 11 nonempty images (ten flat
parts and one multiple-part result) were classified correctly.
The 10 flat parts had 0.029 mm mean center error in the Rust runtime.
There are no on-edge evaluation images.

The PyTorch-to-ONNX reference check passed at an absolute tolerance of
0.0001, with no class differences. The largest output differences were
0.0000048 for class log probabilities and 0.00000030 for pose. The daemon's
placement-control path also requires deterministic occupancy agreement;
the normalized evaluation crops alone cannot test that full gate. This model
must not be used as sole evidence that a placed part exists. Keep it in
shadow or agreement-gated use until empty-target errors are resolved with
reviewed data.

The most confident false positive is the `R14` pre-placement image
`placement_execution-b6b9a8d8-a035-43f2-8c15-b1585af075c3-before`.
The four false-positive overlays show model boxes on the bare pad pair,
not on a resistor body. This agrees with their reviewed empty labels.

## Old-run placement accuracy

The 25 stored result images from 9 September show a mean signed offset of
X+0.068 mm and Y-0.198 mm. The Y result is negative in 23 of 25 cases.
Radial RMS error is 0.247 mm, or 0.132 mm after removal of the common mean
offset. The first 12 results average Y-0.226 mm; the last 13 average
Y-0.172 mm. This suggests a common shift, not only random scatter.

These offsets come from top-camera inspection: 21 deterministic results and
four learned results. Both sets have negative mean Y. In 12 paired results,
the median distance between the deterministic and learned center estimates
is 0.182 mm. The inspection method itself is therefore a material source
of uncertainty. The old images cannot separate camera measurement bias from
machine calibration error or motion during release. Do not apply a machine
offset from these numbers without an independent measurement.

Recompute with `python/tools/analyze_placement_accuracy.py` and the retained
`datasets/resistor-population-2026-09-09/placement-images.csv` and
`velocitypnp.db`.

## Reproduction

```bash
cargo run -p velocitypnpd --example evaluate_held_part_shadow -- \
  datasets/held-parts-v3 \
  datasets/held-parts-v3/training/held-part-pose-v3 \
  /tmp/velocitypnp-held-v3-check.png \
  /tmp/velocitypnp-held-v3-check.json

cargo run -p velocitypnp-model \
  --example evaluate_placement_verification -- \
  datasets/placement-verification-v3/merged/training/placement-verification-v3 \
  datasets/placement-verification-v3/merged eval
```
