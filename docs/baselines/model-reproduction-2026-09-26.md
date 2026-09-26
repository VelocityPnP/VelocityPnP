# Clean model reproduction, 2026-09-26

No machine movement was part of this check. No model was activated, approved,
or released.

## Source tree

Both runs used a clean detached checkout on `svl-spark`:

- commit: `61b44e197b7d41b9b5cfcfcfa5717ee7866f27af`
- path: `/home/tsmith/velocitypnp-training/clean-20260926`
- Git dirty state recorded by each manifest: `false`

The pinned NVIDIA training container reported PyTorch 2.14.0a0, CUDA 13.4,
and the NVIDIA GB10 device.

## Held-part pose

The run used the reviewed `held-parts-v3` dataset. It stopped after 143 epochs
and selected epoch 93. The new checkpoint, ONNX file, and training history are
byte-identical to the earlier v3 output.

- dataset SHA-256:
  `dadcb24a3145c3f75f4f2dfc61c0ec78008fa33f7f40dcfca3948cf7ef09ef79`
- checkpoint SHA-256:
  `ddf3cdb107986c34577a0cd62b6764ba3ea3cf4032780f119376987dae4303a0`
- ONNX SHA-256:
  `6e3a50e742b5e6968dbf080a02fbb1d7cb24a5f16b23f4ab8879ffb27e3d08f4`

The PyTorch evaluation classified all 33 evaluation pairs correctly. The
split contains nine bare-nozzle images and 24 flat-part images. It contains no
on-edge parts. Mean center error for the 24 parts was 0.367 pixels at the
192-pixel model input.

The Rust `tract-onnx` runtime also classified all 33 evaluation pairs
correctly. Its mean center error was 0.979 pixels at the 512-pixel review image
size.

## Placement verification

The run used the reviewed merged `placement-verification-v3` dataset. It ran
all 240 epochs and selected epoch 209. The new checkpoint, ONNX file, and
training history are byte-identical to the earlier v3 output.

- dataset SHA-256:
  `cdeac286ff2614166a3a8927346e3932deb1bd12c8c42b1b5dc8a63b5b08a808`
- checkpoint SHA-256:
  `cc04b76a5471078b072aa46ec6939f9826edf86484e9e049dae1da8520d0aa6c`
- ONNX SHA-256:
  `eb5302231998b3a0dba92c2746c76bfc3ebc82c45c347c5b0d49568eb63ba232`

PyTorch-to-ONNX parity passed for all 21 evaluation images. There were no
class differences. Maximum absolute output differences were 0.0000048 for
class log probabilities and 0.00000030 for pose.

The Rust runtime reproduced the earlier evaluation result. It classified 17
of 21 images correctly. All 11 nonempty images were correct. Four of ten empty
targets were called flat parts, and one false positive passed the model-only
confidence gate. Mean center error for ten flat parts was 0.029 mm. The
agreement-gated daemon policy remains necessary.

## Release state

These outputs correct the dirty-source provenance in the published candidate
manifests. They do not improve the weights. Keep them as local candidates
until the combined source-component review, clean retrain, independent feeder
holdout, and runtime checks are complete. Then publish all approved packages
under a new immutable model release. Do not replace the existing release.
