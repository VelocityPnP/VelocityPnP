# Model distribution

VelocityPnP uses immutable model packages from
[VelocityPnP/velocitypnp-models](https://github.com/VelocityPnP/velocitypnp-models).
The source repository tracks an exact model set in `models.lock.json`. Model
weights stay outside Git.

## Validate and cache

The current lock selects three approved packages for guarded runtime use. Lock
schema v2 gives each package its own immutable release tag, so one model can
advance without republishing the other roles:

Install the Python package to get the model manager:

```sh
python3 -m pip install -e python
velocitypnp-models validate models.lock.json
velocitypnp-models activate models.lock.json
velocitypnp-models current
```

From a source checkout, `python3 python/tools/model_cache.py` provides the same
commands.

The default cache is `$XDG_CACHE_HOME/velocitypnp/models`. If
`XDG_CACHE_HOME` is unset, the default is
`~/.cache/velocitypnp/models`. Use `--cache-dir` before the subcommand to
select a different cache. Public release assets download over HTTPS. A private
fork requires an installed and authenticated GitHub CLI.

The cache uses the package SHA-256 as its directory key. The current source
component pin is `source-component-pose/2026-09-27.1`; the held-part and
placement pins remain at `2026-09-14.1`. A sync verifies the
archive size and checksum, package inventory, training manifest, runtime
contract, dataset digest, checkpoint, and ONNX digest. A corrupt cached archive
or package moves to a quarantine directory before a verified replacement is
installed.

`sync` does not select a model. `activate` first performs the same verified
download, then atomically selects the exact lock. Use it only when the daemon
is stopped and the exact runtime set is ready to change.

## Approved runtime scope

The current packages are the best available models. Approval is for their
existing guarded runtime roles only. It does not give a model sole authority
for occupancy, placement, or unrestricted feeder decisions.

| Role | Validation evidence | Required runtime guard |
| --- | --- | --- |
| Source component | The independent 21-image flat/empty holdout had no class errors. The evaluation occupancy precision was 1.0 and recall was 0.951. Independent on-edge recall and localization are not measured. | Use the learned center only inside the existing confidence, offset, and disagreement bounds. Mark a pocket empty only when learned and deterministic results agree. |
| Held part | The clean Rust evaluation classified all 33 bare/flat pairs correctly. The set has no on-edge parts. | Keep non-clear condition and pose results in shadow or behind the existing package, shape, and vacuum checks. |
| Placement verification | The clean Rust evaluation classified all 11 nonempty images correctly, but called four of ten empty targets flat. | Require deterministic agreement. Never use this model as sole evidence that a placed part exists. |

These limits remain part of the approval. New data can support broader model
authority later, but it does not block guarded use of the current packages.

Clean reproduction is complete for the three current packages. The held-part
and placement packages reproduced their checkpoint, ONNX file, and history
byte for byte. Source-component v6 records a clean source commit and identical
repeat artifacts. See `docs/baselines/model-reproduction-2026-09-26.md` and
`docs/baselines/source-component-pose-v6.json`.

Reproduction was a provenance check. Its goal was to prove that a clean,
identified source tree can rebuild the released files. It was not a request to
train a better held-part model.

The operator approved this guarded set on 2026-10-03. Model-catalog commit
`156e2c98e5b220a613ab13c150a56661088bce53` records the approval. The current
lock points to that exact commit. Approval did not change a package ID,
artifact, or checksum.

Validate and activate the exact lock:

```sh
velocitypnp-models validate models.lock.json
velocitypnp-models activate models.lock.json
velocitypnp-models current
```

Restart the daemon. By default, it reads the active selection from the same
user's model cache. It rechecks the lock digest, approval state, package file
inventory, package and manifest metadata, and every retained file checksum
before it loads any model. A selected corrupt or incomplete package stops
startup instead of silently disabling learned inference.

`/api/v1/health` reports the selection mode, lock checksum, catalog commit,
package IDs, archive checksums, manifest checksums, checkpoint checksums, and
ONNX checksums. The per-model status also reports the loaded ONNX checksum.
Approval and selection do not change daemon acceptance policy.

## Runtime selection and rollback

A runtime lock must set `selection_policy` to `runtime`, and every model in it
must have `status: approved`. Activation writes one atomic cache state file
after all packages pass verification:

```sh
velocitypnp-models activate models.lock.json
velocitypnp-models current
velocitypnp-models rollback
```

Rollback selects the prior verified lock snapshot. It does not download files
or alter package contents. Restart the daemon after selection or rollback.

Set `VELOCITYPNP_MODEL_CACHE_DIR` or pass `--model-cache-dir` when the model
manager and daemon do not use the same home directory. An installed system
service uses `/var/lib/velocitypnp/models`. Explicit per-model directory
options remain available for diagnostics and bypass the managed selection.
Pass `--disable-managed-models` to start without learned models.

Publishing, caching, validation, approval, selection, and activation are
separate operations. A later model or broader control role needs a new reviewed
catalog update that records its allowed runtime scope.
