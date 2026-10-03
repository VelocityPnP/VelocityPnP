# Model distribution

VelocityPnP uses immutable model packages from
`VelocityPnP/velocitypnp-models`. The source repository tracks an exact model
set in `models.lock.json`. Model weights stay outside Git.

## Validate and cache

The current lock selects three approved packages for guarded runtime use. Lock
schema v2 gives each package its own immutable release tag, so one model can
advance without republishing the other roles:

```sh
python3 python/tools/model_cache.py validate models.lock.json
python3 python/tools/model_cache.py sync models.lock.json
```

The default cache is `$XDG_CACHE_HOME/velocitypnp/models`. If
`XDG_CACHE_HOME` is unset, the default is
`~/.cache/velocitypnp/models`. Use `--cache-dir` before the subcommand to
select a different cache. A private model repository requires an authenticated
GitHub CLI.

The cache uses the package SHA-256 as its directory key. The current source
component pin is `source-component-pose/2026-09-27.1`; the held-part and
placement pins remain at `2026-09-14.1`. A sync verifies the
archive size and checksum, package inventory, training manifest, runtime
contract, dataset digest, checkpoint, and ONNX digest. A corrupt cached archive
or package moves to a quarantine directory before a verified replacement is
installed.

Sync does not select a model. It prints the verified package directory for
each role. Use `activate` only when the daemon is stopped and the exact runtime
set is ready to change.

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

Validate, sync, and activate the exact lock:

```sh
python3 python/tools/model_cache.py validate models.lock.json
python3 python/tools/model_cache.py sync models.lock.json
python3 python/tools/model_cache.py activate models.lock.json
python3 python/tools/model_cache.py current
```

Restart the daemon with the selected package directories. Confirm all three
roles are ready. Approval and selection must not change daemon acceptance
policy.

## Runtime selection and rollback

A runtime lock must set `selection_policy` to `runtime`, and every model in it
must have `status: approved`. Activation writes one atomic cache state file
after all packages pass verification:

```sh
python3 python/tools/model_cache.py activate models.lock.json
python3 python/tools/model_cache.py current
python3 python/tools/model_cache.py rollback
```

Rollback selects the prior verified lock snapshot. It does not download files
or alter package contents. A daemon must be restarted with the package
directories from `current`; this tool does not change daemon arguments or
acceptance policy.

Publishing, caching, validation, approval, selection, and activation are
separate operations. A later model or broader control role needs a new reviewed
catalog update that records its allowed runtime scope.
