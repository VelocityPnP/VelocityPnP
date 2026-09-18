# Model distribution

VelocityPnP uses immutable model packages from
`VelocityPnP/velocitypnp-models`. The source repository tracks an exact model
set in `models.lock.json`. Model weights stay outside Git.

## Validate and cache

The current lock is cache-only because all three packages are candidates:

```sh
python3 python/tools/model_cache.py validate models.lock.json
python3 python/tools/model_cache.py sync models.lock.json
```

The default cache is `$XDG_CACHE_HOME/velocitypnp/models`. If
`XDG_CACHE_HOME` is unset, the default is
`~/.cache/velocitypnp/models`. Use `--cache-dir` before the subcommand to
select a different cache. A private model repository requires an authenticated
GitHub CLI.

The cache uses the package SHA-256 as its directory key. A sync verifies the
archive size and checksum, package inventory, training manifest, runtime
contract, dataset digest, checkpoint, and ONNX digest. A corrupt cached archive
or package moves to a quarantine directory before a verified replacement is
installed.

Sync does not select a model. It prints the verified package directory for
each role. The current lock has `selection_policy: cache_only`, so `activate`
must fail.

## Runtime selection and rollback

A future runtime lock must set `selection_policy` to `runtime`, and every model
in it must have `status: approved`. Activation writes one atomic cache state
file after all packages pass verification:

```sh
python3 python/tools/model_cache.py activate approved-models.lock.json
python3 python/tools/model_cache.py current
python3 python/tools/model_cache.py rollback
```

Rollback selects the prior verified lock snapshot. It does not download files
or alter package contents. A daemon must be restarted with the package
directories from `current`; this tool does not change daemon arguments or
acceptance policy.

Publishing, caching, validation, approval, selection, and activation are
separate operations. Do not change a candidate to approved without independent
evaluation and a reviewed catalog update.
