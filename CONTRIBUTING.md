# Contributing

Safety-critical changes require tests that demonstrate the rejected path as
well as the successful path. Never weaken state, homing, bounds, or lease checks
to make a UI flow easier.

Run all checks in [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) before submitting a
change. Hardware reports should include the machine mechanics revision,
controller identity, firmware response, calibration revision, command journal,
and relevant images. Remove unrelated personal or network information from
support bundles.

OpenPnP and Opulo sources are GPL-3.0. This Apache-2.0 repository may implement
documented protocols and import user-owned configuration, but contributors must
not copy GPL implementation code or assets into it.

