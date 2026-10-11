# Versioning and release policy

## Version numbers

OpenRoad Dynamics follows [Semantic Versioning](https://semver.org/) with the usual pre-1.0 rules:

- **0.MINOR.PATCH** while the API and the handling are still settling. A minor bump (0.3 → 0.4) may
  change the public C++ API, `VehicleDefinition` fields, or how cars drive. A patch bump (0.3.0 → 0.3.1)
  fixes defects without intentionally changing the API or the golden baseline.
- **1.0** will be tagged once the Unreal Engine plugin has been compiled and validated against the
  headless core on at least one supported UE version (roadmap M1), and once the public headers in
  `src/core/include/ord/` are frozen for that major version.

The version lives in **one place** for the core: `project(OpenRoadDynamics VERSION x.y.z)` in the
top-level `CMakeLists.txt`. The plugin's `VersionName` in `OpenRoadDynamics.uplugin` must match the core's
`MAJOR.MINOR` when the plugin is released.

## "Handling changes" are breaking changes

For a physics library, changing how a car drives matters as much as changing an API. Any change that
moves a value in `benchmarks/baseline.csv` beyond its tolerance must:

1. Regenerate the file with `ordbench --write-baseline` **in the same pull request** as the physics
   change. `tests/test_baseline.cpp` fails otherwise.
2. Explain in the PR description which metrics moved, by how much, and why the new behaviour is more
   correct. "Tests were failing" is not a reason.
3. Be listed under **Handling changes** in the release notes.

A patch release must not contain baseline changes, except to fix a defect that the release notes name.

## Cutting a release

1. `main` is green on every CI job (Linux GCC/Clang, sanitizers, macOS, Windows).
2. Update the version in `CMakeLists.txt` (and the `.uplugin` if the plugin is part of the release).
3. Regenerate `docs/benchmarks/baseline-results.md` with `ordbench` and update the status block in
   `README.md`: test count, platforms, and plugin compile status. Never describe the Unreal integration
   as working unless it has been compiled and driven in Unreal for that release.
4. Tag `vX.Y.Z` on the merge commit and publish a GitHub release with these sections: **Added**,
   **Fixed**, **Handling changes** (baseline diffs), **Known limitations**, and **Unreal status**
   (compiled on which UE versions, or "not compiled").
5. Never move or delete a published tag; fix mistakes with a new patch release.

## Support

Only the latest minor version on `main` receives fixes while the project is pre-1.0 (see
[SECURITY.md](../SECURITY.md)).
