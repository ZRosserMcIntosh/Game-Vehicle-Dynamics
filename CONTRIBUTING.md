# Contributing to OpenRoad Dynamics

Thanks for your interest. OpenRoad Dynamics is a long-term engineering project to build a
world-class, open-source vehicle physics and driving system. Correctness, measurement, and
maintainability matter more than feature count.

## Ground rules

1. **Evidence before code.** Every new physical model must cite its source (paper, book,
   standard, or public engineering article) in a header comment or in `docs/research/`.
2. **Every model ships with a test.** A new force model, integrator, or assist needs at
   least one numerical test in `tests/` (analytic expectation, conservation check, or
   benchmark-scenario tolerance).
3. **The core is engine-independent.** Code under `src/core/` must not include Unreal,
   Chaos, or any other engine header. Engine integration lives in adapter layers.
4. **No hidden assists.** Driver aids live in the `assists` module and act only through
   inputs or physically modelled actuators (brake pressure, engine torque cut, steering
   input). Never scale tire grip to "help" the player. See
   [`docs/decisions/one-simulation-many-experiences.md`](docs/decisions/one-simulation-many-experiences.md).
5. **Determinism matters.** Fixed timestep, no wall-clock reads, no unordered iteration
   that affects results, no allocation in the step loop.

## Licensing and provenance

- All contributions are licensed under the MIT License (see `LICENSE`).
- You may incorporate third-party code **only** if its license is MIT-compatible
  (MIT, BSD-2/3, zlib, Apache-2.0 with NOTICE handling, Boost). Record it in
  `THIRD_PARTY_NOTICES.md` and preserve its copyright header verbatim.
- **Never** contribute: Unreal Engine source code, Marketplace/Fab assets, GPL/LGPL code,
  leaked or decompiled game code, extracted game data, copyrighted Forza material, or
  datasets without a clear redistribution license.
- Telemetry from commercial games may be used only for private behavioural comparison,
  recorded by yourself through officially provided features. Do not commit raw captures;
  commit only derived, aggregate metrics with a description of how they were obtained.

## Development workflow

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure
```

- C++20, warnings as errors in CI (`-Wall -Wextra -Wpedantic`).
- Format with the repository `.clang-format`.
- One logical change per pull request; include benchmark output when touching physics.
- Architecture-level changes need an ADR in `docs/decisions/`.

## Reporting physics bugs

Please include: vehicle definition, preset/assist configuration, timestep, the scenario
(ideally a `tests/` benchmark), expected behaviour with a source, and observed telemetry.
