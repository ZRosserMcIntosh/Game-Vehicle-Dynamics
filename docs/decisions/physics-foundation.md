# ADR-0001: Physics foundation

- **Status:** Accepted (2026-10-10)
- **Deciders:** project maintainers
- **Inputs:** [kinetiforge-analysis.md](../research/kinetiforge-analysis.md), [alternatives.md](../research/alternatives.md), [forza-physics.md](../research/forza-physics.md), [forza-handling-philosophy.md](../research/forza-handling-philosophy.md)

## Context

OpenRoad Dynamics must deliver two things with one simulation. The first is Forza Horizon-style accessibility. The second, with aids off, is Forza Motorsport-style consequences. It must run inside a very large UE5 open world with rain, mixed surfaces, multiplayer and hundreds of traffic cars. It must also be reusable outside our game.

These requirements shape the foundation:

1. **A physically meaningful tire model.** Every handling trait must come from parameters with physical meaning (cornering stiffness, peak μ, load sensitivity, relaxation length). Curves drawn to taste don't qualify.
2. **A drivetrain coupled with the tires numerically.** It must stay stable at 120 Hz and from standstill to 400 km/h, without energy creep.
3. **Assists separable from physics.** No aid may touch the force models (see [one-simulation-many-experiences.md](one-simulation-many-experiences.md)).
4. **Deterministic and testable without Unreal.** Benchmarks run in CI, on Linux, in seconds.
5. **Safe on the physics thread, and scalable.** No `UObject` access in the step, no per-step allocation, and a level-of-detail (LOD) path for traffic.
6. **MIT-clean provenance.**

## Options considered

| | A. Depend on KinetiForge and extend it | B. Copy selected KinetiForge parts (MIT, attributed) | C. New core from published maths | D. Hybrid: new core, interchangeable modules | Chaos Vehicles (UE built-in) |
|---|---|---|---|---|---|
| Tire model | Curve LUT plus constraint cap, not Pacejka | Same, unless rewritten | Magic Formula family, swappable | Same as C, plus optional alternative tire models | Simple friction / cornering stiffness |
| Drivetrain integration | Sequential, one substep lag ("energy accumulation" fixes in history) | Would need rewriting | Coupled implicit solve | Same as C | Adequate for games; source can't be redistributed |
| Testable outside UE | No (`UActorComponent` everywhere) | Partly, after rewriting | Yes | Yes | No |
| Thread safety | Mutates `UObject`s from the physics thread with no sync (author notes it is "not safe") | Bridge must be replaced | Designed in | Designed in | Engine-managed |
| Scaling to traffic | ~27 components per car, no LOD | — | LOD tiers designed in | LOD tiers designed in | Moderate; no LOD tiers |
| Suspension kinematics | **Strong**: MacPherson, double wishbone, solid axle | **Reusable** with attribution | Must be built | Adopt KinetiForge's geometric solvers as one implementation, re-derived or attributed | Spring/damper only |
| License | MIT | MIT (preserve notices) | Ours (MIT) | MIT | Unreal EULA: no redistribution of engine code |
| Time to first drive | Fast | Medium | Slowest | Slow | Fastest |
| Long-term ceiling | Limited by architecture | Medium | High | **Highest** | Limited by model and EULA |

Other candidates (details in [alternatives.md](../research/alternatives.md)):

- **Jolt Physics (MIT)** and **PhysX 5 vehicle (Apache-2.0 as of 5.11; BSD-3 earlier)** are well-engineered, license-compatible vehicle modules. They are tied to their own rigid-body engines, though, and their tire models are deliberately simple. They are **references and test oracles, not foundations**: inside UE5 the rigid body is Chaos, and our differentiator is the tire, drivetrain and assist stack.
- **Project Chrono (BSD-3)** has research-grade tire models (Pacejka 2002, TMeasy, Fiala) and full multibody suspensions. It is too heavy for 300 cars, but ideal as an **offline validation oracle**.
- **GPL simulators** (TORCS, Speed Dreams, VDrift, Rigs of Rods) are study-only and **excluded from the codebase**.

## Decision

**We choose D, built mostly as C.** OpenRoad Dynamics gets an **engine-independent C++20 physics core** (`src/core`) implemented from published mathematics, with interchangeable modules behind narrow interfaces. Unreal Engine 5 integration is a thin adapter plugin (planned: `plugin/OpenRoadDynamics`) that:

- runs the core inside Chaos' async physics callback at a fixed rate,
- implements `GroundQuery` with Chaos scene queries and physical materials,
- applies the core's chassis forces and torques to the Chaos rigid body. Chaos stays authoritative for collisions between bodies; the core owns wheel, tire, suspension, drivetrain and assists.

Selected KinetiForge algorithms may be adopted under MIT with attribution, starting with the **suspension geometry solvers**. They will be ported into engine-independent form, tested against analytic cases, and credited in `THIRD_PARTY_NOTICES.md`. Nothing is copied yet. The "do not copy" list in [kinetiforge-analysis.md](../research/kinetiforge-analysis.md) §15 (sprung-mass computation, AsyncTickPhysics bridge) is binding.

**Chaos Vehicles** remains the **A/B behavioural baseline** inside UE, used only as a running comparison. No Chaos source enters this repository.

## What the proof of concept already validates

The v0.1 core in this repository (all numbers from `ordbench` and `ord_tests`, see [baseline-results.md](../benchmarks/baseline-results.md)):

- Magic Formula tire with load sensitivity, combined-slip weighting functions and relaxation-length transient slip. Tested for small-slip stiffness, peak μ, friction-ellipse bounds and progressive breakaway.
- **Drivetrain/wheel/slip solved as one linearly implicit system with stick/slip friction rows** (clutch, brakes, LSD, locked diff). Stable at **60 Hz**. 120 Hz results are within 2% of 1000 Hz. It holds a car on a 20% grade with <2 cm drift. An open diff splits torque exactly.
- Cost: ~4–5 µs per full-fidelity vehicle step on one core of the CI-class machine (one 4-core cloud container; not a target platform). That is ~1.8 ms of CPU per simulated second at 360 Hz.
- ESC/ABS/TCS act only through actuators. The FMVSS-126-style sine-with-dwell test fails with ESC off and passes with ESC on for the RWD reference car.

## Consequences

**Positive**
- The whole handling stack is testable in CI without Unreal, and benchmarks are reproducible bit for bit.
- Physics-thread safety: the core owns its state; the adapter only marshals inputs, outputs and ground queries.
- LOD tiers and network replication can be designed around plain-data state.
- Clean MIT provenance.

**Negative / risks**
- Building suspension kinematics, multi-point contact, tire thermal and wear models, and the UE adapter is substantial work (see [roadmap](../roadmap.md)).
- Two rigid-body owners (Chaos for collisions, the core for chassis integration in headless tests) need a precise contract. In UE the core will *not* integrate the chassis; it will output forces to Chaos. Headless integration is for tests and servers. This split must be covered by an adapter test comparing both paths.
- The Magic Formula needs parameter sets. Real MF coefficient files (`.tir`) are often proprietary or unclearly licensed, so we will ship only fictional or derived parameter sets with documented provenance.

## Revisit when

- the UE adapter's force-application path shows divergence from headless integration over 1% on the benchmark suite, or
- profiling shows the per-vehicle cost exceeds the [performance budget](../architecture/open-world-and-lod.md#performance-budget).
