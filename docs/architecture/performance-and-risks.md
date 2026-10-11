# Performance budget and risk register

This document tracks the technical risks named at the end of the foundation work. For each it gives the solution chosen and the evidence. Numbers come from [baseline-results.md](../benchmarks/baseline-results.md). They were measured single-threaded on a 4-core Intel Xeon @ 2.1 GHz cloud container, so they are indicative, not console measurements.

## Budget (per 60 fps frame, one worker thread)

| Item | Budget | Measured (v0.2) |
|---|---|---|
| Player car (LOD0, 60 Hz host × 4 substeps, 5-point contact, prefetch) | 0.10 ms | ~0.045 ms |
| 16 LOD1 AI cars (120 Hz, 1 sample, coarse strip each) | 0.50 ms | ~0.20 ms |
| 300 LOD2 traffic cars (30 Hz kinematic, IDM) | 0.50 ms | ~0.12 ms |
| 1000 LOD2 traffic cars | — | ~0.40 ms |
| **Total, 1 + 16 + 300** | **2.0 ms** | **~0.36 ms** |
| Engine ground rays, 1 + 16 + 300 | ≤ 150 / frame | ~20 + 66 + 18 = **~104 / frame** |

Not yet measured: the UE-side cost of each ray (Chaos scene query), the cost of applying forces to Chaos bodies, and render-side work. These are measured in the UE validation plan ([plugin README](../../plugin/OpenRoadDynamics/README.md)).

## Risks

### R1. Chaos body vs core chassis divergence

- **Risk:** in Unreal, Chaos integrates the chassis. Results might differ from our headless benchmarks.
- **Solution:**
  - `ChassisMode::External`: the core outputs force and torque about the CoM; the host integrates and writes the state back.
  - The unsprung DOF is formulated so the host body simply has `mass = VehicleDefinition::mass`. The body receives tire normal force minus the wheel's relative inertia; no mass scaling or hidden torques.
  - Gravity can be host-applied.
  - All frame and unit conversions live in `host_frame.hpp`, with unit tests for the axial-vector and quaternion mirror rules.
- **Evidence (tests):**
  - External mode with the same integrator reproduces internal mode to 1e-9.
  - A *different* host integrator (trapezoidal, exponential map) stays within 1% over an 8 s drive.
  - A 60 Hz host with 4 substeps matches 240 Hz internal within 1%.
- **Remaining:** the real Chaos integrator, solver iterations and any damping the project leaves enabled. Validate in UE against the headless benchmarks (target < 1%). The plugin sets linear and angular damping to 0.

### R2. Raycast cost in UE

- **Risk:** samples × wheels × rate scene queries.
- **Solution:**
  - Per-frame ground prefetch strips with adaptive density (ADR-0003).
  - LOD1 uses one coarse strip per car. LOD2 uses one ray per car every 8 steps.
  - Paving uses a procedural micro-texture instead of dense collision.
- **Evidence:**
  - Player car: ~960 rays/s on smooth roads and ~1,360 on curbs, against 4,800–20,000 with direct per-substep queries.
  - Whole scene with 1 + 16 + 300 cars: ~104 rays per frame.

### R3. Thread safety in UE

- **Risk:** physics-thread code touching game objects or the world. This is KinetiForge's failure mode.
- **Solution:**
  - The core owns plain-data state only.
  - All scene queries happen on the game thread during prefetch. The physics thread reads an immutable, double-buffered cache.
  - Input and output cross threads through Chaos sim-callback input/output objects (plugin).
- **Remaining:** UE API verification, listed in the plugin README.

### R4. Fixed-rate choice (120 / 240 / 360 Hz)

- **Resolved by decoupling.** The world stays at 60–120 Hz; the vehicle substeps.
- On smooth roads the rate barely matters above 120 Hz.
- On sharp impacts, 240 Hz is within ~7% of 1000 Hz and 360 Hz within ~1%, as long as the 5-point patch is used.
- Default: 240 Hz (×4). Offer 360 Hz with 8 samples as a "simulation quality" setting.

### R5. Many vehicles

- **Solution:** three tiers with a bounded LOD1 pool and analytic placement.
  - Promotion costs ~4 rays and no settling steps.
  - The traffic tier is allocation-free after warm-up, with O(n log n) lane ordering.
- **Evidence:**
  - Promotion and demotion: zero speed or yaw-rate jump, < 0.13 g vertical transient, < 0.4 m lane error.
  - 1,000 traffic cars: ~0.4 ms per frame.

### R6. Wet roads

- **Status:** still a placeholder (`wetSpeedFalloff`).
- **Plan (M3):** a host-supplied water-film depth per contact, speed- and depth-dependent μ, and hydroplaning onset, calibrated from published wet-grip literature with citations.
- The ground strips already carry `SurfaceProperties` per sample, so water depth can ride along at no extra query cost.

### R7. Tire-parameter licensing

- **Risk:** real Magic Formula coefficient sets (`.tir`) are usually proprietary or unclearly licensed.
- **Solution:** `generateTireParams(TireSpec)` builds plausible parameters from a tire's public size and category (e.g. `245/40R18`, Performance). It uses documented heuristics within textbook ranges ([tire-parameter-generation.md](../research/tire-parameter-generation.md)).
- Only parameter sets we generate or fit ourselves, with recorded provenance, go into the repository.

### R8. UE plugin not compiled

- **Status:** the plugin source exists (`plugin/OpenRoadDynamics`) but **has not been compiled**. There is no Unreal Engine in the development container.
- The plugin README lists every API assumption to verify, along with a validation plan.
- The core it wraps is fully tested headless.

### R9. Large worlds and multi-level roads

- **Precision:** handled. The core is double precision, and UE5 Large World Coordinates are double precision too.
- **Open:** prefetch strips start 1 m above the wheel's current contact, which suits stacked interchanges (viadutos) when driving on a deck. Benchmarks for deck edges, ramps and tunnels are still to do.

### R10. Untrusted vehicle data and numerical faults (added 2026-10-11)

- **Found by fuzzing:** 232 of 3,000 randomly perturbed definitions passed validation and then produced NaN. Causes included 0.1 mm suspension travel, ×1000 tire radius, steering lock beyond 90°, non-finite torque-curve points, wheels heavier than their corner's share of the car, and a 70 t vehicle on car springs.
- **Fix:**
  - `VehicleDefinition::validate()` now enforces documented physical ranges, a per-wheel unsprung-mass limit, and a static-support check.
  - A **runtime fault guard** (`Vehicle::faulted()`) keeps the last finite state and zeroes forces if a step ever produces NaN/Inf. Hosts never receive non-finite forces.
- **Evidence:** a fuzz regression test (400 cases per run) requires zero NaN *and* zero faults for definitions that validate. A corrupted-ground test checks that NaN ground data faults the vehicle cleanly.

### R11. Hidden assist effects (added 2026-10-11)

- **Found:** disabling ABS mid-stop left its brake-pressure reduction in place for ~0.1 s while reporting ABS as inactive. That is a hidden aid. Switching presets while driving also changed the steering map instantly, producing a 124 m/s³ lateral jerk.
- **Fix:** disabled ABS stops modulating immediately. A change of steering mapping crossfades the road-wheel angle linearly over 0.3 s. Physical aids still switch instantly.
- **Evidence:** regression tests check full brake torque on the step after disabling ABS, and a bounded per-step steering change during preset switches.
