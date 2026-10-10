# Architecture overview

Status: v0.1 proof of concept. This document describes what exists now
(✅), what is designed but not built (🧭), and open questions (❓).

## Layers

```
┌──────────────────────────────────────────────────────────────────────┐
│ Game (Brazil open world)        UI, cameras, audio, AI driving, net  │
├──────────────────────────────────────────────────────────────────────┤
│ 🧭 UE5 adapter plugin  plugin/OpenRoadDynamics                         │
│   UOpenRoadVehicleComponent · Chaos async callback · GroundQuery via  │
│   Chaos scene queries · PhysicalMaterial → SurfaceProperties ·        │
│   force application to Chaos body · replication · debug overlay       │
├──────────────────────────────────────────────────────────────────────┤
│ ✅ ord_core  (C++20, no engine headers, no allocation in step)        │
│   Vehicle (LOD0) ─ tire · suspension · drivetrain solver · aero       │
│   AssistSystem ─ input shaping · ABS · TCS · ESC · transmission       │
│   Telemetry · FixedStepper · GroundQuery interface                    │
│   🧭 LOD1/LOD2 vehicle models · state snapshot/restore for netcode     │
├──────────────────────────────────────────────────────────────────────┤
│ ✅ ord_scenarios + ordbench + ord_tests  (headless, deterministic)    │
└──────────────────────────────────────────────────────────────────────┘
```

Rule: **`src/core` never includes an engine header.** The host supplies:
- a `GroundQuery` (raycast → hit point, normal, `SurfaceProperties`, surface id),
- a fixed `dt`,
- `DriverInput`.

The core returns state, forces and telemetry.

## Conventions

- **ISO 8855 vehicle axes**: X forward, Y left, Z up, right-handed. SI units, radians.
- Unreal is left-handed (X forward, **Y right**, Z up) and uses **centimetres**. The adapter converts at the boundary: `y_ord = -y_ue`, `len_ord = len_ue / 100`. Handedness flips the sign of yaw rates and lateral quantities. The adapter will have a dedicated unit test.
- Double precision everywhere in the core (`ord::Real = double`).

## One fixed step (`Vehicle::step`) ✅

1. **Assists** turn `DriverInput` into `ActuatorCommands`, using sensor values from the previous step.
2. **Suspension**: per corner, one ray from above the full-bump hardpoint along −Z_body. Compression is computed, then spring + preload, bump stop, bump/rebound damping and anti-roll bars.
3. **Tire load and lateral transient slip**: `tanα'` is advanced with the implicit relaxation ODE.
4. **Drivetrain / wheel / longitudinal-slip solve**: one small linear system, solved implicitly with stick/slip friction rows ([drivetrain-solver.md](drivetrain-solver.md)).
5. **Forces**: combined-slip Magic Formula forces at each contact point (normal + longitudinal + lateral), self-aligning torque, aero drag and axle downforce, gravity.
6. **Chassis integration**: semi-implicit Euler with gyroscopic term (headless mode). In UE, step 6 is replaced by applying the summed force and torque to the Chaos body (🧭).
7. **Telemetry** frame.

## Timestep ✅/🧭

- The core assumes a fixed `dt`. `FixedStepper` provides accumulator stepping with a spiral-of-death cap and an interpolation alpha.
- Measured (v0.1, [baseline-results.md](../benchmarks/baseline-results.md)): results converge at 120 Hz to within ~1–2% of 1000 Hz. Even 60 Hz is stable. This comes from the implicit wheel/drivetrain solve and the implicit relaxation ODEs.
- Proposed defaults: **LOD0 player car 240 Hz** (headroom for curbs, cobblestones and multi-point contact at 300+ km/h), **LOD1 120 Hz**, **LOD2 30–60 Hz kinematic**. 360 Hz is supported but not assumed better: the deciding factor will be contact sampling on rough surfaces (spatial resolution: at 400 km/h, 240 Hz = 0.46 m between samples, 360 Hz = 0.31 m). Benchmark issue: see [roadmap](../roadmap.md).

## Threading 🧭

- One `Vehicle` is single-threaded and owns all its state. Different vehicles share no mutable state, so a pool of vehicles parallelises trivially (task per vehicle, or per batch of LOD1 cars).
- In UE the step runs inside the Chaos async physics callback (`ISimCallbackObject` / async physics tick) on the physics thread. Inputs come in and outputs go out through double-buffered plain structs. **No `UObject` access from the physics thread.** KinetiForge's unsynchronised approach is the specific failure mode to avoid ([kinetiforge-analysis.md](../research/kinetiforge-analysis.md) §9).
- `GroundQuery::raycast` must be callable from the physics thread. Chaos scene queries on the physics thread are the intended backend. ❓ Verify the lock and read semantics on the target UE version.

## Large worlds 🧭

- UE5 Large World Coordinates use doubles for positions. The core also uses doubles, so absolute positions across a ~1,000 km map (São Paulo ↔ Florianópolis) keep sub-millimetre precision (2^-52 × 10^6 m ≈ 2×10^-10 m).
- Even so, the core only ever needs *local* geometry (hardpoints, contact points relative to the CoM). The adapter may rebase each vehicle to a local origin per step to keep floats safe in any float-precision path (e.g. Chaos internals, if they use single precision).
- Physics is independent of streaming: the core needs only `GroundQuery`. If a ground tile isn't loaded, the adapter must supply a fallback (last known plane or a low-res heightfield) and flag it in telemetry. Wheels never fall through the world because a tile is late.

## Determinism and networking 🧭

- ✅ The core is deterministic for the same build, inputs and `dt` (test: `vehicle: simulation is bit-for-bit deterministic`). Cross-platform bit-equality is not promised; floating-point flags, compilers and `libm` differ.
- 🧭 Planned model: **server-authoritative with client prediction** for the local player's car. `VehicleState` (body + wheel spin + slip states + drivetrain + assist state) will be serialisable for snapshot/restore and replay. Remote cars run LOD1 with interpolation and extrapolation from snapshots.

## Data-driven vehicles ✅/🧭

- ✅ `VehicleDefinition` is plain data with `validate()`, which rejects non-finite or out-of-range values (vehicle files are untrusted input, see [SECURITY.md](../../SECURITY.md)).
- 🧭 JSON/TOML vehicle files with schema validation. In UE, a `UDataAsset` wrapper converts to `VehicleDefinition`.

## Module status

| Module | v0.1 | Next |
|---|---|---|
| Tire | Reduced MF + load sensitivity + combined slip + relaxation | Camber, MF 6.x full coefficients, thermal/pressure/wear, multi-point contact patch |
| Suspension | Vertical spring/damper per corner, ARB, bump stop, massless wheel | Unsprung mass DOF, kinematic curves (camber/toe vs travel), geometric solvers (MacPherson, double wishbone, solid axle), anti-dive/squat |
| Drivetrain | Engine curve, idle governor, limiter, clutch, gearbox, open/locked/clutch-LSD axle diffs, AWD open centre (+ locked/LSD centre) | Turbo lag/boost, hybrid/EV motors, viscous couplings, torque-vectoring, stall/start |
| Aero | Drag, front/rear downforce | Ride-height sensitivity, yaw, active aero, slipstream |
| Assists | Steering shaping, ABS, TCS, ESC, countersteer aid, auto/manual shifting, auto clutch | Brake assist, launch, FFB, reference speed estimator |
| LOD | — | LOD1 simplified, LOD2 kinematic traffic |
| UE adapter | — | Plugin skeleton, Chaos integration, debug overlay |
