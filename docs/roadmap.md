# Engineering roadmap

Prioritised by "what unlocks measurable driving quality soonest". Each item has acceptance criteria tied to benchmarks in [benchmark-plan.md](benchmarks/benchmark-plan.md). The GitHub issues mirror this list.

## M0 — Foundation (this session) ✅

- [x] Research: Forza public physics, Forza telemetry, KinetiForge audit, alternatives & licensing
- [x] ADRs: physics foundation; one simulation, many experiences
- [x] Engine-independent core v0.1: MF tire + transient slip, implicit drivetrain solver, suspension, aero, assists, presets, telemetry
- [x] Scenario library, 35 tests, `ordbench` report, CI

## M1 — Unreal Engine 5 adapter (next)

1. **Plugin skeleton** `plugin/OpenRoadDynamics` (Runtime + Editor modules). `UOpenRoadVehicleComponent` owns an `ord::Vehicle`. Build on UE 5.4+. *Accept:* compiles on Win64 with UE 5.4/5.5/5.6. Car drives on the proving-ground level.
2. **Chaos async integration**: core steps inside the async physics callback. `GroundQuery` via physics-thread scene queries. Forces and torque go to the Chaos body (core chassis integration disabled). *Accept:* headless-vs-UE benchmark divergence < 1% on B1/B2/B4/B5. No `UObject` access on the physics thread.
3. **Coordinate/unit adapter** (cm, left-handed) with unit tests.
4. **Physical Material → SurfaceProperties** data asset.
5. **Debug overlay**: per-wheel forces, slip, loads, assist activity. Telemetry CSV capture.
6. **Proving-ground level** mirroring `TestTrackGround`.
7. **Chaos Vehicles A/B baseline** car in the same level.

## M2 — Contact & suspension fidelity

1. **Unsprung mass DOF** per corner (wheel hop, harshness). *Accept:* B20, B22 natural frequencies within 3% of analytic.
2. **Multi-point contact patch** (5 → 8 samples, perimeter + centre, weighted plane fit). *Accept:* B19 curb load spikes converge with sample count. Rate × samples matrix published.
3. **Kinematic suspension**: camber/toe/caster vs travel curves, then geometric solvers (MacPherson, double wishbone, solid axle), adapting KinetiForge's MIT solvers with attribution. *Accept:* analytic-geometry tests.
4. Anti-dive / anti-squat / roll-centre load paths.
5. **Fixed-rate decision**: 120/240/360 Hz on rough-road benchmarks → default for LOD0.

## M3 — Tire depth

1. Camber thrust; full MF 6.x coefficient support (`.tir`-style import, our own parameter sets only).
2. Wet model: water-film depth input, speed-dependent μ, hydroplaning onset (B21).
3. Loose surfaces (gravel/dirt/grass curve shapes, rolling resistance).
4. Thermal (surface/carcass), pressure and wear. Off by default outside SIMULATION. No refuelling gameplay.
5. Self-aligning torque (MF Mz) → **force feedback** pipeline.

## M4 — Drivetrain & powertrains

Turbo boost/lag, supercharger, EV motor envelopes, hybrid, torque converter, viscous couplings, torque vectoring, engine stall/start, manual-clutch realism.

## M5 — Scale & multiplayer

1. LOD1 model + LOD2 kinematic traffic + transitions (B24, B25).
2. State snapshot/restore. Server-authoritative prediction and reconciliation prototype.
3. Parallel vehicle stepping, SIMD-friendly batch for LOD2.

## M6 — Validation & experience

1. Real-vehicle comparison with an open parameter set (e.g. Chrono BMW E90 data).
2. Forza Data Out capture tool (our own recordings) + comparison report (B4 lateral-g curve, B2).
3. Playtest programme; FUN-mode tuning; drift (B17).
4. Assist refinements: brake assist, launch control, reference-speed estimator.

## Next five engineering priorities

1. UE5 plugin skeleton + Chaos async integration (M1.1–M1.3)
2. Unsprung mass DOF (M2.1)
3. Multi-point contact patch + curb/pothole benchmarks (M2.2, B19–B20)
4. Debug overlay + telemetry capture in UE (M1.5)
5. LOD1 vehicle model + scale benchmark (M5.1)
