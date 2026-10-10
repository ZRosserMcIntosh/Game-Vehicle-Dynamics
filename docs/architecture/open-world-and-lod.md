# Open-world requirements, fidelity tiers and budgets

Target: a continuous Brazilian world covering São Paulo, Rio de Janeiro, Curitiba, Florianópolis and the highways between them (BR-116, BR-101, BR-376, …). That means dense city traffic, cobblestone historic centres, favela hill roads, mountain serras and long-distance expressways.

## Surfaces

The core sees every surface as `SurfaceProperties` from the host's `GroundQuery`. That keeps world content (UE Physical Materials) decoupled from physics. v0.1 has `frictionScale`, `rollingResistanceScale` and a placeholder `wetSpeedFalloff`.

| Surface | Grip (relative to dry asphalt; literature ranges, to be calibrated) | Needs | Status |
|---|---|---|---|
| New asphalt, dry | 1.0 | — | ✅ |
| Worn / polished asphalt | 0.8–0.95 | per-material scale | ✅ (scale) |
| Concrete | 0.9–1.0 | expansion-joint bumps (geometry) | ✅ scale, 🧭 joints |
| Paralelepípedo / cobblestone | 0.6–0.8 dry, much lower wet | high-frequency vertical input → **multi-point contact + unsprung mass** | 🧭 |
| Gravel / dirt | 0.5–0.7, loose-surface MF shape | different curve *shape* (peak at high slip), rolling resistance, ploughing | 🧭 |
| Grass | 0.35–0.5 | rolling resistance, low peak | ✅ scale |
| Wet asphalt | 0.5–0.8, speed-dependent | **water-film depth** → speed-dependent μ, hydroplaning onset | 🧭 (placeholder only) |
| Painted lines, manhole covers, wet | 0.3–0.5 | small-patch surfaces → per-contact-point surface | 🧭 |

Grip values are typical engineering ranges for road-tire friction, cited only as starting points. Each will get a sourced calibration entry in `docs/research/` before use.

**Rain plan** (🧭): world rain intensity and drainage give a water-film depth per surface patch, provided by the host. The tire computes a speed- and depth-dependent μ reduction and a hydroplaning onset. Hydroplaning speed scales with √(inflation pressure) in the classic NASA/Horne aircraft-tire relation; passenger-car tires with tread deviate strongly from it, so treat it only as a scaling guide. Calibrate with published road-tire wet-grip literature. FM 2023 and FH also show puddles as discrete hazards (see [forza-physics.md](../research/forza-physics.md)).

## Road imperfections

Potholes, speed bumps (lombadas), curbs, drainage gutters and cobblestones are *geometry* sampled by the tire. They are never faked with noise. FM7 reportedly faked curb noise, and FM 2023 replaced that with 8 contact points per tire (Turn 10 via press, Medium–High confidence).

Plan:
1. 🧭 **Unsprung mass DOF** per corner, so the wheel can leave and re-strike the ground and harshness is physical.
2. 🧭 **Multi-point contact patch**: N samples (start with 5: centre + 4 edges; evaluate 8) at the patch perimeter and centre. Contact plane and effective height come from a weighted fit, which acts as a geometric low-pass filter (envelope) over road features shorter than the patch. Benchmarks: curb strike, 10 cm pothole at 60/120 km/h, cobblestone stretch, timestep × sample-count matrix.
3. 🧭 Sweep test (sphere or cylinder) as an alternative contact query for large obstacles.

## Fidelity tiers (LOD)

| Tier | Who | Model | Rate | Est. cost / car / sim-second |
|---|---|---|---|---|
| **LOD0** | Player car (+ replay/photo car) | Full: this core | 240 Hz (default), 120–360 Hz configurable | ~1.2 ms at 240 Hz (5 µs/step measured, v0.1) |
| **LOD1** | Nearby AI and remote players (≤ ~16 within ~150 m) | Same tire/drivetrain maths. Simplified: 1 contact point, no multi-point patch, no thermal, lumped drivetrain | 120 Hz (or 60 Hz) | target ≤ 0.3 ms |
| **LOD2** | Ambient traffic (hundreds) | Kinematic path follower with a friction-circle limit and a simple longitudinal model. Wakes up to LOD1 on proximity or collision | 30 Hz, batched | target ≤ 10 µs |
| **LOD3** | Distant / off-screen traffic | Lane-graph agents, no physics | event-driven | ~0 |

Transitions must preserve velocity and yaw rate and must not pop. State mapping LOD1 ↔ LOD0 is planned with tests (🧭). A collision always promotes both cars to at least LOD1.

### Performance budget

Proposal for a current-gen console-class CPU, to be validated on target hardware:

- Vehicle physics total ≤ **2.0 ms per 60 fps frame** of one worker thread.
- LOD0: 1 car at 240 Hz ≈ 4 steps/frame × 5 µs = 0.02 ms (desktop measurement; console CPUs ~2× slower, so budget 0.05 ms).
- LOD1: 16 cars × 2 steps/frame × ~1.5 µs ≈ 0.05 ms.
- LOD2: 300 cars × 0.5 steps/frame × ~5 µs ≈ 0.75 ms, batched and SIMD-friendly.
- Plus ground queries (raycasts): **likely the dominant real cost in UE**, so budget and measure them separately.

The v0.1 measurement is from `ordbench` on a 4-core Linux cloud container (CI-class, not a gaming PC or console). Treat the absolute numbers as indicative only.

## Fixed timestep: 120 vs 240 vs 360 Hz

Measured on flat ground (v0.1, sports RWD, SIMULATION preset):

| Rate | 0-100 (s) | 100-0 (m) | Skidpad (g) | Step-steer yaw (rad/s) | µs/step |
|---|---|---|---|---|---|
| 60 | 6.27 | 47.4 | 0.976 | 0.260 | ~5.5 |
| 120 | 6.22 | 47.9 | 0.975 | 0.260 | ~4.4 |
| 240 | 6.19 | 48.1 | 0.975 | 0.260 | ~4.2 |
| 360 | 6.24 | 48.2 | 0.975 | 0.260 | ~4.2 |
| 1000 | 6.25 | 48.3 | 0.975 | 0.260 | ~4.1 |

Source: [baseline-results.md](../benchmarks/baseline-results.md). µs/step is wall time on a 4-core Intel Xeon @ 2.1 GHz cloud container, single thread, noisy at this scale.

Conclusion so far: **on smooth ground, rate barely matters** above 120 Hz, thanks to the implicit solver. The case for 240–360 Hz is **spatial sampling of rough ground at high speed** and input latency. It is not integration accuracy. That needs the rough-road benchmarks (roadmap M2) before choosing a default. The 1000 Hz 0–100 differs from 360 Hz in the 3rd digit partly because of wheelspin chaos at launch with no TCS. Tests check timestep sensitivity with TCS on.

## Multiplayer

See [overview.md](overview.md#determinism-and-networking-). Physics independence from streaming and deterministic stepping are prerequisites, both satisfied by the core design.

## Refuelling

Fuel is not modelled as a gameplay constraint (by request). A fuel mass term (affecting weight distribution) may be added for SIMULATION realism, default off.
