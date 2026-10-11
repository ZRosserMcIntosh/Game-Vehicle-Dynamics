# ADR-0003: Ground prefetch, rate decoupling and surface texture

- **Status:** Accepted (2026-10-11)
- **Context documents:** [physics-foundation.md](physics-foundation.md), [open-world-and-lod.md](../architecture/open-world-and-lod.md), [performance-and-risks.md](../architecture/performance-and-risks.md)
- **Evidence:** [baseline-results.md](../benchmarks/baseline-results.md) ("Rough roads", "Fidelity tiers"), tests in `tests/test_contact.cpp`, `tests/test_host.cpp`, `tests/test_lod.cpp`

## Problem

Three risks from ADR-0001 point to the same design question.

1. **Raycast cost.** A Forza-style multi-point contact patch at a high physics rate means samples × wheels × rate scene queries per car: 8 × 4 × 360 = 11,520 per second. In Unreal those are Chaos scene queries, which will likely dominate the vehicle's CPU cost.
2. **Thread safety.** In Unreal the vehicle steps on the physics thread. Scene queries there need care, and KinetiForge shows what goes wrong when world access from the physics thread is not designed in.
3. **World physics rate.** Chaos' async fixed step applies to the **whole scene**. Running the world at 360 Hz to get good tires would multiply the cost of every rigid body in the city.

## Decision

### 1. Ground prefetch strips (`GroundPrefetchCache`)

Once per **render frame**, on the thread that owns the world (the game thread in UE), the host casts a small adaptive set of rays along each wheel's predicted path:

- The strip covers the contact patch plus `horizon` seconds of travel (default 1.5 frames).
- **Smooth surfaces:** a single coarse line (0.35 m spacing), storing heights and normals, so cross-fall and camber are kept.
- **Rough surfaces:** a dense grid (3 lines, 0.06 m). A strip counts as rough if a material has `SurfaceProperties::roughness`, a ray misses, a coarse sample deviates from its neighbours by more than 4 mm, or normals change by more than 0.03 rad. An untagged pothole is found this way (tested).
- Physics substeps and contact samples query the strips only. A query outside every strip is **extrapolated** from the nearest strip and counted as a miss. A wheel never falls through the world because of a late tile or a hitch.

The physics thread reads immutable data, which makes it thread-safe by construction. Engine ray counts become independent of physics rate and contact sample count.

### 2. Vehicle-local substepping (`Vehicle::stepSubstepped`)

In `ChassisMode::External` the vehicle advances its own copy of the chassis through *k* internal substeps per host step. It then hands the host the impulse-equivalent average force and torque. The world can stay at 60–120 Hz while tires, suspension and drivetrain run at 240–480 Hz.

### 3. Procedural surface micro-texture (`SurfaceProperties::texture*`)

Paving stones (paralelepípedos), worn concrete and rumble strips are usually *flat* in a game's collision mesh. Representing them as dense collision geometry is expensive to author, to query and to prefetch. A sampled strip at 6 cm spacing aliases a 15 cm stone pitch, so the error doubles (measured).

Instead, a material can declare a stone pitch and height. The core adds a deterministic, world-position-based height under each contact sample. That is multiplayer-safe and costs zero queries. Real geometry such as curbs, potholes and speed bumps stays geometry. This is still physically modelled vertical excitation, filtered by the contact patch. It is not FM7-style noise injected into forces.

## Evidence (v0.2, sports RWD, SIMULATION)

| Configuration | Curb RMS az (g) | Pothole RMS az (g) | Texture cobbles RMS az (g) | Engine rays/s (curb) |
|---|---|---|---|---|
| Direct, 1000 Hz, 5 samples (reference) | 0.314 | 0.106 | 0.024 | 20,000 |
| Direct, 240 Hz, 5 samples | 0.292 | 0.101 | 0.024 | 4,800 |
| **Host 60 Hz × 4 substeps, 5 samples, prefetch** | **0.293** | **0.109** | **0.024** | **1,363** |

- Smooth roads with prefetch use ~960 rays/s per car (4 wheels × ~4 samples × 60 fps), with no misses. The 0–100 time is unchanged within 1% (test).
- 60 Hz host with 4 substeps matches a 240 Hz internal run within 1% on a mixed drive (test). Rough roads are within 10% of the 1000 Hz reference (test).
- Point contact (1 sample) does not converge on cobblestones and overstates harshness by ~4×. The 5-sample patch converges at 120 Hz.

## Default configuration (recommended)

| Tier | World step | Vehicle rate | Contact | Ground |
|---|---|---|---|---|
| LOD0 player | 60 Hz (UE default async step can stay) | ×4 = 240 Hz | 5 samples, unsprung DOF | prefetch strip per wheel per frame |
| LOD0 "simulation quality" option | 60 Hz | ×6 = 360 Hz | 8 samples | same |
| LOD1 AI | 60 Hz | 120 Hz | 1 sample, massless | one coarse strip per car per frame |
| LOD2 traffic | — | 30 Hz kinematic | — | 1 ray per car every 8 steps |

The rate choice question from ADR-0001 is answered this way. With the multi-point patch, implicit solvers and prefetch, rate mainly buys accuracy on sharp impacts: curbs are within ~7% at 240 Hz and ~1% at 360 Hz. It no longer costs engine queries, and it no longer forces the world rate up.

## Consequences and limits

- Peak forces inside substeps are not visible to the host. Impulses are exact, but host-side contact or damage logic that wants per-substep peaks must read them from vehicle telemetry.
- The strip is a local height field. Overhangs, tunnels and multi-level interchanges (common on Brazilian highways) need the host to cast from the right level. The strip starts 1 m above the wheel's current contact, so it samples the deck the car is on. Bridge edges and very steep transitions are not yet benchmarked (roadmap).
- Potholes smaller than the coarse spacing (0.35 m) on *untagged* smooth materials can be missed. Content rule: tag defect decals and meshes with a rough physical material.
- Strip reuse across frames (overlap) could halve rays again. Not done yet.
