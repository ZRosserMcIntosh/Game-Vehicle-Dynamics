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

## Update 2026-10-11: coupling audit and tire envelope probes

An independent audit tested the main assumption here: that a 60 Hz host body receiving only the *average* force of 4 vehicle substeps behaves like a chassis coupled to the suspension at the full rate.

**Findings**

1. **The coupling itself is sound.** On an 8 cm curb at 120 km/h, host-mode results match internal mode at the *same vehicle rate* within ~4% (peak heave 0.0102 m vs 0.0099 m at 240 Hz). The host's pose differs from the vehicle's own substep prediction by well under a millimetre per step on rough roads.
   - New diagnostic: `Vehicle::hostSync()` reports this error, so the same check can run against Chaos.
   - On each sync, unsprung wheels now keep their absolute position and velocity, and the suspension absorbs the difference instead of the tire penetration jumping.
   - The measured effect of this is small.
2. **The real defect was rate convergence of the contact model on sharp edges.** With the flat-footprint patch, peak heave over the curb varied non-monotonically with rate: 0.0086, 0.0099, 0.0109 and 0.0087 m at 120, 240, 480 and 1000 Hz.
   - A near-vertical step sampled by discrete vertical rays is an event whose effect depends on where the step lands relative to the timesteps.
   - The earlier "within 10%" result held only at 60 km/h, and partly because the host-mode metric sampled acceleration at 60 Hz while the reference sampled at 1000 Hz.
3. **Fix: envelope probes** (`VehicleFidelity::envelopeProbes`, default 3 per side). These are radial samples at 0.40, 0.55 and 0.70 of the radius ahead of and behind the patch, measured against the tire's circular profile.
   - They carry zero load on flat ground (tested). They pick up a step before the patch reaches it, with a radial force that gives the longitudinal impact.
   - This is the geometric enveloping idea of Pacejka (2012) ch. 10 and Schmeitz's cam-based enveloping model, in a simplified discrete form.
   - With the ground cache, probes cost lookups, not engine rays. The prefetch strip length grows to cover them.
4. **Rate-fair metrics** were added to the rough-road scenarios: peak CoM heave, and vertical acceleration averaged over 60 Hz windows.

**Result.** Through the production ground path (prefetch cache), the 8 cm curb at 120 km/h is within ±7% across 240–2000 Hz with probes; without them it was off by up to 21%. At 240 Hz it is within 2% of 2000 Hz (regression test).

The 60 Hz host × 4 configuration now stays within 10% of a 1000 Hz reference on curbs at both 60 and 120 km/h. The comparison is like for like (both cached, rate-fair metrics).

**Remaining limit.** Raw, perfectly sharp steps fed directly (not through the cache's 6 cm sampling) still show some rate aliasing. A continuous effective-road-profile evaluation (moving cams along the profile) would remove it, at higher cost. This is tracked on the roadmap.

**Hitches.** `GroundPrefetchCache::setHorizon()` lets the host size the strips from measured frame times. A 100 ms game-thread hitch at 200 km/h degrades gracefully: extrapolation keeps the car on the road and misses are counted (test). Strips cannot be extended *during* an unforeseen hitch, so extrapolation is the safety net.

**Model sensitivity (open).** Patch layouts converge with rate, each to its own value. On the 8 cm / 120 km/h curb, peak heave converges to ~0.0110 m with 5 samples and ~0.0079 m with the 8- or 9-sample layouts: the edge response depends ~30% on how stiffness is spread across the footprint.
- This is a modelling choice that needs calibration against measured tire data (cleat/enveloping tests). It is not a numerical error.
- Until then, 5 samples remain the default. The Forza-like 8-point option is **not** claimed to be more accurate.
