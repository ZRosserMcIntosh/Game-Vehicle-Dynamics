# OpenRoad Dynamics

**An open-source vehicle physics and driving system for Unreal Engine 5, built on an engine-independent C++ core.**

One physically grounded simulation. Configurable assists make it accessible,
and turning them off makes it unforgiving. The aim: Forza Horizon-level
enjoyment on a gamepad and Forza Motorsport-level consequences with every aid
off, in a giant open world.

> **Status: research + v0.1 proof of concept (October 2026).**
> The engine-independent core builds and passes 35 automated tests on Linux
> (GCC 13, Clang 18, ASan/UBSan). **There is no Unreal Engine plugin yet.**
> It is the next milestone. Nothing here is affiliated with or endorsed by
> Microsoft, Turn 10 Studios, Playground Games or Epic Games. "Forza" is used
> only to describe publicly documented design goals; we don't claim
> equivalence with any game.

The first user is a larger open-world game set in southeastern and southern
Brazil (São Paulo, Rio de Janeiro, Curitiba, Florianópolis and the highways
connecting them). The system is designed as a reusable library and plugin for
any developer.

## Principles

1. **One simulation, many experiences.** FUN, SPORT and SIMULATION are bundles of individually configurable aids. They never change tire grip or any other physics ([ADR-0002](docs/decisions/one-simulation-many-experiences.md)).
2. **Physics you can measure.** Every handling claim maps to a reproducible benchmark ([plan](docs/benchmarks/benchmark-plan.md), [current results](docs/benchmarks/baseline-results.md)).
3. **Engine-independent core, thin engine adapter.** `src/core` has no engine headers, is deterministic, and doesn't allocate in the step ([ADR-0001](docs/decisions/physics-foundation.md)).
4. **Clean provenance.** MIT, implemented from published mathematics. No engine source, no game data, no leaked material ([THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)).

## What exists today (v0.1 core)

| Area | Implemented | Verified by |
|---|---|---|
| Tire | Reduced Pacejka Magic Formula: load sensitivity, combined-slip weighting functions, relaxation-length transient slip, standstill stiffness, surface friction scaling | `tests/test_tire.cpp` |
| Drivetrain | Engine torque curve, idle governor, rev limiter, engine braking, clutch, gearbox, open / locked / clutch-pack LSD, AWD open centre split. **All shafts, brakes and tire slip are solved in one implicit system with stick/slip friction** | `tests/test_drivetrain.cpp` |
| Chassis | 6-DOF rigid body, per-corner raycast suspension (spring, preload, bump/rebound damping, bump stops), anti-roll bars, Ackermann steering, aero drag + axle downforce | `tests/test_vehicle.cpp` |
| Assists | Speed-sensitive gamepad steering, input shaping, ABS, TCS, ESC, countersteer aid, auto/manual shifting, auto clutch. All act through real actuators only and report every intervention | `tests/test_assists.cpp` |
| Benchmarks | 0–100, 100–0 (dry/wet, ABS on/off), skidpad, step steer, FMVSS 126-style sine-with-dwell, slope hold, speed bump, drop test, coast-down, timestep sweep, top speed, fuzzing, determinism | `tests/test_benchmarks.cpp`, `tools/ordbench` |

Selected results for the fictional 300 kW RWD reference car at 360 Hz ([full report](docs/benchmarks/baseline-results.md)):

| | FUN | SPORT | SIMULATION |
|---|---|---|---|
| 0–100 km/h | 5.2 s | 4.7 s | 6.2 s (wheelspin, no TCS) |
| 100–0 km/h | 39.7 m (ABS) | 39.7 m (ABS) | 48.2 m (locked wheels) |
| Skidpad R40 | 0.95 g | 1.01 g | 0.98 g |

- **Timestep:** 120 Hz results are within 2% of 1000 Hz. The solver stays stable even at 60 Hz.
- **Cost:** ~4–5 µs per full-fidelity vehicle step on one core of a 2.1 GHz cloud Xeon.
- **ESC:** in the sine-with-dwell test, the RWD car spins out (27° sideslip) at a 9° road-wheel input with ESC off, and passes the FMVSS 126 yaw-rate ratios with ESC on.

These are fictional cars tested against plausibility envelopes, not real-world validation.

## Build

Requires CMake ≥ 3.20 and a C++20 compiler.

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure      # 35 tests, ~1 s
./build/ordbench                                # Markdown benchmark report
./build/ordbench --csv-dir /tmp/traces          # per-step telemetry CSV for plotting
```

### Minimal use of the core

```cpp
#include "ord/presets.hpp"
#include "ord/vehicle.hpp"
#include "reference_vehicles.hpp"

ord::Vehicle car(ord::vehicles::referenceSportsRwd(),
                 ord::makePreset(ord::DrivingPreset::Sport));
ord::FlatGround ground;               // hosts implement ord::GroundQuery
car.reset({0, 0, 0.6}, 0.0, ground);

ord::DriverInput in;
in.throttle = 1.0;
for (int i = 0; i < 360 * 5; ++i)
    car.step(1.0 / 360.0, in, ground);   // fixed timestep
double kmh = ord::msToKmh(car.telemetry().speed);
```

## Repository layout

```
src/core/          engine-independent physics core (ord::)
src/vehicles/      fictional reference vehicle definitions
src/scenarios/     reproducible benchmark manoeuvres
tools/ordbench/    benchmark report generator
tests/             unit, regression and benchmark tests (no dependencies)
docs/research/     Forza public research, telemetry, KinetiForge audit, alternatives
docs/architecture/ architecture, drivetrain solver, presets, open world & LOD
docs/benchmarks/   benchmark plan, telemetry channels, baseline results
docs/decisions/    architecture decision records
docs/roadmap.md    milestones and priorities
```

## Research

| Document | Contents |
|---|---|
| [forza-physics.md](docs/research/forza-physics.md) | What Turn 10 / Playground have publicly said about tires (8 contact points at 360 Hz in FM 2023), suspension, drivetrain, assists, with evidence class and confidence |
| [forza-handling-philosophy.md](docs/research/forza-handling-philosophy.md) | How Forza makes a sim-based model accessible; implications for our presets |
| [forza-telemetry.md](docs/research/forza-telemetry.md) | Official Data Out format (Sled/Dash), what lawful self-recorded telemetry can and can't validate |
| [kinetiforge-analysis.md](docs/research/kinetiforge-analysis.md) | Source-level audit of the MIT KinetiForge UE5 plugin: capability matrix, bugs, reusable parts |
| [alternatives.md](docs/research/alternatives.md) | Chaos Vehicles, PhysX 5, Jolt, Chrono, Bullet, GPL simulators: verified licenses and reuse options |

Research caveat: the sandbox used for this research couldn't load several
official pages directly. Findings marked "(snippet)" or "(extract)" come from
search-engine excerpts and should be checked against the live pages before
being quoted.

## Roadmap

Next: UE5 plugin with Chaos async integration → unsprung mass and multi-point
contact patch → tire depth (wet, loose surfaces, thermal) → LOD tiers for
hundreds of traffic cars → multiplayer. See [docs/roadmap.md](docs/roadmap.md).

## Contributing & license

See [CONTRIBUTING.md](CONTRIBUTING.md) and [SECURITY.md](SECURITY.md).
Licensed under the [MIT License](LICENSE).
