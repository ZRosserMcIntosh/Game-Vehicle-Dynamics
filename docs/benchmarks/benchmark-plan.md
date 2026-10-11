# Benchmark and validation plan

> Principle: **measure before building.** Every handling claim must map to a
> reproducible scenario with an objective metric. Subjective "feel" is
> evaluated separately and never mixed with physical-accuracy metrics.

Implementation: [`src/scenarios/`](../../src/scenarios/) (shared by tests and tools),
[`tests/test_benchmarks.cpp`](../../tests/test_benchmarks.cpp) (regression bands),
[`tools/ordbench`](../../tools/ordbench/main.cpp) (report → [baseline-results.md](baseline-results.md)).

## 1. Three kinds of evidence

| Kind | Question | Method | Example |
|---|---|---|---|
| **Numerical correctness** | Does the code solve the equations it claims? | Analytic cases, conservation, convergence vs timestep | Open-diff torque split, energy in drop test, 120 vs 1000 Hz |
| **Physical plausibility / accuracy** | Does the car behave like a real car with these parameters? | Compare with published vehicle data, textbook relations, regulations' test procedures | Skidpad ≈ μg, understeer gradient sign and magnitude, FMVSS 126 |
| **Experience** | Is it fun / readable / Forza-like? | Blind playtests, structured questionnaires, controlled comparison with user-recorded Forza telemetry | FUN-mode spin rate per hour, recovery success |

## 2. Scenario catalogue

✅ implemented in v0.1 · 🧭 planned

| # | Scenario | Procedure | Metrics | Status |
|---|---|---|---|---|
| B1 | **0–100 km/h** | Standstill, full throttle, auto shifts | time 0–60/0–100, peak driven slip, TCS cut | ✅ |
| B2 | **100–0 km/h braking** | Steady 100 km/h, full brake, throttle off | distance, mean decel, max sideslip, ABS active %, wheel lock | ✅ |
| B3 | **Wet vs dry braking** | B2 on surface with frictionScale 0.75 + wet falloff | distance ratio | ✅ (placeholder wet model) |
| B4 | **Skidpad (R = 40 m)** | Pure pursuit + radius PI, speed ramp 0.15 m/s² | max steady lateral g within ±1.5 m of radius, sideslip at max | ✅ |
| B5 | **Step steer** (ISO 7401-style) | 80 km/h, step input, throttle held | steady yaw rate, 10–90% rise time, overshoot, understeer gradient | ✅ |
| B6 | **Sine with dwell** (FMVSS 126-style) | 80 km/h, 0.7 Hz sine, 500 ms dwell, throttle released, amplitude sweep | yaw ratio @1.0 s/1.75 s, max sideslip, spin | ✅ |
| B7 | **Standstill hold** | 20% grade, brakes held / released in neutral | drift (m) | ✅ |
| B8 | **Speed bump** | 8 cm × 0.5 m sine bump at 40 km/h | peak compression, peak vertical g, airborne time, finiteness | ✅ |
| B9 | **Drop test** | 0.25 m drop, brakes held | bounce peaks, mechanical energy never increases | ✅ |
| B10 | **Coast without losses** | Neutral, drag = rolling resistance = 0, 100 km/h | speed loss rate, no energy gain | ✅ |
| B11 | **Timestep sensitivity** | B1, B2, B4, B5 at 60/120/240/360/480/1000 Hz | deviation vs 1000 Hz; cost µs/step | ✅ |
| B12 | **Max-speed stability** | Full throttle to drag-limited top speed | v_max vs analytic, yaw/roll noise at > 250 km/h | ✅ (test) |
| B13 | **Fuzz robustness** | Random inputs on bumpy terrain, 60 & 360 Hz, 3 cars | no NaN/Inf, no solver failure | ✅ (test) |
| B14 | **Determinism** | Same inputs twice | bit-identical state | ✅ (test) |
| B15 | **ISO 3888-2 obstacle avoidance / double lane change** | Path-following driver, increasing entry speed | max clean entry speed, peak sideslip, cone hits | 🧭 |
| B16 | **Slalom** (18 m cones) | Path follower, fixed speeds | max speed, peak yaw, lateral error | 🧭 |
| B17 | **Drift initiation & recovery** | Scripted: clutch kick / handbrake / power-over; then countersteer driver | initiation success, sustained angle, recovery time; FUN vs SIM | 🧭 |
| B18 | **Throttle-on / lift-off oversteer** | Steady cornering at 0.6 g, throttle step / lift | yaw rate deviation | 🧭 |
| B19 | **Curb strike** | Real curb geometry at 60/120 km/h, 1 and N contact points | load spike, contact continuity, timestep × sample matrix | ✅ (5-point patch, rate × samples matrix) |
| B20 | **Pothole / cobblestone** | 10 cm pothole; cobblestone profile | wheel hop, load variation, harshness RMS | ✅ (pothole + cobblestones, geometry and texture) |
| B21 | **Aquaplaning onset** | Water film depth sweep, speed ramp | onset speed vs film depth | 🧭 (needs wet model) |
| B22 | **Suspension frequency response** | Heave/pitch sine sweep | natural frequencies vs analytic √(k/m) | ✅ (heave and wheel hop vs analytic quarter car) |
| B23 | **Controller vs wheel** | Same manoeuvre, gamepad pipeline vs direct-angle pipeline, scripted "human-like" inputs | path error, steering workload, spins | 🧭 |
| B24 | **AI/traffic LOD transitions** | LOD0↔LOD1↔LOD2 swaps at speed | velocity/yaw discontinuity | ✅ |
| B25 | **Scale test** | 1 LOD0 + 16 LOD1 + 300 LOD2 | ms/frame, ground-query count | ✅ (1 + 16 + 100/300/1000) |

Additional checks added in v0.2: ground-cache accuracy vs direct queries, engine-ray budgets, cache misses, host-integration parity (same and different integrators), 60 Hz host + substeps vs internal 240 Hz. See `tests/test_contact.cpp`, `tests/test_host.cpp` and `tests/test_lod.cpp`.

## 3. Initial test environment

Before any Brazilian city content is built, everything runs on an analytic **proving ground**. It exists headless today as `TestTrackGround`; a UE level will mirror it 1:1 (🧭).

```
 ┌────────────────────────────── 1.2 km straight (braking strip, 0-100, top speed) ─────────────────┐
 │  dry asphalt  │ wet patch (B3) │ concrete joints │ cobblestone (B20) │ speed bumps (B8) │ potholes │
 └───────────────────────────────────────────────────────────────────────────────────────────────────┘
   ◯ skidpad R40 (B4) + wet skidpad      ⟂ lane-change box (B15)     ⋯⋯ slalom (B16)
   ⟋ 20% / 30% grades (B7)                ▭ curbs: flat / sausage / FIA-style (B19)
```

All surfaces are flat analytic height functions or tiled patches, so headless tests and the UE level produce comparable numbers.

## 4. Pass criteria philosophy

- **Correctness tests** have tight tolerances from analytic expectations, e.g. open-diff ratio ±1%, energy within 1%.
- **Plausibility tests** use envelopes from textbook ranges for the *fictional* reference cars: 0–100 within 3.8–6 s for the 300 kW RWD, braking 0.9–1.15 g with ABS, skidpad 0.85–1.12 g with μ ≈ 1.1 tires, understeer gradient > 0. These guard against regressions. They are not claims of matching any real car.
- **Accuracy vs real vehicles** (🧭): adopt one or two open parameter sets with published test data. Candidates are listed in [alternatives.md §9](../research/alternatives.md), e.g. the BMW E90 set in Project Chrono derived from SAE 2007-01-0818. Compare B2/B4/B5. Report error with sources.
- **Comparison with Forza** (🧭): only through user-recorded Data Out telemetry, following [forza-telemetry.md](../research/forza-telemetry.md). This gives behavioural similarity (e.g. lateral-g vs speed curves, braking distances). It never claims equivalence of internals.

## 5. Subjective evaluation (🧭)

- Blind A/B sessions: FUN vs SPORT vs SIMULATION, and our car vs Chaos Vehicles baseline in the same UE level.
- Questionnaire: predictability, recoverability, sense of speed, car-to-car distinctiveness, frustration events.
- Logged metrics during free roam: spins per hour, off-road excursions, assist intervention rate (from `AssistReport`).
- The developer's target as a reference point: a no-assist sim-level lap pace benchmark (the lead has a 5:45 FM5 Nordschleife lap in the Lotus F1 car, no assists). SIMULATION must reward that skill level; FUN must not require it.

## 6. CI

`ctest` runs all ✅ scenarios (≈1 s total in Release). The benchmark report is regenerated by hand and committed when physics changes. A CI job that diffs the report and flags changes over 1% is planned.
