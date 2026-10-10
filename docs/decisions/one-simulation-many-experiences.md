# ADR-0002: One simulation, many experiences

- **Status:** Accepted (2026-10-10)
- **Inputs:** [forza-handling-philosophy.md](../research/forza-handling-philosophy.md)

## Context

We want a casual player to have a great time cruising São Paulo on a gamepad. We also want a sim racer with every aid off to face real consequences. Building two engines ("arcade" and "sim") doubles the work and splits the handling feel. It also means skills learned in one mode don't carry over.

Public Forza material points the same way (confidence levels in the research docs):
- Forza Horizon's "Normal" steering is described as layers that change the input and some that change dynamics. "Simulation" steering removes them, and on a wheel the input layers are always off (FH6 support page, via search snippet, Medium–High).
- Turn 10 presents its more realistic FM 2023 physics as making cars "easier for all of our players to drive" (Medium). A coherent model is easier to read than a hacked one.

## Decision

1. **There is exactly one vehicle simulation.** `VehicleDefinition` describes the physical car. No preset, difficulty or assist may change tire grip, mass, inertia, aero, or any force-model parameter. This is enforced by construction: `AssistConfig` has no access to the definition. It is also guarded by a test (`presets: the vehicle definition is identical under every preset`).
2. **Assists act only through real actuators:** steering angle, throttle/engine torque, per-wheel brake torque, clutch and gear selection. ABS, TCS and ESC are modelled as controllers using only sensor-like inputs: wheel speeds, yaw rate, steering angle, and, for now, a reference speed.
3. **Aids with no real-car equivalent are labelled as such** (e.g. `countersteerAssist`, speed-sensitive steering) and are separate toggles. Their interventions are reported in telemetry.
4. **Every aid is individually configurable.** Presets (`FUN`, `SPORT`, `SIMULATION`) are only named bundles of `AssistConfig` values. A player can run SIMULATION steering with ABS on and TCS/ESC off. This is tested.
5. **Input interpretation is device-aware.** A steering wheel never receives speed-sensitive steering or steering-rate limiting in any preset. The physical wheel *is* the steering angle.
6. **Physical realism of aids.** When enabled in SIMULATION, ABS/TCS/ESC behave like the real systems they model (pressure modulation, torque cut, single-wheel braking), with their real side-effects.

## Explicitly rejected

- Grip multipliers per difficulty.
- Hidden yaw damping or "stability torque" applied directly to the chassis. If ever added for accessibility, it must be its own named, default-off aid that reports its torque. The FH6 description of a yaw-inertia-modifying layer shows this is a design choice some games make. We choose to make it visible.
- Preset-specific vehicle definitions.

## Consequences

- FUN mode cannot raise tire grip. It can only shape the driver's requests (steering range scaled to the slip-angle limit, throttle smoothing, countersteer help) and use real actuators (ESC single-wheel braking, TCS). Where ESC's brake-generated yaw moment helps an understeering car use its rear tires better, the gain is physical and reported (see [driving-presets.md](../architecture/driving-presets.md#measured-differences-v01-reference-sports-rwd-360-hz)).
- Cars still differ in FUN: a FWD hatch and a RWD coupe have different limits, balance and traction.
- We can't claim "Forza-equivalent" feel until we measure it (see [benchmark-plan.md](../benchmarks/benchmark-plan.md) and [forza-telemetry.md](../research/forza-telemetry.md)).
