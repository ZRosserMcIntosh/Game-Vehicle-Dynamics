# Driving presets and configurable assists

Presets are **named bundles of `AssistConfig` values**. They never touch the
physical car. Source: [`src/core/include/ord/presets.hpp`](../../src/core/include/ord/presets.hpp),
[`assists.hpp`](../../src/core/include/ord/assists.hpp). Rationale:
[ADR-0002](../decisions/one-simulation-many-experiences.md).

## Pipeline

```
DriverInput (device)  ──►  AssistSystem  ──►  ActuatorCommands  ──►  Vehicle physics
 steer/throttle/brake       input shaping         steer angle             (identical for
 clutch/shift/handbrake     steering limit        throttle                 every preset)
                            countersteer aid      per-wheel brake torque
                            transmission          clutch, gear
                            ABS / TCS / ESC
                                 │
                                 └──► AssistReport (every intervention, per step)
```

## Parameter reference

| Parameter | Kind | Meaning | Real-car equivalent? |
|---|---|---|---|
| `steeringMode` | input | `Direct`: input → road-wheel angle, linear. `SpeedSensitive`: full stick = angle for the tightest sustainable radius + target front slip angle | Direct: yes. SpeedSensitive: **no** (gamepad aid) |
| `steeringExponent` | input | Input shaping `|x|^e` (finer control near centre) | no |
| `steerRate`, `steerReturnRate` | input | Max steering input change per second (towards / back to centre) | no (thumbstick smoothing) |
| `targetFrontSlipAngle` | input | Slip angle allowed at the SpeedSensitive limit (≈ tire peak) | no |
| `throttleRate` | input | Max throttle change per second | partly (drive-by-wire maps) |
| `abs`, `absTargetSlip` | electronic | Per-wheel brake-pressure modulation holding slip near the target | **yes** |
| `tractionControl` | electronic | `Off`/`Sport`/`Full`: engine-torque cut on driven-wheel slip (Sport allows more slip) | **yes** |
| `stabilityControl` | electronic | `Off`/`Sport`/`Full`: yaw-rate and sideslip control by braking single wheels plus torque cut | **yes** |
| `countersteerAssist` | aid | 0..1 blend of front wheels toward the direction of travel when the rear slides | **no** (labelled) |
| `shiftMode`, `autoClutch`, `launchRpm` | transmission | Automatic vs manual shifting; automatic clutch vs pedal | yes (automatic / DCT / manual) |

## The three presets (v0.1 values)

| Setting | FUN | SPORT | SIMULATION |
|---|---|---|---|
| Steering, gamepad | SpeedSensitive, exp 1.4, rate 2.5/s (return 4/s), slip target 0.10 rad | SpeedSensitive, exp 1.2, rate 4/s (return 6/s), slip target 0.14 rad | Direct, linear, unfiltered |
| Steering, wheel | Direct (always) | Direct (always) | Direct (always) |
| Throttle smoothing | 3 /s | 6 /s | none |
| ABS | on | on | **off** |
| Traction control | Full | Sport | **off** |
| Stability control | Full | **off** | **off** |
| Countersteer aid (gamepad) | 0.6 | 0.25 | **0** |
| Shifting | automatic | automatic | manual (sequential), auto clutch |
| Manual clutch | — | — | opt-in (`autoClutch = false`) |

Each row is independently overridable, e.g. "SIMULATION + ABS":

```cpp
AssistConfig cfg = makePreset(DrivingPreset::Simulation, InputDevice::SteeringWheel);
cfg.abs = true;   // keep ABS, everything else stays off
```

### Design intent per preset

- **FUN** (default open world): forgiving and believable. Limits what the driver *asks* for (steering range, throttle rate). It never adds grip. Slides are caught by ESC and the countersteer aid, not by fake yaw damping. Cars stay distinct because limits come from their physics.
- **SPORT** (Horizon-inspired sweet spot): realistic traction, more direct steering, no ESC, light countersteer help on a gamepad. Drift-capable: TCS Sport allows 15% slip.
- **SIMULATION**: no aids, no filtering. The driver owns every input. ABS/TCS/ESC exist only if deliberately enabled and then behave like the real systems.

## Measured differences (v0.1, reference sports RWD, 360 Hz)

From [baseline-results.md](../benchmarks/baseline-results.md):

| | FUN | SPORT | SIMULATION |
|---|---|---|---|
| 0-100 km/h | 5.2 s (TCS Full cuts throttle) | 4.7 s | 6.2 s (heavy wheelspin, peak slip ≈ 3.8) |
| 100-0 km/h | 39.7 m (ABS) | 39.7 m (ABS) | 48.2 m (locked wheels) |
| Skidpad R40 | 0.95 g | 1.01 g | 0.98 g |

Assists can't change tire grip, but they *can* change how well the car uses it.
On the RWD car, FUN (0.95 g) sits below SIMULATION (0.98 g). The SPORT/SIMULATION
difference comes from TCS preventing power oversteer during the speed-holding
throttle inputs. On the understeering FWD hatch, FUN reaches 0.89 g vs 0.85 g:
ESC brakes the inside rear wheel, and the extra yaw moment lets the rear axle
contribute more. Real ESC and brake-based torque vectoring do the same, so this is
physically legitimate. It is tracked so that aid tuning never turns into a
hidden handicap for SIMULATION players.

## Planned (roadmap)

- Braking assist (brake-pressure ramp), launch control, anti-stall, auto-blip.
- Optional "stability aid" (yaw-damping torque) as a separate, labelled, default-off aid if playtests demand it (ADR-0002).
- Force-feedback pipeline from the tire aligning torque (`TireForces::mz`), with wheel-specific settings.
- Per-device response curves and dead zones exposed to the player.
- Reference speed estimator from wheel speeds and an IMU, replacing the current ground-truth reference speed used by ABS/TCS.
