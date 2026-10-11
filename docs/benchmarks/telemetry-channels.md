# Telemetry channels

Source of truth: [`telemetry.hpp`](../../src/core/include/ord/telemetry.hpp). CSV output (`writeCsvHeader` / `writeCsvRow`) uses the column names below. Units are SI. Axes are ISO 8855: X forward, Y left, Z up.

## Vehicle (per step)

| Column | Unit | Meaning |
|---|---|---|
| `time` | s | simulation time since reset |
| `x,y,z` | m | body origin, world |
| `speed_ms`, `speed_kmh` | m/s, km/h | longitudinal velocity of the CoM (body X) |
| `vy` | m/s | lateral velocity (body Y) |
| `sideslip_rad` | rad | β = atan2(vy, |vx|), 0 below 0.5 m/s |
| `yaw_rate` | rad/s | body-frame Z angular velocity (+ = left) |
| `roll`, `pitch` | rad | small-angle extraction from orientation |
| `ax, ay, az` | m/s² | kinematic CoM acceleration, body frame (gravity *not* included) |
| `kinetic_energy` | J | translational + rotational chassis KE |

## Driver vs actuator

| Column | Unit | Meaning |
|---|---|---|
| `steer_in`, `throttle_in`, `brake_in` | – | raw driver input |
| `steer_angle` | rad | reference road-wheel angle after assists |
| `throttle_out` | 0..1 | throttle after TCS / ESC / smoothing |
| `gear`, `clutch` | –, 0..1 | selected gear (−1 R, 0 N), clutch engagement |
| `engine_rpm`, `engine_torque` | rpm, N·m | crank speed; net crank torque produced |

## Assist interventions

| Column | Unit | Meaning |
|---|---|---|
| `tcs_cut` | 0..1 | fraction of throttle removed by TCS |
| `esc_yaw_error` | rad/s | measured − reference yaw rate |
| `countersteer` | rad | angle added by the countersteer aid |
| `wN_abs` | bool | ABS modulating wheel N |
| `wN_esc` | N·m | ESC brake torque on wheel N |
| `solver_iters` | – | drivetrain active-set iterations this step (diagnostic) |

## Per wheel (`wN_` prefix, N = 0 FL, 1 FR, 2 RL, 3 RR for the reference cars)

| Column | Unit | Meaning |
|---|---|---|
| `contact` | bool | tire touching ground |
| `comp` | m | suspension compression (0 = full droop) |
| `fz` | N | tire normal load |
| `omega` | rad/s | wheel spin |
| `kappa` | – | transient longitudinal slip ratio |
| `alpha` | rad | transient slip angle |
| `fx`, `fy` | N | tire force in the wheel/contact frame |
| `brake` | N·m | commanded brake torque (after ABS/ESC) |

## Mapping to Forza Data Out

See [forza-telemetry.md](../research/forza-telemetry.md) for the field mapping. Note that Forza's slip fields are *normalised* (≥ 1 = beyond grip), whereas ours are physical slip ratio and slip angle. Comparisons must use derived quantities (lateral g, braking distance, yaw rate), not raw slip values.
