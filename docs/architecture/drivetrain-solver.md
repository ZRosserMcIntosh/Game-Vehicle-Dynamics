# Drivetrain and wheel-spin solver

Source: [`drivetrain.hpp`](../../src/core/include/ord/drivetrain.hpp), [`drivetrain.cpp`](../../src/core/src/drivetrain.cpp).

## Why a coupled implicit solve

A wheel's spin is coupled to the ground through the tire's longitudinal slip stiffness. Linearised, the time constant of that coupling is

τ ≈ I_w · |v_x| / (K_x · r²)

For I_w = 1.2 kg·m², K_x = 10⁵ N, r = 0.33 m and v_x = 1 m/s, τ ≈ 0.1 ms, far below any affordable step (2.8 ms at 360 Hz). Explicit integration of that coupling is unstable at low speed. The usual fixes are heavy substepping or clamping slip, and both change the physics. The engine, clutch and differentials add further stiff couplings between shafts. KinetiForge exchanges these torques sequentially, one substep apart, and its history shows repeated energy-accumulation fixes ([kinetiforge-analysis.md](../research/kinetiforge-analysis.md) §3).

We instead solve **all rotating shafts, all longitudinal slip states and all drivetrain constraints together, implicitly, each step**.

## Unknowns

- ω_e engine crank, ω_g gearbox input, ω_i for each wheel (angular velocities at the end of the step),
- κ'_i transient longitudinal slip of each wheel,
- λ multipliers of the active constraint rows.

## Equations (one step, backward Euler)

**Shaft i** (inertia I_i):

I_i (ω_i − ω_i⁰)/Δt = T_i^ext − r_i F_x,i + Σ_c J_ci λ_c

with the tire force linearised about the current slip:

F_x,i ≈ F_x(κ'_i⁰) + K_i (κ'_i − κ'_i⁰) + c_i (r_i ω_i − v_x,i),   K_i = max(0, ∂F_x/∂κ)

The c_i term is low-speed carcass damping. It fades out above ~3 m/s.

**Transient slip** (relaxation length σ, Pacejka 2012 ch. 7):

σ (κ'_i − κ'_i⁰)/Δt + |v_x,i| κ'_i = r_i ω_i − v_x,i

In steady state κ' equals the practical slip ratio (r ω − v_x)/|v_x|. At v_x = 0 the equation integrates slip displacement, which gives the tire its standstill stiffness. There is no singularity at zero speed.

**Equality rows** (always active when a gear is engaged):

ω_g − Σ_a s_a · G · R_a · ½(ω_a,L + ω_a,R) = 0

where G is the gear ratio, R_a the final drive of axle a, and s_a the open-centre torque share (Σ s_a = 1). The multiplier of this row automatically gives an **exact equal torque split** across each open differential and the s_a split across axles.

**Friction rows** (clutch, each brake, each LSD / locked axle diff, centre coupling), each with Jacobian row j and torque capacity C ≥ 0:
- *stuck*: constraint j·ω = 0, multiplier λ with |λ| ≤ C,
- *slipping*: known generalised force f = −C·sign(j·ω⁰) applied as jᵀf.

| Row | j | Capacity C |
|---|---|---|
| Clutch | e_engine − e_gearbox | engagement × max torque |
| Brake (wheel i) | e_i (relative to chassis) | commanded brake torque (after ABS/ESC) |
| Clutch-pack LSD | e_L − e_R | preload + ramp(power/coast) × axle input torque |
| Locked diff | e_L − e_R | ∞ (10⁹) |
| Centre LSD / locked | ½(e_FL + e_FR) − ½(e_RL + e_RR) | preload + ramp × input, or ∞ |

## Active-set iteration (stick/slip)

1. Start from last step's stick/slip states. A slipping row whose relative speed is ≈ 0 starts stuck.
2. Assemble and solve the dense system (Gaussian elimination with partial pivoting, fixed capacity, no allocation).
3. A stuck row with |λ| > C becomes slipping with f = clamp(λ, ±C). A slipping row whose post-solve relative velocity has the same sign as f (friction would reverse the motion) becomes stuck.
4. Repeat until no change (≤ 12 iterations; typically 1–2).

Stuck rows get a tiny compliance (10⁻⁷ rad/s per N·m) on the diagonal, so redundant locks stay well-posed. Example: locked diff plus both brakes locked.

## Properties verified by tests (`tests/test_drivetrain.cpp`)

- Open diff: equal torque, so a 4× heavier wheel accelerates 4× slower. Engine speed = ratio × mean wheel speed.
- Locked diff: equal wheel speeds.
- Lossless energy check: engine work = Σ ½Iω² within 1% (backward Euler dissipates slightly).
- Brakes hold a wheel exactly at zero against sub-capacity drive torque.
- The clutch slips above capacity and locks when the speeds meet.
- A linear tire with K_x = 10⁵ N, near standstill, at **60 Hz**, converges to the analytic steady force (3000 N) without oscillation.

## Limitations and next steps

- The engine torque is evaluated explicitly at ω⁰. That's fine for smooth curves; a limiter cut is a discontinuity, handled with hysteresis.
- Lumped gearbox efficiency is applied to positive engine torque only.
- LSD capacity uses the previous step's input torque (one-step lag on the capacity only, not on the dynamics).
- No engine stall/start model; the crank speed is clamped ≥ 0.
- Planned: torque converter (automatic), viscous coupling (speed-dependent capacity row), electric motors (torque-speed envelope on a shaft), hybrid coupling, torque vectoring.
