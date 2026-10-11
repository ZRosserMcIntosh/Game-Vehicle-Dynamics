# KinetiForge Vehicle System: source-code audit

> **Subject.** KinetiForge Vehicle System, an Unreal Engine 5 vehicle plugin by Zhengyi Miao. <https://github.com/myoozy/KinetiForge-Vehicle-System>
>
> **Revision audited.** `df4fbf21056461f5926fc376aeaebfbc9b903b4b` (HEAD, 2026-08-08, "Fix: Jittering of high speed steering scale at high tick rate. (github issue #13)"). The local clone is shallow and has 54 commits visible. 52 are by `myoozy` and 2 by `M1AO`.
>
> **License.** MIT. `LICENSE` reads "Copyright (c) 2025 Zhengyi Miao (github.com/myoozy)", while source headers read "Copyright (c) 2026 Zhengyi Miao". The bundled `Source/AsyncTickPhysics` module is a separate MIT work, "Copyright (c) 2022 Alex Craig", whose notice is in `Source/AsyncTickPhysics/README.md`.
>
> **Audit date.** 2026-10-10.
>
> **Method.** This was a static read of the C++ source only: about 17.5k lines in `Source/KinetiForge` and `Source/AsyncTickPhysics`, plus `KinetiForge.uplugin`, the `*.Build.cs` files, `Config/DefaultEngine.ini`, `README.md`, `git log`, and selected commit diffs. **We did not build, run or profile KinetiForge.** No Unreal Engine is installed in this environment. Every behavioural statement below comes from reading code. Items marked **(suspected)** are inferences we could not confirm without running the code. Line numbers refer to the audited commit. Code excerpts are kept short and quoted only to support analysis.
>
> **Purpose.** This audit informs the OpenRoad Dynamics build/reuse decision: (A) depend on KinetiForge, (B) incorporate selected MIT parts with attribution, (C) implement independently from published math, or (D) a hybrid with interchangeable implementations. Each option is compared against UE5 Chaos Vehicles.

---

## Executive summary

KinetiForge is an ambitious, actively maintained, single-author UE5 plugin. Its suspension kinematics are richer than Chaos Vehicles': real-time MacPherson and double-wishbone linkage solves, solid axles, and instant-centre anti-dive/anti-squat/anti-roll. It also has a detailed engine (turbo, anti-lag, stall and push-start, backfire) and a pragmatic tire force scheme that stays stable at low speed. Several ideas are worth studying. However, it is **not a suitable foundation** for OpenRoad Dynamics. The main reasons follow.

- **The tire is not a Pacejka model.** The longitudinal and lateral curves are artist-drawn `UCurveFloat`s, baked into 64-sample linear look-up tables. The final force is the force needed to cancel slip velocity in one step, clamped by the curve value (`VehicleWheelSolver.cpp:574-612, 670-772`). This is stable and game-friendly, but cornering stiffness is not a true parameter. The lateral table spans 0-90° with 64 samples, about 1.4° per sample, which is coarse exactly where high-speed handling lives.
- **The drivetrain is not implicitly integrated as a whole.** The README says "1D physics, implicit integration". In the code, only the clutch spring-damper torque and the tire relaxation are backward-Euler. Engine, clutch, gearbox, differentials and wheels exchange torques in a **staggered, explicit, one-substep-lagged** pass (block Gauss-Seidel, one iteration) at a default 240 Hz substep (`VehicleDriveAssemblyComponent.cpp:742-895`). The commit history shows repeated "energy accumulation" fixes, which are symptoms of this partitioning.
- **ESC/stability control is absent.** Only a per-wheel `SetEspBrakeTorque` hook exists (`VehicleWheelComponent.h:430`), and the README says "ESP (not yet implemented)". ABS is a per-wheel proportional slip limiter. TCS is a per-axle proportional torque cut using average slip. `VehicleADASComponent` is unrelated to ABS/TCS: it is AI driving helpers (cruise control, autopilot, path following).
- **Threading is unsafe by design.** Physics runs inside a Chaos sim-callback on the physics thread and directly mutates and reads `UObject` component state with no synchronization (`AsyncTickCallback.cpp:25,34`). The author acknowledges this in `VehicleAsyncTickComponent.h:48`: "runs on worker thread (not safe but much better performance)". The game thread concurrently reads the same state for animation, auto-gearbox and timers.
- **Scaling is limited.** A 4-wheel car is about 27 `UActorComponent`s, about 8 of which tick on the game thread. All vehicles run serially in a single physics callback. There is no LOD, no sleeping and no batching. For hundreds of traffic cars this would need a rewrite.
- **Multiplayer has concrete bugs.** `ServerStartVehicleEngine`/`ServerShutVehicleEngine` are declared `NetMulticast`, not `Server` (`VehicleDriveAssemblyComponent.h:134-141`). `ServerVehicleEngineOperationMode` is `ReplicatedUsing` but never registered with `DOREPLIFETIME` (`.cpp:124-129`). Every axis input is a *reliable* RPC with no change detection. There is no prediction or resimulation integration, and gear shifts use game-thread timers.
- **The math cannot be lifted out cleanly.** It is written against UE types: `FVector3f`, `FMath`, `FRichCurve`, `UCurveFloat`, `FHitResult`, `UWorld` traces and Chaos particle handles. It also mixes cm and m units and has no tests and no CI.
- **UE version.** All 333 `.uasset` files were saved with **UE 5.3** (`++UE5+Release-5.3`). There are no engine-version guards in code. Compatibility with 5.4-5.7 is unverified.

**Recommendation: option D, implemented mostly as C.** Build an engine-independent core from published math, with a pluggable tire-model interface. Use KinetiForge only as a design reference and as a source of a few small, well-understood algorithms. These should be either re-derived, or copied with the MIT notice preserved where verbatim reuse saves real effort: suspension linkage solvers, the contact-point effective mass, the implicit clutch spring-damper formula, the LUT "artist tire" with similarity-style combined slip as a FUN-preset model, the constraint-capped low-speed tire regularizer, and the AI speed-planning helpers. Do **not** depend on KinetiForge (A). Do not copy `ComputeSprungMasses` without first checking its provenance; see section 15.

---

## Repository map (what was read)

| Area | Files (lines) | Role |
|---|---|---|
| Tire | `VehicleWheelSolver.{h,cpp}` (144+771), `VehicleWheelStructs.h` (355) | Slip, relaxation, combined slip, ABS, wheel spin |
| Suspension | `VehicleSuspensionSolver.{h,cpp}` (401+2310), `VehicleSuspensionStructs.h` (433) | Traces, kinematics, unsprung mass, spring/damper, anti-geometry |
| Wheel glue | `VehicleWheelComponent.{h,cpp}` (614+1317) | UObject wrapper, force application, animation |
| Axle / diff | `VehicleAxleAssemblyComponent.{h,cpp}` (323+982), `VehicleDifferentialComponent.{h,cpp}` (108+300), `VehicleAxleStructs.h` | Steering, Ackermann, TCS, LSD, transfer case, solid axle |
| Powertrain | `VehicleEngineComponent`, `VehicleClutchComponent`, `VehicleGearboxComponent`, `VehicleDrivetrainStructs.h` | Engine, turbo, clutch models, gearbox |
| Orchestration | `VehicleDriveAssemblyComponent.{h,cpp}` (348+1567), `VehicleAsyncTickComponent`, `VehicleWheelCoordinatorComponent` | Substep loop, input, replication, sprung mass, aero dispatch |
| Aero | `VehicleAeroBaseComponent`, `VehicleAirfoilComponent`, `VehicleAeroVolumeComponent` | Airfoil CL/CD, quadratic drag/damping |
| AI helpers | `VehicleADASComponent.{h,cpp}` (251+1578) | Cruise control, autopilot, path following |
| Async bridge | `Source/AsyncTickPhysics/*` (Alex Craig, MIT 2022, with KinetiForge additions) | Chaos `TSimCallbackObject` invoking components on the physics thread |
| Utilities | `VehicleUtilities.{h,cpp}` | `FVehicleLUT`, `SafeDivide`, chassis state fetch |

---

## 1. Tire model

### 1.1 Not Pacejka: curve look-up tables with a constraint cap

The README advertises a "slip-friction curve (Pacejka-style)" and "combined slip behavior similar to TMeasy". In code, the tire has no Magic Formula coefficients.

- `FVehicleTireConfig::Fx` and `Fy` are `UCurveFloat*` assets (`VehicleWheelStructs.h:107-119`). Fx is a function of slip ratio and Fy of slip angle in degrees, and both output a friction coefficient.
- They are baked into `FVehicleTireLUT<64>` look-up tables (`VehicleWheelStructs.h:167-275`). Fx is sampled over slip ratio [0, 1]. Fy is sampled over [0°, 90°] (`VehicleWheelSolver.cpp:239-265`). Lookup is linear interpolation (`FVehicleLUT::FastEval`, `VehicleUtilities.h:391-423`).
- At bake time the "optimal slip" is found by the maximum-deviation-from-secant method. "Linear stiffness" is the maximum finite-difference slope before that index (`VehicleWheelStructs.h:221-263`). Negative samples are clamped to zero (`:207-210`).

The force itself is built in two steps (`SolveTireForce`, `VehicleWheelSolver.cpp:670-772`).

1. **Constraint force.** This is the force that would cancel the contact slip velocity in one step.
   - Lateral (`:603-612`): `F_y,c = v_slip,y · m_eff,y / Δt_macro`.
   - Longitudinal (`:574-601`): the drive force plus a brake force (clamped to ±T_b/R), plus the "ground interaction" force needed to bring the wheel to pure rolling in one substep:

     ```
     TorqueFromGroundInteraction = (1/Δt_sub) · I_eff · (ω − Vx/R)
     ForceRequiredToBringToStop  = −(Vx·m_eff,x/Δt_macro + T_d/R + F_ground)
     ConstraintForce = T_d/R + clamp(ForceRequiredToBringToStop, ±T_b/R) + F_ground
     ```
2. **Cap.** Each component is clamped to the curve-derived limit `MaxFx·AvailableGrip·LUT(slip)`. It is zeroed if the transient slip and the constraint force disagree in sign (`:739-752`):

   ```
   MFTireForce.X = TransientSlip.X * ConstraintTireForce.X > 0.f ?
       FMath::Clamp(ConstraintTireForce.X, -Fx, Fx) : 0.f;
   ```

In practice, at any meaningful slip the constraint force (`m_eff·v/Δt`, very large) exceeds the cap, so **force ≈ curve value × load**. Near zero slip the constraint term dominates and behaves like stiction. This makes the tire robust at standstill and on slopes. The trade-off is that the force is not proportional to slip times cornering stiffness in the linear region. It follows a piecewise-linear 64-sample curve, rate-limited by a velocity constraint that depends on Δt. The effective "stiffness" therefore depends on time step and mass, not on tire data.

A curve-capped constraint with a sign gate is a known game technique: the "friction-clamped velocity constraint" also used in impulse-based solvers. It is legitimate for FUN/SPORT feel, but it is not a physically parameterized tire (contrast with Pacejka MF 5.2/6.1, TMeasy or Fiala).

### 1.2 Combined slip

The combined slip uses a normalized-slip ("similarity") approach (`VehicleWheelSolver.cpp:704-752`).

```
NormalizedSlip = |(κ, α_n)| ⊙ (Cx, Cy)          // α_n = slip angle / 90°
σ = |NormalizedSlip|
dir = normalize(NormalizedSlip ⊙ ((1−bias)/MaxFx, bias/MaxFy))
Fx = MaxFx·Grip·lerp(LUTx(|κ|), LUTx(σ/Cx)·dir.x, LatToLongInterference)
```

This resembles TMeasy-style combined slip, as the README claims. Two blend factors, `LongitudinalToLateralInterference` and `LateralToLongitudinalInterference` (`VehicleWheelStructs.h:80-93`), interpolate between pure-slip and coupled behaviour, which is useful as an assist dial. `SlipInputScale = FrictionMultiplier / DynFrictionMultiplier` (`:726`) keeps the curve's linear slope independent of surface friction. If only one curve exists, a friction ellipse clip is applied (`:753-769`).

### 1.3 Load sensitivity

`CalculateAvailableGrip` (`VehicleWheelSolver.cpp:656-668`):

```
Fz0 = StaticSprungMass · 9.81          // hard-coded g, ignores world gravity
r   = Fz / Fz0
b   = (1 − s) / (2 + 2s)                // s = WheelLoadInfluenceFactor
Grip = μ · Fz0 · r / (1 + b·r)
```

This is a saturating, degressive load curve, which is qualitatively correct. Three issues stand out.

- The tooltip says s = 0 means "wheel load will be constant" (`VehicleWheelStructs.h:64-69`). The formula with s = 0 gives `r/(1+0.5r)`, which is not constant. The documentation and the code disagree.
- At nominal load (r = 1) grip is `1/(1+b)` < 1 unless s = 1, so it is not normalized at Fz0.
- If the sprung mass was never set, because the coordinator was missing (logged at `VehicleWheelComponent.cpp:126`), then Fz0 = 0, `SafeDivide` returns 0, and **grip is zero**.

### 1.4 Low-speed handling

- The slip ratio used for ABS/TCS is denominator-floored at `max(|Vx|, |ωR|, 1 m/s)` (`VehicleWheelSolver.cpp:462`, and `:287` for the predicted value).
- The slip angle blends a small-angle approximation with `asin(−v̂y)` for |v|² between 0.01 and 0.1 m²/s² (`:445-452`).
- The transient-slip relaxation uses a floor `MinVx = 0.1 m/s` (`:555-556`).
- Wheel-spin integration prevents friction from flipping the sign of the relative slip, and detects lock by zero-crossing (`WheelAcceleration`, `:354-406`, lock at `:401`).
- Slope holding: `CalculateGravityCompensationOnSlope` adds `LatForceDir.Z·Fz` laterally, and longitudinally when locked, clamped by brake force (`:614-654`).

The low-speed behaviour is deliberately engineered and looks robust. The constraint-capped force is the main reason.

### 1.5 Relaxation length and transient behaviour

The relaxation uses implicit (backward-Euler) integration of a first-order relaxation ODE (`UpdateTransientSlip`, `:535-572`):

```
s_{n+1} = (s_n + v_slip·Δt/L) / (1 + |Vx|·Δt/L)
```

This solves dσ/dt = (v_slip − |Vx|σ)/L, the classic single-contact-point relaxation model (Pacejka, *Tire and Vehicle Dynamics*, ch. 7). It is unconditionally stable, and it is textbook math rather than a novel algorithm. L is a constant `FVector2f(0.1, 0.2)` m (`VehicleWheelStructs.h:61-62`). It does not depend on load or speed. At 111 m/s with Δt = 1/240 s, the time constant L/V of about 1.8 ms is shorter than a substep, so transients effectively disappear at high speed. That is physically plausible.

### 1.6 Friction surfaces

- Surface μ comes from `UPhysicalMaterial::Friction` (`VehicleSuspensionSolver.cpp:1203-1215`). It is combined with the tire's `FrictionMultiplier` by a `Constant/Average/Multiply/Min/Max` mode (`VehicleWheelSolver.cpp:315-342`).
- There are no per-surface curves, wet/rain model, water depth, rolling resistance per surface, surface roughness or deformable-terrain behaviour. "Rolling resistance" is a constant brake torque per wheel (`VehicleWheelStructs.h:36-40`, applied at `VehicleWheelSolver.cpp:115`).
- **Camber thrust** is new since the README's "No camber thrust" was written. It is modelled as a lateral contact-patch drift `q = dy/dx` taken from a curve and capped at |tan γ| (`:466-533`, `:408-429`). The README's "Known Limitations" section is therefore stale.

### 1.7 Assessment

| Aspect | Verdict |
|---|---|
| Physical fidelity | Low to medium. Artist curves, constraint-capped, Δt-dependent near zero slip |
| Robustness | High at low speed and at standstill |
| High-speed resolution | Weak. The Fy LUT step is about 1.43°, linearly interpolated, with only about 3-5 samples below a typical road-tire peak (4-8°) (**suspected** feel impact) |
| Reuse value | Medium, as a FUN/arcade tire option and a low-speed regularizer |

---

## 2. Suspension geometry

### 2.1 Types

`EVehicleIndependentSuspensionType` (`VehicleSuspensionStructs.h:10-15`) has three values.

- **StraightLine** (fork-style): `ComputeStraightSuspension`, `VehicleSuspensionSolver.cpp:1571-1604`.
- **MacPherson**: one lower arm plus a strut. The steer axis is simplified to the strut axis (`:1606-1644`, comment at `:1619`).
- **DoubleWishbone**: lower arm, upper arm and knuckle length (`:1707-1766`).

Solid axles are supported only through the axle assembly (`EVehicleAxleSuspensionType::Solid`, `VehicleAxleAssemblyComponent.cpp:222-273`).

### 2.2 Kinematics

- **Lower arm.** The ball joint is found from an exact circle-sphere intersection: the arm circle about the pivot axis against a sphere of radius equal to the strut length from the top mount. The solver picks the root closest to the previous frame, and clamps when there is no intersection (`SolveLowerWishbone`, `:1482-1569`).
- **Upper arm.** A circle-circle construction in the plane normal to the upper pivot axis, constrained by knuckle length, degrades gracefully when unsolvable (`SolveUpperWishbone`, `:1646-1705`).
- **Solid axle.** A Newton-Raphson solve on (axle centre z, roll angle), 5 iterations, with an analytic Jacobian (`SolveSolidAxlePosture`, `:845-927`). There is a harmless typo at `:863-864` (`GuessDeltaY = L.Y − L.Y`), where the variables are unused.
- **Wheel orientation.** Hub rotation is `Steering · SteerAxisBias · LUT(camber/toe/caster) · StaticSpindle` (`:1631, :1753`). Geometric camber change therefore emerges from steer-axis inclination, plus optional camber/toe/caster curves against compression ratio (`GetCamberToeCasterFromLUTs`, `:767-789`; `FVehicleLUT<16>`).
- **Missing.** There is no tie-rod or bump-steer linkage, no compliance or bushings, and no kingpin offset or scrub radius beyond what the hub offset implies.
- **Suspected inconsistency.** `GetLowerWishboneState`/`GetUpperWishboneState` (`:811-843`) do not mirror the mount Y by side sign, but `SolveLowerWishbone` does (`:1487-1492`). These getters feed animation and debug drawing only (`VehicleWheelComponent.cpp:1109,1124`; `VehicleSuspensionSolver.cpp:439-440`).

### 2.3 Contact detection

Five trace modes are available (`VehicleSuspensionStructs.h:18-25`; implementations at `VehicleSuspensionSolver.cpp:1218-1417`). All are single queries via `FPhysicsInterface::RaycastSingle`/`GeomSweepSingle` against `UWorld`.

- **LineTrace.** Starts at the *outer* side of the wheel (`:1229-1232`), so it is a single contact point.
- **SphereTrace.** Uses a wheel-radius sphere, with a second, narrower sphere if the hit is outside the tire width.
- **BoxTrace.** A box rotated 45°.
- **SphereTraceNoRefinement.**
- **MultiSphereTrace.** Sphere, then box, then `GeomSweepMulti`. This allocates a `TArray<FHitResult>` per call (`:1385`).

When the hit distance is below the radius (bottomed out), the code falls back to a line trace (`:1472-1479`). There is one query (or a few) per wheel per physics step, with no multi-point patch and no tire-carcass envelope. At 400 km/h and 120 Hz the car travels about 0.93 m per step. Potholes and cobblestones shorter than that are aliased, because the strut limit is linearly interpolated between successive trace results (`:1137-1138`).

### 2.4 Spring, damper and unsprung mass

**Optional unsprung mass.** This is active if `SuspensionAndBrakeMass > 0`. It integrates strut length with an internal fixed 1/480 s substep (`UpdateStrutLength`, `:1043-1201`). The integration is semi-implicit, with implicit damping and stiffness terms in the denominator:

```
v = (v + F_static/m·dt) / (1 + c·dt/m + k·dt²/m)
```

`F_static` includes spring, preload, swaybar, gravity and a fictitious force from chassis acceleration. Ground contact is a **hard kinematic limit** (`:1171-1184`): there is no tire radial stiffness or damping. Inputs are therefore transmitted rigidly. This is harsh on cobblestones (**suspected**). The README says unsprung mass "requires 120Hz Async Physics".

**Force to chassis.** The force is applied along the impact normal (`ComputeSuspensionForce`, `:2207-2310`), with stability clamps `k ≤ 4·m_eff/Δt²` and `c ≤ m_eff/Δt` (`:2238-2255`).

- **(suspected) unit mismatch.** The spring config appears to be in N/cm: critical damping uses `sqrt(k·100/m)` (`:2197`), and compression is in cm. The clamps are computed in SI per metre, which would make them about 100× more lenient than intended, effectively inactive.
- **Bump stop and preload.** A bump stop with a second spring is used along the normal (`:2290-2302`). Preload is capped by the force needed to hold the car (`:2281-2287`).

**Contact-point effective mass.** This uses the standard rigid-body formula `1/(1/m + (r×d)·I⁻¹(r×d))` along the longitudinal, lateral and normal directions (`CalculatePointEffectiveMass3D`, `:2106-2189`). The longitudinal and lateral masses are then scaled by the wheel's load fraction (`:2304-2309`). This partitions chassis mass across wheels so that independent per-wheel constraints do not overshoot together, which is a cheap Jacobi-like split.

### 2.5 Anti-roll bars

The anti-roll bar is a simple linear coupling on hub Z difference between the two wheels of an axle (`:2262-2266`, and inside unsprung integration at `:1144-1147`). The stiffness is `AxleConfig.SwaybarStiffness` (default 200, presumably N/cm; `VehicleAxleStructs.h:47`). The other wheel's hub Z is read from the *previous* step (`VehicleAxleAssemblyComponent.cpp:349-361`), so the coupling is explicit.

### 2.6 Anti-dive, anti-squat and anti-roll geometry

- **Geometric terms.** These come from 2D instant centres computed from arm velocity normals (`SolveSwingArmSlope2D`, `:1827-1881`; the MacPherson and double-wishbone variants are at `:1883-1956`).
- **Curve terms.** These are added from `AntiDiveCurve`/`AntiSquatCurve`/`AntiRollCurve` (`ComputeAntiPitchRollGeometry`, `:1958-2069`).
- The result is a **jacking force** added to the normal force:

  ```
  JackingForceChassisZ = (TireForceChassis.X * FinalPitchSlope) + (TireForceChassis.Y * FinalRollSlope);
  ```
- **(suspected) instability.** The curve term is converted to a slope with `SafeDivide(TrueCOMHeight, DynamicLx)` (`:2059-2060`). `SafeDivide` only guards |x| < 1e-8. A wheel whose contact patch is longitudinally close to the CoM, such as the middle axle of a 6×6 or a short-wheelbase vehicle, gets a very large slope and jacking force.

### 2.7 Assessment

This is KinetiForge's strongest area. The linkage solvers are compact, closed-form or Newton-based, and well commented, and the anti-geometry idea is good. They depend heavily on `FVector3f`/`FQuat4f`/`FTransform3f` and on context structs that contain `FHitResult`, but the math itself is portable.

---

## 3. Drivetrain simulation and its integration scheme

### 3.1 What the solver actually does

`UVehicleDriveAssemblyComponent::UpdatePhysics` (`VehicleDriveAssemblyComponent.cpp:742-895`) runs once per physics step Δt (default config is 120 Hz async). It splits the step into `N = ceil(Δt / SubstepDeltaTime)` substeps, with `SubstepDeltaTime = 1/240 s` by default (`VehicleDriveAssemblyComponent.h:88`). The flag `bDrivetrainSubstepping` (`.h:86`) is never read. It is dead configuration.

**PreStep, once per Δt.** For each axle, update steering, run the suspension (trace, kinematics, forces), and call wheel `PreStep`, which freezes contact frame, contact velocity, load and grip for the step (`VehicleAxleAssemblyComponent.cpp:317-405`; `VehicleWheelSolver.cpp:22-86`).

**Substep, N times, each of length h.**

1. Read the clutch torque `T_c` **computed at the end of the previous substep** (`:797`).
2. Engine: `ω_e += h/J_e·(T_ind − T_c + T_starter)`. Then apply friction with a sign-preserving zero-cross clamp (`VehicleEngineComponent.cpp:198-208`).
3. Gearbox: `T_out = (T_c + T_P2)·i_g·η` (`VehicleGearboxComponent.cpp:304-311`).
4. Transfer case: `T_k = i_tc·T_out·w_k + λ·J_k(ω̄ − ω_k)/h`. Here ω̄ is the momentum-weighted mean of the drive axles (`VehicleDifferentialComponent.cpp:62-171`).
5. Axle: TCS scaling, then the differential, `T_L,R = ½·i_d·T ∓ λ·J_red(ω_L − ω_R)/h` (`VehicleDifferentialComponent.cpp:260-289`).
6. Wheel `Substep`: compute tire force from ω at the start of the substep, then explicit-Euler ω with clamps (`VehicleWheelSolver.cpp:88-139, 354-406`).
7. Upward pass:
   - `ω_axle = ½·i_d(ω_L + ω_R)`.
   - Gearbox input `ω = i_g·i_tc·mean(ω_axle)`.
   - Reflected inertia `J = (ΣJ_wheel/i_d² + J_shaft)/i_tc²/i_g²`.
   - Shaft stiffness is reflected the same way (`VehicleGearboxComponent.cpp:313-328`).
8. Clutch: compute the next `T_c` using one of three models (`VehicleClutchComponent.cpp:30-113`):
   - **ConstraintLock**: `clamp(Δω·J_red/h, ±cap)`. This is the velocity-matching torque for a one-step lock.
   - **FrictionClutch**: an implicit spring-damper with lumped series stiffness from clutch and driveshaft:

     ```
     T = (K·θ + (K·h + D)·Δω·lock) / (1 + (K·h + D)·h / J_red)
     ```

     This is exactly the backward-Euler solution of `T_{n+1} = K(θ + Δω_{n+1}h) + DΔω_{n+1}` with `Δω_{n+1} = Δω_n − T_{n+1}h/J_red`. It is a nice, reusable closed form.
   - **FluidCoupling**: `T = D·Δω / (1 + D·h/J_red)`.

**PostStep.** Average the tire impulses over the substeps into one force. Apply tire plus suspension force to the chassis as **one impulse per Δt** (`VehicleWheelComponent.cpp:229-289`, `:266-272`).

### 3.2 Characterization

- **Partitioned, staggered and explicit.** Each block (engine, clutch, gearbox, diffs, wheels) is advanced with torques from the other blocks that are either one substep old or computed in the same pass. This is a single block Gauss-Seidel iteration, not an implicit monolithic solve. Action and reaction are consistent, since the engine and gearbox see the same `T_c`, so momentum exchange is conservative. The torque is lagged by h.
- **Lock torques assume a free-spinning wheel side.** `J_red` on the gearbox side includes only wheel and driveshaft inertia, not the vehicle's translational inertia reflected through gripping tires (about m·R²). Lock torques are therefore computed as if the wheels were in the air. The tire constraint then pulls in the opposite direction, which is a typical source of oscillation and energy drift. The history shows this: `a520005` ("too much energy was accumulating when the clutch was partially engaged"), `c95397b` and `0a8242b` ("energy conservation ... limited-slip differential"), and `021621e` ("Refactor clutch and driveshaft compliance").
- **The chassis is frozen during substeps.** Contact velocity Vx is fixed in `PreStep`. Substepping refines wheel spin and drivetrain against a fixed chassis velocity. Tire-chassis coupling is resolved only at physics-step rate.
- **Driveshaft compliance is lumped.** It appears only inside the clutch spring model. There is no per-axle twist state.
- **LSD "lock ratio" depends on Δt.** λ removes a fixed fraction of Δω *per substep*. The same λ therefore locks harder per second at higher substep rates (**suspected**; this follows directly from the formula).

### 3.3 Verdict

The drivetrain is a workable game drivetrain, but the "implicit integration" claim applies only to local sub-models. For OpenRoad, which needs SIMULATION fidelity, a stiff locked driveline at 300+ km/h and multiplayer consistency, a small monolithic implicit solve is the right design. Projected Gauss-Seidel with several iterations, or a direct solve of the 1D drivetrain linear system including tire linearization, would replace this.

---

## 4. Engine, gearbox, clutch and differential components

### 4.1 Engine (`VehicleEngineComponent.cpp`)

- **Torque.** Indicated torque is `throttle·(friction + T_max·LUT(rpm))`, using a 32-sample LUT over [idle, max] rpm (`:62-71`; `VehicleEngineComponent.h:52`).
- **Friction and engine braking.** Friction is `StartFriction + k·rpm`, and pumping loss is `k_p·rpm·(1 − throttle)` (`:54-56`). This gives physically motivated engine braking that depends on throttle.
- **Turbo.** Spool is interpolated toward an rpm × throttle target. Boost pressure follows spool² with blow-off and anti-lag logic (`:77-191`). These are phenomenological `FInterpTo` lags, not a compressor map.
- **States.** The engine has starter, stall, push-start and reverse-rotation states (`:450-533`). The idle controller uses `EstimateThrottleForTorque`, which solves a quadratic (`:312-365`). The rev limiter cuts spark with a timer (`:458-464`).
- **Exhaust and backfire.** These are cosmetic, and use `FMath::FRand()` on the physics thread (`:297`). That is non-deterministic, uses shared RNG state, and is not thread-safe.
- **Bug.** `TickComponent` removes delegates from `TurboBlowOffCallbacks`/`BackfiringCallbacks` *while range-iterating them* (`:389-399, :405-415`). UE asserts on this ("Array has changed during ranged-for iteration") in builds with checks enabled.

### 4.2 Gearbox (`VehicleGearboxComponent.cpp`)

- **Ratios are not a table.** Ratios are generated from `FirstGear`, `TopGear` and `GearRatioBias` by power interpolation (`:366-426`). Reverse ratios mirror the forward ones (`:420-423`). A real gearbox (for example a specific Brazilian hatchback) cannot be entered directly.
- **Shifting.** During a shift the ratio is 0 (neutral) for `ShiftDelay`, implemented with a **game-thread `FTimerManager`** timer (`:251-266`) while the physics thread reads `CurrentGearRatio` (**race**). There is rev-match blip, spark cut for sequential upshifts, an automatic clutch, and an "arcade" N/D/R auto-selection (`VehicleDriveAssemblyComponent.cpp:154-202, 318-478`).
- **Automatic gearbox.** Evaluated on the game thread every `AutomaticGearboxRefreshTime` (0.5 s), and only when locally controlled (`:320-321`). It dereferences `Cast<APawn>(GetOwner())` without a null check, so a non-Pawn owner such as a traffic actor would crash.
- **Missing.** No torque converter (the closest is the viscous "FluidCoupling" clutch), no CVT, no dual-clutch overlap.

### 4.3 Clutch

See section 3.1. There are three models. Capacity is `EngineMaxTorque·Capacity` (`VehicleClutchComponent.cpp:143`). Lock is `1 − pedal` and is forced to 0 in neutral (`:145`).

### 4.4 Differential (`VehicleDifferentialComponent.cpp`)

- **One type only:** a "lock-ratio" LSD with separate drive and coast ratios (`VehicleDrivetrainStructs.h`, `FVehicleLimitedSlipDifferentialConfig`, lines 419-429). λ = 0 is open and λ = 1 is velocity-locked.
- **No torque limit.** There is no preload, ramp angles, clutch-pack friction torque limit, Torsen torque-bias ratio, or viscous coupling. The transfer torque is a fraction of the velocity-equalizing torque with **no torque cap** (`:278-287`).
- **Transfer case.** The same class doubles as the transfer case or centre differential, with momentum-weighted locking across N axles (`:62-171`).
- **Hybrid hooks.** P1, P2, P3 and P4 electric-motor torque injection points exist (`VehicleEngineComponent.cpp:196`, `VehicleGearboxComponent.cpp:309`, `VehicleAxleAssemblyComponent.cpp:449`, `VehicleWheelSolver.cpp:109`).

---

## 5. ABS and traction control

`VehicleADASComponent` **is not ABS/TCS.** It is a set of static helpers for AI driving: `UpdateCruiseControl`, `SpeedUpVehicle`, `SlowDownVehicle`, `UpdateDirectDrive`, `UpdateAutoPilotSimple/NavMesh/HumanLike`, `GetMaxSpeedToBrake`, `GetMaxSpeedToTurn` and `FixPathCollisions` (`VehicleADASComponent.h:46-246`). Its own timer tick is compiled out (`#if 0`, `VehicleADASComponent.cpp:40-53`). `UpdateAutoPilotHumanLike` is documented as "Do not call ... still under developing" (`.h:226-228`).

### 5.1 ABS (per wheel, inside the tire solver)

The code is `PredictSlipAndUpdateABS`, `VehicleWheelSolver.cpp:276-313`.

- It predicts ω from the last angular acceleration and computes a predicted slip ratio.
- It triggers if all of these hold: enabled, braking, on the ground, |Vx| > `ActivationSpeed` (5 m/s), |κ| > `OptimalSlip` (0.1), and the slip sign is opposite to the velocity sign.
- Action: `T_b ← T_b·clamp(1 − (|κ| − κ*)·Sensitivity, 0, 1)`.
- This is stateless proportional pressure reduction. There is no pressure build/hold/dump cycle, no hydraulic dynamics and no wheel-deceleration threshold. `OptimalSlip` is a fixed parameter. It is not derived from the tire curve's peak, even though the LUT computes `OptimalSlipIndex`, which the ABS does not use.

### 5.2 TCS (per axle)

The code is `UpdateTCS`, `VehicleAxleAssemblyComponent.cpp:177-205`.

- The trigger uses the **average** |predicted slip| of the two wheels of an axle. It requires Vx·T > 0 and |Vx| > 1 m/s, then scales axle drive torque proportionally.
- There is no per-wheel brake-based traction control. With an open or low-lock differential and one wheel on wet cobblestone, the average can stay below threshold while one wheel spins (**suspected** weak case).
- TCS is inactive below 1 m/s, which is exactly when a launch needs it.
- There is no engine torque-reduction channel. The axle torque is cut after the engine has already produced it, so engine rpm flares instead.

### 5.3 Other assists

- **Counter-steer assist** adds steering proportional to chassis slip angle (`VehicleAxleAssemblyComponent.cpp:103-151`).
- **High-speed steering scale** is an input curve on speed (`VehicleDriveAssemblyComponent.cpp:268-292`).
- **Auto-hold**, **burnout line-lock**, **automatic clutch** and **rev-match** are also present.
- These map well to OpenRoad's "individually configurable aids" concept, but every aid is implemented ad hoc in a different component.

---

## 6. ESC / stability control

**Confirmed absent.** The repository has no yaw-rate reference model, no sideslip estimator, no per-wheel brake allocation for yaw moment, and no understeer/oversteer arbitration. What exists:

- `UVehicleWheelComponent::SetEspBrakeTorque` (`VehicleWheelComponent.h:430`) writes `BrakeTorqueFromESP`. The ABS-filtered brake path adds it (`VehicleWheelSolver.cpp:111`). Nothing in the plugin calls it.
- The README says "Open APIs for P1/P2/P3/P4 electric motors and ESP (not yet implemented)".
- The counter-steer assist (section 5.3) is a steering-side drift aid, not ESC.

OpenRoad must implement ESC itself in every scenario: a bicycle-model yaw reference, a β estimator, brake-moment allocation and an engine-torque request.

---

## 7. Aerodynamics

**Base dispatcher** (`VehicleAeroBaseComponent.cpp:61-158`). It computes the apparent wind at the centre of pressure, including chassis rotation and motion of "virtual parts" (active aero) obtained by finite-differencing a game-thread-set transform. It returns force plus `r × F` torque. The coordinator sums all aero components and applies **one impulse per physics step** (`VehicleWheelCoordinatorComponent.cpp:527-580`). Wind is **hard-coded zero** (`:546`).

**Airfoil** (`VehicleAirfoilComponent.cpp:22-91`).

- CL and CD versus AoA come from `FRuntimeFloatCurve`, evaluated via `FRichCurve::Eval` every step ("TODO: Bake LUTs", `:19`). Induced drag is `CL²/(π·e·AR)`.
- Lift direction is `WingRight × WindDir`.
- There is no ground effect or ride-height sensitivity, no stall hysteresis, and no yaw/sideslip dependence beyond geometry. Air density is a constant 1.225 (`VehicleAeroBaseComponent.h:61`).

**Aero volume** (`VehicleAeroVolumeComponent.cpp:22-87`). It applies per-local-axis quadratic drag `½ρ·Cd·A·v|v|` and quadratic rotational damping using `L³` reference lengths.

**Missing.** No slipstream or drafting, which matters for highway traffic. No downforce map against pitch and ride height. No crosswind, which matters for trucks passing and for bridges.

**Assessment.** The aero is simple, correct as far as it goes, and easy to re-derive.

---

## 8. Numerical integration and timestep requirements

| Element | Rate | Scheme |
|---|---|---|
| Chaos rigid body (chassis) | Async fixed step. The plugin config ships 120 Hz (`Config/DefaultEngine.ini`: `AsyncFixedTimeStepSize=0.008333`, `bTickPhysicsAsync=True`) | Chaos PBD |
| Suspension trace, kinematics, forces | Once per physics step | Explicit force, with the stability clamps of section 2.4 |
| Unsprung mass | Internal 1/480 s (`VehicleSuspensionSolver.cpp:1072`) | Semi-implicit |
| Drivetrain and wheel spin | `SubstepDeltaTime` 1/240 s (`VehicleDriveAssemblyComponent.h:88`) | Staggered explicit (section 3) |
| Tire relaxation | Per substep | Backward Euler |
| Clutch spring | Per substep | Backward Euler |
| Chassis force application | Once per physics step, as an averaged impulse | — |

The README says to use a physics step ≤ 1/60 s (1/90 to 1/120 recommended), "never go below 30 Hz", and to prefer async physics over Chaos substepping. Unsprung mass "requires 120Hz".

**What degrades at low rate.** This list is inferred from the formulas, not measured.

- All constraint forces scale as `m/Δt`. At low rates, stiction-like behaviour becomes softer and the curve cap dominates sooner.
- LSD and center-diff λ is per substep, so locking strength depends on rate.
- The swaybar and the other-wheel coupling are one step behind.
- Ray sampling distance grows (about 1.85 m per step at 400 km/h and 60 Hz).
- The suspension uses `RayStrutVelocity = ΔLimit/Δt` from successive traces (`:1064`). At curbs and step edges this velocity spikes and feeds the damper. The damping clamp may be ineffective because of the suspected unit issue.
- The game-thread `UpdateWheelAnim` interpolates using `MacroDeltaTimeInv` (`VehicleWheelComponent.cpp:1020`), so rendering also depends on the physics rate.

---

## 9. Async physics interaction and thread safety

**Mechanism.**

- `FAsyncTickManager` registers one `TSimCallbackObject` per physics scene (`AsyncTickManager.cpp:117-131`).
- Each frame, `ScenePreTick` copies the list of registered `TWeakObjectPtr<UAsyncTickActorComponent>` into the producer input (`:133-162`).
- On the physics thread, `OnPreSimulate_Internal` resolves each weak pointer and calls the virtual `NativeAsyncTick` (`AsyncTickCallback.cpp:3-35`).
- `UVehicleAsyncTickComponent::NativeAsyncTick` then calls `DriveAssembly->UpdatePhysics`, `WheelCoordinator->UpdateAeros` and `SpringArm->UpdatePhysics` **directly on UObjects** (`VehicleAsyncTickComponent.cpp:103-154`).

**Race and lifetime findings.** There are no synchronization primitives anywhere in the plugin. A search for `FCriticalSection`, `FScopeLock`, atomics and `TAtomic` finds only commented-out `TAtomic` lines in the spring arm.

1. **The physics thread reads game-thread-owned UObject state.**
   - `GetRelativeTransform()` (`VehicleWheelComponent.cpp:324`; `VehicleAeroBaseComponent.cpp:79`).
   - `TWeakObjectPtr::Get()` on chassis, wheels and axles, many times per step.
   - `UPhysicalMaterial` via weak pointer (`VehicleSuspensionSolver.cpp:1208`).
   - `HitComponent->IsSimulatingPhysics()` (`VehicleWheelComponent.cpp:278-279`).
   - Component config structs that the editor can rewrite.
2. **The game thread reads physics-written state with no double buffering.** Examples: `Suspension.State` and `Wheel.State` in `UpdateWheelAnim` (`VehicleWheelComponent.cpp:1010-1070`), and `LocalLinearVelocity` in the auto-gearbox. **Torn reads** are possible, for example a half-written `FQuat4f` hub rotation, which would only cause a visual glitch.
3. **Writes in the other direction:**
   - `WorldGravityZ` is written on the game thread (`VehicleWheelComponent.cpp:459-467`) and read on the physics thread.
   - Gear ratio changes come from game-thread timers (section 4.2).
   - Engine callback flags are set on the physics thread and consumed on the game thread (`VehicleEngineComponent.cpp:115, 303, 386-416`).
   - In editor builds, LUTs are rebuilt periodically on the game thread while the physics thread reads them (`VehicleWheelComponent.cpp:438-450`; `VehicleEngineComponent.cpp:374-384`).
4. **Garbage-collection hazard (suspected).** With async physics, the physics thread can run concurrently with game-thread GC or component destruction. Calling methods on a UObject obtained from a weak pointer on the physics thread is not GC-safe.
5. **Bridge bugs** (the bridge is Alex Craig's code with KinetiForge modifications):
   - `~FAsyncTickManager` loops `while (Pawns.Num() > 0) RemovePawn(Pawns.Last())`. `RemovePawn` only removes *valid* pointers (`AsyncTickManager.cpp:31-41, 81-95`), so a stale entry makes this an **infinite loop** at scene teardown.
   - `UnregisterCallbacks` frees `AsyncObject` but does not null it (`:122-130`), so a later `RegisterCallbacks` would skip re-creation and `ScenePreTick` would use a freed object (**suspected**; depends on world lifecycle).
   - `UAsyncTickActorComponent::BeginPlay/EndPlay` dereference the manager without a null check (`AsyncTickActorComponent.cpp:14-15, 22-23`).
6. **An alternative path exists.** `bUpdatePhysicsOnGameThread` switches to UE's `AsyncPhysicsTickComponent`, which the code describes as running on a "frozen game thread, thread-safe but performance is bad" (`VehicleAsyncTickComponent.h:53`). It is not a `UPROPERTY`, so it defaults to `false` and cannot be configured from the editor.

**Verdict.** The threading model is the biggest structural obstacle to depending on KinetiForge for a multiplayer open world.

---

## 10. Performance, memory and scaling

**Good practices.**

- Hot loops use stack contexts.
- `TInlineAllocator<8>` holds the axle list (`VehicleDriveAssemblyComponent.cpp:759`).
- LUTs replace curve evaluation for tire, engine and suspension.
- `TRACE_CPUPROFILER_EVENT_SCOPE` markers are present throughout.

**Per-tick costs and allocations.**

- `MultiSphereTrace` allocates a `TArray<FHitResult>` per wheel per step when refinement triggers (`VehicleSuspensionSolver.cpp:1385`).
- `ScenePreTick` refills two `TArray`s per scene per frame (`AsyncTickManager.cpp:147-161`). This is mostly capacity-reusing.
- The airfoil evaluates `FRichCurve` twice per step (no LUT).
- Each wheel step involves:
  - 1 to 3 scene queries.
  - About 4 unsprung substeps.
  - A 3×3 effective-mass computation in 3 directions.
  - Kinematic solves.
  - N tire substeps with several LUT lookups.
  - Many `TWeakObjectPtr` resolutions and virtual calls.
- `FVehicleSuspensionSimContext` embeds a full `FHitResult` and copies it into state each step.

**UObject footprint, counted from the code.** A 4-wheel car has: DriveAssembly, Engine, Clutch, Gearbox, TransferCase, 2 Axles, 2 Diffs, 4 Wheels, 12 auto-generated mesh/hub components (3 per wheel, `VehicleWheelComponent.cpp:155-227`), WheelCoordinator and AsyncTick. That is **about 27 components**. About 8 have game-thread ticks enabled: wheels ×4, engine, drive assembly, coordinator, plus aero/ADAS when present.

**Scaling.** All vehicles run **serially** inside one physics-thread callback. There is no `ParallelFor`, no LOD, no sleeping and no significance management. The content includes a `StressTest_100Cars.umap`, so the author tests about 100 cars, but we have no numbers. For "hundreds of traffic cars" in an open world, a rewrite would be needed: data-oriented state arrays, parallel per-vehicle stepping and simulation-LOD tiers.

---

## 11. Numerical stability at very high speed (300-400 km/h)

| Risk | Where | Assessment |
|---|---|---|
| Contact sampling aliasing | One trace per physics step; strut limit interpolated linearly (`VehicleSuspensionSolver.cpp:1137-1138`) | **High** for cobblestones and potholes: 0.93 m per step at 400 km/h and 120 Hz |
| Rigid wheel-ground constraint | No tire vertical compliance (`:1171-1184`) | Harsh impacts and possible launch on sharp edges (**suspected**) |
| Coarse Fy LUT | 64 samples over 0-90° (`VehicleWheelSolver.cpp:251`) | Piecewise-linear lateral force at the small slip angles (0.3-2°) that dominate high-speed stability (**suspected** feel and stability impact) |
| Slip angle | `asin(−v̂y)` on the normalized 2D velocity (`:431-453`); transient uses tan-like `vy/vx` relaxation then `atan` (`:563`) | Stable at high speed |
| Relaxation | Implicit (`:558-560`) | Stable, with no CFL limit |
| Divisions | `SafeDivide` guards only \|x\| < 1e-8 (`VehicleUtilities.h:135-138`) | Small-but-nonzero denominators pass. Example: the anti-geometry `TrueCOMHeight/DynamicLx` (`VehicleSuspensionSolver.cpp:2059-2060`) |
| Slip-ratio denominator | Floor of 1 m/s (`VehicleWheelSolver.cpp:462`) | Safe; dampens ABS/TCS at low speed only |
| Precision | World positions in `double` (UE LWC `FVector`); velocities and chassis-space values in `float` | Adequate for large worlds |
| Aero | v² forces applied as one impulse per step | Stable; no ride-height feedback loop, so no porpoising risk |
| Steering at speed | The latest fix changed speed averaging (`df4fbf2`, issue #13) | Shows the speed signal feeding assists was wrong until 2026-08 |

---

## 12. UE version compatibility

- **Build files.** `KinetiForge.uplugin` has no `EngineVersion` field, is marked beta, and declares two runtime modules. `KinetiForge.Build.cs` depends on Core, Chaos, ChaosSolverEngine, PhysicsCore, AsyncTickPhysics, Engine, MovieScene and NavigationSystem. Both `Build.cs` files carry the UE template header "Copyright Epic Games, Inc."
- **Asset version.** All 333 assets report `++UE5+Release-5.3`. 5.3 is the minimum for the content. The template map and curves cannot be opened in earlier versions.
- **API surface.** The code uses Chaos internals: `FRigidBodyHandle_Internal`, `FParticleUtilitiesGT/XR`, `GetPhysicsThreadAPI()`, `TSimCallbackObject`, `CreateAndRegisterSimCallbackObject_External`, `FPhysScene_Chaos::OnPhysScenePreTick`, `SetAsyncPhysicsTickEnabled` and `FPhysicsInterface::RaycastSingle/GeomSweepSingle`. These are internal or experimental-ish APIs that Epic has changed between minor versions.
- **No version guards.** There are no `ENGINE_MAJOR_VERSION`/`ENGINE_MINOR_VERSION` guards. Compatibility with UE 5.4-5.7 is **unverified**, because we did not compile it.
- **Migration burden.** Breaking renames are handled with `[CoreRedirects]` that users must copy into their project (`Config/DefaultEngine.ini`). The README warns that Blueprint data is lost otherwise.

---

## 13. Modularity, maintainability, extensibility and testability

**Positive.**

- The solver classes `FVehicleWheelSolver` and `FVehicleSuspensionSolver` are plain C++ with `State`/`Context`/`Config` separation and mostly `static` helper functions.
- The component composition is flexible: multiple drive assemblies per actor, standalone wheels and axles, and hybrid motor hooks.
- The code is reasonably commented, though some comments are mojibake-encoded Chinese.

**Negative.**

- **UObject coupling.** Engine, clutch, gearbox and differential physics live inside `UActorComponent`s. Gearbox shifting depends on `UWorld` timers. Orchestration passes `UVehicleAxleAssemblyComponent*` arrays into differential math (`VehicleDifferentialComponent.cpp:62-171`). The suspension solver takes `UWorld*`, `FCollisionQueryParams` and `UVehicleWheelComponent*`, and debug drawing is mixed into solver classes.
- **UE type dependence.** Tire and suspension math use `FVector3f`, `FQuat4f`, `FTransform3f`, `FMath`, `FRichCurve`, `UCurveFloat`, `TStaticArray`, `FHitResult` and `Chaos::FVec3`/`FMatrix33`.
- **Units are mixed.** Radius is in cm, suspension in cm and probably N/cm, tire math in m and m/s, and impulses are multiplied by 100 (`VehicleWheelComponent.cpp:267`). Some unit conversions are implicit and suspect (section 2.4).
- **No tests and no CI.** There is no test directory and no `.github` workflows.
- **Single maintainer.** 52 of 54 visible commits are by one author. Hot code has been revised repeatedly in short bursts: four "Fixing ... constraint force" commits on 2026-07-05.
- **Stale documentation.** The README still lists "No camber thrust".

**Could the math be lifted out?** Partly.

- **About 300 lines** of tire code (`VehicleWheelSolver.cpp:276-772` minus debug) port with mechanical type substitution.
- **About 800 lines** of suspension kinematics (`SolveLowerWishbone`, `SolveUpperWishbone`, `SolveSolidAxlePosture`, swing-arm/IC slopes, effective mass) port after removing `FHitResult` and context coupling.
- **The drivetrain** would need re-architecture rather than porting.

---

## 14. Bugs, limitations and technical debt

Concrete findings from static reading. **(S)** marks suspected issues.

| # | Finding | Location | Severity |
|---|---|---|---|
| 1 | Client engine start/stop RPCs declared `NetMulticast` instead of `Server`, so calls from a client never reach the server | `VehicleDriveAssemblyComponent.h:134-141` | High (multiplayer) |
| 2 | `ServerVehicleEngineOperationMode` is `ReplicatedUsing` but missing from `GetLifetimeReplicatedProps` | `VehicleDriveAssemblyComponent.cpp:124-129` | High (multiplayer) |
| 3 | Every axis input sent as a **reliable** server RPC plus a reliable multicast on every call, with no change detection or throttling | `VehicleDriveAssemblyComponent.cpp:897-978`; `.h:104-123` | High (bandwidth, reliable-buffer overflow risk) |
| 4 | Physics thread mutates and reads UObjects with no synchronization; game thread reads physics state | Section 9 | High (stability and determinism) |
| 5 | Delegate array modified during ranged-for | `VehicleEngineComponent.cpp:389-399, 405-415` | Medium (assert or crash) |
| 6 | `~FAsyncTickManager` infinite loop if a weak pointer is stale | `AsyncTickManager.cpp:31-41` | Medium |
| 7 | `AsyncObject` not nulled after free **(S)** | `AsyncTickManager.cpp:122-130` | Medium |
| 8 | Null dereferences: manager in `AsyncTickActorComponent::BeginPlay/EndPlay`; `Cast<APawn>(GetOwner())` in auto-gearbox and multicasts | `AsyncTickActorComponent.cpp:14-23`; `VehicleDriveAssemblyComponent.cpp:320-321, 653-654, 680-681` | Medium |
| 9 | Gear changes driven by game-thread timers while the physics thread reads the ratio | `VehicleGearboxComponent.cpp:251-283` | Medium |
| 10 | Suspension clamp units (N/m against N/cm) **(S)** | `VehicleSuspensionSolver.cpp:2238-2255, 2197` | Medium |
| 11 | Anti-geometry slope blow-up when contact is near CoM longitudinally **(S)** | `VehicleSuspensionSolver.cpp:2059-2062` | Medium (multi-axle vehicles) |
| 12 | Load-sensitivity tooltip contradicts formula; hard-coded 9.81; zero grip if sprung mass is unset | `VehicleWheelStructs.h:64-69`; `VehicleWheelSolver.cpp:656-668` | Low to medium |
| 13 | `bDrivetrainSubstepping` unused | `VehicleDriveAssemblyComponent.h:86` | Low |
| 14 | Wishbone getters not side-mirrored **(S)** (animation/debug only) | `VehicleSuspensionSolver.cpp:811-843` | Low |
| 15 | `FMath::FRand()` on the physics thread (cosmetic backfire) | `VehicleEngineComponent.cpp:297` | Low (determinism) |
| 16 | Dead code: `GetVector2dAngleDegrees`, `ComputeCircleIntersection` ("Abandoned"), `TickADAS` empty, ADAS timer `#if 0`, `UpdateAutoPilotHumanLike` unfinished | `VehicleSuspensionSolver.h:124-125`; `VehicleADASComponent.cpp:40-59`; `.h:226-228` | Low |
| 17 | Gearbox cannot take a per-gear ratio table; reverse mirrors forward | `VehicleGearboxComponent.cpp:366-426` | Low to medium (realism) |
| 18 | Stale README (camber thrust now exists) | `README.md` "Known Limitations" | Low |
| 19 | Unused variable typo in solid-axle solver | `VehicleSuspensionSolver.cpp:863-864` | Cosmetic |

**History signals.** The log contains, in sequence:

- `fb7c5b3` "Fixing brake force constraint"
- `24ccaac` "Fixing longitudinal constraint force again"
- `9846259` "Fixing longitudinal constraint force again..."
- `06c4943` "Fixing problem with suspension", which carried the strut limit across steps
- several "energy conservation" fixes for the clutch and LSD
- lifecycle reverts and re-applies (`11991c4`, `84d55c9`, `439f1a7`, `4a759fc` "Fix Liftcyle bug in packaged version", "Fix a LifeTime issue?")

The core constraint formulation and object lifecycle were still being stabilized as of mid-2026.

---

## Capability matrix

Chaos Vehicles column: our general knowledge of UE 5.x `ChaosVehicles` (`UChaosWheeledVehicleMovementComponent`). It was **not re-verified in this audit** and should be checked against the target engine version.

| Feature | KinetiForge status | Quality / notes | Chaos Vehicles (ref.) | Reuse candidate? |
|---|---|---|---|---|
| Tire force model | Curve LUT (64 samples) capped constraint | Robust, not physically parameterized; coarse Fy | Simple slip-based model with friction ellipse | **Partial**: FUN-preset "artist tire" option; re-implement |
| Combined slip | Similarity/normalized slip plus interference blend | Decent; tunable | Friction circle/ellipse | Idea yes (re-derive) |
| Load sensitivity | `r/(1+br)` | Normalization quirk; docs mismatch | Basic | No (use MF/TMeasy formulation) |
| Relaxation length | Backward-Euler first-order | Correct, standard | Limited or none | Re-derive (textbook) |
| Low-speed model | Constraint cap, slip floors, lock detection | Good | Has low-speed handling, limited | **Yes, as concept** (regularizer) |
| Surface friction | PhysMat friction × tire μ, combine modes | No wet or roughness model | PhysMat friction | No |
| Camber thrust | Drift curve, capped | New, plausible | No | Idea |
| Suspension types | Straight, MacPherson, double wishbone, solid axle | Strong | Raycast spring/damper | **Yes**: linkage solvers |
| Camber/toe curves | LUT against compression | Good | Limited | Idea |
| Contact detection | Line, sphere, box, multi-sphere, single query | No patch or envelope; aliasing at speed | Ray or sphere sweep | No (need multi-point/envelope) |
| Unsprung mass | 480 Hz semi-implicit, rigid ground | No tire radial compliance | No | Idea only |
| Anti-roll bar | Linear hub-Z coupling, one step lag | Basic | Yes (basic) | Re-derive |
| Anti-dive/squat/roll | Instant centre plus curves → jacking | Good idea; blow-up risk | No | **Yes** (with guards) |
| Contact effective mass | 3-direction rigid-body formula | Standard, clean | Internal | **Yes** (standard math) |
| Engine | Torque LUT, friction and pumping, idle, stall, turbo, anti-lag | Rich for games | Torque curve, simple | Partial (turbo/anti-lag state machine for FUN) |
| Clutch | Constraint, implicit spring-damper, viscous | Spring-damper closed form is good | Simple | **Yes**: implicit spring-damper formula |
| Gearbox | Interpolated ratios, shift delay, rev-match, auto | No ratio table; game-thread timers | Ratio table, auto/manual | No |
| Differential | Lock-ratio LSD, drive/coast, N-axle transfer | No torque cap; Δt-dependent | Open, FWD/RWD/AWD split | No (implement clutch-pack/Torsen/viscous) |
| Drivetrain integration | Staggered explicit, one-substep lag, 240 Hz | Energy-drift history | Simple | No |
| ABS | Per-wheel proportional slip limiter | Basic; no hydraulics | Toggle (basic) | Re-derive |
| TCS | Per-axle average-slip torque cut | Weak cases (split μ, launch) | Toggle (basic) | No |
| ESC | **Absent** (hook only) | — | Arcade "stability" style aids exist (verify) | n/a, must build |
| Counter-steer/steer assists | Yes | Ad hoc | Arcade controls (verify) | Idea |
| Aero | Airfoil CL/CD, quadratic drag/damping, active parts | No ground effect, wind or draft | Drag and downforce coefficients | Re-derive |
| Async physics | Chaos sim-callback calling UObjects | **Unsafe** | Engine-integrated async path | No |
| Multiplayer | Reliable input RPCs, no prediction; RPC bugs | Weak | Movement replication; network physics in recent versions (verify) | No |
| LOD / many vehicles | None; serial | Weak | Some (sleep, mechanical-sim toggles; verify) | No |
| Engine independence | None (UE types throughout) | — | None | n/a |
| Tests / CI | None | — | Engine-tested | n/a |
| AI driving helpers | Cruise control, autopilot, braking and turn speed planners | Useful utilities | No | **Yes**: `GetMaxSpeedToBrake`, `GetMaxSpeedToTurn` (trivial math, re-derive or copy) |

---

## 15. Lawful reuse

### 15.1 Licensing facts

- KinetiForge is MIT-licensed, "Copyright (c) 2025 Zhengyi Miao" in `LICENSE`, with 2026 headers in source files. MIT permits copying, modification and relicensing within an MIT project, provided **the copyright notice and permission notice are included in all copies or substantial portions**.
- `Source/AsyncTickPhysics` is a **separate MIT work**, "Copyright (c) 2022 Alex Craig", with its notice in `Source/AsyncTickPhysics/README.md`, not in the root `LICENSE`. Any reuse of it needs that notice too. We recommend **not** reusing it at all (section 9).
- The `*.Build.cs`, `KinetiForge.{h,cpp}` and `AsyncTickPhysics.{h,cpp}` files carry the UE plugin-template header "Copyright Epic Games, Inc. All Rights Reserved." These are trivial boilerplate. Do not copy them, and write our own module files.
- **Provenance caution: `UVehicleWheelCoordinatorComponent::ComputeSprungMasses`** (`VehicleWheelCoordinatorComponent.cpp:270-465`). It has the same structure as the sprung-mass utility in Epic's Chaos Vehicles plugin as we remember it: single-point, two-point and N-point Lagrange-multiplier cases. Epic's code is under the UE EULA, not MIT. **We did not compare against Epic's source in this audit**, so this is unverified. Either way, the math (minimum-variance load distribution under force and moment balance) is generic. OpenRoad should **re-derive it from first principles and not copy this function.**
- Binary `.uasset` content (curves, meshes, maps) is UE-format data, cannot be used in an engine-independent core, and was not inspected.

### 15.2 Candidates and how to adopt them

| Algorithm | Source | Adopt how | Rewrite needed for engine independence |
|---|---|---|---|
| Implicit relaxation-length update | `VehicleWheelSolver.cpp:535-572` | **Re-derive** from Pacejka (textbook ODE); no need to copy | Trivial |
| Constraint-capped tire force with sign gate (low-speed regularizer) | `VehicleWheelSolver.cpp:574-612, 739-752` | Re-implement the concept as an optional low-speed blend in our tire interface. If copying verbatim, keep the MIT notice | Replace `FVector2f`/`FMath`; make it Δt-explicit and unit-clean |
| LUT "artist tire" with similarity combined slip and interference blends | `VehicleWheelSolver.cpp:670-772`; `VehicleWheelStructs.h:167-265` | Candidate **FUN-preset tire model** behind our `ITireModel`. Copy with notice or re-implement. Increase LUT resolution or use non-uniform sampling near the peak | Replace `FRichCurve`/`UCurveFloat` with our own curve type; SI units |
| MacPherson lower-arm circle-sphere solve | `VehicleSuspensionSolver.cpp:1482-1569` | Copy with notice, or re-derive (about 80 lines of geometry) | Own vec3 type; remove context struct |
| Double-wishbone upper-arm solve | `:1646-1705` | Same | Same |
| Solid-axle Newton posture solve | `:845-927` | Re-derive (2×2 Newton); copying offers little | Same |
| Instant-centre anti-pitch/roll slopes | `:1803-1956, 1958-2069` | Copy or re-derive. **Add guards** against near-zero lever arms | Same; decouple from `FHitResult` |
| Contact-point effective mass, 3 directions | `:2106-2189` | Standard rigid-body formula; re-derive | Own mat3 type |
| Load-fraction mass partitioning for per-wheel constraints | `:2304-2309` | Idea; re-implement | — |
| Implicit clutch spring-damper torque | `VehicleClutchComponent.cpp:30-76` | Standard backward-Euler closed form; re-derive, cite as inspiration | Trivial |
| Engine friction, pumping loss, idle throttle solve | `VehicleEngineComponent.cpp:48-71, 312-365` | Re-derive (simple) | Trivial |
| Turbo spool, blow-off and anti-lag state machine (feel and audio) | `VehicleEngineComponent.cpp:77-191` | Optional FUN feature; copy with notice or re-implement | Replace `FMath::FInterpTo` |
| Optimal-slip detection on curve LUT (max deviation from secant) | `VehicleWheelStructs.h:221-263` | Small utility; re-implement | Trivial |
| AI speed planners `GetMaxSpeedToBrake`, `GetMaxSpeedToTurn` | `VehicleADASComponent.cpp:83-121` | Kinematics one-liners; re-derive | Trivial |

**Attribution practice if anything is copied verbatim.** Add a `THIRD_PARTY_NOTICES.md` entry with the full KinetiForge MIT text, listing both years (2025 from `LICENSE`, 2026 from file headers). Keep a header comment in each derived file naming the origin file and the commit `df4fbf2`. Record the decision in `docs/decisions/`.

---

## 16. Recommendation

### 16.1 Options compared

| Option | Pros | Cons |
|---|---|---|
| **A. Depend on and extend KinetiForge** | Fastest first drive; rich suspension and engine; active author | UObject-bound and thread-unsafe core; no ESC; no LOD or batching; multiplayer bugs; non-physical tire parameterization; UE 5.3-era internal Chaos APIs with no version guards; single-maintainer bus factor; no tests. Contradicts our engine-independent core requirement. A fork would diverge quickly |
| **B. Incorporate selected MIT components** | Saves effort on geometry solvers and some feel features; lawful with notice | The valuable parts are small and mostly textbook. Copying UE-typed code still needs rewriting. Provenance check needed for one function |
| **C. Independent core from published math** | Clean SI units; testable headless; deterministic and thread-safe by construction; real tire models (MF/TMeasy) and a monolithic implicit drivetrain; LOD tiers for traffic; full ownership | Most upfront work; must build all aids (ABS/TCS/ESC) and the UE adapter ourselves |
| **D. Hybrid with interchangeable implementations** | Keeps C's architecture. Pluggable `ITireModel`, `ISuspensionKinematics`, `IDrivetrainSolver` and assist modules allow an "artist tire" (KinetiForge-style) for FUN and MF/TMeasy for SIMULATION. Chaos Vehicles can stay as a reference baseline or benchmark in the UE adapter | Interface design cost; must keep implementations behaviourally comparable |

### 16.2 Choice: D, built as C, with targeted B

1. **Architecture as in C.** Use an engine-independent C++ core with SI units, explicit state structs, no UObjects, and deterministic fixed-step stepping. Expose a ray/shape query interface and per-vehicle data-oriented state for parallel stepping and LOD.
2. **Interchangeable implementations as in D.**
   - Tire: `ITireModel` with MF 6.x or TMeasy for SIMULATION, a curve-LUT "artist" model for FUN/SPORT (KinetiForge-inspired, higher LUT resolution), plus a shared low-speed regularizer.
   - Suspension: geometry modules for MacPherson, double wishbone, solid axle and LUT-only.
   - Drivetrain: a monolithic implicit solver.
   - Assists: ABS/TCS/ESC as separate configurable modules. This maps directly onto the FUN / SPORT / SIMULATION presets with per-aid overrides.
3. **Targeted B.** Port or re-derive the small items in section 15.2: suspension linkage solvers, IC anti-geometry with guards, contact effective mass, the clutch implicit formula, the turbo state machine and the AI speed planners. Preserve MIT notices for anything copied verbatim. **Do not** copy `ComputeSprungMasses` (provenance) or the AsyncTickPhysics bridge (unsafe).
4. **Chaos Vehicles as a benchmark.** Keep it as a baseline for comparison tests in the UE adapter (handling metrics, CPU per vehicle), not as a runtime dependency of the core.

### 16.3 Gaps to fill regardless of choice

KinetiForge does not solve any of these for us:

- ESC.
- Wet and rain grip with water depth, and surface roughness for cobblestones.
- Multi-point or envelope tire contact (for example a cylinder-sampled patch) for potholes at speed.
- Tire radial compliance.
- Real differential models: clutch-pack with preload and ramps, Torsen TBR, viscous.
- Per-gear ratio tables and a torque converter.
- Slipstream and crosswind aero.
- Network prediction and reconciliation.
- Simulation LOD for hundreds of vehicles.
- A headless test suite (steady-state cornering, step-steer, braking-in-turn, split-μ, 400 km/h straight-line stability).

---

## Appendix A: Key equations as implemented (short excerpts)

- **Transient slip** (`VehicleWheelSolver.cpp:558-560`):
  `TransientSlip = (TransientSlip + SlipVelocity*dt/L) / (1 + AbsVx*dt/L)`
- **Load sensitivity** (`:665-667`):
  `b = (1-s)/(2+2s); LoadScale = r/(1+b*r); Grip = mu*LoadScale*Fz0`
- **Lateral constraint** (`:608-609`):
  `F = LatSlipVelocity * (1/dt_macro) * m_eff_lat`
- **Clutch spring-damper** (`VehicleClutchComponent.cpp:56-60`):
  `T = (K*theta + (K*dt + D)*dw) / (1 + (K*dt + D)*dt/J)`
- **Differential transfer** (`VehicleDifferentialComponent.cpp:278-287`):
  `T_xfer = LockRatio * (J_L*J_R/(J_L+J_R)) * (w_L - w_R) / dt`
- **Unsprung semi-implicit step** (`VehicleSuspensionSolver.cpp:1152-1168`):
  `v = (v + F/m*dt) / (1 + c*dt/m + k*dt^2/m)`
- **Load sensitivity reference load** (`VehicleWheelSolver.cpp:662-663`):
  `Fz0 = StaticSprungMass * 9.81` (hard-coded g)

## Appendix B: Limits of this audit

- The audit is static only. There are no runtime measurements, CPU profiles or handling tests, and no compile check against any UE version.
- We could not inspect the default curve assets (`DefaultFx`, `DefaultFy`, etc.), so the shapes of the shipped tire curves are unknown.
- Chaos Vehicles comparisons come from general knowledge and were not re-verified against a specific engine release.
- The clone is shallow, so authorship statistics cover only the visible history.
