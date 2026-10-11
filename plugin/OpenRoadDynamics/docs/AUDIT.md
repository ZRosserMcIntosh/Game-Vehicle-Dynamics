# UE5 adapter audit (v0.2 draft)

Scope: everything under `plugin/OpenRoadDynamics/Source`, read as unfamiliar code. The core (`src/core`) was read but not changed.

> **Nothing here was compiled or run in Unreal Engine.** The repository has no Unreal Engine. Each item says which of these applies:
> - **source**: established by reading the code.
> - **headless**: exercised by `Extras/HeadlessCheck` (`./run.sh`). That harness compiles the adapter's engine-free files (`OrdVehicleSim.cpp`, `OrdFrame.h`, `OrdGroundHorizon.h`) with GCC and Clang against small stand-ins for `FVector`/`FQuat`/`TArray`. It drives them with a rigid-body integrator written in UE axes and units.
> - **unverified**: depends on Chaos or engine behaviour that only a real build can show.

Status values:
- **OK**: no defect found at source level.
- **Defect-fixed**: a defect was fixed in this pass.
- **Risk-unverifiable**: correct as far as the source shows, but it depends on engine behaviour.
- **Defect-open**: a known defect that is not fixed yet.

Line numbers refer to the files as of this audit.

## Summary

| # | Item | Status |
|---|---|---|
| 1 | Sim-callback registration and lifetime | Defect-fixed + Risk-unverifiable |
| 2 | Rigid-body handle / proxy lifetime on the physics thread | Defect-fixed + Risk-unverifiable |
| 3 | Physics-thread safety, no UObject on the physics thread | OK |
| 4 | Centre-of-mass representation (X/R vs CoM, V) | OK (source + headless) + Risk-unverifiable, now self-diagnosing |
| 5 | Inertia tensor conversion | OK + Risk-unverifiable (principal frame) |
| 6 | Force/torque application | Defect-fixed (minor) + Risk-unverifiable |
| 7 | Async fixed step, substeps, async disabled | OK + Risk-unverifiable |
| 8 | Game-thread/physics-thread synchronisation | OK + Risk-unverifiable |
| 9 | Handedness and unit conversions | OK (source + headless) |
| 10 | Ground-cache publication and reclamation | Defect-fixed |
| 11 | Ground-cache freshness under frame hitches | Defect-fixed |
| 12 | Construction, destruction, cleanup | Defect-fixed |
| 13 | BeginPlay ordering with the physics body | Defect-fixed |
| 14 | Multiple vehicles, PIE restart, GC | OK + Risk-unverifiable |
| 15 | Kinematic/static steps and host-sync bookkeeping | Defect-fixed |
| 16 | Header compile defect (`OrdVehicleSim.h`) | Defect-fixed (found by the headless build) |
| 17 | 60 Hz world + 4 substeps parity | Defect-open (core design; reported, not an adapter bug) |

---

## 1. Chaos sim-callback registration and lifetime

**Evidence**
- Creation: `OrdChaosBridge.cpp:211` calls `CreateAndRegisterSimCallbackObject_External<FOrdSimCallback>()`. The vehicle is handed over through an atomic pointer at `:74`, and the physics thread takes it at `:123`. The callback owns the vehicle; its destructor at `:67` deletes a vehicle that was never taken.
- Release: `Unregister()` at `:225` calls `UnregisterAndFreeSimCallbackObject_External`. Chaos frees the object, and with it `FOrdVehicleSim` and `ord::Vehicle`. The component never deletes the vehicle itself.
- Callers: `EndPlay` (`OpenRoadVehicleComponent.cpp:144`), the body-destroyed handler (`:294`), the new `OnUnregister` backstop (`:159`), and `~FOrdChaosVehicleLink` through `TPimplPtr` destruction.

**Defects fixed**
- *Use-after-free during world teardown or GC.* `~FOrdChaosVehicleLink` runs when the component is garbage-collected, and that can happen after the world's physics scene and solver are gone. The old code still called `Solver->Unregister…` on a dangling solver. `Unregister()` now first checks that the world, kept as a weak pointer, still has this exact solver (`:236`). If not, it logs and leaves the callback to the solver's own teardown.
- *No backstop for unregistration without EndPlay.* Added `OnUnregister()`, which deactivates while the world is still alive.
- *Implicit template options.* The base is now `TSimCallbackObject<In, Out, ESimCallbackOptions::Presimulate>` (`:63`). It no longer relies on the default options including Presimulate.

**Ordering at EndPlay, actor destroy, level unload and PIE end (source reasoning).** `AActor::RouteEndPlay` runs before `UnregisterAllComponents`. Our unregistration is therefore queued in the frame's marshalled data before the body proxy's destruction.

**Check in a real UE build**
- The `TSimCallbackObject` third template parameter compiles. Either `Reset()` on input and output is called by the pool, or a never-released `shared_ptr` shows up as growing `TransientCacheAllocations`.
- Which thread runs `~FOrdSimCallback` (breakpoint), and that no `OnPreSimulate_Internal` call follows it.
- Destroy 50 vehicles while driving, under `-stompmalloc`.
- End PIE while driving; then close the editor world with PIE never started.
- Does the Chaos solver delete callbacks that are still registered when it is destroyed? If not, the teardown path above leaks one callback per vehicle per world teardown. A leak is acceptable; a crash is not.

## 2. Rigid-body handle access on the physics thread (proxy lifetime)

**Evidence.** Each frame the game thread reads `FBodyInstance::GetPhysicsActorHandle()` (`OrdChaosBridge.cpp:268`) and sends the raw `FSingleParticlePhysicsProxy*` in the input (`:292`). The physics thread calls `GetPhysicsThreadAPI()` on it (`:144`), which is null until the particle exists on the physics thread.

**Defects fixed**
- *Same-frame destruction race.* The old handler only unregistered. If `OnComponentPhysicsStateChanged(Destroyed)` fires after `TermBody()` in the same frame, this frame's input still carried the old proxy, and the physics step for this frame could dereference it. The handler now calls `ClearBody()` (`OpenRoadVehicleComponent.cpp:310`, `OrdChaosBridge.cpp:256`). That re-pushes this frame's input with `Proxy = nullptr`. The proxy's destruction, the null-proxy input and the unregistration all travel in the same frame's data. Inputs are consumed in order, so no step after the destruction sees the old pointer, and this holds whichever order `TermBody` and the broadcast happen in. `Unregister()` also pushes a null-proxy input first.
- *The vehicle stayed dead after any physics-state re-creation* (for example a collision-profile change or `RecreatePhysicsState`). The delegate was removed on deactivation, so `Created` was never seen. The delegate now stays bound from activation until EndPlay (`BindBody`, `:168`). `Created` schedules a re-activation on the next tick (`:318`, `:334`), which rebuilds the vehicle from the body's current state.
- `SetSimulatePhysics(false)` does not destroy the proxy; the body becomes kinematic. See item 15.

**Check in a real UE build**
- `OnComponentPhysicsStateChanged` signature and whether `Destroyed`/`Created` fire around `TermBody`/`InitBody`. Put breakpoints in the handler and in `FBodyInstance::TermBody`.
- While driving at speed, call `RecreatePhysicsState()` on the root every 0.5 s, and toggle `SetSimulatePhysics` false/true. Run under `-stompmalloc`. Expect a log line "re-activated", no crash and no force spike.
- `GetPhysicsThreadAPI()` returns null (not garbage) for a proxy whose particle is not yet created.

## 3. Physics-thread safety

**Evidence**
- `OnPreSimulate_Internal` (`OrdChaosBridge.cpp:121`) touches only the callback's own members, the copied `FOrdGameInput`, the proxy/handle (Chaos objects meant for the physics thread) and `FOrdVehicleSim`.
- `FOrdVehicleSim` (`OrdVehicleSim.cpp`) contains no UObject and no engine API apart from `UE_LOG`, which is thread-safe, and `check`.
- Console variables are read on the game thread (`OpenRoadVehicleComponent.cpp`, input construction).
- Line traces and physical-material lookup run only on the game thread; `FOrdUnrealGround::raycast` asserts `IsInGameThread()` at `OrdUnrealGround.cpp:24`.

**Status: OK.** Note: debug drawing allocates `TArray` memory on the physics thread when enabled. That is acceptable for debug use.

**Check in a real UE build:** `check(!IsInGameThread())` in `OnPreSimulate_Internal` with async physics on (the README validation plan).

## 4. Centre-of-mass representation

**Best understanding of Chaos.**
- `FRigidBodyHandle_Internal::X()`/`R()` are the particle (actor) frame.
- `CenterOfMass()`/`RotationOfMass()` are the CoM offset and principal frame in that actor frame.
- Integration is done at the CoM: `CoM = X + R·CenterOfMass`, advanced with `V`, and `X` is rebuilt from it.
- So `V()` is the **CoM velocity**, and `AddForce` acts at the CoM.

**Evidence**
- `ReadBodyState` (`OrdChaosBridge.cpp:97–116`) builds the world CoM from `X + R·CenterOfMass()`.
- `ToCoreState` (`OrdVehicleSim.cpp:98–116`) moves the velocity to the core's CoM: `v + ω × (coreCoM − hostCoM)`.
- `Step` moves the torque back: `τ + (coreCoM − hostCoM) × F` (`:239`). The wrench is exact whatever the CoM offset.

**Robustness to the other interpretation (new).** `UpdateVelocityCheck` (`OrdVehicleSim.cpp:119`) compares the finite-difference velocity of the Chaos CoM and of the body origin with `V`. With PBD, `(CoM_n − CoM_{n−1})/dt == V_n` exactly when V is the CoM velocity. Both residuals are in the snapshot, in `GetDiagnostics()` and in a one-time log line about 2 s into play.

**Headless (stand-in host whose V is the CoM velocity):**
- CoM residual: 1e-10 cm/s.
- Origin residual: 21 cm/s while rotating. The check discriminates.
- The trajectory matches a `ChassisMode::Internal` run to 4e-13 m over 8 s at 240 Hz.

**Check in a real UE build.** Read the "coupling check" log line, or the demo readout's V-check, while turning on flat ground. The CoM value should be about 0 and the origin value clearly not. If it is the other way round, set `V` to the origin velocity in `ReadBodyState`, or add `ω × (R·CenterOfMass)`. Shell contacts perturb both values, so judge on clean driving.

## 5. Inertia tensor conversion

**Evidence**
- Asset values are in kg·m² on body axes (`OpenRoadVehicleDefinition.cpp:100`). The diagonal is unchanged by the mirror `M = diag(1, −1, 1)`, because `M·I·M` keeps diagonal entries and order, so it stays `[Ixx, Iyy, Izz]`.
- To UE: `×1e4` gives kg·cm² (`OrdFrame.h:56`, `host_frame.hpp`).
- `ConfigureBody` (`OrdChaosBridge.cpp:352`) scales `InertiaTensorScale` component-wise by wanted/current.

**Risk.** Chaos stores the inertia as a diagonal in the principal frame `RotationOfMass`. If that frame is not the body frame (an asymmetric collision shell), the diagonal order no longer means roll/pitch/yaw, and the scaling is wrong. The core assumes principal axes are body axes. **New:** `RotationOfMass()` is read on the physics thread (`:115`), reported as `RotationOfMassDeg`, and included in the mass-mismatch warning. The demo uses a box, which has an identity principal frame.

**Check in a real UE build.**
- `RotationOfMassDeg` is about 0.
- `HostInertiaKgM2` matches the definition within 5%.
- If the inertia does not take through `InertiaTensorScale`, set it on the physics thread (`SetI`/`SetInvI`) instead.

## 6. Force and torque application

**Evidence.** `Handle->AddForce(F, false)` and `AddTorque(T, false)` are called at `OrdChaosBridge.cpp:174–175`, in world space, in kg·cm/s² and kg·cm²/s². They are called once per step and only when the body is `Dynamic`. This is the physics-thread handle API; `FBodyInstance::AddForce`, with its `bAccelChange`, is not used.

**Defect fixed (minor).** The calls used the default `bInvalidate = true`, which may wake or dirty the particle. Forces are only applied to bodies that are already Dynamic, so `false` is now passed explicitly.

**Accumulation and clearing.** Chaos is assumed to clear the force accumulators after each integration. If forces persisted, the body would accelerate without bound and the host-sync error would grow every step.

**Check in a real UE build.** The **host-sync error** (`HostSyncPositionErrorCm`, in the log line and the readout) compares where Chaos actually put the chassis with where the core predicted it, given our wrench.
- At 240 Hz with 1 substep it must stay well below 0.1 cm on smooth driving. Headless it is 2e-12 cm, and any error there would come from Chaos integration differences such as gyroscopic terms or solver iterations.
- A wrong unit (×100), a wrong frame (local instead of world), persisting forces or a wrong CoM point show up as errors of centimetres or more, growing with speed.
- This also covers README items 5 and 8.

## 7. Async fixed step and substeps

**Evidence**
- `dt = GetDeltaTime_Internal()` (`OrdChaosBridge.cpp:163`).
- `FOrdVehicleSim` warns once if dt changes (`OrdVehicleSim.cpp:163`).
- Substeps are `ceil(dt · VehicleRateHz)`, clamped to 1..16 (`:222`).
- With the recent core change, `stepSubstepped` always predicts in External mode, also for 1 substep. The adapter calls `setChassisState` once per step before stepping, so `hostSync()` compares the host state with the previous step's prediction.
- With async physics off, the callback still runs at the variable or substepped physics dt. The component warns at BeginPlay, and `FOrdVehicleSim` warns when dt varies.

**Risk.** Behaviour of `GetConsumerInput_Internal()` when one game frame spans several physics steps (null, or the same input again). The code keeps the latest input either way, and shift requests are counters, so each press applies exactly once.

**Check in a real UE build.** Compare `GetDeltaTime_Internal` with `AsyncFixedTimeStepSize`. `PhysicsStepsLastFrame` in the readout should be about frame time / step.

## 8. Game-thread / physics-thread synchronisation

**Evidence**
- Producer: one input per frame in `TG_PrePhysics` (`OpenRoadVehicleComponent.cpp:128`, `:435`).
- Consumer: the latest input is copied on the physics thread.
- Outputs: one snapshot per physics step, all popped every frame (`:357`).
- Snapshots carry plain data. `R.Latest = Snapshot` copies the arrays, which keep their capacity.

**Queue growth.** Outputs are popped every frame, so the queue length is about steps per frame. With async physics the game thread is what hands simulation time to Chaos: a stalled game thread does not make Chaos run ahead indefinitely. A hitch frame then produces at most `MaxPhysicsDeltaTime` worth of steps (unverified). The pool and horizon logic below do not depend on this.

**Check in a real UE build:** busy-wait 100 ms and 500 ms on the game thread at 250 km/h. Record `PhysicsStepsLastFrame` and `GroundCacheAgeSeconds`, and check for no `bFaulted`.

## 9. Handedness and units

**Evidence.** All conversions go through `OrdFrame.h`, which wraps `ord/host_frame.hpp`:
- Polar vectors mirror Y.
- Axial vectors negate X and Z.
- Quaternions are `(w, −x, y, −z)`.
- Steering input and readouts are negated (`OpenRoadVehicleComponent.cpp:420`; `GetWheelState` negates lateral force, slip angle and steer).

**Headless:**
- UE `r × F` equals `TorqueToUe(core r × F)`.
- UE `ω × r` equals `VelocityToUe(core ω × r)`.
- `FQuat::RotateVector` agrees with `Quat::rotate` through the mirror (1e-14).
- The same identities are written as UE automation tests in `Private/Tests/OpenRoadAdapterTests.cpp` (unverified).

**Status: OK.**

## 10. Ground-cache publication and reclamation

**Evidence.**
- A cache is rebuilt only while the pool holds the single reference: `use_count() == 1` followed by an acquire fence (`OpenRoadVehicleComponent.cpp:89–96`).
- Holders are Chaos input objects (until `Reset()`), the callback's latest-input copy, `FOrdVehicleSim::Ground` and the link's `LastInput`.
- **No ABA:** once the count is 1, nobody else holds a `shared_ptr` or a raw pointer to copy from, so the count cannot rise again behind our back.
- `GroundPrefetchCache::misses_` is a mutable atomic counter. Only the physics thread queries the cache, and `rebuild()` never touches it.

**Buffers needed:** one per game frame the physics thread has not consumed yet, plus about 3. When Chaos lags or runs several steps per frame this number grows.

**Defects fixed**
- *Fixed pool of 6, then an allocation every frame.* A lagging physics thread, or a Chaos input pool that does not call `Reset()` promptly, would allocate on every frame. The pool now grows to 16 (`:106`). Beyond that it allocates transient caches and counts them (`TransientCacheAllocations`).
- *Wrong miss counter.* `GroundCacheMisses` reported "the misses of the cache object used this step". Pooled caches are reused and never reset, so the value jumped between pool slots and was not per vehicle. It is now a per-vehicle monotonic total built from per-step deltas (`OrdVehicleSim.cpp:226–228`), plus per-step and per-frame counts.

## 11. Ground-cache freshness under frame hitches

**Defect fixed.** The strip horizon was a fixed 0.075 s. A cache must cover the time from the physics step that requested it until the step that receives the next cache: the output latency, plus one game frame, plus any hitch.
- The physics thread now measures exactly that: `GroundCacheAge = end of step − request time` (`OrdVehicleSim.cpp:232`). The request time travels as `StripRequestTime` → `GroundRequestTime`.
- The game thread adapts the horizon in `OrdGroundHorizon.h:49`: `clamp(max(min, 1.5·peak frame dt, 1.25·peak cache age + step), 1/60 s, 0.25 s)`. Peaks decay at 0.02 s per second, the value is quantised to 1/120 s, and there is 20% hysteresis.
- A horizon change replaces the pooled cache in place, because `Settings` are fixed per cache object (`OpenRoadVehicleComponent.cpp:97–101`).

**Headless (full throttle, two 100 ms hitches 1 s apart, 60 Hz frames, 1-frame output latency):**

| Horizon | Misses in the second hitch |
|---|---|
| Fixed | 248 |
| Adaptive | 0 |

The first, unannounced hitch cannot be predicted. The core extrapolates the nearest strip (flat continuation) and counts the misses. With the default 0.075 s that is safe on flat ground. Over curbs or crests a long first hitch can cause one frame of mis-predicted contact.

**Telemetry.** `FOrdDiagnostics` reports misses (total and last frame), the horizon, cache age and transient allocations. The demo readout shows them.

**Check in a real UE build:** the README frame-hitch tests. The horizon should rise after the first hitch, with no misses on a repeated one. Also measure the trace count per frame with Insights (`OpenRoad_GroundPrefetch`) at 0.25 s and 300 km/h.

## 12. Component construction, destruction and cleanup

**Defects fixed**
- Delegate lifetime: `AddDynamic` could assert on re-activation, and the binding was removed too early. It now uses `BindBody` with `AddUniqueDynamic`, bound from activation to EndPlay.
- Telemetry capture survives a re-activation (`KeptRecorder`, `:253`).
- Solver teardown race: see item 1.
- `OnUnregister` backstop: see item 1.

**Risk.** `TPimplPtr<FOrdComponentRuntime>` as a UCLASS member (README item 15).

## 13. BeginPlay ordering with the physics body

**Evidence.** Component BeginPlay runs after registration, when the root's physics state exists. `ConfigureBody` runs on the game thread before registration, and the physics thread sees the new mass properties a few steps later.

**Defects fixed**
- If the root was not yet simulating at BeginPlay (for example the owner calls `SetSimulatePhysics(true)` in its own BeginPlay afterwards), the vehicle never activated: the tick was disabled and no `Created` event follows `SetSimulatePhysics`. It now keeps ticking and activates once the body simulates.
- The mass-mismatch warning compared the very first snapshot, before `ConfigureBody` had reached the physics thread, so it could fire falsely. It now waits for 0.5 s of simulation.

## 14. Multiple vehicles, PIE restart, GC

**Source review**
- Each component has its own callback, cache pool and runtime, and there is no static mutable state apart from the two console variables. PIE restart therefore gets fresh components.
- The physics thread holds no UObject pointer, so GC cannot invalidate anything it reads.
- `Body` and `Definition` are UPROPERTYs.

**Check in a real UE build:**
- 20 vehicles at once, measuring callback cost with Insights. A batched world-level callback is the documented next step.
- PIE start/stop 20 times while driving, with `-stompmalloc`.

## 15. Kinematic/static steps and host-sync bookkeeping

**Defect fixed.** On Static/Kinematic steps the old bridge skipped the core entirely. When simulation resumed:
- `setChassisState` compared the host state with a stale prediction from before the pause, which inflated the `hostSync` maxima;
- the core carried that stale difference into the unsprung wheels, as long as it was under 5 cm.

`FOrdVehicleSim::SyncOnly` (`OrdVehicleSim.cpp:135`, called at `OrdChaosBridge.cpp:157`) now syncs the chassis on such steps and consumes the prediction. Sleeping bodies still step without the wrench being applied. The resulting one-step difference is small and physically reasonable (the wheel stays where it was).

## 16. Header compile defect

`OrdVehicleSim.h` needs `ord::RigidBodyState` in a declaration while only forward-declaring `ord::Vehicle`. The headless build caught this after the refactor, and a forward declaration fixes it. The UE build would have failed the same way.

## 17. 60 Hz world + 4 vehicle substeps (configuration B): parity

**Headless result**
- The 0–100 km/h time matches the 240 Hz world within 0.33% (6.250 s vs 6.229 s).
- The per-step host-sync position error is 0.14 cm, and the end point of an 8 s throttle/steer/brake script differs by 2.2% (1.1 m of 52 m).

**Explanation.** The host integrates a single 1/60 s step with the impulse-equivalent average wrench. That gives the same velocity as the substeps but a position that differs by O(a·dt²). This is inherent to the core's rate-decoupling design (ADR-0003), not the adapter. The README previously claimed 0.1% for B without saying which metric. **Left open for the lead:** a core-side change could reduce this (for example handing the host the predicted end pose, or a position-corrected wrench). Until then, use configuration A (240 Hz world) for handling parity.
