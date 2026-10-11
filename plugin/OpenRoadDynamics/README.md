# OpenRoad Dynamics: Unreal Engine 5 plugin (v0.2)

This plugin runs the engine-independent OpenRoad Dynamics core (`src/core`) inside Chaos async physics. The core owns the tires, suspension, drivetrain and assists. Chaos owns the chassis rigid body and all collisions between bodies.

> **Status: written against the UE 5.4/5.5 C++ API, but never compiled.** The repository's CI has no Unreal Engine, so the plugin has never been built or run in UE. Everything we could not check is listed under [Unverified API assumptions](#unverified-api-assumptions). Work through that list the first time you build.
>
> What *has* been checked headless (GCC/Clang, with small stand-ins for the UE math types). Rerun it with `Extras/HeadlessCheck/run.sh`:
> - The core amalgamation compiles as one translation unit with `-Wall -Wextra -Wshadow-all -Werror -fno-exceptions -fno-rtti`.
> - `FOrdVehicleSim` (the physics-thread wrapper) is driven by a rigid-body integrator written in **UE axes and units**. With a 240 Hz world it matches a `ChassisMode::Internal` run to 4e-13 m over an 8 s throttle/steer/brake script. Its host-sync error is 2e-12 cm.
> - A 60 Hz world with 4 vehicle substeps matches the 240 Hz world's 0–100 km/h time within 0.4%. Its trajectory drifts about 2% on a hard steering script (`docs/AUDIT.md` item 17).
> - Ground-cache misses are counted per vehicle. With the adaptive strip horizon, a repeated 100 ms game-thread hitch causes no misses; a fixed 0.075 s horizon gives 248.
>
> See `docs/AUDIT.md` for the full engineering audit and `docs/PROVING_GROUND.md` for a drivable demo in an empty project.

## Contents

```
OpenRoadDynamics.uplugin
Source/OpenRoadCore/OpenRoadCore.Build.cs        External (include-path) module: locates the core
Content/Python/build_proving_ground.py          editor script: proving-ground level from basic shapes
docs/AUDIT.md, docs/PROVING_GROUND.md           audit findings; demo setup, parity procedure, troubleshooting
Extras/HeadlessCheck/                           engine-free check of the physics-thread code (not built by UBT)
Source/OpenRoadDynamics/
  Public/OpenRoadVehicleComponent.h              UOpenRoadVehicleComponent (Blueprint API)
  Public/OpenRoadDemoVehicle.h                   AOpenRoadDemoVehicle / AOpenRoadDemoGameMode (demo only)
  Public/OpenRoadVehicleDefinition.h             UOpenRoadVehicleDefinition data asset
  Public/OpenRoadSurfaceTable.h                  UOpenRoadSurfaceTable: UPhysicalMaterial -> surface
  Public/OpenRoadTypes.h                         presets, devices, assist overrides
  Private/OrdChaosBridge.{h,cpp}                 the ONLY file that uses Chaos / body-instance APIs
  Private/OrdVehicleSim.{h,cpp}                  physics-thread vehicle wrapper + cross-thread data
  Private/OrdUnrealGround.{h,cpp}                ord::GroundQuery over UWorld line traces (game thread)
  Private/OrdFrame.h                             FVector/FQuat wrappers over ord/host_frame.hpp
  Private/OrdGroundHorizon.h                     adaptive ground-strip horizon (engine-free)
  Private/Tests/OpenRoadAdapterTests.cpp         UE automation tests, no world needed (unverified)
  Private/OrdConversions.{h,cpp}                 Blueprint enums/structs -> core types
  Private/OrdCoreAmalgamation.cpp                compiles src/core/src/*.cpp unchanged
```

There is no editor module; the data assets are edited with the stock details panel. The demo pawn lives in the runtime module and is the only code that uses Enhanced Input.

## Setup

1. **Package the core** if the plugin will live outside this repository:
   ```sh
   python3 tools/package_plugin.py          # copies src/core + LICENSE + notices into the plugin
   python3 tools/package_plugin.py --check  # CI: amalgamation lists every core source, copy up to date
   ```
   Inside the repository the build finds `src/core` directly, so packaging is optional there.
2. **Copy** `plugin/OpenRoadDynamics` to `<YourProject>/Plugins/OpenRoadDynamics` and regenerate project files. The plugin needs a C++ project.
3. **Project Settings → Engine → Physics → Framerate.** Use one of these two configurations:

   | Setting | A. 240 Hz world | B. 60 Hz world + vehicle substeps (ADR-0003 default) |
   |---|---|---|
   | Tick Physics Async | **on** | **on** |
   | Async Fixed Time Step Size | `0.0041666` | `0.0166667` |
   | Substepping | off | off |
   | Component `VehicleRateHz` | 240 (gives 1 substep) | 240 (gives 4 substeps) |

   B keeps the whole world at 60 Hz while the tires run at 240 Hz. The core integrates its own copy of the chassis across the substeps and hands Chaos the impulse-equivalent average wrench. Without async physics the plugin still runs, but at the variable physics dt, and logs a warning. For parity tests, also set **World Settings → Global Gravity Z = -980.665**, because the core uses 9.80665 m/s² and the UE default is -980.
4. **Create a vehicle definition:** Content Browser → Miscellaneous → Data Asset → `OpenRoadVehicleDefinition`. The defaults describe a generic 1450 kg RWD sports car whose mesh pivot sits on the ground between the axles.
   - **Positions** (centre of mass, axle hardpoints) are in **cm, UE axes, relative to the actor root origin**. An axle's `Hardpoint` is the wheel centre at **full bump**. `|Y|` is the half track: the left wheel is placed at −Y, the right at +Y.
   - **Everything else is SI** (kg, m, N/m, N·s/m, N·m, kg·m²). Steering and toe angles are in **degrees**. Tooltips state the unit of every field.
5. Optionally create an `OpenRoadSurfaceTable` and map your physical materials to friction, rolling resistance, wet falloff, roughness and micro-texture.
6. **Build the pawn:**
   - The root component is a `UPrimitiveComponent` (usually the body's skeletal or static mesh) with **Simulate Physics** on and **scale 1**. Its collision should be the body shell only, with no wheel shapes. The wheels are rays in the core.
   - Add the **OpenRoad Vehicle** component and assign the definition (and surface table).
   - In `BeginPlay` the component overrides the body's mass (`SetMassOverrideInKg`), sets linear and angular damping to 0, moves the centre of mass to the definition's CoM, and scales the inertia tensor to the definition's principal moments. It logs what Chaos actually ended up with.
7. **Bind input** in Blueprint, for example with Enhanced Input:
   ```
   IA_Throttle (Axis1D)  Triggered -> OpenRoadVehicle.SetThrottle(ActionValue)   Completed -> SetThrottle(0)
   IA_Brake    (Axis1D)  Triggered -> SetBrake(ActionValue)                      Completed -> SetBrake(0)
   IA_Steer    (Axis1D)  Triggered -> SetSteering(ActionValue)   // +1 = full RIGHT  Completed -> SetSteering(0)
   IA_Handbrake(Bool)    Started -> SetHandbrake(1)                              Completed -> SetHandbrake(0)
   IA_ShiftUp  (Bool)    Started -> ShiftUp()          IA_ShiftDown Started -> ShiftDown()
   UI: SetPreset(Fun|Sport|Simulation), SetInputDevice(...), SetAssistOverrides(...)
   HUD: GetSpeedKmh(), GetEngineRpm(), GetGear()
   ```
8. **Debug and telemetry:**
   - Console: `openroad.Debug 1` draws contact samples, suspension and tire-force arrows for every vehicle. `openroad.DebugForceScale 0.0002` sets the arrow length in metres per newton.
   - Blueprint: `SetDebugDrawEnabled` draws one vehicle.
   - `StartTelemetryCapture()` / `StopTelemetryCapture()` writes `Saved/OpenRoad/<Actor>_<timestamp>.csv` with the same columns as `ordbench`. There is one row per world step (with substeps, the last substep's frame).

## Architecture

```
 GAME THREAD  (UOpenRoadVehicleComponent::TickComponent, TG_PrePhysics)        PHYSICS THREAD (Chaos async fixed step)
 ───────────────────────────────────────────────────────────────────────        ──────────────────────────────────────────
 (a) PopOutputData_External() loop  ◄── FOrdSimSnapshot (1 per step) ─────────  FOrdSimCallback::OnPreSimulate_Internal()
       latest speed/rpm/gear, telemetry → TelemetryRecorder,                       1. take FOrdVehicleSim on first call (atomic hand-over)
       strip requests, debug primitives                                           2. GetConsumerInput_Internal() (keep latest)
 (b) GroundPrefetchCache::rebuild(FOrdUnrealGround)                              3. proxy->GetPhysicsThreadAPI(): X, R, V, W, CoM
       UWorld::LineTraceSingleByChannel, physical material → surface               4. ISO conversion + CoM transport → setChassisState
       into a pooled cache nobody else references                                 5. Vehicle::stepSubstepped(dt, n, input, *cache)
 (c) GetProducerInputData_External() ── FOrdGameInput (1 per frame) ──────────►  6. chassisForces() → UE units → AddForce/AddTorque
       driver input, shift counters, AssistConfig + revision,                     7. snapshot: telemetry, strip requests, debug draw
       shared_ptr<const GroundPrefetchCache>, debug flags
 (d) DrawDebugLine/Point from the latest snapshot
```

**Ownership and lifetime**

- `ord::Vehicle` is built on the game thread in `BeginPlay`, wrapped in `FOrdVehicleSim`, and handed to the sim callback once through an atomic pointer (release/acquire). After that **only the physics thread touches it**. The game thread reads only snapshots.
- **The physics thread never touches a UObject.** It sees plain structs, a raw particle proxy pointer and an immutable ground cache.
- Ground caches come from a game-thread pool that grows to 16. A cache is rebuilt only when the pool holds the **only** reference (`use_count() == 1` followed by an acquire fence). Once published it is never written again. The physics thread keeps its own `shared_ptr` while it reads.
- **Strip horizon.** The physics thread measures each cache's age: the time since the step whose strip requests built it. The game thread sizes the next strips from that age and its own frame time (`OrdGroundHorizon.h`), between `GroundPredictionSeconds` (minimum) and `MaxGroundPredictionSeconds`.
- `EndPlay`, the body's physics state being destroyed, or `OnUnregister` calls `UnregisterAndFreeSimCallbackObject_External`. Before that, an input with no body is pushed in the same frame. Chaos frees the callback (and with it the vehicle) after the last step that could use it. The component never deletes the vehicle itself. It never calls into a solver that the world no longer owns. If the body's physics state is created again, the vehicle re-activates on the next tick.
- Shift requests are counters, so the physics thread sees exactly one edge per press, however many steps reuse one game-frame input.
- **CoM transport:** if Chaos' centre of mass differs from the core's, the body velocity is moved to the core CoM (`v + ω × r`) and the torque is moved back to the Chaos CoM (`τ + r × F`). The wrench therefore stays exact even if the body setup did not fully take. A mismatch is still logged once, because the mass distribution then differs.

**One callback per vehicle.** Each component registers its own `TSimCallbackObject`. That keeps ownership trivial. A world-level manager that batches all vehicles into one callback (and later parallelises them) is a straightforward next step if callback overhead shows up with many LOD0 cars. LOD1/LOD2 traffic is expected to use `ord::LodManager` instead of components.

**Large worlds.** UE5 positions (LWC) and Chaos `FVec3` are double, and so is the core. Positions are converted directly, with no rebasing. Origin rebasing is not needed for the core: it keeps no world-space state across steps except the ground cache, which is rebuilt every frame. Test it anyway (see the validation plan).

**Build notes**

- *OpenRoadCore is an External module.* The core has no `dllexport`/visibility annotations, and it must stay engine-free and unchanged. A separately compiled OpenRoadCore DLL would therefore export nothing in modular (editor) builds, and OpenRoadDynamics would fail to link. So `OpenRoadCore.Build.cs` only publishes include paths: the packaged `Source/OpenRoadCore/Core`, or else the repository's `src/core`. `OpenRoadDynamics` compiles the core into its own binary through `OrdCoreAmalgamation.cpp`. This deviates from the original "two Runtime modules" plan, and it is the reason the `.uplugin` lists only one module.
- `OpenRoadDynamics` uses `PCHUsage = NoPCHs` and `bUseUnity = false`, so no engine header precedes the amalgamation, and `CppStandard = Cpp20`. Exceptions stay disabled; the core never throws.
- *Warnings.* UE treats shadowing as an error (`-Wshadow` / MSVC C4456–C4459). The core is clean under GCC/Clang `-Wshadow-all -Werror`. The core include roots are added as *system* include paths, which should also silence MSVC `/W4` diagnostics inside the core (unverified). If a core warning still breaks the build, fix it in `src/core`, not in the plugin.

## Unverified API assumptions

Each item below was written from documentation and memory, not compiled. Each line gives the assumption, then what to check. Items 1–12 and 20–22 are in `OrdChaosBridge.cpp` or the component (marked `[UNVERIFIED]` in the code). `docs/AUDIT.md` gives the reasoning per area.

| # | Assumption | What to check |
|---|---|---|
| 1 | `Chaos::TSimCallbackObject<TInput, TOutput, ESimCallbackOptions>` (header `Chaos/SimCallbackObject.h`) accepts `ESimCallbackOptions::Presimulate` as its third argument; we pass it explicitly. `FSimCallbackInput`/`FSimCallbackOutput` live in `Chaos/SimCallbackInput.h`. Input/output types must be default-constructible with a `Reset()` that Chaos calls before pooling. | Template signature in 5.4/5.5. Check that `Reset()` is called: it releases the input's `shared_ptr` to the ground cache. A missing `Reset()` shows up as growing `TransientCacheAllocations` in `GetDiagnostics()`. |
| 2 | `FPhysicsSolverBase::CreateAndRegisterSimCallbackObject_External<T>()` takes no arguments, default-constructs `T` and returns `T*`. `UnregisterAndFreeSimCallbackObject_External(ISimCallbackObject*)` defers the delete until no step can still call the object. | Exact template parameters. On which thread, and when, the object is deleted (breakpoint in `~FOrdSimCallback`). Whether a solver destroyed with callbacks still registered frees them (a leak, not a crash, in our teardown path). |
| 3 | `GetProducerInputData_External()` returns the same input object for the current game frame. `GetConsumerInput_Internal()` returns `const TInput*` and may be null on steps without new input. `GetProducerOutputData_Internal()` returns `TOutput&`. `PopOutputData_External()` returns a handle that is falsy when empty, pops oldest first and returns the output to the pool on destruction. `GetDeltaTime_Internal()` returns the async fixed step. | Read `SimCallbackObject.h`. `PhysicsStepsLastFrame` should equal the physics steps per frame. Check `GetDeltaTime_Internal()` against `AsyncFixedTimeStepSize`. `ClearBody()` relies on the "same object per frame" part, but stays safe if a new object is returned. |
| 4 | `FSingleParticlePhysicsProxy` is in the global namespace. `FBodyInstance::GetPhysicsActorHandle()` returns it (as `FPhysicsActorHandle`). `GetPhysicsThreadAPI()` returns `Chaos::FRigidBodyHandle_Internal*`, or null before the particle exists on the physics thread. | Header `PhysicsProxy/SingleParticlePhysicsProxy.h`. |
| 5 | `FRigidBodyHandle_Internal::X()/R()/V()/W()`: X is the **body (actor) origin**, not the CoM; V is the **velocity of the centre of mass**. 5.4+ may rename these to `GetX()` and so on (set `ORD_CHAOS_USE_GET_ACCESSORS=1`). | Most important semantic check, now built in. The snapshot's **velocity check** compares the finite-difference velocity of the CoM and of the origin with V; the CoM value must be ≈ 0. A large **host-sync error** or `ComMismatchCm` also points here. |
| 6 | `CenterOfMass()` is the body-local CoM offset (cm). `RotationOfMass()` is read for diagnostics only: the core assumes principal axes = body axes. `M()` returns kg. `I()` returns the principal (mass-space) diagonal `TVec3<FRealSingle>` in kg·cm². | `RotationOfMassDeg` ≈ 0 (readout "rotMass"; included in the mass-mismatch warning). If not, use a symmetric collision shell or set the inertia on the physics thread (`SetI`/`SetInvI`). |
| 7 | `ObjectState()` returns `Chaos::EObjectStateType` {Static, Kinematic, Sleeping, Dynamic}. Forces are applied only when `Dynamic`. Static/Kinematic steps only sync the core's chassis (`SyncOnly`). | Enum values and accessor name. |
| 8 | `AddForce(const FVector&, bool bInvalidate)` applies a world-space force at the CoM. `AddTorque` applies a world-space torque. Both accumulate for the current step only and are cleared after integration. We pass `bInvalidate = false`. | **Host-sync error** < 0.1 cm at 240 Hz. Persisting, local-space or mis-scaled forces show up as centimetres of error, growing with speed. Independent test: one step on an isolated body gives Δv = F·dt/m. |
| 9 | `UWorld::GetPhysicsScene()` returns `FPhysScene*` (= `FPhysScene_Chaos`). `GetSolver()` returns `Chaos::FPhysicsSolver*` (= `Chaos::FPBDRigidsSolver`, header `PBDRigidsSolver.h`). | Forward declaration in `OrdChaosBridge.h`. |
| 10 | `FBodyInstance::COMNudge` is a component-local offset from the shape-derived CoM, applied by `UPrimitiveComponent::SetCenterOfMass(FVector, FName)`. `GetBodyInertiaTensor()` returns the current (already scaled) local diagonal inertia. Setting `InertiaTensorScale` and then calling `UpdateMassProperties()` applies the new inertia. | Read the BeginPlay log line "body: mass … CoM … inertia …" and `HostInertiaKgM2`. If the inertia does not take, set it on the physics thread instead. |
| 11 | `UPhysicsSettings::bTickPhysicsAsync` and `AsyncFixedTimeStepSize` are the field names. | `PhysicsEngine/PhysicsSettings.h`. |
| 12 | **Proxy lifetime.** The physics thread receives a raw `FSingleParticlePhysicsProxy*` through each input. It must never see a proxy after Chaos destroyed it. Mitigation: on `OnComponentPhysicsStateChanged(Destroyed)`, an input with no body and the unregistration are pushed in the same frame as the proxy's destruction, so the order of `TermBody` and the broadcast does not matter. `EndPlay` precedes component unregistration. `Created` re-activates the vehicle. | `OnComponentPhysicsStateChanged` is a dynamic multicast with signature `(UPrimitiveComponent*, EComponentPhysicsStateChange::Type)`, fired for both Destroyed and Created. Test `RecreatePhysicsState()` and `SetSimulatePhysics(false/true)` while driving, under `-stompmalloc`. |
| 13 | A `ModuleType.External` module inside a plugin compiles no sources from its directory, so the packaged `Core/src/*.cpp` is not built twice. `PublicSystemIncludePaths` on it propagates to dependents. | If you get duplicate-symbol link errors after packaging, UBT is compiling `Core/src`; move the packaged copy out of the module directory. |
| 14 | `PCHUsageMode.NoPCHs`, `bUseUnity = false`, `CppStandardVersion.Cpp20` and `SetupModulePhysicsSupport(Target)` are accepted in plugin `Build.cs` files. | UBT output. 5.5 may move warning settings to `CppCompileWarningSettings`; none are set here. |
| 15 | `TPimplPtr<FOrdComponentRuntime>` with an incomplete type is a legal UCLASS member (it is not a UPROPERTY; UHT-generated code only needs default construction and destruction). | UHT and `.gen.cpp` compile. Fallback: a raw pointer deleted in `BeginDestroy`. |
| 16 | `TMap<TObjectPtr<UPhysicalMaterial>, FOpenRoadSurface>` works as an edit-anywhere UPROPERTY, and `Find()` accepts a raw pointer. | Editor details panel and lookup. The generator writes it from Python (`unreal.Map`). |
| 17 | `FFileHelper::SaveArrayToFile(TArrayView<const uint8>, const TCHAR*)`, `UE_KINDA_SMALL_NUMBER`, `FVector::ToCompactString()`, `FMath::Max3` and `FMath::CeilToDouble` exist. | Compile. |
| 18 | Gravity (a parity note, not an API). Chaos applies the world gravity (`hostAppliesGravity = true`). The core uses 9.80665 m/s² inside substeps; UE's default is 9.80. | Set Global Gravity Z = −980.665 for parity runs (the proving-ground generator does), or accept a 0.07% difference. |
| 19 | Sleeping. While Chaos keeps the body asleep, the wrench is not applied. The game thread wakes the body on any driver input (`WakeAllRigidBodies`). | A parked car on a slope should not creep. A car woken by a collision should not jump. |
| 20 | Async stepping. With async physics, a stalled game thread does not let Chaos run ahead indefinitely. A hitch frame hands Chaos at most `MaxPhysicsDeltaTime` of simulation. The adaptive horizon does not depend on this; it measures. | `PhysicsStepsLastFrame` and `GroundCacheAgeSeconds` during a 100 ms / 500 ms busy-wait. |
| 21 | Teardown. `TWeakObjectPtr<UWorld>::Get()` returns null for a world being destroyed, and `UWorld::GetPhysicsScene()->GetSolver()` is the solver we registered with while the world lives. | End PIE while driving, with `-stompmalloc`. Close a level with vehicles. Look for "callback not unregistered because its Chaos solver is already gone": that is a leak, not a crash. |
| 22 | Demo: `UInputMappingContext::MapKey` at runtime with `FEnhancedActionKeyMapping::Modifiers`. `UInputModifierDeadZone::LowerThreshold`. `APawn::PawnClientRestart` adds the context. `ConstructorHelpers` finds `/Engine/BasicShapes/{Cube,Cylinder}`. | The demo drives with keyboard and gamepad. See `docs/PROVING_GROUND.md` troubleshooting. |
| 23 | Generator: the editor Python names used in `build_proving_ground.py` (`EditorActorSubsystem`, `LevelEditorSubsystem.new_level`, `DataAssetFactory.data_asset_class`, `set_phys_material_override`, World Settings `default_game_mode`/`global_gravity_z`). | Run it. Cosmetic steps only warn; others raise with the failing name. |
| 24 | Tests: `IMPLEMENT_SIMPLE_AUTOMATION_TEST` with `EAutomationTestFlags::EditorContext \| EngineFilter` compiles on 5.4 (namespaced enum) and 5.5 (enum class). | Session Frontend → Automation → "OpenRoad". |

## Validation plan

**Parity: headless vs UE, target < 1% on every metric.** Use the same vehicle: the asset defaults reproduce `ord::vehicles::referenceSportsRwd()` geometry, with the body origin moved to the ground. Use the SIMULATION preset, a flat plane, `FrictionScale = 1`, Global Gravity Z = −980.665, and both world configurations A and B. Record CSVs with `StartTelemetryCapture`/`StopTelemetryCapture` and compare them with `ordbench` output for the same scenario:

| Scenario | Metric(s) |
|---|---|
| 0–100 km/h, full throttle, auto shift | time to 100 km/h; distance |
| 100–0 km/h, full brake (ABS on and off) | stopping distance; peak decel |
| Skidpad, R = 40 m steady state | lateral g; steer angle; sideslip |
| Step steer at 80 km/h | yaw-rate rise time, overshoot, steady-state yaw rate |

If a metric is off by more than 1%, check the BeginPlay body log and `ComMismatchCm`, mass and inertia first. These cover unverified items 5, 6 and 10.

**Frame hitches.** Stall the game thread for 100 ms and 500 ms, for example with a debug Blueprint node that busy-waits, at 0, 100 and 250 km/h:
- Physics keeps stepping (async), and the car must not fall through or launch. It uses the last ground cache, extrapolating outside its strips.
- Shift presses during the hitch apply exactly once.
- No `bFaulted` snapshots.

**Ground-cache misses.** `GetGroundCacheMisses()` should stay at ~0 when driving at 30, 60 and 120 fps up to 300 km/h, including over curbs and crests. `GetDiagnostics()` shows the adaptive horizon and the measured cache age; the horizon must stay above the age. Misses are expected only during the first frame of an unannounced hitch. Count engine traces per frame with Unreal Insights (`OpenRoad_GroundPrefetch` scope). Expect about 4 wheels × 4–6 rays on smooth roads.

**Large worlds and rebasing.** Repeat the skidpad 100 km from the origin with LWC; results should match near-origin within noise. Drive through a world-origin rebase (if the project uses it). Expect at most one frame of cache misses and no force spike.

**Coupling self-check.** About 2 s into play, each vehicle logs a "coupling check" line: the host-sync error (host-integrated vs predicted chassis) and the velocity-reference check. These values verify items 5 and 8 below on the running engine. `docs/AUDIT.md` items 4 and 6 explain how to read them.

**Threading.**
- Run with `-stompmalloc`, and with many vehicles spawned and destroyed while driving, to catch use-after-free around unregistration (unverified items 2 and 12).
- Build a test variant that `check()`s `IsInGameThread()` in every UObject accessor reached from the component's tick, and `!IsInGameThread()` in `OnPreSimulate_Internal` (with async physics on).

**Assists.** Switch presets while driving. The change must take effect within one physics step, with no state reset (gear and wheel spin continuous).
