// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT

#include "OpenRoadVehicleComponent.h"

#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "HAL/IConsoleManager.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "OpenRoadLog.h"
#include "OpenRoadSurfaceTable.h"
#include "OpenRoadVehicleDefinition.h"
#include "OrdChaosBridge.h"
#include "OrdConversions.h"
#include "OrdFrame.h"
#include "OrdGroundHorizon.h"
#include "OrdUnrealGround.h"
#include "OrdVehicleSim.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ord/ground_cache.hpp"
#include "ord/telemetry_recorder.hpp"
#include "ord/vehicle.hpp"

static TAutoConsoleVariable<int32> CVarOpenRoadDebug(
	TEXT("openroad.Debug"), 0,
	TEXT("OpenRoad Dynamics debug overlay. 0 = per-component setting, 1 = draw every vehicle."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarOpenRoadDebugForceScale(
	TEXT("openroad.DebugForceScale"), 1.0f / 5000.0f,
	TEXT("OpenRoad Dynamics debug overlay: metres of arrow per newton of tire force."),
	ECVF_Default);

/** Game-thread runtime state of one vehicle component. */
struct FOrdComponentRuntime
{
	FOrdChaosVehicleLink Link;

	// ---- Ground caches ----
	// A cache is mutable only while the pool holds the sole reference. Once
	// published in an input it is immutable until every other holder (Chaos'
	// input objects, the callback's latest-input copy, FOrdVehicleSim::Ground)
	// has released it. Holders at any time: one per game frame the physics
	// thread has not consumed yet, plus two on the physics thread. The pool
	// grows on demand up to MaxPoolSize so a lagging physics thread does not
	// force an allocation every frame.
	static constexpr std::size_t MaxPoolSize = 16;
	std::vector<std::shared_ptr<ord::GroundPrefetchCache>> CachePool;
	ord::GroundPrefetchCache::Settings CacheSettings;
	std::vector<ord::GroundStripRequest> Requests;  // latest from the physics thread
	double RequestTime = 0.0;                       // core time those requests describe
	int32 TransientCacheAllocations = 0;

	// ---- Adaptive strip horizon (see UOpenRoadVehicleComponent::bAdaptiveGroundHorizon) ----
	OrdGround::FHorizonState Horizon;
	double FrameCacheAge = 0.0; // s, max age in the last frame (diagnostics)
	uint32 FrameMisses = 0;     // misses in the last frame (diagnostics)
	int32 FrameSteps = 0;       // snapshots consumed in the last frame

	FOrdSimSnapshot Latest;  // last valid physics snapshot
	bool bHasSnapshot = false;
	int32 NumWheels = 0;

	std::unique_ptr<ord::TelemetryRecorder> Recorder;
	double FixedDt = 1.0 / 240.0;
	bool bReportedMassMismatch = false;
	bool bReportedCoupling = false;

	FOrdComponentRuntime() { CachePool.reserve(MaxPoolSize); }

	/** Game thread. Returns a cache no other thread references, configured with CacheSettings. */
	std::shared_ptr<ord::GroundPrefetchCache> AcquireCache()
	{
		for (std::shared_ptr<ord::GroundPrefetchCache>& Slot : CachePool)
		{
			if (Slot.use_count() == 1)
			{
				// The other threads released their references with acq_rel
				// decrements; this fence orders their last reads before our
				// rebuild. No ABA: with use_count() == 1 nobody else holds a
				// shared_ptr (or a raw pointer) to copy from, so the count
				// cannot rise again behind our back.
				std::atomic_thread_fence(std::memory_order_acquire);
				if (Slot->settings().horizon != CacheSettings.horizon)
				{
					// Settings are fixed per cache object: replace it in place
					// (exclusively owned, so this is safe; strip memory regrows once).
					*Slot = ord::GroundPrefetchCache(CacheSettings);
				}
				return Slot;
			}
		}
		if (CachePool.size() < MaxPoolSize)
		{
			CachePool.push_back(std::make_shared<ord::GroundPrefetchCache>(CacheSettings));
			return CachePool.back();
		}
		// Every pooled cache is still referenced (physics thread far behind): correct, but allocates.
		++TransientCacheAllocations;
		return std::make_shared<ord::GroundPrefetchCache>(CacheSettings);
	}

	/** Game thread, once per frame after consuming the snapshots (see OrdGroundHorizon.h). */
	void UpdateHorizon(double FrameDt, double PhysicsTimeThisFrame, bool bAdaptive, double MinHorizon, double MaxHorizon)
	{
		CacheSettings.horizon = OrdGround::UpdateHorizon(Horizon, FrameDt, PhysicsTimeThisFrame, FrameCacheAge, FixedDt,
		                                                 bAdaptive, MinHorizon, MaxHorizon);
	}
};

UOpenRoadVehicleComponent::UOpenRoadVehicleComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;  // inputs must be pushed before this frame's physics advance
}

// ============================================================================
// Lifecycle
// ============================================================================

void UOpenRoadVehicleComponent::BeginPlay()
{
	Super::BeginPlay();
	if (!ActivateVehicle() && !bPendingReactivate)
	{
		SetComponentTickEnabled(false);
	}
}

void UOpenRoadVehicleComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// EndPlay runs before the owner's components are unregistered (actor
	// destroy, level unload, PIE end), i.e. before the body's proxy is queued
	// for destruction: the unregistration below reaches Chaos first.
	if (IsCapturingTelemetry())
	{
		StopTelemetryCapture();
	}
	DeactivateVehicle(TEXT("EndPlay"));
	BindBody(nullptr);
	bPendingReactivate = false;
	Super::EndPlay(EndPlayReason);
}

void UOpenRoadVehicleComponent::OnUnregister()
{
	if (Runtime && Runtime->Link.IsRegistered())
	{
		DeactivateVehicle(TEXT("component unregistered"));
	}
	Super::OnUnregister();
}

void UOpenRoadVehicleComponent::BindBody(UPrimitiveComponent* NewBody)
{
	if (Body == NewBody)
	{
		return;
	}
	if (Body)
	{
		Body->OnComponentPhysicsStateChanged.RemoveDynamic(this, &UOpenRoadVehicleComponent::HandleBodyPhysicsStateChanged);
	}
	Body = NewBody;
	if (Body)
	{
		Body->OnComponentPhysicsStateChanged.AddUniqueDynamic(this, &UOpenRoadVehicleComponent::HandleBodyPhysicsStateChanged);
	}
}

bool UOpenRoadVehicleComponent::ActivateVehicle()
{
	check(IsInGameThread());
	AActor* Owner = GetOwner();
	UWorld* World = GetWorld();
	UPrimitiveComponent* Root = Owner ? Cast<UPrimitiveComponent>(Owner->GetRootComponent()) : nullptr;
	// Listen to the root's physics state even if it cannot be driven yet, so a
	// later (re)creation of the body re-activates the vehicle.
	BindBody(Root);
	if (!World || !Root || !Root->IsSimulatingPhysics())
	{
		UE_LOG(LogOpenRoad, Error, TEXT("OpenRoad: %s needs a root UPrimitiveComponent with Simulate Physics enabled%s."),
		       *GetNameSafe(Owner), Root ? TEXT(" (the vehicle activates as soon as it simulates)") : TEXT(""));
		// Simulation may be switched on later (SetSimulatePhysics(true) does not
		// re-create the physics state, so no Created event would arrive): keep
		// ticking and retry once the body simulates.
		bPendingReactivate = Root != nullptr;
		return false;
	}
	if (!Definition)
	{
		UE_LOG(LogOpenRoad, Error, TEXT("OpenRoad: %s has no vehicle definition asset."), *GetNameSafe(Owner));
		return false;
	}

	ord::VehicleDefinition Def;
	FString Error;
	if (!Definition->ToCore(Def, Error))
	{
		UE_LOG(LogOpenRoad, Error, TEXT("OpenRoad: invalid vehicle definition: %s"), *Error);
		return false;
	}

	ord::VehicleFidelity Fidelity;  // LOD0 defaults, including the envelope probes
	Fidelity.unsprungMass = bUnsprungMass;
	Fidelity.contactSamples = ContactSamples;
	Fidelity.telemetry = true;

	auto Vehicle = std::make_unique<ord::Vehicle>(Def, OrdConvert::MakeAssistConfig(Preset, InputDevice, AssistOverrides), Fidelity);
	if (!Vehicle->isValid())
	{
		UE_LOG(LogOpenRoad, Error, TEXT("OpenRoad: vehicle rejected: %s"), UTF8_TO_TCHAR(Vehicle->error().c_str()));
		return false;
	}
	// Chaos integrates the chassis and applies gravity; the core supplies the wheel/aero wrench.
	Vehicle->setChassisMode(ord::ChassisMode::External, /*hostAppliesGravity*/ true);

	FString BodyReport;
	if (!FOrdChaosVehicleLink::ConfigureBody(*Root, Def.mass, OrdFrame::PositionToUe(Def.centerOfMass),
	                                         OrdFrame::InertiaToUe(Def.inertia), BodyReport))
	{
		UE_LOG(LogOpenRoad, Warning, TEXT("OpenRoad: %s body setup incomplete: %s"), *GetNameSafe(Owner), *BodyReport);
	}
	else
	{
		UE_LOG(LogOpenRoad, Log, TEXT("OpenRoad: %s body: %s"), *GetNameSafe(Owner), *BodyReport);
	}

	// Initial state, so the first ground strips are requested in the right place.
	// (The vehicle is still exclusively ours: it is handed to the physics thread below.)
	ord::RigidBodyState Initial;
	Initial.position = OrdFrame::PositionFromUe(Root->GetComponentLocation());
	Initial.orientation = OrdFrame::RotationFromUe(Root->GetComponentQuat());
	Initial.velocity = OrdFrame::VelocityFromUe(Root->GetPhysicsLinearVelocity());
	Initial.angularVelocity = OrdFrame::AngularVelocityFromUe(Root->GetPhysicsAngularVelocityInRadians());
	Vehicle->setChassisState(Initial);

	// A re-activation keeps an ongoing telemetry capture.
	std::unique_ptr<ord::TelemetryRecorder> KeptRecorder = Runtime ? std::move(Runtime->Recorder) : nullptr;
	Runtime = MakePimpl<FOrdComponentRuntime>();
	FOrdComponentRuntime& R = *Runtime;
	R.Recorder = std::move(KeptRecorder);
	R.Horizon.Horizon = FMath::Max(static_cast<double>(GroundPredictionSeconds), 1.0 / 60.0);
	R.CacheSettings.horizon = R.Horizon.Horizon;
	R.NumWheels = static_cast<int32>(Def.wheels.size());
	Vehicle->groundPrefetchRequests(R.Requests);
	R.RequestTime = Vehicle->time();

	float FixedStep = 0.0f;
	if (FOrdChaosVehicleLink::GetAsyncPhysicsSettings(FixedStep) && FixedStep > 0.0f)
	{
		R.FixedDt = FixedStep;
	}
	else
	{
		UE_LOG(LogOpenRoad, Warning,
		       TEXT("OpenRoad: 'Tick Physics Async' is off. The vehicle runs at the variable physics dt; enable async "
		            "physics with a fixed step (e.g. 1/240 s) for reproducible handling (see plugin README)."));
	}

	if (!R.Link.Register(*World, *Root, MakeUnique<FOrdVehicleSim>(std::move(Vehicle)), Error))
	{
		UE_LOG(LogOpenRoad, Error, TEXT("OpenRoad: %s could not attach to Chaos: %s"), *GetNameSafe(Owner), *Error);
		return false;
	}
	bPendingReactivate = false;
	return true;
}

void UOpenRoadVehicleComponent::DeactivateVehicle(const TCHAR* Reason)
{
	check(IsInGameThread());
	if (Runtime && Runtime->Link.IsRegistered())
	{
		Runtime->Link.Unregister();  // the vehicle is destroyed with the callback, after its last step
		UE_LOG(LogOpenRoad, Verbose, TEXT("OpenRoad: %s deactivated (%s)."), *GetNameSafe(GetOwner()), Reason);
	}
}

void UOpenRoadVehicleComponent::HandleBodyPhysicsStateChanged(UPrimitiveComponent* ChangedComponent,
                                                              EComponentPhysicsStateChange::Type StateChange)
{
	if (ChangedComponent != Body || !HasBegunPlay())
	{
		return;
	}
	if (StateChange == EComponentPhysicsStateChange::Destroyed)
	{
		if (Runtime && Runtime->Link.IsRegistered())
		{
			UE_LOG(LogOpenRoad, Log, TEXT("OpenRoad: %s body physics state destroyed; vehicle deactivated until it is re-created."),
			       *GetNameSafe(GetOwner()));
			// Whether this fires before or after the body's TermBody(), the proxy
			// is destroyed with THIS frame's physics data; the input without a
			// body and the unregistration travel in the same frame's data.
			Runtime->Link.ClearBody();
			DeactivateVehicle(TEXT("body physics state destroyed"));
		}
	}
	else if (StateChange == EComponentPhysicsStateChange::Created)
	{
		// Deferred to the next tick: re-configuring mass properties from inside
		// the body's physics-state creation is avoided on purpose.
		bPendingReactivate = true;
		SetComponentTickEnabled(true);
	}
}

// ============================================================================
// Per-frame flow (game thread, TG_PrePhysics)
// ============================================================================

void UOpenRoadVehicleComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	TRACE_CPUPROFILER_EVENT_SCOPE(OpenRoad_VehicleTick);

	if (!Runtime || !Runtime->Link.IsRegistered())
	{
		// Re-activate once the body's physics state exists again.
		if (bPendingReactivate && IsValid(Body) && Body->IsSimulatingPhysics() && Body->GetBodyInstance() &&
		    Body->GetBodyInstance()->IsValidBodyInstance())
		{
			bPendingReactivate = false;
			if (ActivateVehicle())
			{
				UE_LOG(LogOpenRoad, Log, TEXT("OpenRoad: %s re-activated on its new physics body."), *GetNameSafe(GetOwner()));
			}
		}
		return;
	}
	if (!IsValid(Body))
	{
		DeactivateVehicle(TEXT("body component gone"));
		return;
	}
	FOrdComponentRuntime& R = *Runtime;

	// (a) Consume every physics snapshot since last frame (one per physics step).
	R.FrameCacheAge = 0.0;
	R.FrameMisses = 0;
	double PhysicsTimeThisFrame = 0.0;
	R.FrameSteps = R.Link.ConsumeOutputs([&R, &PhysicsTimeThisFrame](const FOrdSimSnapshot& Snapshot)
	{
		PhysicsTimeThisFrame += Snapshot.DeltaTime;
		if (Snapshot.NumStripRequests > 0)
		{
			R.Requests.assign(Snapshot.StripRequests.begin(), Snapshot.StripRequests.begin() + Snapshot.NumStripRequests);
			R.RequestTime = Snapshot.StripRequestTime;
		}
		if (!Snapshot.bValid)
		{
			return;
		}
		R.FrameCacheAge = FMath::Max(R.FrameCacheAge, Snapshot.GroundCacheAge);
		R.FrameMisses += Snapshot.GroundCacheMissesStep;
		R.Latest = Snapshot;
		R.bHasSnapshot = true;
		if (R.Recorder && Snapshot.bHasTelemetry)
		{
			R.Recorder->push(Snapshot.Telemetry);
		}
	});

	// Checked after 0.5 s of simulation: the BeginPlay body setup reaches the physics thread a few steps late.
	if (R.bHasSnapshot && !R.bReportedMassMismatch && Definition && R.Latest.SimTime > 0.5)
	{
		const double MassError = std::abs(R.Latest.HostMassKg - Definition->MassKg) / FMath::Max(1.0f, Definition->MassKg);
		const FVector InertiaError = (R.Latest.HostInertiaKgM2 - Definition->InertiaKgM2) / Definition->InertiaKgM2;
		if (R.Latest.ComMismatchCm > 1.0 || MassError > 0.01 || InertiaError.GetAbsMax() > 0.05 || R.Latest.RotationOfMassDeg > 1.0)
		{
			UE_LOG(LogOpenRoad, Warning,
			       TEXT("OpenRoad: %s Chaos mass properties differ from the definition (CoM off by %.1f cm, mass %.1f kg, "
			            "inertia %s kg*m^2, principal frame rotated %.1f deg). Forces stay consistent, but handling parity "
			            "with headless runs will suffer."),
			       *GetNameSafe(GetOwner()), R.Latest.ComMismatchCm, R.Latest.HostMassKg,
			       *R.Latest.HostInertiaKgM2.ToCompactString(), R.Latest.RotationOfMassDeg);
		}
		R.bReportedMassMismatch = true;
	}
	// One-time coupling report after ~2 s of simulation (README items 5 and 8).
	if (R.bHasSnapshot && !R.bReportedCoupling && R.Latest.SimTime > 2.0)
	{
		UE_LOG(LogOpenRoad, Log,
		       TEXT("OpenRoad: %s coupling check: host-sync error %.3f cm / %.3f deg (max %.3f cm); velocity check CoM %.2f cm/s, "
		            "origin %.2f cm/s (the ~0 one is what Chaos V() refers to)."),
		       *GetNameSafe(GetOwner()), R.Latest.HostSyncPositionErrorCm, R.Latest.HostSyncAngleErrorDeg,
		       R.Latest.HostSyncMaxPositionErrorCm, R.Latest.VelocityCheckComCmS, R.Latest.VelocityCheckOriginCmS);
		R.bReportedCoupling = true;
	}

	// (b) Ground: one batch of line traces into a fresh (unshared) cache, with
	// a strip horizon that covers the measured latency until the next cache.
	R.UpdateHorizon(DeltaTime, PhysicsTimeThisFrame, bAdaptiveGroundHorizon, GroundPredictionSeconds, MaxGroundPredictionSeconds);
	std::shared_ptr<ord::GroundPrefetchCache> Cache = R.AcquireCache();
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(OpenRoad_GroundPrefetch);
		const FOrdUnrealGround Ground(GetWorld(), GroundTraceChannel.GetValue(), GetOwner(), SurfaceTable);
		Cache->rebuild(Ground, R.Requests.data(), R.Requests.size());
	}

	// (c) Publish this frame's input. From here on the cache is immutable.
	FOrdGameInput Input;
	Input.Driver.throttle = Throttle;
	Input.Driver.brake = Brake;
	Input.Driver.steer = -Steering;  // UE: + = right; core (ISO): + = left
	Input.Driver.handbrake = Handbrake;
	Input.Driver.clutch = Clutch;
	Input.Driver.device = OrdConvert::ToCore(InputDevice);
	Input.ShiftUpCount = ShiftUpCount;
	Input.ShiftDownCount = ShiftDownCount;
	Input.Assists = OrdConvert::MakeAssistConfig(Preset, InputDevice, AssistOverrides);
	Input.AssistsRevision = AssistsRevision;
	Input.Ground = std::move(Cache);
	Input.GroundRequestTime = R.RequestTime;
	Input.VehicleRateHz = VehicleRateHz;
	Input.bDebugDraw = bDebugDraw || CVarOpenRoadDebug.GetValueOnGameThread() != 0;
	Input.DebugForceScale = CVarOpenRoadDebugForceScale.GetValueOnGameThread();

	R.Link.RefreshBody(*Body);
	R.Link.PushInput(Input);

	// A sleeping body ignores forces; wake it when the driver asks for something.
	const bool bDriverActive = Throttle > 0.0f || Brake > 0.0f || FMath::Abs(Steering) > 0.0f || Handbrake > 0.0f;
	if (bDriverActive && !Body->IsAnyRigidBodyAwake())
	{
		Body->WakeAllRigidBodies();
	}

	if (Input.bDebugDraw)
	{
		DrawDebug();
	}
}

void UOpenRoadVehicleComponent::DrawDebug() const
{
#if ENABLE_DRAW_DEBUG
	const UWorld* World = GetWorld();
	if (!World || !Runtime || !Runtime->bHasSnapshot)
	{
		return;
	}
	const FOrdSimSnapshot& S = Runtime->Latest;
	for (const FOrdDebugLine& Line : S.DebugLines)
	{
		DrawDebugLine(World, Line.A, Line.B, Line.Color, /*bPersistent*/ false, /*LifeTime*/ -1.0f, /*DepthPriority*/ 0, /*Thickness*/ 1.5f);
	}
	for (const FOrdDebugPoint& Point : S.DebugPoints)
	{
		DrawDebugPoint(World, Point.Position, Point.Size, Point.Color, /*bPersistent*/ false, /*LifeTime*/ -1.0f);
	}
#endif
}

// ============================================================================
// Blueprint API
// ============================================================================

void UOpenRoadVehicleComponent::SetThrottle(float Value) { Throttle = FMath::Clamp(Value, 0.0f, 1.0f); }
void UOpenRoadVehicleComponent::SetBrake(float Value) { Brake = FMath::Clamp(Value, 0.0f, 1.0f); }
void UOpenRoadVehicleComponent::SetSteering(float Value) { Steering = FMath::Clamp(Value, -1.0f, 1.0f); }
void UOpenRoadVehicleComponent::SetHandbrake(float Value) { Handbrake = FMath::Clamp(Value, 0.0f, 1.0f); }
void UOpenRoadVehicleComponent::SetClutch(float Value) { Clutch = FMath::Clamp(Value, 0.0f, 1.0f); }
void UOpenRoadVehicleComponent::ShiftUp() { ++ShiftUpCount; }
void UOpenRoadVehicleComponent::ShiftDown() { ++ShiftDownCount; }

void UOpenRoadVehicleComponent::SetPreset(EOpenRoadPreset NewPreset)
{
	Preset = NewPreset;
	++AssistsRevision;
}

void UOpenRoadVehicleComponent::SetInputDevice(EOpenRoadInputDevice NewDevice)
{
	InputDevice = NewDevice;
	++AssistsRevision;
}

void UOpenRoadVehicleComponent::SetAssistOverrides(const FOpenRoadAssistOverrides& NewOverrides)
{
	AssistOverrides = NewOverrides;
	++AssistsRevision;
}

void UOpenRoadVehicleComponent::SetAbsEnabled(bool bEnabled)
{
	AssistOverrides.bOverrideAbs = true;
	AssistOverrides.bAbs = bEnabled;
	++AssistsRevision;
}

void UOpenRoadVehicleComponent::SetTractionControl(EOpenRoadIntervention Level)
{
	AssistOverrides.bOverrideTractionControl = true;
	AssistOverrides.TractionControl = Level;
	++AssistsRevision;
}

void UOpenRoadVehicleComponent::SetStabilityControl(EOpenRoadIntervention Level)
{
	AssistOverrides.bOverrideStabilityControl = true;
	AssistOverrides.StabilityControl = Level;
	++AssistsRevision;
}

void UOpenRoadVehicleComponent::SetShiftMode(EOpenRoadShiftMode Mode)
{
	AssistOverrides.bOverrideShiftMode = true;
	AssistOverrides.ShiftMode = Mode;
	++AssistsRevision;
}

namespace
{
EOpenRoadIntervention FromCore(ord::InterventionLevel Level)
{
	switch (Level)
	{
	case ord::InterventionLevel::Off: return EOpenRoadIntervention::Off;
	case ord::InterventionLevel::Sport: return EOpenRoadIntervention::Sport;
	case ord::InterventionLevel::Full: return EOpenRoadIntervention::Full;
	}
	return EOpenRoadIntervention::Off;
}
}  // namespace

FOpenRoadAssistState UOpenRoadVehicleComponent::GetAssistState() const
{
	FOpenRoadAssistState Out;
	const ord::AssistConfig Config = OrdConvert::MakeAssistConfig(Preset, InputDevice, AssistOverrides);
	Out.bAbs = Config.abs;
	Out.TractionControl = FromCore(Config.tractionControl);
	Out.StabilityControl = FromCore(Config.stabilityControl);
	Out.ShiftMode = Config.shiftMode == ord::ShiftMode::Manual ? EOpenRoadShiftMode::Manual : EOpenRoadShiftMode::Automatic;
	if (Runtime && Runtime->bHasSnapshot && Runtime->Latest.bHasTelemetry)
	{
		const ord::AssistReport& Report = Runtime->Latest.Telemetry.assists;
		for (std::size_t I = 0; I < ord::kMaxWheels; ++I)
		{
			Out.bAbsActive |= Report.absActive[I];
			Out.bEscActive |= Report.escBrakeTorque[I] > 0.0;
		}
		Out.TcsThrottleCut = static_cast<float>(Report.tcsThrottleCut);
	}
	return Out;
}

float UOpenRoadVehicleComponent::GetSpeedKmh() const
{
	return Runtime && Runtime->bHasSnapshot ? static_cast<float>(Runtime->Latest.SpeedKmh) : 0.0f;
}

float UOpenRoadVehicleComponent::GetEngineRpm() const
{
	return Runtime && Runtime->bHasSnapshot ? static_cast<float>(Runtime->Latest.EngineRpm) : 0.0f;
}

int32 UOpenRoadVehicleComponent::GetGear() const
{
	return Runtime && Runtime->bHasSnapshot ? Runtime->Latest.Gear : 0;
}

bool UOpenRoadVehicleComponent::IsVehicleActive() const
{
	return Runtime && Runtime->Link.IsRegistered();
}

int64 UOpenRoadVehicleComponent::GetGroundCacheMisses() const
{
	return Runtime && Runtime->bHasSnapshot ? static_cast<int64>(Runtime->Latest.GroundCacheMisses) : 0;
}

int32 UOpenRoadVehicleComponent::GetNumWheels() const
{
	return Runtime ? Runtime->NumWheels : 0;
}

bool UOpenRoadVehicleComponent::GetWheelState(int32 WheelIndex, FOpenRoadWheelState& OutState) const
{
	OutState = FOpenRoadWheelState();
	if (!Runtime || !Runtime->bHasSnapshot || !Runtime->Latest.bHasTelemetry)
	{
		return false;
	}
	const ord::TelemetryFrame& T = Runtime->Latest.Telemetry;
	if (WheelIndex < 0 || static_cast<std::size_t>(WheelIndex) >= FMath::Min<std::size_t>(T.wheelCount, ord::kMaxWheels))
	{
		return false;
	}
	const ord::WheelTelemetry& W = T.wheels[static_cast<std::size_t>(WheelIndex)];
	OutState.bContact = W.contact;
	OutState.LoadN = static_cast<float>(W.load);
	OutState.SlipRatio = static_cast<float>(W.slipRatio);
	// Lateral quantities change sign with the Y mirror between ISO and UE.
	OutState.SlipAngleDeg = static_cast<float>(-FMath::RadiansToDegrees(W.slipAngle));
	OutState.LongitudinalForceN = static_cast<float>(W.fx);
	OutState.LateralForceN = static_cast<float>(-W.fy);
	OutState.ForceOverLoad = W.load > 1.0 ? static_cast<float>(std::sqrt(W.fx * W.fx + W.fy * W.fy) / W.load) : 0.0f;
	OutState.CompressionCm = static_cast<float>(W.compression * ord::host::kUeUnitsPerMetre);
	OutState.SteerAngleDeg = static_cast<float>(-FMath::RadiansToDegrees(W.steerAngle));
	OutState.AngularVelocity = static_cast<float>(W.omega);
	OutState.bAbsActive = T.assists.absActive[static_cast<std::size_t>(WheelIndex)];
	OutState.SurfaceId = W.surfaceId;
	return true;
}

FOpenRoadDiagnostics UOpenRoadVehicleComponent::GetDiagnostics() const
{
	FOpenRoadDiagnostics D;
	if (!Runtime)
	{
		return D;
	}
	const FOrdComponentRuntime& R = *Runtime;
	D.GroundHorizonSeconds = static_cast<float>(R.CacheSettings.horizon);
	D.GroundCacheAgeSeconds = static_cast<float>(R.FrameCacheAge);
	D.GroundCacheMissesLastFrame = static_cast<int32>(R.FrameMisses);
	D.TransientCacheAllocations = R.TransientCacheAllocations;
	D.PhysicsStepsLastFrame = R.FrameSteps;
	if (R.bHasSnapshot)
	{
		const FOrdSimSnapshot& S = R.Latest;
		D.GroundCacheMisses = static_cast<int64>(S.GroundCacheMisses);
		D.HostSyncPositionErrorCm = static_cast<float>(S.HostSyncPositionErrorCm);
		D.HostSyncAngleErrorDeg = static_cast<float>(S.HostSyncAngleErrorDeg);
		D.HostSyncMaxPositionErrorCm = static_cast<float>(S.HostSyncMaxPositionErrorCm);
		D.VelocityCheckComCmS = static_cast<float>(S.VelocityCheckComCmS);
		D.VelocityCheckOriginCmS = static_cast<float>(S.VelocityCheckOriginCmS);
		D.ComMismatchCm = static_cast<float>(S.ComMismatchCm);
		D.HostMassKg = static_cast<float>(S.HostMassKg);
		D.HostInertiaKgM2 = S.HostInertiaKgM2;
		D.RotationOfMassDeg = static_cast<float>(S.RotationOfMassDeg);
		D.bFaulted = S.bFaulted;
	}
	return D;
}

void UOpenRoadVehicleComponent::SetDebugDrawEnabled(bool bEnabled)
{
	bDebugDraw = bEnabled;
}

void UOpenRoadVehicleComponent::StartTelemetryCapture()
{
	if (!Runtime)
	{
		UE_LOG(LogOpenRoad, Warning, TEXT("OpenRoad: telemetry capture needs an active vehicle."));
		return;
	}
	const std::size_t Capacity = static_cast<std::size_t>(FMath::Max(1.0, std::ceil(TelemetryCaptureSeconds / Runtime->FixedDt)));
	Runtime->Recorder = std::make_unique<ord::TelemetryRecorder>(Capacity);
}

FString UOpenRoadVehicleComponent::StopTelemetryCapture()
{
	if (!Runtime || !Runtime->Recorder)
	{
		return FString();
	}
	std::unique_ptr<ord::TelemetryRecorder> Recorder = std::move(Runtime->Recorder);
	if (Recorder->size() == 0)
	{
		return FString();
	}

	std::ostringstream Csv;
	Recorder->writeCsv(Csv);
	const std::string Text = Csv.str();

	const FString Directory = FPaths::ProjectSavedDir() / TEXT("OpenRoad");
	IFileManager::Get().MakeDirectory(*Directory, /*Tree*/ true);
	const FString Path = Directory / FString::Printf(TEXT("%s_%s.csv"), *GetNameSafe(GetOwner()),
	                                                 *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")));
	const TArrayView<const uint8> Bytes(reinterpret_cast<const uint8*>(Text.data()), static_cast<int32>(Text.size()));
	if (!FFileHelper::SaveArrayToFile(Bytes, *Path))
	{
		UE_LOG(LogOpenRoad, Error, TEXT("OpenRoad: could not write telemetry to %s"), *Path);
		return FString();
	}
	UE_LOG(LogOpenRoad, Log, TEXT("OpenRoad: wrote %d telemetry frames to %s"), static_cast<int32>(Recorder->size()), *Path);
	return Path;
}

bool UOpenRoadVehicleComponent::IsCapturingTelemetry() const
{
	return Runtime && Runtime->Recorder != nullptr;
}
