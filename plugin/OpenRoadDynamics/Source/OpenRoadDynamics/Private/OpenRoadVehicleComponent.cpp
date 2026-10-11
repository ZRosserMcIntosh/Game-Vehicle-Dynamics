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

	// Ground caches. A cache is mutable only while the pool holds the sole
	// reference: once published in an input it is immutable until the physics
	// thread and the Chaos input pool have released it.
	static constexpr int32 PoolSize = 6;
	std::array<std::shared_ptr<ord::GroundPrefetchCache>, PoolSize> CachePool;
	ord::GroundPrefetchCache::Settings CacheSettings;
	std::vector<ord::GroundStripRequest> Requests;  // latest from the physics thread
	int32 TransientCacheAllocations = 0;

	FOrdSimSnapshot Latest;  // last valid physics snapshot
	bool bHasSnapshot = false;

	std::unique_ptr<ord::TelemetryRecorder> Recorder;
	double FixedDt = 1.0 / 240.0;
	bool bReportedMassMismatch = false;

	std::shared_ptr<ord::GroundPrefetchCache> AcquireCache()
	{
		for (std::shared_ptr<ord::GroundPrefetchCache>& Slot : CachePool)
		{
			if (!Slot)
			{
				Slot = std::make_shared<ord::GroundPrefetchCache>(CacheSettings);
				return Slot;
			}
			if (Slot.use_count() == 1)
			{
				// The other threads released their references with acq_rel
				// decrements; this fence orders their last reads before our rebuild.
				std::atomic_thread_fence(std::memory_order_acquire);
				return Slot;
			}
		}
		// Every pooled cache is still referenced (inputs not yet recycled): correct, but allocates.
		++TransientCacheAllocations;
		return std::make_shared<ord::GroundPrefetchCache>(CacheSettings);
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
	if (!ActivateVehicle())
	{
		SetComponentTickEnabled(false);
	}
}

void UOpenRoadVehicleComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (IsCapturingTelemetry())
	{
		StopTelemetryCapture();
	}
	DeactivateVehicle(TEXT("EndPlay"));
	Super::EndPlay(EndPlayReason);
}

bool UOpenRoadVehicleComponent::ActivateVehicle()
{
	check(IsInGameThread());
	AActor* Owner = GetOwner();
	UWorld* World = GetWorld();
	UPrimitiveComponent* Root = Owner ? Cast<UPrimitiveComponent>(Owner->GetRootComponent()) : nullptr;
	if (!World || !Root || !Root->IsSimulatingPhysics())
	{
		UE_LOG(LogOpenRoad, Error, TEXT("OpenRoad: %s needs a root UPrimitiveComponent with Simulate Physics enabled."),
		       *GetNameSafe(Owner));
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

	ord::VehicleFidelity Fidelity;
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

	Runtime = MakePimpl<FOrdComponentRuntime>();
	FOrdComponentRuntime& R = *Runtime;
	R.CacheSettings.horizon = GroundPredictionSeconds;
	Vehicle->groundPrefetchRequests(R.Requests);

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

	Body = Root;
	Root->OnComponentPhysicsStateChanged.AddDynamic(this, &UOpenRoadVehicleComponent::HandleBodyPhysicsStateChanged);
	return true;
}

void UOpenRoadVehicleComponent::DeactivateVehicle(const TCHAR* Reason)
{
	check(IsInGameThread());
	if (Body)
	{
		Body->OnComponentPhysicsStateChanged.RemoveDynamic(this, &UOpenRoadVehicleComponent::HandleBodyPhysicsStateChanged);
	}
	if (Runtime && Runtime->Link.IsRegistered())
	{
		Runtime->Link.Unregister();  // the vehicle is destroyed with the callback, after its last step
		UE_LOG(LogOpenRoad, Verbose, TEXT("OpenRoad: %s deactivated (%s)."), *GetNameSafe(GetOwner()), Reason);
	}
	Body = nullptr;
}

void UOpenRoadVehicleComponent::HandleBodyPhysicsStateChanged(UPrimitiveComponent* ChangedComponent,
                                                              EComponentPhysicsStateChange::Type StateChange)
{
	if (StateChange == EComponentPhysicsStateChange::Destroyed && ChangedComponent == Body)
	{
		UE_LOG(LogOpenRoad, Warning, TEXT("OpenRoad: %s body physics state destroyed; vehicle deactivated."),
		       *GetNameSafe(GetOwner()));
		DeactivateVehicle(TEXT("body physics state destroyed"));
		SetComponentTickEnabled(false);
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
		return;
	}
	if (!IsValid(Body))
	{
		DeactivateVehicle(TEXT("body component gone"));
		return;
	}
	FOrdComponentRuntime& R = *Runtime;

	// (a) Consume every physics snapshot since last frame (one per physics step).
	R.Link.ConsumeOutputs([&R](const FOrdSimSnapshot& Snapshot)
	{
		if (Snapshot.NumStripRequests > 0)
		{
			R.Requests.assign(Snapshot.StripRequests.begin(), Snapshot.StripRequests.begin() + Snapshot.NumStripRequests);
		}
		if (!Snapshot.bValid)
		{
			return;
		}
		R.Latest = Snapshot;
		R.bHasSnapshot = true;
		if (R.Recorder && Snapshot.bHasTelemetry)
		{
			R.Recorder->push(Snapshot.Telemetry);
		}
	});

	if (R.bHasSnapshot && !R.bReportedMassMismatch && Definition)
	{
		const double MassError = std::abs(R.Latest.HostMassKg - Definition->MassKg) / FMath::Max(1.0f, Definition->MassKg);
		const FVector InertiaError = (R.Latest.HostInertiaKgM2 - Definition->InertiaKgM2) / Definition->InertiaKgM2;
		if (R.Latest.ComMismatchCm > 1.0 || MassError > 0.01 || InertiaError.GetAbsMax() > 0.05)
		{
			UE_LOG(LogOpenRoad, Warning,
			       TEXT("OpenRoad: %s Chaos mass properties differ from the definition (CoM off by %.1f cm, mass %.1f kg, "
			            "inertia %s kg*m^2). Forces stay consistent, but handling parity with headless runs will suffer."),
			       *GetNameSafe(GetOwner()), R.Latest.ComMismatchCm, R.Latest.HostMassKg, *R.Latest.HostInertiaKgM2.ToCompactString());
		}
		R.bReportedMassMismatch = true;
	}

	// (b) Ground: one batch of line traces into a fresh (unshared) cache.
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
