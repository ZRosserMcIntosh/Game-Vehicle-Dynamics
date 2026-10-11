// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// All version-sensitive Chaos and FBodyInstance usage lives in this file.
// Written against the public UE 5.4/5.5 API from documentation and memory, NOT
// compiled here: every call marked [UNVERIFIED] is listed in the plugin README
// ("Unverified API assumptions") together with what to check.

#include "OrdChaosBridge.h"

#include "Chaos/SimCallbackInput.h"          // [UNVERIFIED] header location of FSimCallbackInput/Output
#include "Chaos/SimCallbackObject.h"         // [UNVERIFIED] TSimCallbackObject
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "PBDRigidsSolver.h"                 // [UNVERIFIED] Chaos::FPBDRigidsSolver (FPhysicsSolver)
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"  // FSingleParticlePhysicsProxy, Chaos::FRigidBodyHandle_Internal

#include "OpenRoadLog.h"
#include "OrdVehicleSim.h"

#include <atomic>

// UE 5.4 started renaming particle accessors (X() -> GetX(), ...). Set to 1 if
// the X()/R()/V()/W() accessors of FRigidBodyHandle_Internal are deprecated or
// removed in your engine version.
#ifndef ORD_CHAOS_USE_GET_ACCESSORS
#define ORD_CHAOS_USE_GET_ACCESSORS 0
#endif

// ---------------------------------------------------------------------------
// Marshalled data. Chaos pools these objects and calls Reset() before reuse,
// which also releases the input's reference to the ground cache.
// ---------------------------------------------------------------------------

struct FOrdAsyncInput : public Chaos::FSimCallbackInput
{
	FSingleParticlePhysicsProxy* Proxy = nullptr;  // the vehicle body (game-thread handle value, read on PT)
	FOrdGameInput Game;

	void Reset()
	{
		Proxy = nullptr;
		Game = FOrdGameInput();
	}
};

struct FOrdAsyncOutput : public Chaos::FSimCallbackOutput
{
	FOrdSimSnapshot Snapshot;

	void Reset() { Snapshot.Reset(); }
};

// ---------------------------------------------------------------------------
// Sim callback: runs on the physics thread before every Chaos step.
// Default options of TSimCallbackObject include ESimCallbackOptions::Presimulate. [UNVERIFIED]
// ---------------------------------------------------------------------------

class FOrdSimCallback final : public Chaos::TSimCallbackObject<FOrdAsyncInput, FOrdAsyncOutput>
{
public:
	virtual ~FOrdSimCallback() override
	{
		// Runs after Chaos has unregistered us, when no step can use the vehicle.
		delete PendingSim.exchange(nullptr, std::memory_order_acq_rel);
	}

	/** Game thread, once, right after creation. Release/acquire pairs with OnPreSimulate_Internal. */
	void HandOver_External(TUniquePtr<FOrdVehicleSim> InSim)
	{
		delete PendingSim.exchange(InSim.Release(), std::memory_order_acq_rel);
	}

private:
	virtual void OnPreSimulate_Internal() override;

	std::atomic<FOrdVehicleSim*> PendingSim{nullptr};

	// ---- Physics thread only ----
	TUniquePtr<FOrdVehicleSim> Sim;
	FOrdGameInput LatestInput;
	FSingleParticlePhysicsProxy* LatestProxy = nullptr;
	bool bHasInput = false;
};

namespace
{
FOrdBodyState ReadBodyState(const Chaos::FRigidBodyHandle_Internal& Handle)
{
	FOrdBodyState S;
#if ORD_CHAOS_USE_GET_ACCESSORS
	S.Position = Handle.GetX();         // [UNVERIFIED] body origin, world
	S.Rotation = Handle.GetR();
	S.LinearVelocity = Handle.GetV();   // [UNVERIFIED] velocity of the centre of mass
	S.AngularVelocity = Handle.GetW();
#else
	S.Position = Handle.X();            // [UNVERIFIED] body (actor) origin, not the CoM
	S.Rotation = Handle.R();
	S.LinearVelocity = Handle.V();      // [UNVERIFIED] velocity of the centre of mass
	S.AngularVelocity = Handle.W();
#endif
	// [UNVERIFIED] CenterOfMass() is the body-local CoM offset, RotationOfMass() ignored
	// (the core assumes principal axes = body axes).
	S.CenterOfMass = S.Position + S.Rotation.RotateVector(FVector(Handle.CenterOfMass()));
	S.MassKg = Handle.M();                       // [UNVERIFIED]
	S.InertiaKgCm2 = FVector(Handle.I());        // [UNVERIFIED] TVec3<FRealSingle>, local, kg*cm^2
	return S;
}
}  // namespace

void FOrdSimCallback::OnPreSimulate_Internal()
{
	if (FOrdVehicleSim* Incoming = PendingSim.exchange(nullptr, std::memory_order_acq_rel))
	{
		Sim.Reset(Incoming);
	}

	// [UNVERIFIED] May be null on steps for which no new game-thread input exists;
	// we then keep using the latest one (edge-triggered fields are counters).
	if (const FOrdAsyncInput* Input = GetConsumerInput_Internal())
	{
		LatestInput = Input->Game;
		LatestProxy = Input->Proxy;
		bHasInput = true;
	}

	if (!Sim || !bHasInput)
	{
		return;
	}

	FOrdAsyncOutput& Output = GetProducerOutputData_Internal();

	Chaos::FRigidBodyHandle_Internal* Handle = LatestProxy ? LatestProxy->GetPhysicsThreadAPI() : nullptr;
	if (!Handle)
	{
		Output.Snapshot.Reset();  // body not (yet) on the physics thread
		return;
	}

	const Chaos::EObjectStateType State = Handle->ObjectState();  // [UNVERIFIED]
	if (State == Chaos::EObjectStateType::Static || State == Chaos::EObjectStateType::Kinematic)
	{
		Output.Snapshot.Reset();
		return;
	}

	const FOrdBodyState Body = ReadBodyState(*Handle);
	const FOrdBodyWrench Wrench = Sim->Step(static_cast<double>(GetDeltaTime_Internal()), Body, LatestInput, Output.Snapshot);

	// A sleeping body keeps sleeping (the game thread wakes it on driver input);
	// forcing it awake every step would defeat Chaos' sleeping.
	if (Wrench.bApply && State == Chaos::EObjectStateType::Dynamic)
	{
		// [UNVERIFIED] world-space force at the CoM and world-space torque, both
		// accumulated for this step only and cleared by Chaos after integration.
		Handle->AddForce(Wrench.Force);
		Handle->AddTorque(Wrench.Torque);
	}
}

// ---------------------------------------------------------------------------
// FOrdChaosVehicleLink (game thread)
// ---------------------------------------------------------------------------

FOrdChaosVehicleLink::~FOrdChaosVehicleLink()
{
	Unregister();
}

bool FOrdChaosVehicleLink::Register(UWorld& World, UPrimitiveComponent& Body, TUniquePtr<FOrdVehicleSim> Sim, FString& OutError)
{
	check(IsInGameThread());
	check(Sim.IsValid());
	Unregister();

	FPhysScene* Scene = World.GetPhysicsScene();
	Chaos::FPhysicsSolver* PhysicsSolver = Scene ? Scene->GetSolver() : nullptr;  // [UNVERIFIED]
	if (!PhysicsSolver)
	{
		OutError = TEXT("the world has no Chaos physics solver");
		return false;
	}

	RefreshBody(Body);
	if (!Proxy)
	{
		OutError = FString::Printf(TEXT("'%s' has no physics body (is Simulate Physics enabled?)"), *Body.GetName());
		return false;
	}

	// [UNVERIFIED] template creates the object with its default constructor and
	// registers it; the solver owns it until UnregisterAndFreeSimCallbackObject_External.
	FOrdSimCallback* NewCallback = PhysicsSolver->CreateAndRegisterSimCallbackObject_External<FOrdSimCallback>();
	if (!NewCallback)
	{
		OutError = TEXT("CreateAndRegisterSimCallbackObject_External failed");
		return false;
	}
	NewCallback->HandOver_External(MoveTemp(Sim));

	Callback = NewCallback;
	Solver = PhysicsSolver;
	return true;
}

void FOrdChaosVehicleLink::Unregister()
{
	if (Callback && Solver)
	{
		check(IsInGameThread());
		// [UNVERIFIED] queued: the solver stops calling us before freeing, so the
		// vehicle is destroyed with the callback, after the last step that used it.
		Solver->UnregisterAndFreeSimCallbackObject_External(Callback);
	}
	Callback = nullptr;
	Solver = nullptr;
	Proxy = nullptr;
}

void FOrdChaosVehicleLink::RefreshBody(UPrimitiveComponent& Body)
{
	check(IsInGameThread());
	FBodyInstance* BodyInstance = Body.GetBodyInstance();
	// [UNVERIFIED] FPhysicsActorHandle == FSingleParticlePhysicsProxy* under Chaos.
	Proxy = BodyInstance ? BodyInstance->GetPhysicsActorHandle() : nullptr;
}

void FOrdChaosVehicleLink::PushInput(const FOrdGameInput& Input)
{
	check(IsInGameThread());
	if (!Callback)
	{
		return;
	}
	// [UNVERIFIED] returns this frame's input object (same object on repeated calls within a frame).
	if (FOrdAsyncInput* AsyncInput = Callback->GetProducerInputData_External())
	{
		AsyncInput->Proxy = Proxy;
		AsyncInput->Game = Input;
	}
}

int32 FOrdChaosVehicleLink::ConsumeOutputs(TFunctionRef<void(const FOrdSimSnapshot&)> Visitor)
{
	check(IsInGameThread());
	if (!Callback)
	{
		return 0;
	}
	int32 Count = 0;
	// [UNVERIFIED] pops outputs oldest-first; the handle returns the output to the pool when it goes out of scope.
	while (auto Output = Callback->PopOutputData_External())
	{
		Visitor(Output->Snapshot);
		++Count;
	}
	return Count;
}

bool FOrdChaosVehicleLink::ConfigureBody(UPrimitiveComponent& Body, double MassKg, const FVector& CenterOfMassLocalCm,
                                         const FVector& InertiaKgCm2, FString& OutMessage)
{
	check(IsInGameThread());
	FBodyInstance* BodyInstance = Body.GetBodyInstance();
	if (!BodyInstance || !BodyInstance->IsValidBodyInstance())
	{
		OutMessage = TEXT("root component has no valid body instance");
		return false;
	}

	bool bOk = true;
	FString Notes;

	if (!Body.GetComponentScale().Equals(FVector::OneVector, 1e-3))
	{
		bOk = false;
		Notes += TEXT(" Root component scale is not 1: definition positions are interpreted in unscaled cm.");
	}

	// Damping would be a force the core does not know about.
	Body.SetLinearDamping(0.0f);
	Body.SetAngularDamping(0.0f);

	Body.SetMassOverrideInKg(NAME_None, static_cast<float>(MassKg), /*bOverrideMass*/ true);

	// Centre of mass: COMNudge offsets the CoM computed from the collision shapes. [UNVERIFIED: component-local]
	const FTransform& Transform = Body.GetComponentTransform();
	const FVector CurrentComLocal = Transform.InverseTransformPosition(Body.GetCenterOfMass());
	const FVector Nudge = BodyInstance->COMNudge + (CenterOfMassLocalCm - CurrentComLocal);
	Body.SetCenterOfMass(Nudge, NAME_None);

	// Inertia: scale the shape-derived principal inertia to the definition's values. [UNVERIFIED]
	const FVector CurrentInertia = BodyInstance->GetBodyInertiaTensor();
	if (CurrentInertia.X > UE_KINDA_SMALL_NUMBER && CurrentInertia.Y > UE_KINDA_SMALL_NUMBER && CurrentInertia.Z > UE_KINDA_SMALL_NUMBER)
	{
		BodyInstance->InertiaTensorScale = BodyInstance->InertiaTensorScale * (InertiaKgCm2 / CurrentInertia);
		BodyInstance->UpdateMassProperties();
	}
	else
	{
		bOk = false;
		Notes += TEXT(" Could not read the body inertia; Chaos keeps its shape-derived inertia.");
	}

	const FVector ComAfter = Transform.InverseTransformPosition(Body.GetCenterOfMass());
	const FVector InertiaAfter = BodyInstance->GetBodyInertiaTensor();
	OutMessage = FString::Printf(
		TEXT("mass %.1f kg; CoM %s cm (wanted %s); inertia %s kg*cm^2 (wanted %s).%s"),
		Body.GetMass(), *ComAfter.ToCompactString(), *CenterOfMassLocalCm.ToCompactString(),
		*InertiaAfter.ToCompactString(), *InertiaKgCm2.ToCompactString(), *Notes);
	return bOk;
}

bool FOrdChaosVehicleLink::GetAsyncPhysicsSettings(float& OutFixedStep)
{
	const UPhysicsSettings* Settings = UPhysicsSettings::Get();
	OutFixedStep = Settings ? Settings->AsyncFixedTimeStepSize : 0.0f;  // [UNVERIFIED] field names
	return Settings && Settings->bTickPhysicsAsync;
}
