// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// The ONLY place that touches version-sensitive Chaos / body-instance APIs.
// Everything Chaos-specific is in OrdChaosBridge.cpp; this header exposes a
// small, Chaos-free interface to the vehicle component.
//
// Threading model (one link per vehicle):
//
//   game thread (TG_PrePhysics)                 physics thread (Chaos async step)
//   ---------------------------                 ---------------------------------
//   ConsumeOutputs()  <-- FOrdSimSnapshot ----  OnPreSimulate_Internal():
//   rebuild ground cache (UWorld traces)          read body (FRigidBodyHandle_Internal)
//   PushInput()       --- FOrdGameInput  ---->    FOrdVehicleSim::Step()
//                         (+ shared_ptr<const     AddForce / AddTorque
//                          GroundPrefetchCache>)  write snapshot
//
// Inputs/outputs are marshalled by Chaos' TSimCallbackObject (one input per
// game frame, one output per physics step). The game thread never touches the
// ord::Vehicle after Register(); the physics thread never touches a UObject.
#pragma once

#include "CoreMinimal.h"
#include "Templates/Function.h"
#include "Templates/UniquePtr.h"

class FOrdSimCallback;
class FOrdVehicleSim;
class FSingleParticlePhysicsProxy;
class UPrimitiveComponent;
class UWorld;
struct FOrdGameInput;
struct FOrdSimSnapshot;

namespace Chaos
{
class FPBDRigidsSolver;
}

class FOrdChaosVehicleLink
{
public:
	FOrdChaosVehicleLink() = default;
	~FOrdChaosVehicleLink();

	FOrdChaosVehicleLink(const FOrdChaosVehicleLink&) = delete;
	FOrdChaosVehicleLink& operator=(const FOrdChaosVehicleLink&) = delete;

	/**
	 * Game thread. Registers a sim callback with the world's Chaos solver and
	 * hands Sim to the physics thread (ownership transfer; the caller must not
	 * keep any pointer into it).
	 */
	bool Register(UWorld& World, UPrimitiveComponent& Body, TUniquePtr<FOrdVehicleSim> Sim, FString& OutError);

	/**
	 * Game thread. Unregisters the callback. Chaos frees it (and the vehicle it
	 * owns) on its own schedule once no physics step can still be using it.
	 * Safe to call repeatedly.
	 */
	void Unregister();

	bool IsRegistered() const { return Callback != nullptr; }

	/** Game thread. Re-reads the body's physics proxy (bodies can be recreated). */
	void RefreshBody(UPrimitiveComponent& Body);
	bool HasBody() const { return Proxy != nullptr; }

	/** Game thread. Publishes this frame's input (overwrites an earlier push in the same frame). */
	void PushInput(const FOrdGameInput& Input);

	/** Game thread. Visits every snapshot produced since the last call, oldest first. */
	int32 ConsumeOutputs(TFunctionRef<void(const FOrdSimSnapshot&)> Visitor);

	/**
	 * Game thread. Makes the Chaos body agree with the core's mass properties:
	 * mass override, zero damping, centre of mass (via the CoM nudge) and
	 * principal inertia (via InertiaTensorScale). Returns false with a message
	 * when something could not be applied; the physics thread additionally
	 * reports what Chaos actually uses (FOrdSimSnapshot::Host*).
	 */
	static bool ConfigureBody(UPrimitiveComponent& Body, double MassKg, const FVector& CenterOfMassLocalCm,
	                          const FVector& InertiaKgCm2, FString& OutMessage);

	/** Project physics settings. Returns bTickPhysicsAsync; OutFixedStep = Async Fixed Time Step Size. */
	static bool GetAsyncPhysicsSettings(float& OutFixedStep);

private:
	FOrdSimCallback* Callback = nullptr;
	Chaos::FPBDRigidsSolver* Solver = nullptr;
	FSingleParticlePhysicsProxy* Proxy = nullptr;
};
