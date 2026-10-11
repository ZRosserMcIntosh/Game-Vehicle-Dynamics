// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// Physics-thread side of one vehicle, plus the plain data that crosses threads.
//
// Nothing in this file touches a UObject or a Chaos type. The Chaos bridge
// (OrdChaosBridge.cpp) reads the rigid body into FOrdBodyState, calls
// FOrdVehicleSim::Step() and applies the returned FOrdBodyWrench.
//
// Ownership: FOrdVehicleSim (and the ord::Vehicle inside it) is created on the
// game thread in BeginPlay, then handed to the physics thread exactly once
// through an atomic pointer (FOrdSimCallback::HandOver_External). From then on
// it is touched ONLY by the physics thread, and it is destroyed together with
// the sim callback object after the callback has been unregistered.
#pragma once

#include "CoreMinimal.h"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "ord/ground_cache.hpp"
#include "ord/inputs.hpp"
#include "ord/assists.hpp"
#include "ord/telemetry.hpp"

namespace ord
{
class Vehicle;
struct RigidBodyState;
}

/** Rigid-body state as Chaos reports it (UE frame and units). */
struct FOrdBodyState
{
	FVector Position = FVector::ZeroVector;        // body (actor root) origin, world, cm
	FQuat Rotation = FQuat::Identity;              // body -> world
	FVector CenterOfMass = FVector::ZeroVector;    // Chaos centre of mass, world, cm
	FVector LinearVelocity = FVector::ZeroVector;  // velocity of the Chaos CoM, cm/s
	FVector AngularVelocity = FVector::ZeroVector; // world, rad/s (UE axes)
	double MassKg = 0.0;                           // diagnostics
	FVector InertiaKgCm2 = FVector::ZeroVector;    // diagnostics: Chaos local diagonal inertia
	double RotationOfMassDeg = 0.0;                // diagnostics: angle of Chaos' principal frame vs body axes
};

/** World-space force at the Chaos CoM and torque about it, Chaos units. */
struct FOrdBodyWrench
{
	FVector Force = FVector::ZeroVector;   // kg*cm/s^2
	FVector Torque = FVector::ZeroVector;  // kg*cm^2/s^2
	bool bApply = false;
};

/** Game thread -> physics thread, one per game frame. Plain data + one immutable shared cache. */
struct FOrdGameInput
{
	ord::DriverInput Driver;        // shiftUp/shiftDown are ignored; use the counters below
	uint32 ShiftUpCount = 0;        // incremented per press; edges are derived on the physics thread
	uint32 ShiftDownCount = 0;
	ord::AssistConfig Assists;
	uint32 AssistsRevision = 0;     // the physics thread applies Assists when this changes
	std::shared_ptr<const ord::GroundPrefetchCache> Ground;  // immutable once published
	double GroundRequestTime = 0.0; // core time of the state the strips were requested for (FOrdSimSnapshot::StripRequestTime)
	double VehicleRateHz = 240.0;   // vehicle-local rate: substeps = ceil(dt * rate), 1 at a 240 Hz world step
	bool bDebugDraw = false;
	double DebugForceScale = 1.0 / 5000.0;  // m of arrow per N
};

struct FOrdDebugLine
{
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	FColor Color = FColor::White;
};

struct FOrdDebugPoint
{
	FVector Position = FVector::ZeroVector;
	float Size = 4.0f;  // screen pixels
	FColor Color = FColor::White;
};

/** Physics thread -> game thread, one per physics step. */
struct FOrdSimSnapshot
{
	bool bValid = false;          // the vehicle stepped this physics step
	bool bGroundMissing = false;  // no ground cache had arrived yet
	bool bFaulted = false;        // non-finite forces were produced and not applied
	double SimTime = 0.0;         // core time, s
	double DeltaTime = 0.0;       // physics (world) step, s
	int32 Substeps = 1;           // vehicle substeps inside this world step

	double SpeedKmh = 0.0;        // longitudinal, body frame
	double EngineRpm = 0.0;
	int32 Gear = 0;               // -1 reverse, 0 neutral, 1..N

	bool bHasTelemetry = false;
	ord::TelemetryFrame Telemetry;

	int32 NumStripRequests = 0;
	std::array<ord::GroundStripRequest, ord::kMaxWheels> StripRequests{};
	double StripRequestTime = 0.0;   // core time of the chassis state the requests describe (start of this step)

	// Ground-cache health. A "miss" is a contact query outside every strip,
	// answered by extrapolating the nearest strip (no engine trace on this thread).
	uint64 GroundCacheMisses = 0;    // cumulative for this vehicle (monotonic)
	uint32 GroundCacheMissesStep = 0;// misses during this step
	double GroundCacheAge = 0.0;     // s: end of this step minus GroundRequestTime of the cache used.
	                                 // The strip horizon must exceed this (adaptive horizon feeds on it).

	// Host coupling (ord::Vehicle::hostSync()): host-integrated vs core-predicted
	// chassis at the start of this step. Near zero when Chaos applied our wrench
	// exactly as assumed (frame, units, CoM, per-step force clearing).
	double HostSyncPositionErrorCm = 0.0;
	double HostSyncAngleErrorDeg = 0.0;
	double HostSyncMaxPositionErrorCm = 0.0;
	double HostSyncMaxAngleErrorDeg = 0.0;

	// Velocity reference check (README item 5): finite-difference velocity of the
	// Chaos CoM and of the body origin over the last step, minus the velocity
	// Chaos reported. The one that stays ~0 tells what V() refers to.
	double VelocityCheckComCmS = -1.0;    // -1 = not available this step
	double VelocityCheckOriginCmS = -1.0;
	double RotationOfMassDeg = 0.0;

	// Mass-property agreement between the core and Chaos (validation).
	double ComMismatchCm = 0.0;
	double HostMassKg = 0.0;
	FVector HostInertiaKgM2 = FVector::ZeroVector;

	TArray<FOrdDebugLine> DebugLines;   // world, UE frame
	TArray<FOrdDebugPoint> DebugPoints;

	/** Keeps array capacity (pooled outputs are reused). */
	void Reset();
};

class FOrdVehicleSim
{
public:
	/** Game thread, before hand-over. The vehicle must be valid and in ChassisMode::External. */
	explicit FOrdVehicleSim(std::unique_ptr<ord::Vehicle> InVehicle);
	~FOrdVehicleSim();

	FOrdVehicleSim(const FOrdVehicleSim&) = delete;
	FOrdVehicleSim& operator=(const FOrdVehicleSim&) = delete;

	/**
	 * Physics thread. One fixed step: body state in, wrench out, snapshot filled.
	 * The wrench must be applied to the same body for this same step.
	 */
	FOrdBodyWrench Step(double Dt, const FOrdBodyState& Body, const FOrdGameInput& Input, FOrdSimSnapshot& Out);

	/**
	 * Physics thread. The body is not simulated this step (static/kinematic):
	 * only syncs the core's chassis to it, so the next Step() neither compares
	 * against a stale prediction nor carries a stale wheel state.
	 */
	void SyncOnly(const FOrdBodyState& Body);

private:
	void FillStripRequests(FOrdSimSnapshot& Out, double RequestTime);
	ord::RigidBodyState ToCoreState(const FOrdBodyState& Body, ord::Vec3& OutComOffset) const;
	void UpdateVelocityCheck(double Dt, const FOrdBodyState& Body, FOrdSimSnapshot& Out);

	std::unique_ptr<ord::Vehicle> Vehicle;
	std::shared_ptr<const ord::GroundPrefetchCache> Ground;  // latest cache received
	double GroundRequestTime = 0.0;
	uint64 GroundMissesTotal = 0;
	FVector PrevHostCom = FVector::ZeroVector;
	FVector PrevHostOrigin = FVector::ZeroVector;
	bool bHavePrevBody = false;
	std::vector<ord::GroundStripRequest> RequestScratch;
	uint32 LastShiftUpCount = 0;
	uint32 LastShiftDownCount = 0;
	uint32 AppliedAssistsRevision = 0;
	double FirstDt = 0.0;
	bool bWarnedVariableDt = false;
};
