// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT

#include "OrdVehicleSim.h"

#include "OpenRoadLog.h"
#include "OrdFrame.h"

#include <cmath>

#include "ord/debug_draw.hpp"
#include "ord/vehicle.hpp"

namespace
{
/** Records the core's debug primitives into the step snapshot (physics thread, plain data only). */
class FOrdSnapshotDebugSink final : public ord::DebugDrawSink
{
public:
	explicit FOrdSnapshotDebugSink(FOrdSimSnapshot& InOut) : Out(InOut) {}

	virtual void line(const ord::Vec3& A, const ord::Vec3& B, ord::Color C) override
	{
		FOrdDebugLine& L = Out.DebugLines.AddDefaulted_GetRef();
		L.A = OrdFrame::PositionToUe(A);
		L.B = OrdFrame::PositionToUe(B);
		L.Color = OrdFrame::ColorToUe(C);
	}

	virtual void point(const ord::Vec3& P, ord::Real Size, ord::Color C) override
	{
		FOrdDebugPoint& D = Out.DebugPoints.AddDefaulted_GetRef();
		D.Position = OrdFrame::PositionToUe(P);
		D.Size = static_cast<float>(FMath::Clamp(Size * 200.0, 3.0, 16.0));  // metres -> screen pixels (approximate)
		D.Color = OrdFrame::ColorToUe(C);
	}

private:
	FOrdSimSnapshot& Out;
};
}  // namespace

void FOrdSimSnapshot::Reset()
{
	bValid = false;
	bGroundMissing = false;
	bFaulted = false;
	SimTime = 0.0;
	DeltaTime = 0.0;
	SpeedKmh = 0.0;
	EngineRpm = 0.0;
	Gear = 0;
	Substeps = 1;
	bHasTelemetry = false;
	NumStripRequests = 0;
	StripRequestTime = 0.0;
	GroundCacheMisses = 0;
	GroundCacheMissesStep = 0;
	GroundCacheAge = 0.0;
	HostSyncPositionErrorCm = 0.0;
	HostSyncAngleErrorDeg = 0.0;
	HostSyncMaxPositionErrorCm = 0.0;
	HostSyncMaxAngleErrorDeg = 0.0;
	VelocityCheckComCmS = -1.0;
	VelocityCheckOriginCmS = -1.0;
	RotationOfMassDeg = 0.0;
	ComMismatchCm = 0.0;
	HostMassKg = 0.0;
	HostInertiaKgM2 = FVector::ZeroVector;
	DebugLines.Reset();
	DebugPoints.Reset();
	// Telemetry and StripRequests are overwritten before use (guarded by the flags/counts above).
}

FOrdVehicleSim::FOrdVehicleSim(std::unique_ptr<ord::Vehicle> InVehicle)
	: Vehicle(std::move(InVehicle))
{
	check(Vehicle && Vehicle->isValid());
	check(Vehicle->chassisMode() == ord::ChassisMode::External);
	RequestScratch.reserve(ord::kMaxWheels);
}

FOrdVehicleSim::~FOrdVehicleSim() = default;

void FOrdVehicleSim::FillStripRequests(FOrdSimSnapshot& Out, double RequestTime)
{
	RequestScratch.clear();
	Vehicle->groundPrefetchRequests(RequestScratch);
	const std::size_t Count = FMath::Min<std::size_t>(RequestScratch.size(), ord::kMaxWheels);
	for (std::size_t I = 0; I < Count; ++I)
	{
		Out.StripRequests[I] = RequestScratch[I];
	}
	Out.NumStripRequests = static_cast<int32>(Count);
	Out.StripRequestTime = RequestTime;
}

ord::RigidBodyState FOrdVehicleSim::ToCoreState(const FOrdBodyState& Body, ord::Vec3& OutComOffset) const
{
	// ---- Chaos body -> core chassis state (ISO, metres; host_frame.hpp math) ----
	const ord::VehicleDefinition& Def = Vehicle->definition();
	ord::RigidBodyState State;
	State.position = OrdFrame::PositionFromUe(Body.Position);
	State.orientation = OrdFrame::RotationFromUe(Body.Rotation);
	State.orientation.normalize();
	State.angularVelocity = OrdFrame::AngularVelocityFromUe(Body.AngularVelocity);

	// The core's CoM (from the definition) and Chaos' CoM should coincide (the
	// component configures the body), but any residual offset is handled
	// exactly: velocity is transported to the core CoM here, and the torque is
	// transported back to the Chaos CoM in Step().
	const ord::Vec3 HostCom = OrdFrame::PositionFromUe(Body.CenterOfMass);
	const ord::Vec3 CoreCom = State.position + State.orientation.rotate(Def.centerOfMass);
	OutComOffset = CoreCom - HostCom;
	State.velocity = OrdFrame::VelocityFromUe(Body.LinearVelocity) + ord::cross(State.angularVelocity, OutComOffset);
	return State;
}

void FOrdVehicleSim::UpdateVelocityCheck(double Dt, const FOrdBodyState& Body, FOrdSimSnapshot& Out)
{
	// Chaos (PBD) derives V from the position change of the step, so with a
	// constant CoM offset (Com_n - Com_n-1) / dt == V_n when V is the CoM
	// velocity; the origin version matches instead if V were the origin velocity.
	// Contacts of the body shell (velocity solve, restitution) perturb both.
	if (bHavePrevBody && Dt > 0.0)
	{
		Out.VelocityCheckComCmS = ((Body.CenterOfMass - PrevHostCom) / Dt - Body.LinearVelocity).Size();
		Out.VelocityCheckOriginCmS = ((Body.Position - PrevHostOrigin) / Dt - Body.LinearVelocity).Size();
	}
	PrevHostCom = Body.CenterOfMass;
	PrevHostOrigin = Body.Position;
	bHavePrevBody = true;
}

void FOrdVehicleSim::SyncOnly(const FOrdBodyState& Body)
{
	if (!Vehicle)
	{
		return;
	}
	ord::Vec3 ComOffset;
	Vehicle->setChassisState(ToCoreState(Body, ComOffset));
	bHavePrevBody = false;  // the body may have been moved kinematically
}

FOrdBodyWrench FOrdVehicleSim::Step(double Dt, const FOrdBodyState& Body, const FOrdGameInput& Input, FOrdSimSnapshot& Out)
{
	FOrdBodyWrench Wrench;
	Out.Reset();
	Out.DeltaTime = Dt;
	if (!Vehicle || !(Dt > 0.0))
	{
		return Wrench;
	}

	// The core assumes a fixed dt (async physics with a fixed step).
	if (FirstDt == 0.0)
	{
		FirstDt = Dt;
	}
	else if (!bWarnedVariableDt && std::abs(Dt - FirstDt) > 1e-7)
	{
		bWarnedVariableDt = true;
		UE_LOG(LogOpenRoad, Warning,
		       TEXT("OpenRoad: physics dt changed from %.6f to %.6f s. Enable 'Tick Physics Async' with a fixed "
		            "'Async Fixed Time Step Size' for reproducible handling."),
		       FirstDt, Dt);
	}

	if (Input.AssistsRevision != AppliedAssistsRevision)
	{
		Vehicle->assists().setConfig(Input.Assists);
		AppliedAssistsRevision = Input.AssistsRevision;
	}
	if (Input.Ground && Input.Ground != Ground)
	{
		Ground = Input.Ground;  // keeps the immutable cache alive while we read it
		GroundRequestTime = Input.GroundRequestTime;
	}

	UpdateVelocityCheck(Dt, Body, Out);
	Out.RotationOfMassDeg = Body.RotationOfMassDeg;

	ord::Vec3 ComOffset;
	const ord::RigidBodyState State = ToCoreState(Body, ComOffset);
	// Also records hostSync() against the prediction of the previous step and
	// carries unsprung wheels across small pose differences (core behaviour).
	Vehicle->setChassisState(State);

	const ord::Vehicle::HostSyncStats& Sync = Vehicle->hostSync();
	Out.HostSyncPositionErrorCm = Sync.positionError * ord::host::kUeUnitsPerMetre;
	Out.HostSyncAngleErrorDeg = FMath::RadiansToDegrees(Sync.angleError);
	Out.HostSyncMaxPositionErrorCm = Sync.maxPositionError * ord::host::kUeUnitsPerMetre;
	Out.HostSyncMaxAngleErrorDeg = FMath::RadiansToDegrees(Sync.maxAngleError);

	Out.ComMismatchCm = ord::length(ComOffset) * ord::host::kUeUnitsPerMetre;
	Out.HostMassKg = Body.MassKg;
	Out.HostInertiaKgM2 = Body.InertiaKgCm2 / (ord::host::kUeUnitsPerMetre * ord::host::kUeUnitsPerMetre);
	Out.GroundCacheMisses = GroundMissesTotal;

	const double StartTime = Vehicle->time();
	if (!Ground)
	{
		Out.bGroundMissing = true;
		FillStripRequests(Out, StartTime);
		return Wrench;
	}

	// Edge-triggered shifts: one edge per counter change, however many physics
	// steps reuse the same game-thread input.
	ord::DriverInput Driver = Input.Driver;
	Driver.shiftUp = Input.ShiftUpCount != LastShiftUpCount;
	Driver.shiftDown = Input.ShiftDownCount != LastShiftDownCount;
	LastShiftUpCount = Input.ShiftUpCount;
	LastShiftDownCount = Input.ShiftDownCount;

	// Vehicle-local substepping (ADR-0003). In ChassisMode::External the core
	// always integrates its own copy of the chassis (also for 1 substep) and
	// returns the impulse-equivalent average wrench for the whole world step;
	// the predicted end state feeds hostSync() at the next setChassisState().
	const double RateHz = FMath::Max(1.0, Input.VehicleRateHz);
	const int32 Substeps = FMath::Clamp(static_cast<int32>(std::ceil(Dt * RateHz - 1e-6)), 1, 16);
	// Misses are counted per cache object (which is pooled and reused), so only
	// the difference across this step is attributed to this vehicle. Only this
	// thread queries the cache, so no other increment can interleave.
	const uint64 MissesBefore = Ground->misses();
	Vehicle->stepSubstepped(Dt, Substeps, Driver, *Ground);
	const uint64 MissesStep = Ground->misses() - MissesBefore;
	GroundMissesTotal += MissesStep;
	Out.GroundCacheMissesStep = static_cast<uint32>(FMath::Min<uint64>(MissesStep, MAX_uint32));
	Out.GroundCacheMisses = GroundMissesTotal;
	Out.GroundCacheAge = (StartTime + Dt) - GroundRequestTime;
	Out.Substeps = Substeps;

	// ---- Core forces -> Chaos wrench ----
	// Core: world force at the core CoM and torque about it (gravity excluded,
	// Chaos applies it). Chaos: force at its CoM, torque about its CoM.
	const ord::ChassisForces& Forces = Vehicle->chassisForces();
	const ord::Vec3 TorqueAboutHostCom = Forces.torque + ord::cross(ComOffset, Forces.force);
	if (ord::isFinite(Forces.force) && ord::isFinite(TorqueAboutHostCom))
	{
		Wrench.Force = OrdFrame::ForceToUe(Forces.force);
		Wrench.Torque = OrdFrame::TorqueToUe(TorqueAboutHostCom);
		Wrench.bApply = true;
	}
	else
	{
		Out.bFaulted = true;
	}

	// ---- Snapshot ----
	Out.bValid = true;
	Out.SimTime = Vehicle->time();
	Out.SpeedKmh = ord::msToKmh(Vehicle->forwardSpeed());
	Out.EngineRpm = Vehicle->drivetrain().telemetry().engineRpm;
	Out.Gear = Vehicle->assists().currentGear();
	if (Vehicle->fidelity().telemetry)
	{
		Out.Telemetry = Vehicle->telemetry();
		Out.bHasTelemetry = true;
	}
	// Requests describe the chassis at StartTime (stepSubstepped restores the
	// host-owned start state); the age of the next cache is measured from it.
	FillStripRequests(Out, StartTime);

	if (Input.bDebugDraw)
	{
		FOrdSnapshotDebugSink Sink(Out);
		ord::DebugDrawOptions Options;
		Options.forceScale = Input.DebugForceScale;
		ord::drawVehicle(*Vehicle, Sink, Options);
	}
	return Wrench;
}
