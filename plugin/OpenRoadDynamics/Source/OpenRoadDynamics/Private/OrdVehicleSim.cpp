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
	GroundCacheMisses = 0;
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

void FOrdVehicleSim::FillStripRequests(FOrdSimSnapshot& Out)
{
	RequestScratch.clear();
	Vehicle->groundPrefetchRequests(RequestScratch);
	const std::size_t Count = FMath::Min<std::size_t>(RequestScratch.size(), ord::kMaxWheels);
	for (std::size_t I = 0; I < Count; ++I)
	{
		Out.StripRequests[I] = RequestScratch[I];
	}
	Out.NumStripRequests = static_cast<int32>(Count);
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
	if (Input.Ground)
	{
		Ground = Input.Ground;  // keeps the immutable cache alive while we read it
	}

	// ---- Chaos body -> core chassis state (ISO, metres) ----
	const ord::VehicleDefinition& Def = Vehicle->definition();
	ord::RigidBodyState State;
	State.position = OrdFrame::PositionFromUe(Body.Position);
	State.orientation = OrdFrame::RotationFromUe(Body.Rotation);
	State.orientation.normalize();
	State.angularVelocity = OrdFrame::AngularVelocityFromUe(Body.AngularVelocity);

	// The core's CoM (from the definition) and Chaos' CoM should coincide (the
	// component configures the body), but any residual offset is handled
	// exactly: velocity is transported to the core CoM, and the torque is
	// transported back to the Chaos CoM below.
	const ord::Vec3 HostCom = OrdFrame::PositionFromUe(Body.CenterOfMass);
	const ord::Vec3 CoreCom = State.position + State.orientation.rotate(Def.centerOfMass);
	const ord::Vec3 ComOffset = CoreCom - HostCom;
	State.velocity = OrdFrame::VelocityFromUe(Body.LinearVelocity) + ord::cross(State.angularVelocity, ComOffset);
	Vehicle->setChassisState(State);

	Out.ComMismatchCm = ord::length(ComOffset) * ord::host::kUeUnitsPerMetre;
	Out.HostMassKg = Body.MassKg;
	Out.HostInertiaKgM2 = Body.InertiaKgCm2 / (ord::host::kUeUnitsPerMetre * ord::host::kUeUnitsPerMetre);

	if (!Ground)
	{
		Out.bGroundMissing = true;
		FillStripRequests(Out);
		return Wrench;
	}

	// Edge-triggered shifts: one edge per counter change, however many physics
	// steps reuse the same game-thread input.
	ord::DriverInput Driver = Input.Driver;
	Driver.shiftUp = Input.ShiftUpCount != LastShiftUpCount;
	Driver.shiftDown = Input.ShiftDownCount != LastShiftDownCount;
	LastShiftUpCount = Input.ShiftUpCount;
	LastShiftDownCount = Input.ShiftDownCount;

	// Vehicle-local substepping (ADR-0003): with a 240 Hz world step this is one
	// plain step(); with a 60 Hz world step the core runs 4 internal substeps and
	// returns the impulse-equivalent average wrench for the whole world step.
	const double RateHz = FMath::Max(1.0, Input.VehicleRateHz);
	const int32 Substeps = FMath::Clamp(static_cast<int32>(std::ceil(Dt * RateHz - 1e-6)), 1, 16);
	Vehicle->stepSubstepped(Dt, Substeps, Driver, *Ground);
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
	FillStripRequests(Out);
	Out.GroundCacheMisses = Ground->misses();

	if (Input.bDebugDraw)
	{
		FOrdSnapshotDebugSink Sink(Out);
		ord::DebugDrawOptions Options;
		Options.forceScale = Input.DebugForceScale;
		ord::drawVehicle(*Vehicle, Sink, Options);
	}
	return Wrench;
}
