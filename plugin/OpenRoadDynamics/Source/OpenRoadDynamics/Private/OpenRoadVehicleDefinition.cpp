// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT

#include "OpenRoadVehicleDefinition.h"

#include "OrdConversions.h"
#include "OrdFrame.h"

#include <string>

#include "ord/vehicle_def.hpp"

namespace
{
ord::DifferentialType ToCore(EOpenRoadDifferential D)
{
	switch (D)
	{
	case EOpenRoadDifferential::Open: return ord::DifferentialType::Open;
	case EOpenRoadDifferential::Locked: return ord::DifferentialType::Locked;
	case EOpenRoadDifferential::ClutchLsd: return ord::DifferentialType::ClutchLsd;
	}
	return ord::DifferentialType::Open;
}

ord::CenterCoupling ToCore(EOpenRoadCenterCoupling C)
{
	switch (C)
	{
	case EOpenRoadCenterCoupling::Open: return ord::CenterCoupling::Open;
	case EOpenRoadCenterCoupling::Locked: return ord::CenterCoupling::Locked;
	case EOpenRoadCenterCoupling::ClutchLsd: return ord::CenterCoupling::ClutchLsd;
	}
	return ord::CenterCoupling::Open;
}

ord::TransmissionKind ToCore(EOpenRoadTransmission T)
{
	switch (T)
	{
	case EOpenRoadTransmission::Automatic: return ord::TransmissionKind::Automatic;
	case EOpenRoadTransmission::Sequential: return ord::TransmissionKind::Sequential;
	case EOpenRoadTransmission::DualClutch: return ord::TransmissionKind::DualClutch;
	case EOpenRoadTransmission::ManualHPattern: return ord::TransmissionKind::ManualHPattern;
	}
	return ord::TransmissionKind::Automatic;
}
}  // namespace

UOpenRoadVehicleDefinition::UOpenRoadVehicleDefinition()
{
	// Generic front-engine RWD sports car. Mesh pivot on the ground; CoM 45 cm up;
	// wheel centres at full bump 5 cm below the CoM; 2.55 m wheelbase, 1.6 m track.
	FOpenRoadAxle Front;
	Front.Name = TEXT("Front");
	Front.Hardpoint = FVector(125.0, 80.0, 40.0);
	Front.Travel = 0.18f;
	Front.SpringRate = 45000.0f;
	Front.BumpDamping = 3200.0f;
	Front.ReboundDamping = 4800.0f;
	Front.AntiRollBarRate = 22000.0f;
	Front.MaxSteerAngleDeg = 31.513f;  // 0.55 rad
	Front.MaxBrakeTorque = 2800.0f;
	Front.HandbrakeTorque = 0.0f;
	Front.Tire.Radius = 0.33f;
	Front.Tire.MuX = 1.15f;
	Front.Tire.MuY = 1.10f;
	Front.bDriven = false;

	FOpenRoadAxle Rear = Front;
	Rear.Name = TEXT("Rear");
	Rear.Hardpoint = FVector(-130.0, 80.0, 40.0);
	Rear.SpringRate = 50000.0f;
	Rear.AntiRollBarRate = 12000.0f;
	Rear.MaxSteerAngleDeg = 0.0f;
	Rear.MaxBrakeTorque = 1600.0f;
	Rear.HandbrakeTorque = 2500.0f;
	Rear.Tire.Radius = 0.335f;
	Rear.Tire.MuX = 1.18f;
	Rear.Tire.MuY = 1.12f;
	Rear.bDriven = true;
	Rear.TorqueShare = 1.0f;
	Rear.Differential = EOpenRoadDifferential::ClutchLsd;

	Axles = {Front, Rear};

	TorqueCurve = {
		FVector2D(800.0, 260.0), FVector2D(1000.0, 300.0), FVector2D(2000.0, 480.0), FVector2D(3000.0, 520.0),
		FVector2D(5000.0, 520.0), FVector2D(6500.0, 440.0), FVector2D(7200.0, 380.0),
	};
	ForwardGearRatios = {3.30f, 2.20f, 1.60f, 1.25f, 1.00f, 0.82f};
}

bool UOpenRoadVehicleDefinition::ToCore(ord::VehicleDefinition& Out, FString& OutError) const
{
	Out = ord::VehicleDefinition();
	Out.name = TCHAR_TO_UTF8(*VehicleName);
	Out.mass = MassKg;
	Out.centerOfMass = OrdFrame::PositionFromUe(CenterOfMass);
	Out.inertia = ord::Vec3(InertiaKgM2.X, InertiaKgM2.Y, InertiaKgM2.Z);  // diagonal: mirror-invariant
	Out.steeringAckermann = SteeringAckermann;

	if (Axles.Num() == 0 || Axles.Num() > static_cast<int32>(ord::kMaxAxles))
	{
		OutError = FString::Printf(TEXT("%s: needs 1..%d axles"), *GetName(), static_cast<int32>(ord::kMaxAxles));
		return false;
	}

	double ShareSum = 0.0;
	int32 DrivenCount = 0;
	for (const FOpenRoadAxle& Axle : Axles)
	{
		if (Axle.bDriven)
		{
			ShareSum += FMath::Max(0.0f, Axle.TorqueShare);
			++DrivenCount;
		}
	}

	for (int32 AxleIndex = 0; AxleIndex < Axles.Num(); ++AxleIndex)
	{
		const FOpenRoadAxle& Axle = Axles[AxleIndex];

		ord::WheelDef Wheel;
		Wheel.suspension.travel = Axle.Travel;
		Wheel.suspension.springRate = Axle.SpringRate;
		Wheel.suspension.springPreload = Axle.SpringPreload;
		Wheel.suspension.bumpDamping = Axle.BumpDamping;
		Wheel.suspension.reboundDamping = Axle.ReboundDamping;
		Wheel.suspension.bumpStopRate = Axle.BumpStopRate;
		Wheel.tire = OrdConvert::ToCore(Axle.Tire);
		Wheel.maxSteerAngle = FMath::DegreesToRadians(static_cast<double>(Axle.MaxSteerAngleDeg));
		Wheel.staticToe = FMath::DegreesToRadians(static_cast<double>(Axle.StaticToeDeg));
		Wheel.maxBrakeTorque = Axle.MaxBrakeTorque;
		Wheel.handbrakeTorque = Axle.HandbrakeTorque;
		Wheel.unsprungMass = Axle.UnsprungMassKg;

		// UE Y points right: the left wheel sits at -|Y| (which becomes +Y in ISO).
		const double HalfTrack = FMath::Abs(Axle.Hardpoint.Y);
		ord::WheelDef Left = Wheel;
		Left.suspension.hardpoint = OrdFrame::PositionFromUe(FVector(Axle.Hardpoint.X, -HalfTrack, Axle.Hardpoint.Z));
		Left.isLeft = true;
		ord::WheelDef Right = Wheel;
		Right.suspension.hardpoint = OrdFrame::PositionFromUe(FVector(Axle.Hardpoint.X, HalfTrack, Axle.Hardpoint.Z));
		Right.isLeft = false;

		const int LeftIndex = static_cast<int>(Out.wheels.size());
		Out.wheels.push_back(Left);
		Out.wheels.push_back(Right);

		ord::AxleDef AxleDef;
		AxleDef.leftWheel = LeftIndex;
		AxleDef.rightWheel = LeftIndex + 1;
		AxleDef.antiRollBarRate = Axle.AntiRollBarRate;
		Out.axles.push_back(AxleDef);

		if (Axle.bDriven)
		{
			ord::DrivenAxle Driven;
			Driven.axleIndex = static_cast<std::size_t>(AxleIndex);
			Driven.torqueShare = ShareSum > 0.0 ? FMath::Max(0.0f, Axle.TorqueShare) / ShareSum : 1.0 / DrivenCount;
			Driven.differential.type = ToCore(Axle.Differential);
			Driven.differential.finalDrive = FinalDrive;
			Driven.differential.preload = Axle.LsdPreload;
			Driven.differential.powerRamp = Axle.LsdPowerRamp;
			Driven.differential.coastRamp = Axle.LsdCoastRamp;
			Out.drivetrain.drivenAxles.push_back(Driven);
		}
	}

	ord::EngineDef& Engine = Out.drivetrain.engine;
	Engine.torqueCurve.clear();
	for (const FVector2D& Point : TorqueCurve)
	{
		Engine.torqueCurve.push_back({Point.X, Point.Y});
	}
	Engine.idleRpm = IdleRpm;
	Engine.limiterRpm = LimiterRpm;
	Engine.inertia = EngineInertia;
	Engine.frictionTorque = EngineFrictionTorque;
	Engine.frictionPerRadS = EngineFrictionPerRadS;
	Out.drivetrain.clutch.maxTorque = ClutchMaxTorque;

	ord::GearboxDef& Gearbox = Out.drivetrain.gearbox;
	Gearbox.forwardRatios.clear();
	for (const float Ratio : ForwardGearRatios)
	{
		Gearbox.forwardRatios.push_back(Ratio);
	}
	Gearbox.reverseRatio = ReverseGearRatio;
	Gearbox.inputInertia = GearboxInputInertia;
	Gearbox.efficiency = GearboxEfficiency;
	Gearbox.shiftTime = ShiftTime;
	Gearbox.kind = ToCore(Transmission);

	Out.drivetrain.center = ToCore(CenterCoupling);
	Out.drivetrain.centerPreload = CenterPreload;
	Out.drivetrain.centerRamp = CenterRamp;

	Out.aero.dragArea = DragArea;
	Out.aero.frontDownforceArea = FrontDownforceArea;
	Out.aero.rearDownforceArea = RearDownforceArea;
	Out.aero.airDensity = AirDensity;

	const std::string Error = Out.validate();
	if (!Error.empty())
	{
		OutError = FString::Printf(TEXT("%s: %s"), *GetName(), UTF8_TO_TCHAR(Error.c_str()));
		return false;
	}
	return true;
}

bool UOpenRoadVehicleDefinition::Validate(FString& OutError) const
{
	ord::VehicleDefinition Scratch;
	return ToCore(Scratch, OutError);
}
