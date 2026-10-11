// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// Automation tests that need no world: frame/unit conversions and the
// definition/assist mapping to the core. NOT COMPILED OR RUN in this repository
// (no Unreal Engine in CI). The same conversion identities are checked headless
// against stand-in types by Extras/HeadlessCheck.
//
// Run: Session Frontend > Automation > filter "OpenRoad", or
//   UnrealEditor-Cmd <Project>.uproject -ExecCmds="Automation RunTests OpenRoad; Quit" -unattended -nullrhi

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "OpenRoadVehicleDefinition.h"
#include "OrdConversions.h"
#include "OrdFrame.h"

#include "ord/presets.hpp"
#include "ord/vehicle_def.hpp"

// Written as an expression (not a typed constant): EAutomationTestFlags is a
// namespaced enum in 5.4 and an enum class in 5.5.
#define ORD_TEST_FLAGS (EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

namespace
{
bool Near(const FVector& A, const FVector& B, double Tolerance) { return (A - B).GetAbsMax() <= Tolerance; }
}  // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FOpenRoadFrameConversionTest, "OpenRoad.Adapter.FrameConversions", ORD_TEST_FLAGS)
bool FOpenRoadFrameConversionTest::RunTest(const FString& Parameters)
{
	// Positions: cm <-> m and the Y mirror.
	const FVector P(123.0, -45.0, 67.0);
	const ord::Vec3 Pc = OrdFrame::PositionFromUe(P);
	TestEqual(TEXT("position x"), Pc.x, 1.23);
	TestEqual(TEXT("position y mirrored"), Pc.y, 0.45);
	TestTrue(TEXT("position round trip"), Near(OrdFrame::PositionToUe(Pc), P, 1e-9));

	// Torque: UE r x F (cm, kg*cm/s^2) equals TorqueToUe of the core's r x F.
	const ord::Vec3 R{0.3, -1.2, 0.5}, F{120.0, 3400.0, -800.0};
	const FVector TorqueUe = FVector::CrossProduct(OrdFrame::PositionToUe(R), OrdFrame::ForceToUe(F));
	TestTrue(TEXT("torque mirror"), Near(TorqueUe, OrdFrame::TorqueToUe(ord::cross(R, F)), 1e-6));

	// Angular velocity: UE w x r equals VelocityToUe of the core's w x r.
	const ord::Vec3 W{0.4, -0.7, 1.1};
	const FVector VUe = FVector::CrossProduct(OrdFrame::AngularVelocityToUe(W), OrdFrame::PositionToUe(R));
	TestTrue(TEXT("angular velocity mirror"), Near(VUe, OrdFrame::VelocityToUe(ord::cross(W, R)), 1e-9));

	// Rotations: FQuat::RotateVector agrees with the core through the mirror.
	const ord::Quat Q = ord::Quat::fromAxisAngle({0.2, 0.9, -0.4}, 0.7);
	const FVector A = OrdFrame::RotationToUe(Q).RotateVector(OrdFrame::PositionToUe(R));
	TestTrue(TEXT("rotation mirror"), Near(A, OrdFrame::PositionToUe(Q.rotate(R)), 1e-9));

	// A UE yaw to the right (+Yaw) is a clockwise (negative) ISO yaw.
	const ord::Quat Yaw = OrdFrame::RotationFromUe(FQuat(FRotator(0.0, 30.0, 0.0)));
	const ord::Vec3 Fwd = Yaw.rotate({1.0, 0.0, 0.0});
	TestTrue(TEXT("UE +yaw turns ISO forward to the right (-Y)"), Fwd.y < -0.49 && Fwd.y > -0.51);

	// Inertia: diagonal, scale only.
	TestTrue(TEXT("inertia kg*m^2 -> kg*cm^2"), Near(OrdFrame::InertiaToUe({520.0, 2050.0, 2250.0}), FVector(5.2e6, 2.05e7, 2.25e7), 1e-3));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FOpenRoadDefinitionMappingTest, "OpenRoad.Adapter.DefinitionMapping", ORD_TEST_FLAGS)
bool FOpenRoadDefinitionMappingTest::RunTest(const FString& Parameters)
{
	const UOpenRoadVehicleDefinition* Asset = GetDefault<UOpenRoadVehicleDefinition>();
	ord::VehicleDefinition Def;
	FString Error;
	if (!TestTrue(TEXT("defaults convert to a valid core definition"), Asset->ToCore(Def, Error)))
	{
		AddError(Error);
		return false;
	}
	TestEqual(TEXT("mass"), Def.mass, 1450.0);
	TestEqual(TEXT("4 wheels"), static_cast<int32>(Def.wheels.size()), 4);
	TestEqual(TEXT("2 axles"), static_cast<int32>(Def.axles.size()), 2);
	// Wheel 0 is front-left: ISO +Y (UE -Y).
	TestTrue(TEXT("wheel 0 is left"), Def.wheels[0].isLeft && Def.wheels[0].suspension.hardpoint.y > 0.0);
	TestTrue(TEXT("wheel 1 is right"), !Def.wheels[1].isLeft && Def.wheels[1].suspension.hardpoint.y < 0.0);
	TestEqual(TEXT("front hardpoint x, m"), Def.wheels[0].suspension.hardpoint.x, 1.25);
	TestEqual(TEXT("CoM height, m"), Def.centerOfMass.z, 0.45);
	TestTrue(TEXT("inertia kept on the diagonal (roll, pitch, yaw)"),
	         Def.inertia.x == 520.0 && Def.inertia.y == 2050.0 && Def.inertia.z == 2250.0);
	TestEqual(TEXT("steer angle deg -> rad"), Def.wheels[0].maxSteerAngle, FMath::DegreesToRadians(31.513), 1e-4);
	TestEqual(TEXT("rear axle driven"), static_cast<int32>(Def.drivetrain.drivenAxles.size()), 1);
	TestEqual(TEXT("driven axle index"), static_cast<int32>(Def.drivetrain.drivenAxles[0].axleIndex), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FOpenRoadAssistMappingTest, "OpenRoad.Adapter.AssistMapping", ORD_TEST_FLAGS)
bool FOpenRoadAssistMappingTest::RunTest(const FString& Parameters)
{
	FOpenRoadAssistOverrides None;
	const ord::AssistConfig Sim = OrdConvert::MakeAssistConfig(EOpenRoadPreset::Simulation, EOpenRoadInputDevice::Gamepad, None);
	const ord::AssistConfig CoreSim = ord::makePreset(ord::DrivingPreset::Simulation, ord::InputDevice::Gamepad);
	TestEqual(TEXT("SIMULATION: ABS as core"), Sim.abs, CoreSim.abs);
	TestTrue(TEXT("SIMULATION: manual gearbox as core"), Sim.shiftMode == CoreSim.shiftMode);

	FOpenRoadAssistOverrides Overrides;
	Overrides.bOverrideAbs = true;
	Overrides.bAbs = true;
	Overrides.bOverrideShiftMode = true;
	Overrides.ShiftMode = EOpenRoadShiftMode::Automatic;
	Overrides.bOverrideStabilityControl = true;
	Overrides.StabilityControl = EOpenRoadIntervention::Full;
	const ord::AssistConfig O = OrdConvert::MakeAssistConfig(EOpenRoadPreset::Simulation, EOpenRoadInputDevice::Keyboard, Overrides);
	TestTrue(TEXT("override ABS"), O.abs);
	TestTrue(TEXT("override gearbox"), O.shiftMode == ord::ShiftMode::Automatic);
	TestTrue(TEXT("override ESC"), O.stabilityControl == ord::InterventionLevel::Full);
	TestTrue(TEXT("TCS untouched (SIMULATION: off)"), O.tractionControl == ord::InterventionLevel::Off);

	FOpenRoadSurface Wet;
	Wet.FrictionScale = 0.75f;
	Wet.WetSpeedFalloff = 0.004f;
	const ord::SurfaceProperties S = OrdConvert::ToCore(Wet);
	TestEqual(TEXT("wet friction scale"), S.frictionScale, 0.75, 1e-6);
	TestEqual(TEXT("wet speed falloff"), S.wetSpeedFalloff, 0.004, 1e-6);
	return true;
}

#endif  // WITH_DEV_AUTOMATION_TESTS
