// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// Data asset mirroring the important fields of ord::VehicleDefinition
// (vehicle_def.hpp, tire.hpp, drivetrain.hpp).
//
// Units: POSITIONS use Unreal conventions (centimetres, UE axes: X forward,
// Y right, Z up, relative to the actor's root component origin). EVERYTHING ELSE
// is SI (kg, m, N, N/m, N*s/m, N*m, kg*m^2, rpm for engine speed) except
// steering/toe angles, which are in degrees. Fields not exposed here keep the
// core defaults.
#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"

#include "OpenRoadVehicleDefinition.generated.h"

namespace ord
{
struct VehicleDefinition;
}

UENUM(BlueprintType)
enum class EOpenRoadDifferential : uint8
{
	Open,
	Locked,
	/** Clutch-type limited slip: locking torque = preload + ramp * |input torque|. */
	ClutchLsd,
};

UENUM(BlueprintType)
enum class EOpenRoadCenterCoupling : uint8
{
	Open,
	Locked,
	ClutchLsd,
};

UENUM(BlueprintType)
enum class EOpenRoadTransmission : uint8
{
	Automatic,
	Sequential,
	DualClutch,
	ManualHPattern,
};

/** Reduced Magic Formula tire parameters (subset of ord::TireParams). SI units. */
USTRUCT(BlueprintType)
struct OPENROADDYNAMICS_API FOpenRoadTireParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire", meta = (ClampMin = "0.1", ToolTip = "Loaded rolling radius, m (SI)."))
	float Radius = 0.33f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire", meta = (ClampMin = "100", ToolTip = "Nominal load Fz0, N."))
	float NominalLoad = 3600.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire", meta = (ClampMin = "0.1", ToolTip = "Longitudinal peak friction at nominal load."))
	float MuX = 1.15f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire", meta = (ToolTip = "Load sensitivity of MuX: mu = mu0 * (1 + s * (Fz - Fz0) / Fz0)."))
	float MuXLoadSensitivity = -0.08f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire", meta = (ClampMin = "0.1", ToolTip = "Lateral peak friction at nominal load."))
	float MuY = 1.10f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire", meta = (ToolTip = "Load sensitivity of MuY."))
	float MuYLoadSensitivity = -0.10f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire|Shape", meta = (ClampMin = "0.1", ToolTip = "Magic Formula shape factor C, longitudinal."))
	float ShapeX = 1.65f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire|Shape", meta = (ClampMax = "1.0", ToolTip = "Magic Formula curvature factor E, longitudinal (<= 1)."))
	float CurvatureX = 0.10f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire|Shape", meta = (ClampMin = "0.1", ToolTip = "Magic Formula shape factor C, lateral."))
	float ShapeY = 1.35f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire|Shape", meta = (ClampMax = "1.0", ToolTip = "Magic Formula curvature factor E, lateral (<= 1)."))
	float CurvatureY = -0.40f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire|Stiffness", meta = (ClampMin = "1.0", ToolTip = "Longitudinal slip stiffness per unit load: Kx = value * Fz."))
	float LongStiffnessPerLoad = 18.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire|Stiffness", meta = (ClampMin = "1.0", ToolTip = "Peak cornering stiffness in units of Fz0 per rad."))
	float CornerStiffness = 16.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire|Stiffness", meta = (ClampMin = "0.1", ToolTip = "Load (in Fz0) at which cornering stiffness peaks."))
	float CornerStiffnessLoad = 1.9f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire|Transient", meta = (ClampMin = "0.01", ToolTip = "Longitudinal relaxation length, m."))
	float RelaxationLengthX = 0.15f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire|Transient", meta = (ClampMin = "0.01", ToolTip = "Lateral relaxation length, m."))
	float RelaxationLengthY = 0.45f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire", meta = (ClampMin = "0.0", ToolTip = "Rolling resistance coefficient (dimensionless)."))
	float RollingResistance = 0.012f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire", meta = (ClampMin = "0.0", ToolTip = "Pneumatic trail at zero slip angle, m."))
	float PneumaticTrail = 0.035f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire", meta = (ClampMin = "0.05", ToolTip = "Wheel + tire + hub + rotor spin inertia, kg*m^2."))
	float WheelInertia = 1.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire|Vertical", meta = (ClampMin = "1000", ToolTip = "Radial stiffness of the contact patch, N/m (unsprung-mass fidelity)."))
	float VerticalStiffness = 250000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire|Vertical", meta = (ClampMin = "0.0", ToolTip = "Radial damping, N*s/m."))
	float VerticalDamping = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire|Vertical", meta = (ClampMin = "0.0", ToolTip = "Contact patch half length along the wheel heading, m (multi-point contact)."))
	float PatchHalfLength = 0.09f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire|Vertical", meta = (ClampMin = "0.0", ToolTip = "Contact patch half width, m."))
	float PatchHalfWidth = 0.10f;
};

/** One axle: a mirrored left/right wheel pair sharing suspension, tire, brakes and ARB. */
USTRUCT(BlueprintType)
struct OPENROADDYNAMICS_API FOpenRoadAxle
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle")
	FName Name = TEXT("Axle");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Suspension", meta = (ToolTip = "Wheel-centre position at FULL BUMP, cm, UE axes, relative to the actor root origin. |Y| is the half track: the left wheel is placed at -|Y|, the right wheel at +|Y| (UE Y points right)."))
	FVector Hardpoint = FVector(125.0, 80.0, 40.0);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Suspension", meta = (ClampMin = "0.01", ToolTip = "Suspension travel from full droop to full bump, m (SI)."))
	float Travel = 0.18f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Suspension", meta = (ClampMin = "1000", ToolTip = "Spring rate at the wheel, N/m."))
	float SpringRate = 45000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Suspension", meta = (ClampMin = "0.0", ToolTip = "Spring preload at full droop, N."))
	float SpringPreload = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Suspension", meta = (ClampMin = "0.0", ToolTip = "Bump (compression) damping, N*s/m."))
	float BumpDamping = 3200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Suspension", meta = (ClampMin = "0.0", ToolTip = "Rebound (extension) damping, N*s/m."))
	float ReboundDamping = 4800.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Suspension", meta = (ClampMin = "0.0", ToolTip = "Bump stop rate beyond full bump, N/m."))
	float BumpStopRate = 300000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Suspension", meta = (ClampMin = "0.0", ToolTip = "Anti-roll bar rate at the wheel, N/m of left/right travel difference."))
	float AntiRollBarRate = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Steering", meta = (ClampMin = "0.0", ClampMax = "60.0", ToolTip = "Road-wheel angle at full lock, DEGREES. 0 = not steered."))
	float MaxSteerAngleDeg = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Steering", meta = (ClampMin = "-5.0", ClampMax = "5.0", ToolTip = "Static toe, DEGREES (+ = toe-in)."))
	float StaticToeDeg = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Brakes", meta = (ClampMin = "0.0", ToolTip = "Brake torque per wheel at 100% pedal, N*m."))
	float MaxBrakeTorque = 2000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Brakes", meta = (ClampMin = "0.0", ToolTip = "Handbrake torque per wheel at 100%, N*m."))
	float HandbrakeTorque = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Wheel", meta = (ClampMin = "0.0", ToolTip = "Unsprung mass per wheel, kg. Part of the vehicle's total mass."))
	float UnsprungMassKg = 40.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Wheel")
	FOpenRoadTireParams Tire;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Drive", meta = (ToolTip = "Engine torque reaches this axle through its differential."))
	bool bDriven = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Drive", meta = (EditCondition = "bDriven", ClampMin = "0.0", ToolTip = "Open-centre torque share. Shares of driven axles are normalised to sum to 1."))
	float TorqueShare = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Drive", meta = (EditCondition = "bDriven"))
	EOpenRoadDifferential Differential = EOpenRoadDifferential::Open;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Drive", meta = (EditCondition = "bDriven", ClampMin = "0.0", ToolTip = "Clutch LSD preload, N*m."))
	float LsdPreload = 80.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Drive", meta = (EditCondition = "bDriven", ClampMin = "0.0", ToolTip = "Clutch LSD ramp on drive (fraction of input torque)."))
	float LsdPowerRamp = 0.45f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axle|Drive", meta = (EditCondition = "bDriven", ClampMin = "0.0", ToolTip = "Clutch LSD ramp on overrun (fraction of input torque)."))
	float LsdCoastRamp = 0.25f;
};

/**
 * Physical description of a car. Defaults describe a generic front-engine,
 * rear-wheel-drive sports car (1450 kg, 2.55 m wheelbase) whose mesh pivot sits
 * on the ground between the axles.
 */
UCLASS(BlueprintType)
class OPENROADDYNAMICS_API UOpenRoadVehicleDefinition : public UDataAsset
{
	GENERATED_BODY()

public:
	UOpenRoadVehicleDefinition();

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Vehicle")
	FString VehicleName = TEXT("Generic Sports RWD");

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Vehicle|Mass", meta = (ClampMin = "50", ToolTip = "Total mass including unsprung masses, kg. Applied to the root body with SetMassOverrideInKg."))
	float MassKg = 1450.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Vehicle|Mass", meta = (ToolTip = "Centre of mass, cm, UE axes, relative to the actor root origin."))
	FVector CenterOfMass = FVector(0.0, 0.0, 45.0);

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Vehicle|Mass", meta = (ToolTip = "Principal moments of inertia about the CoM, kg*m^2 (SI): X = roll, Y = pitch, Z = yaw."))
	FVector InertiaKgM2 = FVector(520.0, 2050.0, 2250.0);

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Vehicle|Steering", meta = (ClampMin = "0.0", ClampMax = "1.0", ToolTip = "0 = parallel steer, 1 = full Ackermann."))
	float SteeringAckermann = 0.5f;

	/** Front to rear. Each axle produces a left wheel (index 2i) and a right wheel (2i+1). Max 4 axles. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Vehicle|Axles")
	TArray<FOpenRoadAxle> Axles;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Engine", meta = (ToolTip = "Wide-open-throttle net crank torque. X = rpm, Y = torque in N*m. Rpm must increase; at least 2 points."))
	TArray<FVector2D> TorqueCurve;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Engine", meta = (ClampMin = "300"))
	float IdleRpm = 850.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Engine", meta = (ClampMin = "1000", ToolTip = "Fuel cut above this engine speed, rpm."))
	float LimiterRpm = 7200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Engine", meta = (ClampMin = "0.01", ToolTip = "Crank + flywheel inertia, kg*m^2."))
	float EngineInertia = 0.18f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Engine", meta = (ClampMin = "0.0", ToolTip = "Engine friction torque T = a + b*omega: constant part a, N*m."))
	float EngineFrictionTorque = 18.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Engine", meta = (ClampMin = "0.0", ToolTip = "Engine friction torque: speed-proportional part b, N*m per rad/s."))
	float EngineFrictionPerRadS = 0.035f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Clutch", meta = (ClampMin = "10", ToolTip = "Clutch torque capacity when fully engaged, N*m."))
	float ClutchMaxTorque = 800.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Gearbox", meta = (ToolTip = "Forward gear ratios, 1st first."))
	TArray<float> ForwardGearRatios;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Gearbox", meta = (ClampMin = "0.1"))
	float ReverseGearRatio = 3.2f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Gearbox", meta = (ClampMin = "0.001", ToolTip = "Clutch disc + gearbox input inertia, kg*m^2."))
	float GearboxInputInertia = 0.03f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Gearbox", meta = (ClampMin = "0.5", ClampMax = "1.0"))
	float GearboxEfficiency = 0.93f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Gearbox", meta = (ClampMin = "0.0", ToolTip = "Torque interruption per shift, s."))
	float ShiftTime = 0.15f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Gearbox")
	EOpenRoadTransmission Transmission = EOpenRoadTransmission::DualClutch;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Differential", meta = (ClampMin = "1.0", ToolTip = "Final drive ratio, applied to every driven axle's differential."))
	float FinalDrive = 3.5f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Differential", meta = (ToolTip = "Centre coupling; only used with exactly two driven axles."))
	EOpenRoadCenterCoupling CenterCoupling = EOpenRoadCenterCoupling::Open;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Differential", meta = (ClampMin = "0.0", ToolTip = "Centre LSD preload, N*m."))
	float CenterPreload = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drivetrain|Differential", meta = (ClampMin = "0.0", ToolTip = "Centre LSD ramp."))
	float CenterRamp = 0.3f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aero", meta = (ClampMin = "0.0", ToolTip = "Drag area Cd*A, m^2."))
	float DragArea = 0.68f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aero", meta = (ToolTip = "Front axle lift area Cl*A, m^2 (positive = downforce)."))
	float FrontDownforceArea = 0.10f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aero", meta = (ToolTip = "Rear axle lift area Cl*A, m^2 (positive = downforce)."))
	float RearDownforceArea = 0.15f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aero", meta = (ClampMin = "0.5", ClampMax = "1.5", ToolTip = "Air density, kg/m^3."))
	float AirDensity = 1.225f;

	/**
	 * Converts to the core definition (ISO 8855 axes, metres) and validates it.
	 * Returns false with a human-readable reason when the result is invalid.
	 */
	bool ToCore(ord::VehicleDefinition& Out, FString& OutError) const;

	/** Blueprint/editor helper: true when the asset converts to a valid core definition. */
	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Vehicle")
	bool Validate(FString& OutError) const;
};
