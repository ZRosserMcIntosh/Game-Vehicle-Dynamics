// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// Blueprint-facing enums and the per-aid assist override struct. These mirror
// ord::DrivingPreset / ord::InputDevice / ord::AssistConfig (presets.hpp,
// assists.hpp); conversion lives in Private/OrdConversions.cpp.
#pragma once

#include "CoreMinimal.h"

#include "OpenRoadTypes.generated.h"

/** Named assist bundles. Presets never change the physical car (VehicleDefinition). */
UENUM(BlueprintType)
enum class EOpenRoadPreset : uint8
{
	Fun,
	Sport,
	Simulation,
};

/** Input device; presets choose different steering interpretation per device. */
UENUM(BlueprintType)
enum class EOpenRoadInputDevice : uint8
{
	Gamepad,
	SteeringWheel,
	Keyboard,
};

UENUM(BlueprintType)
enum class EOpenRoadSteeringMode : uint8
{
	/** Input maps linearly to road-wheel angle (wheels, "simulation steering"). */
	Direct,
	/** Gamepad-friendly: range limited by speed and peak slip angle, rate limited. */
	SpeedSensitive,
};

UENUM(BlueprintType)
enum class EOpenRoadIntervention : uint8
{
	Off,
	Sport,
	Full,
};

UENUM(BlueprintType)
enum class EOpenRoadShiftMode : uint8
{
	Automatic,
	Manual,
};

/**
 * Per-aid overrides applied on top of the selected preset. Every aid acts only
 * through real actuators (steering, throttle, brakes, clutch, gear).
 */
USTRUCT(BlueprintType)
struct OPENROADDYNAMICS_API FOpenRoadAssistOverrides
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideSteeringMode = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideSteeringMode", ToolTip = "How steering input becomes road-wheel angle."))
	EOpenRoadSteeringMode SteeringMode = EOpenRoadSteeringMode::Direct;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideSteeringExponent = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideSteeringExponent", ClampMin = "0.5", ClampMax = "3.0", ToolTip = "Input shaping |x|^e * sign(x). 1 = linear."))
	float SteeringExponent = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideSteerRate = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideSteerRate", ClampMin = "0.0", ToolTip = "Max change of steering input per second (1/s). 0 = unlimited."))
	float SteerRate = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideThrottleRate = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideThrottleRate", ClampMin = "0.0", ToolTip = "Max change of throttle per second (1/s). 0 = unlimited."))
	float ThrottleRate = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideAbs = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideAbs", ToolTip = "Anti-lock brakes."))
	bool bAbs = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideTractionControl = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideTractionControl", ToolTip = "Traction control intervention level."))
	EOpenRoadIntervention TractionControl = EOpenRoadIntervention::Off;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideStabilityControl = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideStabilityControl", ToolTip = "Stability control (ESC) intervention level."))
	EOpenRoadIntervention StabilityControl = EOpenRoadIntervention::Off;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideCountersteerAssist = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideCountersteerAssist", ClampMin = "0.0", ClampMax = "1.0", ToolTip = "0..1. Driving aid with no real-car equivalent: blends the front wheels toward the direction of travel in slides."))
	float CountersteerAssist = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideShiftMode = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideShiftMode", ToolTip = "Automatic or manual gear selection."))
	EOpenRoadShiftMode ShiftMode = EOpenRoadShiftMode::Automatic;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideAutoClutch = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideAutoClutch", ToolTip = "When off, the driver must operate the clutch pedal (SetClutch)."))
	bool bAutoClutch = true;
};
