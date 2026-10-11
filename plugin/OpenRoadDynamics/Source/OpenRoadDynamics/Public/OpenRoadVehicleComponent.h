// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// UOpenRoadVehicleComponent: drives the owner's simulating root body with the
// OpenRoad Dynamics core, inside Chaos async physics.
//
// Requirements:
//  * the owner's root component is a UPrimitiveComponent with Simulate Physics on
//    (unit scale; its origin is the vehicle body origin of the definition);
//  * Project Settings > Physics: Tick Physics Async = true, a fixed Async Fixed
//    Time Step Size (0.004166 = 240 Hz, or 0.016666 = 60 Hz with VehicleRateHz
//    240 for vehicle-local substepping), Chaos substepping off.
//
// Data flow (see Private/OrdChaosBridge.h for the full diagram):
//   game thread, TG_PrePhysics: read physics snapshots -> rebuild ground cache
//   with line traces -> push driver input + cache; physics thread: read body ->
//   step the core -> AddForce/AddTorque -> write snapshot.
// The ord::Vehicle is owned by the physics thread after BeginPlay.
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/EngineTypes.h"
#include "OpenRoadTypes.h"
#include "Templates/PimplPtr.h"

#include "OpenRoadVehicleComponent.generated.h"

class UOpenRoadSurfaceTable;
class UOpenRoadVehicleDefinition;
struct FOrdComponentRuntime;

UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent, DisplayName = "OpenRoad Vehicle"))
class OPENROADDYNAMICS_API UOpenRoadVehicleComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UOpenRoadVehicleComponent();

	// ---------------------------------------------------------------- Setup

	/** Physical car. Read once in BeginPlay (changing it later has no effect until the next BeginPlay). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Vehicle")
	TObjectPtr<UOpenRoadVehicleDefinition> Definition;

	/** Physical material -> surface mapping. Optional (default surface = dry asphalt). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Ground")
	TObjectPtr<UOpenRoadSurfaceTable> SurfaceTable;

	/** Channel for the ground line traces. A dedicated "VehicleGround" trace channel is recommended. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Ground")
	TEnumAsByte<ECollisionChannel> GroundTraceChannel = ECC_Visibility;

	/**
	 * Minimum seconds of predicted wheel travel covered by each per-frame ground
	 * strip, s. The strip must cover the time from the physics step that
	 * requested it until the physics step that receives the NEXT cache.
	 * With bAdaptiveGroundHorizon the horizon grows above this value from the
	 * measured cache age and game-frame time.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Ground", meta = (ClampMin = "0.01", ClampMax = "0.5"))
	float GroundPredictionSeconds = 0.075f;

	/**
	 * Adapt the strip horizon to the measured latency:
	 * horizon = clamp(max(GroundPredictionSeconds, 1.5 * peak frame time, 1.25 * peak cache age + one physics step),
	 *                 1/60 s, MaxGroundPredictionSeconds); peaks decay by 0.02 s per second (OrdGroundHorizon.h).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Ground")
	bool bAdaptiveGroundHorizon = true;

	/** Upper bound of the adaptive horizon, s (longer strips cost more line traces per frame). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Ground", meta = (ClampMin = "0.02", ClampMax = "1.0", EditCondition = "bAdaptiveGroundHorizon"))
	float MaxGroundPredictionSeconds = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Driving")
	EOpenRoadPreset Preset = EOpenRoadPreset::Sport;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Driving")
	EOpenRoadInputDevice InputDevice = EOpenRoadInputDevice::Gamepad;

	/** Per-aid overrides on top of Preset. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Driving")
	FOpenRoadAssistOverrides AssistOverrides;

	/**
	 * Vehicle-local simulation rate, Hz. Each Chaos step runs ceil(step * rate)
	 * vehicle substeps (ADR-0003): 1 with a 240 Hz async step, 4 with a 60 Hz one.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Fidelity", meta = (ClampMin = "30", ClampMax = "1000"))
	float VehicleRateHz = 240.0f;

	/** Per-corner wheel DOF with tire vertical stiffness (LOD0). Off = massless wheels (LOD1). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Fidelity")
	bool bUnsprungMass = true;

	/** Contact samples per wheel: 1, 5, 8 or 9. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Fidelity", meta = (ClampMin = "1", ClampMax = "9"))
	int32 ContactSamples = 5;

	/** Draw contact, suspension and tire-force debug primitives (also: console variable openroad.Debug 1). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Debug")
	bool bDebugDraw = false;

	/** Telemetry ring-buffer length for StartTelemetryCapture, s of simulation. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad|Telemetry", meta = (ClampMin = "1.0", ClampMax = "600.0"))
	float TelemetryCaptureSeconds = 30.0f;

	// ---------------------------------------------------------------- Driver input (game thread)

	/** 0..1 */
	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Input")
	void SetThrottle(float Value);

	/** 0..1 */
	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Input")
	void SetBrake(float Value);

	/** -1..1, POSITIVE = RIGHT (Unreal convention; converted to the core's + = left). */
	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Input")
	void SetSteering(float Value);

	/** 0..1 */
	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Input")
	void SetHandbrake(float Value);

	/** 0..1 pedal position, 1 = fully pressed (disengaged). Only used without auto clutch. */
	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Input")
	void SetClutch(float Value);

	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Input")
	void ShiftUp();

	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Input")
	void ShiftDown();

	// ---------------------------------------------------------------- Assists

	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Driving")
	void SetPreset(EOpenRoadPreset NewPreset);

	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Driving")
	void SetInputDevice(EOpenRoadInputDevice NewDevice);

	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Driving")
	void SetAssistOverrides(const FOpenRoadAssistOverrides& NewOverrides);

	/** Convenience: overrides ABS on top of the preset. */
	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Driving")
	void SetAbsEnabled(bool bEnabled);

	/** Convenience: overrides the traction-control level on top of the preset. */
	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Driving")
	void SetTractionControl(EOpenRoadIntervention Level);

	/** Convenience: overrides the stability-control (ESC) level on top of the preset. */
	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Driving")
	void SetStabilityControl(EOpenRoadIntervention Level);

	/** Convenience: overrides automatic/manual gear selection on top of the preset. */
	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Driving")
	void SetShiftMode(EOpenRoadShiftMode Mode);

	UFUNCTION(BlueprintPure, Category = "OpenRoad|Driving")
	EOpenRoadPreset GetPreset() const { return Preset; }

	/** Effective settings (preset + overrides, applied on the next physics step) and the latest aid activity. */
	UFUNCTION(BlueprintPure, Category = "OpenRoad|Driving")
	FOpenRoadAssistState GetAssistState() const;

	// ---------------------------------------------------------------- Readouts (latest physics snapshot)

	/** Longitudinal speed, km/h. */
	UFUNCTION(BlueprintPure, Category = "OpenRoad|State")
	float GetSpeedKmh() const;

	UFUNCTION(BlueprintPure, Category = "OpenRoad|State")
	float GetEngineRpm() const;

	/** -1 reverse, 0 neutral, 1..N forward. */
	UFUNCTION(BlueprintPure, Category = "OpenRoad|State")
	int32 GetGear() const;

	/** True while the core is driving the body. */
	UFUNCTION(BlueprintPure, Category = "OpenRoad|State")
	bool IsVehicleActive() const;

	/** Cumulative ground-cache misses of this vehicle (should stay ~0; see README "Validation plan"). */
	UFUNCTION(BlueprintPure, Category = "OpenRoad|Debug")
	int64 GetGroundCacheMisses() const;

	/** Wheels of the vehicle (2 per axle), 0 when inactive. Index 2i = left, 2i+1 = right of axle i. */
	UFUNCTION(BlueprintPure, Category = "OpenRoad|State")
	int32 GetNumWheels() const;

	/** Latest per-wheel readout. Returns false (and a default state) for an invalid index or no data yet. */
	UFUNCTION(BlueprintPure, Category = "OpenRoad|State")
	bool GetWheelState(int32 WheelIndex, FOpenRoadWheelState& OutState) const;

	/** Coupling, mass-property and ground-cache health of the latest frame. */
	UFUNCTION(BlueprintPure, Category = "OpenRoad|Debug")
	FOpenRoadDiagnostics GetDiagnostics() const;

	// ---------------------------------------------------------------- Debug and telemetry

	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Debug")
	void SetDebugDrawEnabled(bool bEnabled);

	UFUNCTION(BlueprintPure, Category = "OpenRoad|Debug")
	bool IsDebugDrawEnabled() const { return bDebugDraw; }

	/** Starts recording every physics step into a ring buffer of TelemetryCaptureSeconds. */
	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Telemetry")
	void StartTelemetryCapture();

	/** Stops recording and writes Saved/OpenRoad/<Actor>_<timestamp>.csv. Returns the file path ("" if nothing was written). */
	UFUNCTION(BlueprintCallable, Category = "OpenRoad|Telemetry")
	FString StopTelemetryCapture();

	UFUNCTION(BlueprintPure, Category = "OpenRoad|Telemetry")
	bool IsCapturingTelemetry() const;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	/** Backstop: a component unregistered without EndPlay must still unregister its Chaos callback while the world lives. */
	virtual void OnUnregister() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	bool ActivateVehicle();
	void DeactivateVehicle(const TCHAR* Reason);
	void BindBody(UPrimitiveComponent* NewBody);
	void DrawDebug() const;

	/**
	 * Destroyed: forgets the proxy in this frame's input and unregisters (the
	 * physics thread holds a raw proxy pointer). Created: re-activates the
	 * vehicle on the next tick (state is rebuilt from the body; gear, wheel
	 * spin and slip start fresh).
	 */
	UFUNCTION()
	void HandleBodyPhysicsStateChanged(UPrimitiveComponent* ChangedComponent, EComponentPhysicsStateChange::Type StateChange);

	/** The simulating root body we drive and whose physics-state delegate we are bound to. */
	UPROPERTY(Transient)
	TObjectPtr<UPrimitiveComponent> Body;

	/** Set when the body's physics state was (re)created while the vehicle was inactive. */
	bool bPendingReactivate = false;

	// Driver state (game thread). Shift presses are counters so the physics
	// thread sees exactly one edge per press whatever the step/frame ratio.
	float Throttle = 0.0f;
	float Brake = 0.0f;
	float Steering = 0.0f;
	float Handbrake = 0.0f;
	float Clutch = 0.0f;
	uint32 ShiftUpCount = 0;
	uint32 ShiftDownCount = 0;
	uint32 AssistsRevision = 0;

	/** Game-thread runtime state (link to the physics thread, ground caches, telemetry). */
	TPimplPtr<FOrdComponentRuntime> Runtime;
};
