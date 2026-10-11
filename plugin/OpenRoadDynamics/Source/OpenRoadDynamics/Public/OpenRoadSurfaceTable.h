// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// Maps Unreal physical materials to the core's SurfaceProperties (tire.hpp).
#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "PhysicalMaterials/PhysicalMaterial.h"

#include "OpenRoadSurfaceTable.generated.h"

/** Per-surface multipliers handed to the tire model. Grip lives in the tire, the surface only scales it. */
USTRUCT(BlueprintType)
struct OPENROADDYNAMICS_API FOpenRoadSurface
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface", meta = (ClampMin = "0.0", ClampMax = "2.0", ToolTip = "Multiplies the tire peak friction (dry asphalt = 1)."))
	float FrictionScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface", meta = (ClampMin = "0.0", ToolTip = "Multiplies the tire rolling resistance coefficient."))
	float RollingResistanceScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface", meta = (ClampMin = "0.0", ToolTip = "Crude wet-road falloff, SI (s/m): mu *= 1 / (1 + WetSpeedFalloff * v[m/s]). 0 = dry."))
	float WetSpeedFalloff = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface", meta = (ClampMin = "0.0", ToolTip = "Geometric roughness hint for the ground cache (0 = smooth road; > 0.05 = curbs, cobbles, gravel: sampled densely). Does not change grip."))
	float Roughness = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Micro-texture", meta = (ClampMin = "0.0", ClampMax = "0.05", ToolTip = "Procedural stone height for surfaces whose collision is flat (paving stones, rumble strips), m (SI). 0 = off. Deterministic in world position; costs no traces."))
	float TextureAmplitude = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Micro-texture", meta = (ClampMin = "0.02", ToolTip = "Stone pitch of the micro-texture, m (SI)."))
	float TextureWavelength = 0.15f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Micro-texture", meta = (ClampMin = "0.0", ClampMax = "1.0", ToolTip = "0 = identical stones, 1 = stone heights vary 0..Amplitude."))
	float TextureVariation = 0.4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface", meta = (ToolTip = "Free id reported per wheel in telemetry (audio, VFX, analytics)."))
	int32 SurfaceId = 0;
};

UCLASS(BlueprintType)
class OPENROADDYNAMICS_API UOpenRoadSurfaceTable : public UDataAsset
{
	GENERATED_BODY()

public:
	/** Used when the hit has no physical material or the material is not in the table. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Surfaces")
	FOpenRoadSurface DefaultSurface;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Surfaces")
	TMap<TObjectPtr<UPhysicalMaterial>, FOpenRoadSurface> Surfaces;

	/** Game thread only. Returns DefaultSurface when Material is null or unmapped. */
	const FOpenRoadSurface& FindSurface(const UPhysicalMaterial* Material) const;

	UFUNCTION(BlueprintPure, Category = "OpenRoad|Surfaces")
	FOpenRoadSurface GetSurface(UPhysicalMaterial* Material) const { return FindSurface(Material); }
};
