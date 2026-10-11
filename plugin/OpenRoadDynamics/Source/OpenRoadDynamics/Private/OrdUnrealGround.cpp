// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT

#include "OrdUnrealGround.h"

#include "Engine/HitResult.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "OpenRoadSurfaceTable.h"
#include "OrdConversions.h"
#include "OrdFrame.h"
#include "PhysicalMaterials/PhysicalMaterial.h"

FOrdUnrealGround::FOrdUnrealGround(const UWorld* InWorld, ECollisionChannel InChannel, const AActor* IgnoredActor,
                                   const UOpenRoadSurfaceTable* InSurfaces)
	: World(InWorld)
	, Channel(InChannel)
	, Surfaces(InSurfaces)
	, QueryParams(SCENE_QUERY_STAT(OpenRoadGround), /*bTraceComplex*/ false, IgnoredActor)
{
	QueryParams.bReturnPhysicalMaterial = true;
}

ord::GroundHit FOrdUnrealGround::raycast(const ord::Vec3& Origin, const ord::Vec3& Dir, ord::Real MaxDistance) const
{
	check(IsInGameThread());

	ord::GroundHit Out;
	if (!World)
	{
		return Out;
	}

	const FVector Start = OrdFrame::PositionToUe(Origin);
	const FVector End = OrdFrame::PositionToUe(Origin + Dir * MaxDistance);

	FHitResult Hit;
	++TraceCount;
	if (!World->LineTraceSingleByChannel(Hit, Start, End, Channel, QueryParams) || !Hit.bBlockingHit)
	{
		return Out;
	}

	Out.hit = true;
	Out.distance = Hit.Distance / ord::host::kUeUnitsPerMetre;
	Out.point = OrdFrame::PositionFromUe(Hit.ImpactPoint);
	Out.normal = ord::normalized(OrdFrame::DirectionFromUe(Hit.ImpactNormal));

	FOpenRoadSurface Surface;
	if (Surfaces)
	{
		Surface = Surfaces->FindSurface(Hit.PhysMaterial.Get());
	}
	Out.surface = OrdConvert::ToCore(Surface);
	Out.surfaceId = Surface.SurfaceId;
	return Out;
}
