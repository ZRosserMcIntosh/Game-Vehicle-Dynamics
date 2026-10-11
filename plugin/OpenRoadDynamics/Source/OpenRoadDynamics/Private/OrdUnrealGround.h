// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// ord::GroundQuery backed by UWorld line traces. GAME THREAD ONLY.
//
// The physics thread never calls this class: once per frame the vehicle
// component rebuilds an ord::GroundPrefetchCache from it on the game thread and
// hands the immutable cache to the physics thread (see OrdChaosBridge.h).
// This deliberately narrows the core's "thread-safe for concurrent const calls"
// contract: a constructed instance lives on the game-thread stack for one
// rebuild and is used by one thread only.
#pragma once

#include "CoreMinimal.h"
#include "CollisionQueryParams.h"
#include "Engine/EngineTypes.h"

#include "ord/ground.hpp"

class AActor;
class UOpenRoadSurfaceTable;
class UWorld;

class FOrdUnrealGround final : public ord::GroundQuery
{
public:
	/**
	 * @param World        world to trace in (must outlive this object)
	 * @param Channel      trace channel (e.g. ECC_Visibility or a project "Vehicle Ground" channel)
	 * @param IgnoredActor the vehicle itself
	 * @param Surfaces     optional physical material -> surface mapping
	 */
	FOrdUnrealGround(const UWorld* World, ECollisionChannel Channel, const AActor* IgnoredActor, const UOpenRoadSurfaceTable* Surfaces);

	virtual ord::GroundHit raycast(const ord::Vec3& Origin, const ord::Vec3& Dir, ord::Real MaxDistance) const override;

	/** Engine traces issued through this object. */
	int32 NumTraces() const { return TraceCount; }

private:
	const UWorld* World = nullptr;
	ECollisionChannel Channel = ECC_Visibility;
	const UOpenRoadSurfaceTable* Surfaces = nullptr;
	FCollisionQueryParams QueryParams;
	mutable int32 TraceCount = 0;
};
