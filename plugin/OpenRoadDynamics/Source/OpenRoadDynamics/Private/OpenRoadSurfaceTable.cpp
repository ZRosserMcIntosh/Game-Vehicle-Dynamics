// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT

#include "OpenRoadSurfaceTable.h"

const FOpenRoadSurface& UOpenRoadSurfaceTable::FindSurface(const UPhysicalMaterial* Material) const
{
	if (Material)
	{
		if (const FOpenRoadSurface* Found = Surfaces.Find(const_cast<UPhysicalMaterial*>(Material)))
		{
			return *Found;
		}
	}
	return DefaultSurface;
}
