// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// Blueprint types -> core types. Game thread (pure functions, no state).
#pragma once

#include "CoreMinimal.h"
#include "OpenRoadSurfaceTable.h"
#include "OpenRoadTypes.h"
#include "OpenRoadVehicleDefinition.h"

#include "ord/assists.hpp"
#include "ord/inputs.hpp"
#include "ord/presets.hpp"
#include "ord/tire.hpp"

namespace OrdConvert
{
ord::DrivingPreset ToCore(EOpenRoadPreset Preset);
ord::InputDevice ToCore(EOpenRoadInputDevice Device);
ord::SteeringMode ToCore(EOpenRoadSteeringMode Mode);
ord::InterventionLevel ToCore(EOpenRoadIntervention Level);
ord::ShiftMode ToCore(EOpenRoadShiftMode Mode);

/** Preset bundle for the device, then every enabled override on top. */
ord::AssistConfig MakeAssistConfig(EOpenRoadPreset Preset, EOpenRoadInputDevice Device, const FOpenRoadAssistOverrides& Overrides);

ord::SurfaceProperties ToCore(const FOpenRoadSurface& Surface);

/** Starts from the core defaults, so unexposed fields keep them. */
ord::TireParams ToCore(const FOpenRoadTireParams& Tire);
}  // namespace OrdConvert
