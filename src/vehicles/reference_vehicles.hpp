// OpenRoad Dynamics — reference vehicle definitions.
// SPDX-License-Identifier: MIT
//
// Generic, fictional vehicles with plausible parameters for benchmarking.
// They are NOT models of any real car and contain no manufacturer or game
// data. Values are round numbers in typical ranges from textbook sources
// (e.g. Milliken & Milliken, "Race Car Vehicle Dynamics", 1995; Gillespie,
// "Fundamentals of Vehicle Dynamics", 1992).
#pragma once

#include "ord/vehicle_def.hpp"

namespace ord::vehicles {

// ~1450 kg, ~300 kW turbocharged RWD sports coupe with a clutch-pack LSD.
VehicleDefinition referenceSportsRwd();

// ~1250 kg, ~110 kW FWD hatchback with an open differential.
VehicleDefinition referenceHatchFwd();

// ~1600 kg, ~280 kW AWD saloon, 40/60 open centre split, rear LSD.
VehicleDefinition referenceSaloonAwd();

}  // namespace ord::vehicles
