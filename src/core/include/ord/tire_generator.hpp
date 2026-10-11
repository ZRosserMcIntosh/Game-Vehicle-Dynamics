// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Plausible TireParams from a tire's public description (size + category),
// so vehicles can be authored without proprietary tire data (.tir files from
// tire makers / MF-Tyre are usually licensed). The mappings are engineering
// heuristics fitted to typical textbook ranges; they are NOT measurements of
// any product. See docs/research/tire-parameter-generation.md.
#pragma once

#include <optional>
#include <string_view>

#include "ord/tire.hpp"

namespace ord {

enum class TireCategory { Eco, Touring, Performance, UltraHighPerformance, SemiSlick, Slick, AllTerrain };

struct TireSpec {
    Real sectionWidthMm = 225;   // e.g. 245 in 245/40R18
    Real aspectRatio = 45;       // e.g. 40
    Real rimDiameterIn = 18;     // e.g. 18
    TireCategory category = TireCategory::Performance;
    Real nominalLoad = 4000;     // N: use the static corner load of the car
};

// Parses "245/40R18" or "245/40 R18" (category/load stay at defaults).
std::optional<TireSpec> parseTireSize(std::string_view s);

TireParams generateTireParams(const TireSpec& spec);

}  // namespace ord
