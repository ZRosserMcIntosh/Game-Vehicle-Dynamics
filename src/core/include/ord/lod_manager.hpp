// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Fidelity tiers for many vehicles (docs/architecture/open-world-and-lod.md):
//   LOD0  player car(s): full Vehicle, owned by the game (not by this class)
//   LOD1  nearby AI / remote cars: Vehicle with VehicleFidelity::lod1(),
//         LaneFollowDriver, one coarse ground strip per car per frame
//   LOD2  ambient traffic: kinematic TrafficSystem agents
// The manager promotes the nearest LOD2 agents inside a radius (bounded pool,
// hysteresis on demotion) using analytic placement, so swaps cost a handful
// of ground rays and no settling steps.
#pragma once

#include <memory>
#include <vector>

#include "ord/ground_cache.hpp"
#include "ord/traffic.hpp"
#include "ord/vehicle.hpp"

namespace ord {

struct LodSettings {
    std::size_t maxLod1 = 16;
    Real promoteRadius = 150.0;   // m
    Real demoteRadius = 190.0;    // m (hysteresis)
    Real lod1Hz = 120.0;
    Real lod2Hz = 30.0;
    VehicleFidelity lod1Fidelity = VehicleFidelity::lod1();
    AssistConfig lod1Assists;     // AI drivers: ABS/TCS/ESC on, direct steering
    GroundPrefetchCache::Settings lod1Ground;

    LodSettings() {
        lod1Assists.abs = true;
        lod1Assists.tractionControl = InterventionLevel::Full;
        lod1Assists.stabilityControl = InterventionLevel::Full;
        lod1Ground.smoothSpacing = 1.5;
        lod1Ground.roughLines = 2;
        lod1Ground.roughSpacing = 0.5;
        lod1Ground.maxSamplesPerStrip = 12;
        lod1Ground.margin = 0.4;
    }
};

struct LodStats {
    std::size_t lod1Active = 0;
    std::size_t lod2Active = 0;
    std::uint64_t promotions = 0, demotions = 0;
    std::uint64_t lod1Steps = 0, lod2Steps = 0;
    Real lod1Seconds = 0, lod2Seconds = 0;  // wall-clock time spent (CPU cost)
};

class LodManager {
public:
    LodManager(TrafficSystem& traffic, std::vector<VehicleDefinition> models, const LodSettings& s = {});

    // One render frame: steps LOD2 and LOD1 at their rates, then rebalances.
    // `world` is the host's ground query (counted by the caller if desired).
    void update(Real frameDt, const Vec3& focus, const GroundQuery& world);

    struct Lod1Car {
        bool used = false;
        int agent = -1;
        int model = -1;
        std::unique_ptr<Vehicle> vehicle;
        LaneFollowDriver driver;
        GroundPrefetchCache ground;
    };
    const std::vector<Lod1Car>& lod1() const { return lod1_; }
    const LodStats& stats() const { return stats_; }
    TrafficSystem& traffic() { return traffic_; }

    // Exposed for transition tests.
    void promote(std::size_t agent, const GroundQuery& world);
    void demote(std::size_t slot);

private:
    Real desiredSpeedFor(const Lod1Car& c) const;

    TrafficSystem& traffic_;
    std::vector<VehicleDefinition> models_;
    LodSettings settings_;
    std::vector<Lod1Car> lod1_;
    std::vector<GroundStripRequest> requests_;
    std::vector<std::pair<Real, std::size_t>> candidates_;  // reused promotion candidates (no per-frame allocation)
    Real acc1_ = 0, acc2_ = 0;
    LodStats stats_;
};

}  // namespace ord
