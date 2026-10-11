// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/lod_manager.hpp"

#include <algorithm>
#include <chrono>

namespace ord {

namespace {
using Clock = std::chrono::steady_clock;
Real seconds(Clock::time_point a, Clock::time_point b) { return std::chrono::duration<Real>(b - a).count(); }
}  // namespace

LodManager::LodManager(TrafficSystem& traffic, std::vector<VehicleDefinition> models, const LodSettings& s)
    : traffic_(traffic), models_(std::move(models)), settings_(s) {
    lod1_.resize(settings_.maxLod1);
    for (auto& c : lod1_) c.ground = GroundPrefetchCache(settings_.lod1Ground);
    requests_.reserve(1);
}

Real LodManager::desiredSpeedFor(const Lod1Car& c) const {
    const auto& agents = traffic_.agents();
    const TrafficSystem::Agent& me = agents[static_cast<std::size_t>(c.agent)];
    const LanePath& lane = traffic_.lanes()[static_cast<std::size_t>(me.lane)];
    const Real s = c.driver.laneS();
    const Real v = std::max(0.0, c.vehicle->forwardSpeed());
    const IdmParams& p = traffic_.settings().idm;
    // Nearest vehicle ahead on the same lane: active LOD2 agents and other LOD1 cars.
    Real gap = -1, vl = 0;
    auto consider = [&](Real os, Real ospeed) {
        Real d = os - s;
        if (lane.loop() && d < 0) d += lane.length();
        if (d <= 0) return;
        const Real g = std::max(0.1, d - p.vehicleLength);
        if (gap < 0 || g < gap) { gap = g; vl = ospeed; }
    };
    for (const auto& a : agents)
        if (a.active && a.lane == me.lane) consider(a.s, a.speed);
    for (const auto& o : lod1_)
        if (o.used && &o != &c && agents[static_cast<std::size_t>(o.agent)].lane == me.lane)
            consider(o.driver.laneS(), o.vehicle->forwardSpeed());
    Real v0 = std::min(me.desiredSpeed, lane.speedLimit());
    const Real k = lane.maxCurvature(s, s + std::max(20.0, 3.0 * v));
    if (k > 1e-5) v0 = std::min(v0, std::sqrt(p.maxLateralAccel / k));
    const Real a = idmAcceleration(p, v, v0, gap, vl);
    return clamp(v + a * 1.0, 0.0, v0);
}

void LodManager::promote(std::size_t agentIndex, const GroundQuery& world) {
    auto free = std::find_if(lod1_.begin(), lod1_.end(), [](const Lod1Car& c) { return !c.used; });
    if (free == lod1_.end()) return;
    TrafficSystem::Agent& a = traffic_.agents()[agentIndex];
    Lod1Car& c = *free;
    if (!c.vehicle || c.model != a.model) {
        c.vehicle = std::make_unique<Vehicle>(models_[static_cast<std::size_t>(a.model)], settings_.lod1Assists,
                                              settings_.lod1Fidelity);
        c.model = a.model;
    }
    Vehicle& v = *c.vehicle;
    v.mutableState().position.z = a.position.z;  // search hint for the placement rays
    v.selectGearForSpeed(a.speed);
    v.placeOnGround(a.position.x, a.position.y, a.yaw, world, a.speed, 0.0, a.yawRate);
    c.driver.reset(a.s);
    c.agent = static_cast<int>(agentIndex);
    c.used = true;
    a.active = false;
    ++stats_.promotions;
}

void LodManager::demote(std::size_t slot) {
    Lod1Car& c = lod1_[slot];
    if (!c.used) return;
    TrafficSystem::Agent& a = traffic_.agents()[static_cast<std::size_t>(c.agent)];
    const LanePath& lane = traffic_.lanes()[static_cast<std::size_t>(a.lane)];
    const Vec3 com = c.vehicle->centerOfMassWorld();
    a.s = lane.project(com, c.driver.laneS());
    a.speed = std::max(0.0, c.vehicle->forwardSpeed());
    a.position = com;
    const Vec3 fwd = c.vehicle->state().orientation.rotate({1, 0, 0});
    a.yaw = std::atan2(fwd.y, fwd.x);
    a.yawRate = c.vehicle->yawRate();
    a.active = true;
    c.used = false;
    ++stats_.demotions;
}

void LodManager::update(Real frameDt, const Vec3& focus, const GroundQuery& world) {
    // ---- LOD2 traffic ----
    const auto t0 = Clock::now();
    traffic_.clearObstacles();
    for (const auto& c : lod1_)
        if (c.used)
            traffic_.addObstacle(traffic_.agents()[static_cast<std::size_t>(c.agent)].lane, c.driver.laneS(),
                                 c.vehicle->forwardSpeed());
    const Real dt2 = 1.0 / settings_.lod2Hz;
    acc2_ += frameDt;
    while (acc2_ >= dt2) {
        traffic_.step(dt2, &world);
        acc2_ -= dt2;
        ++stats_.lod2Steps;
    }
    const auto t1 = Clock::now();
    stats_.lod2Seconds += seconds(t0, t1);

    // ---- LOD1 vehicles: one coarse strip per car per frame, then fixed steps ----
    const Real dt1 = 1.0 / settings_.lod1Hz;
    acc1_ += frameDt;
    const int n1 = static_cast<int>(acc1_ / dt1);
    acc1_ -= n1 * dt1;
    for (auto& c : lod1_) {
        if (!c.used) continue;
        Vehicle& v = *c.vehicle;
        const VehicleDefinition& d = v.definition();
        GroundStripRequest r;
        r.center = v.centerOfMassWorld();
        r.velocity = v.state().velocity;
        r.heading = v.state().orientation.rotate({1, 0, 0});
        r.halfLength = 0.5 * d.wheelbase() + 0.4;
        r.halfWidth = std::abs(d.wheels[0].suspension.hardpoint.y) + 0.2;
        c.ground.rebuild(world, &r, 1);
        const LanePath& lane = traffic_.lanes()[static_cast<std::size_t>(traffic_.agents()[static_cast<std::size_t>(c.agent)].lane)];
        const Real desired = desiredSpeedFor(c);
        for (int k = 0; k < n1; ++k) v.step(dt1, c.driver.drive(v, lane, desired, dt1), c.ground);
        stats_.lod1Steps += static_cast<std::uint64_t>(n1);
    }
    const auto t2 = Clock::now();
    stats_.lod1Seconds += seconds(t1, t2);

    // ---- Rebalance ----
    auto dist2 = [&](const Vec3& p) { return (p.x - focus.x) * (p.x - focus.x) + (p.y - focus.y) * (p.y - focus.y); };
    for (std::size_t i = 0; i < lod1_.size(); ++i)
        if (lod1_[i].used && dist2(lod1_[i].vehicle->centerOfMassWorld()) > settings_.demoteRadius * settings_.demoteRadius)
            demote(i);
    std::size_t freeSlots = 0;
    for (const auto& c : lod1_) freeSlots += c.used ? 0 : 1;
    if (freeSlots > 0) {
        std::vector<std::pair<Real, std::size_t>> cand;
        const auto& agents = traffic_.agents();
        const Real r2 = settings_.promoteRadius * settings_.promoteRadius;
        for (std::size_t i = 0; i < agents.size(); ++i) {
            if (!agents[i].active) continue;
            const Real d = dist2(agents[i].position);
            if (d < r2) cand.emplace_back(d, i);
        }
        const std::size_t n = std::min(freeSlots, cand.size());
        std::partial_sort(cand.begin(), cand.begin() + static_cast<std::ptrdiff_t>(n), cand.end());
        for (std::size_t k = 0; k < n; ++k) promote(cand[k].second, world);
    }
    stats_.lod1Seconds += seconds(t2, Clock::now());

    stats_.lod1Active = 0;
    for (const auto& c : lod1_) stats_.lod1Active += c.used ? 1 : 0;
    stats_.lod2Active = 0;
    for (const auto& a : traffic_.agents()) stats_.lod2Active += a.active ? 1 : 0;
}

}  // namespace ord
