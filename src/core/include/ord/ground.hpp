// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Ground query interface. The core never talks to a physics engine or world
// directly: the host (Unreal adapter, test harness, headless server) implements
// GroundQuery. This keeps the core independent of world streaming and lets the
// same vehicle run against analytic test tracks in unit tests.
#pragma once

#include <functional>
#include <vector>

#include "ord/math.hpp"
#include "ord/tire.hpp"

namespace ord {

struct GroundHit {
    bool hit = false;
    Real distance = 0;   // along the ray
    Vec3 point;          // world
    Vec3 normal{0, 0, 1};
    SurfaceProperties surface;
    int surfaceId = 0;
};

class GroundQuery {
public:
    virtual ~GroundQuery() = default;
    // `dir` is unit length. Must be thread-safe for concurrent const calls.
    virtual GroundHit raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const = 0;
};

// Infinite plane z = height with a single surface.
class FlatGround final : public GroundQuery {
public:
    explicit FlatGround(Real height = 0, SurfaceProperties s = {}) : height_(height), surface_(s) {}
    GroundHit raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const override;

private:
    Real height_;
    SurfaceProperties surface_;
};

// Analytic test track: height field z = h(x, y) plus axis-aligned surface
// patches (e.g. a wet braking strip). Used by tests and the benchmark tool.
class TestTrackGround final : public GroundQuery {
public:
    struct Patch {
        Real xMin, xMax, yMin, yMax;
        SurfaceProperties surface;
        int surfaceId;
    };
    using HeightFn = std::function<Real(Real x, Real y)>;

    explicit TestTrackGround(HeightFn h = nullptr, SurfaceProperties base = {}) : height_(std::move(h)), base_(base) {}
    void addPatch(const Patch& p) { patches_.push_back(p); }
    Real heightAt(Real x, Real y) const { return height_ ? height_(x, y) : 0.0; }
    GroundHit raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const override;

private:
    HeightFn height_;
    SurfaceProperties base_;
    std::vector<Patch> patches_;
};

}  // namespace ord
