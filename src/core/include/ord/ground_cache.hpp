// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Ground query budget tools (docs/decisions/ground-prefetch.md).
//
// GroundPrefetchCache: once per *render frame*, the host casts a small,
// adaptive set of rays along each wheel's predicted path ("strips") into its
// world. Every physics substep and every contact sample then reads the strips
// with no engine scene query at all. This
//   * cuts engine raycasts (one batch per frame instead of
//     samples x wheels x substeps),
//   * moves all scene queries to the thread that owns the world (game thread),
//     so the physics thread only reads immutable data,
//   * makes physics independent of streaming hitches: the strip is the
//     contract.
// Smooth surfaces get a coarse single line (heights + normals). Rough
// surfaces (SurfaceProperties::roughness, or detected height residuals) get a
// dense multi-line grid, so curbs, potholes and cobblestones are resolved where
// they exist.
#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

#include "ord/ground.hpp"

namespace ord {

struct GroundStripRequest {
    Vec3 center;          // current contact-patch centre, world
    Vec3 velocity;        // contact-point velocity, world
    Vec3 heading;         // wheel heading, world (used when nearly stationary)
    Real halfLength = 0.09;
    Real halfWidth = 0.10;
};

// Counts queries forwarded to an inner GroundQuery (benchmarks / budgets).
class CountingGroundQuery final : public GroundQuery {
public:
    explicit CountingGroundQuery(const GroundQuery& inner) : inner_(inner) {}
    GroundHit raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const override {
        count_.fetch_add(1, std::memory_order_relaxed);
        return inner_.raycast(origin, dir, maxDistance);
    }
    std::uint64_t count() const { return count_.load(std::memory_order_relaxed); }
    void resetCount() { count_.store(0, std::memory_order_relaxed); }

private:
    const GroundQuery& inner_;
    mutable std::atomic<std::uint64_t> count_{0};
};

class GroundPrefetchCache final : public GroundQuery {
public:
    struct Settings {
        Real horizon = 1.5 / 60.0;       // s of predicted travel covered (1.5 frames at 60 fps)
        Real margin = 0.15;              // m added around the footprint
        Real smoothSpacing = 0.35;       // m between samples on smooth surfaces (single line)
        Real roughSpacing = 0.06;        // m between samples on rough surfaces
        int roughLines = 3;              // lateral lines on rough surfaces
        Real roughnessThreshold = 0.05;  // SurfaceProperties::roughness that forces dense sampling
        Real residualThreshold = 0.004;  // m: coarse-line height residual that forces dense sampling
        Real normalThreshold = 0.03;     // rad: normal change between coarse samples that forces dense
        int maxSamplesPerStrip = 120;
        Real rayStartHeight = 1.0;       // m above the request centre
        Real rayLength = 2.5;            // m
    };

    GroundPrefetchCache() = default;
    explicit GroundPrefetchCache(const Settings& s) : settings_(s) {}
    GroundPrefetchCache(GroundPrefetchCache&& o) noexcept { *this = std::move(o); }
    GroundPrefetchCache& operator=(GroundPrefetchCache&& o) noexcept {
        settings_ = o.settings_;
        strips_ = std::move(o.strips_);
        stripCount_ = o.stripCount_;
        raysLastRebuild_ = o.raysLastRebuild_;
        roughStrips_ = o.roughStrips_;
        fallback_ = o.fallback_;
        misses_.store(o.misses_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        return *this;
    }

    // Rebuilds all strips from `source`. Call on the thread that owns the world.
    // Reuses memory: no allocation once warmed up for a given request count.
    void rebuild(const GroundQuery& source, const GroundStripRequest* requests, std::size_t count);

    // When set, queries outside every strip are forwarded here (headless use).
    // Leave null in engines where the physics thread must not touch the world:
    // misses are then answered by extrapolating the nearest strip.
    void setFallback(const GroundQuery* fallback) { fallback_ = fallback; }

    GroundHit raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const override;

    // Statistics.
    std::size_t raysLastRebuild() const { return raysLastRebuild_; }
    std::size_t roughStripsLastRebuild() const { return roughStrips_; }
    std::uint64_t misses() const { return misses_.load(std::memory_order_relaxed); }
    const Settings& settings() const { return settings_; }

private:
    struct Sample {
        bool hit = false;
        Real z = 0;
        Vec3 normal{0, 0, 1};
        SurfaceProperties surface;
        int surfaceId = 0;
    };
    struct Strip {
        Vec3 start;     // world point at (u = 0, w = 0); z unused
        Vec3 along;     // unit, horizontal
        Vec3 across;    // unit, horizontal, = up x along
        Real length = 0;
        Real halfWidth = 0;
        int nAlong = 0, nAcross = 1;
        Real dAlong = 1, dAcross = 1;
        bool rough = false;
        std::vector<Sample> samples;  // row-major [across][along]
        const Sample& at(int a, int i) const { return samples[static_cast<std::size_t>(a * nAlong + i)]; }
    };

    void sampleStrip(const GroundQuery& source, Strip& s, Real refZ, int nAlong, int nAcross);
    bool heightAt(const Strip& s, Real x, Real y, Real& z, Vec3& n, const Sample** nearest, bool clampToStrip) const;

    Settings settings_;
    std::vector<Strip> strips_;
    std::size_t stripCount_ = 0;
    std::size_t raysLastRebuild_ = 0;
    std::size_t roughStrips_ = 0;
    const GroundQuery* fallback_ = nullptr;
    mutable std::atomic<std::uint64_t> misses_{0};
};

}  // namespace ord
