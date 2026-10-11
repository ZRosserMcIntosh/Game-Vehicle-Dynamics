// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/ground_cache.hpp"

#include <cmath>

namespace ord {

namespace {
constexpr Real kInsideTolerance = 0.02;  // m
Vec3 horizontal(const Vec3& v) { return {v.x, v.y, 0}; }
}  // namespace

void GroundPrefetchCache::sampleStrip(const GroundQuery& source, Strip& s, Real refZ, int nAlong, int nAcross) {
    s.nAcross = std::max(nAcross, 1);
    // Never exceed the per-strip budget (keeps memory bounded and reserved).
    s.nAlong = std::max(2, std::min(nAlong, std::max(2, settings_.maxSamplesPerStrip / s.nAcross)));
    s.dAlong = s.length / (s.nAlong - 1);
    s.dAcross = s.nAcross > 1 ? 2 * s.halfWidth / (s.nAcross - 1) : 0;
    s.samples.resize(static_cast<std::size_t>(s.nAlong * s.nAcross));
    const Vec3 down{0, 0, -1};
    for (int a = 0; a < s.nAcross; ++a) {
        const Real w = s.nAcross > 1 ? -s.halfWidth + a * s.dAcross : 0.0;
        for (int i = 0; i < s.nAlong; ++i) {
            const Vec3 p = s.start + s.along * (i * s.dAlong) + s.across * w;
            const GroundHit h = source.raycast({p.x, p.y, refZ + settings_.rayStartHeight}, down, settings_.rayLength);
            Sample& out = s.samples[static_cast<std::size_t>(a * s.nAlong + i)];
            out.hit = h.hit;
            out.z = h.point.z;
            out.normal = h.normal;
            out.surface = h.surface;
            out.surfaceId = h.surfaceId;
            ++raysLastRebuild_;
        }
    }
}

void GroundPrefetchCache::rebuild(const GroundQuery& source, const GroundStripRequest* requests, std::size_t count) {
    raysLastRebuild_ = 0;
    roughStrips_ = 0;
    if (strips_.size() < count) {
        strips_.resize(count);
        // Reserve the full per-strip budget once, so steady-state rebuilds never allocate.
        for (Strip& st : strips_) st.samples.reserve(static_cast<std::size_t>(std::max(settings_.maxSamplesPerStrip, 2)));
    }
    stripCount_ = count;
    const Settings& cfg = settings_;
    for (std::size_t k = 0; k < count; ++k) {
        const GroundStripRequest& r = requests[k];
        Strip& s = strips_[k];
        const Vec3 vh = horizontal(r.velocity);
        const Real speed = length(vh);
        Vec3 along = speed > 0.5 ? vh / speed : normalized(horizontal(r.heading));
        if (length(along) < 0.5) along = {1, 0, 0};
        s.along = along;
        s.across = cross(Vec3{0, 0, 1}, along);
        const Real ext = std::max(r.halfLength, r.halfWidth) + cfg.margin;
        s.start = r.center - along * ext;
        s.length = speed * cfg.horizon + 2 * ext;
        s.halfWidth = ext;

        // Pass 1: coarse single line (heights + normals).
        const int coarse = std::max(2, static_cast<int>(std::ceil(s.length / cfg.smoothSpacing)) + 1);
        sampleStrip(source, s, r.center.z, coarse, 1);

        bool rough = false;
        for (int i = 0; i < s.nAlong && !rough; ++i) {
            const Sample& c = s.at(0, i);
            if (!c.hit || c.surface.roughness > cfg.roughnessThreshold) rough = true;
            if (i > 0 && i + 1 < s.nAlong) {
                const Real mid = 0.5 * (s.at(0, i - 1).z + s.at(0, i + 1).z);
                if (std::abs(c.z - mid) > cfg.residualThreshold) rough = true;
            }
            if (i > 0 && std::acos(clamp(dot(c.normal, s.at(0, i - 1).normal), -1.0, 1.0)) > cfg.normalThreshold) rough = true;
        }
        s.rough = rough;
        if (rough) {
            ++roughStrips_;
            const int lines = std::max(cfg.roughLines, 2);
            int n = static_cast<int>(std::ceil(s.length / cfg.roughSpacing)) + 1;
            n = std::max(2, std::min(n, cfg.maxSamplesPerStrip / lines));
            sampleStrip(source, s, r.center.z, n, lines);
        }
    }
}

bool GroundPrefetchCache::heightAt(const Strip& s, Real x, Real y, Real& z, Vec3& n, const Sample** nearest,
                                   bool clampToStrip) const {
    const Vec3 rel{x - s.start.x, y - s.start.y, 0};
    const Real u = dot(rel, s.along);
    const Real w = dot(rel, s.across);
    if (!clampToStrip && (u < -kInsideTolerance || u > s.length + kInsideTolerance ||
                          std::abs(w) > s.halfWidth + kInsideTolerance))
        return false;
    const Real uc = clamp(u, 0.0, s.length);
    const Real fi = uc / s.dAlong;
    const int i0 = std::min(static_cast<int>(fi), s.nAlong - 2);
    const Real t = fi - i0;

    if (s.nAcross == 1) {
        const Sample& s0 = s.at(0, i0);
        const Sample& s1 = s.at(0, i0 + 1);
        if (!s0.hit && !s1.hit) return false;
        const Sample& a = s0.hit ? s0 : s1;
        const Sample& b = s1.hit ? s1 : s0;
        const Real zl = lerp(a.z, b.z, t);
        const Vec3 nn = normalized(a.normal * (1 - t) + b.normal * t);
        // Extend off the line using the sampled surface normal (cross-fall, camber).
        const Vec3 off = s.across * w + s.along * (u - uc);
        z = zl - (nn.x * off.x + nn.y * off.y) / std::max(nn.z, 0.2);
        n = nn;
        *nearest = t < 0.5 ? &a : &b;
        return true;
    }

    const Real wc = clamp(w, -s.halfWidth, s.halfWidth);
    const Real fa = (wc + s.halfWidth) / s.dAcross;
    const int a0 = std::min(static_cast<int>(fa), s.nAcross - 2);
    const Real ta = fa - a0;
    const Sample* q[4] = {&s.at(a0, i0), &s.at(a0, i0 + 1), &s.at(a0 + 1, i0), &s.at(a0 + 1, i0 + 1)};
    const Sample* anyHit = nullptr;
    for (const Sample* p : q)
        if (p->hit) { anyHit = p; break; }
    if (!anyHit) return false;
    Real zq[4];
    for (int k = 0; k < 4; ++k) zq[k] = q[k]->hit ? q[k]->z : anyHit->z;
    const Real z0 = lerp(zq[0], zq[1], t), z1 = lerp(zq[2], zq[3], t);
    const Real dzdu = lerp(zq[1] - zq[0], zq[3] - zq[2], ta) / s.dAlong;
    const Real dzdw = (z1 - z0) / s.dAcross;
    z = lerp(z0, z1, ta) + dzdu * (u - uc) + dzdw * (w - wc);
    n = normalized(Vec3{0, 0, 1} - s.along * dzdu - s.across * dzdw);
    const int k = (t < 0.5 ? 0 : 1) + (ta < 0.5 ? 0 : 2);
    *nearest = q[k]->hit ? q[k] : anyHit;
    return true;
}

GroundHit GroundPrefetchCache::raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const {
    GroundHit h;
    if (dir.z > -1e-6 || stripCount_ == 0) {
        if (fallback_) return fallback_->raycast(origin, dir, maxDistance);
        return h;
    }
    Real z = 0;
    Vec3 n;
    const Sample* nearest = nullptr;
    const Strip* strip = nullptr;
    for (std::size_t k = 0; k < stripCount_; ++k) {
        if (heightAt(strips_[k], origin.x, origin.y, z, n, &nearest, false)) {
            strip = &strips_[k];
            break;
        }
    }
    if (!strip) {
        misses_.fetch_add(1, std::memory_order_relaxed);
        if (fallback_) return fallback_->raycast(origin, dir, maxDistance);
        // Extrapolate from the closest strip: never let a wheel fall through
        // the world because a query landed outside the prefetched area.
        Real best = 1e300;
        for (std::size_t k = 0; k < stripCount_; ++k) {
            const Vec3 c = strips_[k].start + strips_[k].along * (0.5 * strips_[k].length);
            const Real d = (c.x - origin.x) * (c.x - origin.x) + (c.y - origin.y) * (c.y - origin.y);
            if (d < best) { best = d; strip = &strips_[k]; }
        }
        if (!heightAt(*strip, origin.x, origin.y, z, n, &nearest, true)) return h;
    }
    // Mostly-downward ray against the local height field: fixed-point on t.
    Real t = (z - origin.z) / dir.z;
    for (int it = 0; it < 3; ++it) {
        const Vec3 p = origin + dir * t;
        if (!heightAt(*strip, p.x, p.y, z, n, &nearest, true)) break;
        t = (z - origin.z) / dir.z;
    }
    if (t < 0 || t > maxDistance) return h;
    h.hit = true;
    h.distance = t;
    h.point = origin + dir * t;
    h.normal = n;
    h.surface = nearest->surface;
    h.surfaceId = nearest->surfaceId;
    return h;
}

}  // namespace ord
