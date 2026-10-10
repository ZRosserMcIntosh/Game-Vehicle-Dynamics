// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/ground.hpp"

namespace ord {

GroundHit FlatGround::raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const {
    GroundHit h;
    if (dir.z >= -1e-9) return h;
    const Real t = (height_ - origin.z) / dir.z;
    if (t < 0 || t > maxDistance) return h;
    h.hit = true;
    h.distance = t;
    h.point = origin + dir * t;
    h.normal = {0, 0, 1};
    h.surface = surface_;
    return h;
}

GroundHit TestTrackGround::raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const {
    GroundHit h;
    if (dir.z >= -1e-6) return h;
    // Fixed-point iteration on t for a mostly-downward ray against z = h(x, y).
    Real t = (heightAt(origin.x, origin.y) - origin.z) / dir.z;
    for (int i = 0; i < 6; ++i) {
        const Vec3 p = origin + dir * t;
        t = (heightAt(p.x, p.y) - origin.z) / dir.z;
    }
    if (t < 0 || t > maxDistance) return h;
    h.hit = true;
    h.distance = t;
    h.point = origin + dir * t;
    constexpr Real e = 0.01;
    const Real dhdx = (heightAt(h.point.x + e, h.point.y) - heightAt(h.point.x - e, h.point.y)) / (2 * e);
    const Real dhdy = (heightAt(h.point.x, h.point.y + e) - heightAt(h.point.x, h.point.y - e)) / (2 * e);
    h.normal = normalized(Vec3{-dhdx, -dhdy, 1});
    h.surface = base_;
    h.surfaceId = 0;
    for (const Patch& p : patches_) {
        if (h.point.x >= p.xMin && h.point.x <= p.xMax && h.point.y >= p.yMin && h.point.y <= p.yMax) {
            h.surface = p.surface;
            h.surfaceId = p.surfaceId;
        }
    }
    return h;
}

}  // namespace ord
