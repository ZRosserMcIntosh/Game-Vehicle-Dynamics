// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/traffic.hpp"

#include <algorithm>

#include "ord/vehicle.hpp"

namespace ord {

// ---------------- LanePath ----------------

LanePath::LanePath(std::vector<Vec3> points, bool loop, Real speedLimit)
    : pts_(std::move(points)), loop_(loop), speedLimit_(speedLimit) {
    cum_.assign(1, 0.0);
    const std::size_t n = pts_.size();
    const std::size_t segs = loop_ ? n : (n ? n - 1 : 0);
    for (std::size_t i = 0; i < segs; ++i) cum_.push_back(cum_.back() + ord::length(pts_[(i + 1) % n] - pts_[i]));
    const std::size_t nc = static_cast<std::size_t>(length() / kCurvStep) + 1;
    absCurv_.resize(nc);
    for (std::size_t i = 0; i < nc; ++i) absCurv_[i] = std::abs(curvatureAt(static_cast<Real>(i) * kCurvStep));
}

Real LanePath::wrap(Real s) const {
    const Real L = length();
    if (L <= 0) return 0;
    if (loop_) {
        s = std::fmod(s, L);
        return s < 0 ? s + L : s;
    }
    return clamp(s, 0.0, L);
}

std::size_t LanePath::segmentAt(Real s, Real& t) const {
    s = wrap(s);
    const auto it = std::upper_bound(cum_.begin(), cum_.end(), s);
    std::size_t i = it == cum_.begin() ? 0 : static_cast<std::size_t>(it - cum_.begin()) - 1;
    i = std::min(i, cum_.size() - 2);
    const Real segLen = cum_[i + 1] - cum_[i];
    t = segLen > 0 ? (s - cum_[i]) / segLen : 0.0;
    return i;
}

Vec3 LanePath::pointAt(Real s) const {
    if (pts_.size() < 2) return pts_.empty() ? Vec3{} : pts_[0];
    Real t;
    const std::size_t i = segmentAt(s, t);
    return pts_[i] + (pts_[(i + 1) % pts_.size()] - pts_[i]) * t;
}

Vec3 LanePath::tangentAt(Real s) const {
    if (pts_.size() < 2) return {1, 0, 0};
    Real t;
    const std::size_t i = segmentAt(s, t);
    const Vec3 d = pts_[(i + 1) % pts_.size()] - pts_[i];
    return normalized(Vec3{d.x, d.y, 0});
}

Real LanePath::curvatureAt(Real s) const {
    // Menger curvature of three points 2 m apart: exact on circular arcs and
    // insensitive to how finely the polyline is segmented.
    constexpr Real h = 2.0;
    const Vec3 a = pointAt(s - h), b = pointAt(s), c = pointAt(s + h);
    const Real abx = b.x - a.x, aby = b.y - a.y, bcx = c.x - b.x, bcy = c.y - b.y;
    const Real cr = abx * bcy - aby * bcx;
    const Real denom = std::hypot(abx, aby) * std::hypot(bcx, bcy) * std::hypot(c.x - a.x, c.y - a.y);
    return denom > 1e-12 ? 2.0 * cr / denom : 0.0;
}

Real LanePath::maxCurvature(Real s0, Real s1) const {
    if (absCurv_.empty()) return 0;
    Real k = 0;
    const std::size_t n = absCurv_.size();
    for (Real s = s0; s <= s1; s += kCurvStep) {
        const Real w = wrap(s);
        k = std::max(k, absCurv_[std::min(n - 1, static_cast<std::size_t>(w / kCurvStep))]);
    }
    return k;
}

Real LanePath::project(const Vec3& p, Real hint) const {
    const std::size_t n = pts_.size();
    if (n < 2) return 0;
    const std::size_t segs = cum_.size() - 1;
    std::size_t first = 0, count = segs;
    if (hint >= 0) {
        Real t;
        const std::size_t h = segmentAt(hint, t);
        // Search a window of segments around the hint (about +-30 m).
        Real avg = length() / static_cast<Real>(segs);
        const std::size_t win = std::min<std::size_t>(segs, static_cast<std::size_t>(30.0 / std::max(avg, 0.1)) + 2);
        first = (h + segs - win) % segs;
        count = std::min(segs, 2 * win + 1);
    }
    Real best = 1e300, bestS = 0;
    for (std::size_t k = 0; k < count; ++k) {
        const std::size_t i = (first + k) % segs;
        const Vec3 a = pts_[i], b = pts_[(i + 1) % n];
        const Vec3 ab{b.x - a.x, b.y - a.y, 0};
        const Real l2 = dot(ab, ab);
        const Real t = l2 > 0 ? clamp(dot(Vec3{p.x - a.x, p.y - a.y, 0}, ab) / l2, 0.0, 1.0) : 0.0;
        const Real dx = a.x + ab.x * t - p.x, dy = a.y + ab.y * t - p.y;
        const Real d = dx * dx + dy * dy;
        if (d < best) { best = d; bestS = cum_[i] + t * (cum_[i + 1] - cum_[i]); }
    }
    return bestS;
}

LanePath LanePath::circle(const Vec3& c, Real radius, Real speedLimit, int segments) {
    std::vector<Vec3> pts;
    for (int i = 0; i < segments; ++i) {
        const Real a = 2 * kPi * i / segments - kPi / 2;  // starts south of the centre heading +X
        pts.push_back({c.x + radius * std::cos(a), c.y + radius * std::sin(a), c.z});
    }
    return LanePath(std::move(pts), true, speedLimit);
}

// ---------------- IDM ----------------

Real idmAcceleration(const IdmParams& p, Real v, Real v0, Real gap, Real vl) {
    v0 = std::max(v0, 0.1);
    Real a = p.maxAccel * (1.0 - std::pow(std::max(v, 0.0) / v0, 4));
    if (gap > 0) {
        const Real sStar = p.minGap + std::max(0.0, v * p.desiredTimeGap + v * (v - vl) / (2 * std::sqrt(p.maxAccel * p.comfortDecel)));
        a -= p.maxAccel * (sStar / gap) * (sStar / gap);
    }
    return clamp(a, -p.maxDecel, p.maxAccel);
}

// ---------------- TrafficSystem ----------------

int TrafficSystem::addAgent(int lane, Real s, Real speed, Real desiredSpeed, int model) {
    Agent a;
    a.lane = lane;
    a.s = lanes_[static_cast<std::size_t>(lane)].wrap(s);
    a.speed = speed;
    a.desiredSpeed = desiredSpeed;
    a.model = model;
    a.groundTimer = static_cast<int>(agents_.size()) % std::max(1, settings_.groundQueryInterval);  // stagger rays
    const LanePath& l = lanes_[static_cast<std::size_t>(lane)];
    a.position = l.pointAt(a.s);
    const Vec3 t = l.tangentAt(a.s);
    a.yaw = std::atan2(t.y, t.x);
    agents_.push_back(a);
    return static_cast<int>(agents_.size()) - 1;
}

void TrafficSystem::refreshOrder() {
    // Reuses the per-lane buffers: no allocation once warmed up.
    order_.resize(lanes_.size());
    for (auto& l : order_) l.clear();
    rank_.resize(agents_.size());
    for (std::size_t i = 0; i < agents_.size(); ++i)
        if (agents_[i].active) order_[static_cast<std::size_t>(agents_[i].lane)].push_back(static_cast<int>(i));
    for (std::size_t o = 0; o < obstacles_.size(); ++o)
        order_[static_cast<std::size_t>(obstacles_[o].lane)].push_back(-static_cast<int>(o) - 1);
    auto sOf = [&](int id) { return id >= 0 ? agents_[static_cast<std::size_t>(id)].s : obstacles_[static_cast<std::size_t>(-id - 1)].s; };
    for (auto& lane : order_) {
        std::sort(lane.begin(), lane.end(), [&](int a, int b) { return sOf(a) < sOf(b); });
        for (std::size_t k = 0; k < lane.size(); ++k)
            if (lane[k] >= 0) rank_[static_cast<std::size_t>(lane[k])] = static_cast<int>(k);
    }
}

void TrafficSystem::leaderOf(std::size_t agent, Real& gap, Real& leaderSpeed) const {
    gap = -1;
    leaderSpeed = 0;
    const Agent& me = agents_[agent];
    const auto& lane = order_[static_cast<std::size_t>(me.lane)];
    const LanePath& path = lanes_[static_cast<std::size_t>(me.lane)];
    if (lane.size() < 2 || agent >= rank_.size()) return;
    const auto it = lane.begin() + rank_[agent];
    if (*it != static_cast<int>(agent)) return;
    auto next = it + 1;
    if (next == lane.end()) {
        if (!path.loop()) return;
        next = lane.begin();
    }
    const int id = *next;
    const Real ls = id >= 0 ? agents_[static_cast<std::size_t>(id)].s : obstacles_[static_cast<std::size_t>(-id - 1)].s;
    leaderSpeed = id >= 0 ? agents_[static_cast<std::size_t>(id)].speed : obstacles_[static_cast<std::size_t>(-id - 1)].speed;
    Real d = ls - me.s;
    if (d < 0) d += path.length();
    gap = std::max(0.1, d - settings_.idm.vehicleLength);
}

void TrafficSystem::step(Real dt, const GroundQuery* ground) {
    refreshOrder();
    const IdmParams& p = settings_.idm;
    // Accelerations first (all agents see the same state), then integrate.
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        Agent& a = agents_[i];
        if (!a.active) continue;
        const LanePath& lane = lanes_[static_cast<std::size_t>(a.lane)];
        Real gap, vl;
        leaderOf(i, gap, vl);
        const Real look = std::max(20.0, 3.0 * a.speed);
        const Real kAhead = lane.maxCurvature(a.s, a.s + look);
        Real v0 = std::min(a.desiredSpeed, lane.speedLimit());
        if (kAhead > 1e-5) v0 = std::min(v0, std::sqrt(p.maxLateralAccel / kAhead));
        Real acc = idmAcceleration(p, a.speed, v0, gap, vl);
        // Friction circle: longitudinal capability left after cornering.
        const Real aLat = a.speed * a.speed * std::abs(lane.curvatureAt(a.s));
        const Real aMax = p.friction * kGravity;
        const Real aLong = std::sqrt(std::max(0.0, aMax * aMax - aLat * aLat));
        // The friction circle limits *added* acceleration; braking toward a safe
        // speed is always allowed (a kinematic agent must never be stuck too fast).
        a.accel = clamp(acc, -p.maxDecel, std::min(aLong, p.maxAccel));
    }
    for (Agent& a : agents_) {
        if (!a.active) continue;
        const LanePath& lane = lanes_[static_cast<std::size_t>(a.lane)];
        a.speed = std::max(0.0, a.speed + a.accel * dt);
        Real s = a.s + a.speed * dt;
        if (!lane.loop() && s >= lane.length()) { s = lane.length(); a.speed = 0; }
        a.s = lane.wrap(s);
        const Vec3 t = lane.tangentAt(a.s);
        const Real k = lane.curvatureAt(a.s);
        a.yaw = std::atan2(t.y, t.x);
        a.yawRate = a.speed * k;
        Vec3 pos = lane.pointAt(a.s);
        if (ground && --a.groundTimer <= 0) {
            const GroundHit h = ground->raycast({pos.x, pos.y, pos.z + 5.0}, {0, 0, -1}, 10.0);
            ++groundQueries_;
            if (h.hit) a.groundOffset = h.point.z - pos.z;
            a.groundTimer = settings_.groundQueryInterval;
        }
        pos.z += a.groundOffset;
        a.position = pos;
        a.pitch = -settings_.pitchPerAccel * a.accel;
        a.roll = settings_.rollPerAccel * a.speed * a.speed * k;
    }
}

// ---------------- LaneFollowDriver ----------------

DriverInput LaneFollowDriver::drive(const Vehicle& v, const LanePath& lane, Real desiredSpeed, Real dt) {
    DriverInput in;
    const Vec3 com = v.centerOfMassWorld();
    s_ = lane.project(com, s_);
    const Vec3 pl = lane.pointAt(s_);
    const Vec3 t = lane.tangentAt(s_);
    lateralError_ = (com.x - pl.x) * -t.y + (com.y - pl.y) * t.x;  // + = left of the lane

    const Real speed = v.forwardSpeed();
    const Real ld = std::max(6.0, 0.9 * std::abs(speed));
    const Vec3 tgt = lane.pointAt(s_ + ld);
    const Vec3 toT = v.state().orientation.inverseRotate(tgt - com);
    const Real eta = std::atan2(toT.y, toT.x);
    const Real L = v.definition().wheelbase();
    Real maxSteer = 0;
    for (const auto& w : v.definition().wheels) maxSteer = std::max(maxSteer, w.maxSteerAngle);
    const Real delta = std::atan(2.0 * L * std::sin(eta) / ld);
    in.steer = maxSteer > 0 ? clamp(delta / maxSteer, -1.0, 1.0) : 0.0;

    const Real err = desiredSpeed - speed;
    speedIntegral_ = clamp(speedIntegral_ + err * dt, -20.0, 20.0);
    in.throttle = clamp(0.12 + 0.15 * err + 0.03 * speedIntegral_, 0.0, 1.0);
    in.brake = err < -1.0 ? clamp(-0.12 * (err + 1.0), 0.0, 1.0) : 0.0;
    if (in.brake > 0) in.throttle = 0;
    return in;
}

}  // namespace ord
