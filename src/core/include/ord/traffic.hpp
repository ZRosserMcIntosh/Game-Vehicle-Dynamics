// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Lanes, lightweight traffic (LOD2) and an AI lane-following driver (for LOD1
// and test drivers). See docs/architecture/open-world-and-lod.md.
//
// LOD2 agents are *kinematic*: they move along a lane polyline with
// longitudinal dynamics from the Intelligent Driver Model (IDM; M. Treiber,
// A. Hennecke, D. Helbing, "Congested traffic states in empirical observations
// and microscopic simulations", Phys. Rev. E 62, 2000), speed limited by lane
// curvature and a friction circle. They cost well under a microsecond each and
// query the ground only every few steps.
#pragma once

#include <cstdint>
#include <vector>

#include "ord/ground.hpp"
#include "ord/inputs.hpp"
#include "ord/math.hpp"

namespace ord {

class Vehicle;

class LanePath {
public:
    LanePath() = default;
    LanePath(std::vector<Vec3> points, bool loop, Real speedLimit);

    Real length() const { return cum_.empty() ? 0 : cum_.back(); }
    bool loop() const { return loop_; }
    Real speedLimit() const { return speedLimit_; }
    Real wrap(Real s) const;
    Vec3 pointAt(Real s) const;
    Vec3 tangentAt(Real s) const;          // unit, horizontal
    Real curvatureAt(Real s) const;        // 1/m, + = turning left
    Real maxCurvature(Real s0, Real s1) const;
    // Arc length of the closest point to `p`, searching near `hint` (pass a
    // previous result; negative = full search).
    Real project(const Vec3& p, Real hint = -1) const;

    static LanePath circle(const Vec3& center, Real radius, Real speedLimit, int segments = 256);

private:
    std::size_t segmentAt(Real s, Real& t) const;
    std::vector<Vec3> pts_;
    std::vector<Real> cum_;
    std::vector<Real> absCurv_;   // |curvature| sampled every kCurvStep metres (O(1) lookahead queries)
    static constexpr Real kCurvStep = 2.0;
    bool loop_ = false;
    Real speedLimit_ = 30;
};

struct IdmParams {
    Real desiredTimeGap = 1.4;     // s
    Real minGap = 2.5;             // m, bumper to bumper at standstill
    Real maxAccel = 1.6;           // m/s^2
    Real comfortDecel = 2.5;       // m/s^2
    Real maxDecel = 8.0;           // m/s^2, emergency cap
    Real vehicleLength = 4.6;      // m
    Real maxLateralAccel = 3.5;    // m/s^2, comfortable cornering for traffic
    Real friction = 0.9;           // friction-circle limit (x g)
};

// IDM acceleration (free road when gap <= 0).
Real idmAcceleration(const IdmParams& p, Real v, Real v0, Real gap, Real leaderSpeed);

class TrafficSystem {
public:
    struct Agent {
        bool active = true;            // false while promoted to a higher LOD
        int lane = 0;
        int model = 0;                 // index into the host's vehicle models
        Real s = 0;                    // arc length along the lane
        Real speed = 0;                // m/s
        Real desiredSpeed = 25;        // m/s
        Real accel = 0;                // m/s^2 (last step)
        // Derived, for rendering / promotion
        Vec3 position;
        Real yaw = 0, yawRate = 0;
        Real pitch = 0, roll = 0;      // cosmetic body motion
        Real groundOffset = 0;         // ground z minus lane z, refreshed every few steps
        int groundTimer = 0;
    };

    struct Settings {
        IdmParams idm;
        int groundQueryInterval = 8;   // steps between ground rays per agent
        Real pitchPerAccel = 0.006;    // rad per m/s^2 (cosmetic)
        Real rollPerAccel = 0.008;     // rad per m/s^2 (cosmetic)
    };

    TrafficSystem() = default;
    TrafficSystem(std::vector<LanePath> lanes, const Settings& s) : lanes_(std::move(lanes)), settings_(s) {}

    int addAgent(int lane, Real s, Real speed, Real desiredSpeed, int model = 0);
    void step(Real dt, const GroundQuery* ground);

    std::vector<Agent>& agents() { return agents_; }
    const std::vector<Agent>& agents() const { return agents_; }
    const std::vector<LanePath>& lanes() const { return lanes_; }
    const Settings& settings() const { return settings_; }
    std::uint64_t groundQueries() const { return groundQueries_; }

    // Leader of `agent` on its lane (active agents and any extra obstacles
    // registered with setObstacle), returns gap (m, <= 0 if none) and speed.
    void leaderOf(std::size_t agent, Real& gap, Real& leaderSpeed) const;
    // Higher-LOD vehicles occupying lanes, so LOD2 traffic does not drive
    // through them. Cleared by the caller each frame.
    void clearObstacles() { obstacles_.clear(); }
    void addObstacle(int lane, Real s, Real speed) { obstacles_.push_back({lane, s, speed}); }

private:
    void refreshOrder();
    struct Obstacle {
        int lane;
        Real s, speed;
    };
    std::vector<LanePath> lanes_;
    Settings settings_;
    std::vector<Agent> agents_;
    std::vector<Obstacle> obstacles_;
    std::vector<std::vector<int>> order_;  // per lane: indices sorted by s (agents >= 0, obstacles ~idx)
    std::vector<int> rank_;                // agent -> position in its lane's order_
    std::uint64_t groundQueries_ = 0;
};

// Pure-pursuit steering + speed control on a lane: drives a physical Vehicle
// (LOD1 AI, LOD0 autopilot in tests).
class LaneFollowDriver {
public:
    DriverInput drive(const Vehicle& v, const LanePath& lane, Real desiredSpeed, Real dt);
    Real laneS() const { return s_; }
    Real lateralError() const { return lateralError_; }
    void reset(Real s) { s_ = s; speedIntegral_ = 0; }

private:
    Real s_ = -1;
    Real speedIntegral_ = 0;
    Real lateralError_ = 0;
};

}  // namespace ord
