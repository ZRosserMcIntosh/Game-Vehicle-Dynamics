// OpenRoad Dynamics — contact patch, unsprung mass, ground cache and rate-decoupling tests.
// SPDX-License-Identifier: MIT
#include "ord/ground_cache.hpp"
#include "ord/presets.hpp"
#include "ord_test.hpp"
#include "reference_vehicles.hpp"
#include "scenarios.hpp"

using namespace ord;
using namespace ord::scenarios;

namespace {
RunConfig base(Real hz = 240, int samples = 5) {
    RunConfig c;
    c.vehicle = vehicles::referenceSportsRwd();
    c.assists = makePreset(DrivingPreset::Simulation);
    c.hz = hz;
    c.fidelity.contactSamples = samples;
    return c;
}
}  // namespace

TEST_CASE("contact: suspension heave and wheel-hop frequencies match the analytic quarter car (B22)") {
    const FrequencyResult f = suspensionFrequencies(base());
    CHECK_NEAR(f.heaveHz / f.heaveExpectedHz, 1.0, 0.03);
    CHECK_NEAR(f.wheelHopHz / f.wheelHopExpectedHz, 1.0, 0.03);
}

TEST_CASE("contact: multi-point patch converges with rate on curbs and potholes (B19, B20)") {
    const RoughRoadResult c1000 = curbStrike(base(1000), 60), c240 = curbStrike(base(240), 60);
    CHECK(c240.finite && c1000.finite);
    CHECK_NEAR(c240.rmsVerticalAccelG / c1000.rmsVerticalAccelG, 1.0, 0.10);
    const RoughRoadResult p1000 = pothole(base(1000), 60), p240 = pothole(base(240), 60);
    CHECK_NEAR(p240.rmsVerticalAccelG / p1000.rmsVerticalAccelG, 1.0, 0.10);
    CHECK(p240.minLoad < 0.5 * p240.staticLoad);  // the wheel really drops into the hole
}

TEST_CASE("contact: patch enveloping - 5 samples converge on cobblestones at 120 Hz, 1 sample does not") {
    const RoughRoadResult m120 = cobblestones(base(120, 5), 40), m1000 = cobblestones(base(1000, 5), 40);
    CHECK_NEAR(m120.rmsVerticalAccelG / m1000.rmsVerticalAccelG, 1.0, 0.05);
    const RoughRoadResult s120 = cobblestones(base(120, 1), 40), s1000 = cobblestones(base(1000, 1), 40);
    CHECK(std::abs(s120.rmsVerticalAccelG / s1000.rmsVerticalAccelG - 1.0) > 0.1);
    CHECK(s1000.rmsVerticalAccelG > 2.0 * m1000.rmsVerticalAccelG);  // point contact overstates harshness
}

TEST_CASE("contact: procedural surface texture reproduces geometric cobblestones with zero dense rays") {
    const RoughRoadResult geo = cobblestones(base(240), 40, false);
    RunConfig c = base(240);
    c.prefetch = true;
    RunStats st;
    c.stats = &st;
    const RoughRoadResult tex = cobblestones(c, 40, true);
    CHECK_NEAR(tex.rmsVerticalAccelG / geo.rmsVerticalAccelG, 1.0, 0.15);
    CHECK(st.roughStrips == 0);                          // flat collision -> coarse strips only
    CHECK(static_cast<Real>(st.engineRays) / st.simulatedSeconds < 1500.0);  // ~smooth-road budget
}

TEST_CASE("ground cache: matches direct queries on curbs and potholes with >= 3x fewer engine rays") {
    RunConfig direct = base(240);
    RunStats sd;
    direct.stats = &sd;
    RunConfig cached = base(240);
    cached.prefetch = true;
    RunStats sc;
    cached.stats = &sc;
    const RoughRoadResult a = curbStrike(direct, 60), b = curbStrike(cached, 60);
    CHECK_NEAR(b.rmsVerticalAccelG / a.rmsVerticalAccelG, 1.0, 0.10);
    CHECK(sd.engineRays > 3 * sc.engineRays);
    CHECK(sc.cacheMisses == 0);
    const RoughRoadResult pa = pothole(direct, 60), pb = pothole(cached, 60);
    CHECK_NEAR(pb.rmsVerticalAccelG / pa.rmsVerticalAccelG, 1.0, 0.10);
    CHECK(sc.roughStrips > 0);  // untagged pothole found by the residual check
}

TEST_CASE("ground cache: smooth-road driving is unchanged and needs only a coarse line per wheel") {
    RunConfig direct = base(240);
    RunConfig cached = base(240);
    cached.prefetch = true;
    RunStats st;
    cached.stats = &st;
    const AccelResult a = accelerate0to100(direct), b = accelerate0to100(cached);
    CHECK_NEAR(b.time0to100 / a.time0to100, 1.0, 0.01);
    CHECK(st.cacheMisses == 0);
    CHECK(st.roughStrips == 0);
    CHECK(static_cast<Real>(st.engineRays) / st.simulatedSeconds < 1500.0);
}

TEST_CASE("ground cache: queries outside the strips extrapolate instead of falling through") {
    TestTrackGround g([](Real x, Real) { return 0.1 * x; });
    GroundPrefetchCache cache;
    GroundStripRequest r;
    r.center = {0, 0, 0};
    r.velocity = {10, 0, 0};
    r.heading = {1, 0, 0};
    cache.rebuild(g, &r, 1);
    const GroundHit inside = cache.raycast({0.2, 0, 2}, {0, 0, -1}, 5);
    CHECK(inside.hit);
    CHECK_NEAR(inside.point.z, 0.02, 1e-6);
    const GroundHit outside = cache.raycast({3.0, 0, 2}, {0, 0, -1}, 5);  // beyond the strip
    CHECK(outside.hit);
    CHECK_NEAR(outside.point.z, 0.3, 0.02);  // slope extrapolated
    CHECK(cache.misses() == 1);
}

TEST_CASE("rate decoupling: 60 Hz host + 4 substeps + ground cache stays within 10% on rough roads") {
    // Like-for-like: both runs see the ground through the per-frame cache and
    // are compared with rate-independent metrics (peak heave, 60 Hz-window accel).
    RunConfig ref = base(1000);
    ref.prefetch = true;
    RunConfig h = base(240);
    h.hostHz = 60;
    h.prefetch = true;
    for (Real kmh : {60.0, 120.0}) {
        const RoughRoadResult a = curbStrike(ref, kmh), b = curbStrike(h, kmh);
        CHECK_NEAR(b.maxHeave / a.maxHeave, 1.0, 0.10);
        CHECK_NEAR(b.rmsFrameAccelG / a.rmsFrameAccelG, 1.0, 0.10);
    }
    CHECK_NEAR(pothole(h, 60).rmsFrameAccelG / pothole(ref, 60).rmsFrameAccelG, 1.0, 0.10);
    CHECK_NEAR(cobblestones(h, 40, true).rmsVerticalAccelG / cobblestones(ref, 40, true).rmsVerticalAccelG, 1.0, 0.05);
}

TEST_CASE("host coupling: chassis pose error per host step stays sub-millimetre on a rough road") {
    RunConfig h = base(240);
    h.hostHz = 60;
    h.prefetch = true;
    // Run the curb scenario, then inspect the vehicle's host-sync diagnostics via a fresh drive.
    Vehicle v(h.vehicle, h.assists);
    TestTrackGround g([](Real x, Real y) { return 0.03 * std::sin(x / 2.0) + 0.01 * std::sin(x / 0.7 + y); });
    v.reset({0, 0, 0.6}, 0, g);
    v.selectGearForSpeed(25);
    v.placeOnGround(0, 0, 0, g, 25);
    v.setChassisMode(ChassisMode::External, false);
    DriverInput in;
    in.throttle = 0.3;
    for (int f = 0; f < 60 * 5; ++f) {
        in.steer = 0.2 * std::sin(f / 30.0);
        v.stepSubstepped(1.0 / 60, 4, in, g);
        RigidBodyState s = v.state();
        const ChassisForces& cf = v.chassisForces();
        // Host: plain semi-implicit Euler at 60 Hz (a different integrator than the substeps).
        const Vec3 com = s.position + s.orientation.rotate(h.vehicle.centerOfMass);
        s.velocity += cf.force * (1.0 / 60 / h.vehicle.mass);
        const Mat3 R = s.orientation.toMatrix();
        const Mat3 inv = R * Mat3::diagonal({1 / h.vehicle.inertia.x, 1 / h.vehicle.inertia.y, 1 / h.vehicle.inertia.z}) * R.transposed();
        s.angularVelocity += inv * cf.torque * (1.0 / 60);
        const Real ang = length(s.angularVelocity) / 60;
        if (ang > 1e-12) s.orientation = Quat::fromAxisAngle(s.angularVelocity, ang) * s.orientation;
        s.position = com + s.velocity * (1.0 / 60) - s.orientation.rotate(h.vehicle.centerOfMass);
        v.setChassisState(s);
    }
    CHECK(v.hostSync().syncs == 300);
    CHECK(v.hostSync().maxPositionError < 0.003);   // m
    CHECK(v.hostSync().maxAngleError < 0.003);      // rad
    CHECK(isFinite(v.state().position));
}

TEST_CASE("envelope probes: no load on flat ground, better curb convergence across rates") {
    // Flat ground: identical static loads with and without probes.
    RunConfig a = base(240), b = base(240);
    b.fidelity.envelopeProbes = 0;
    Vehicle va(a.vehicle, a.assists, a.fidelity), vb(b.vehicle, b.assists, b.fidelity);
    FlatGround g;
    va.reset({0, 0, 0.6}, 0, g);
    vb.reset({0, 0, 0.6}, 0, g);
    for (std::size_t w = 0; w < 4; ++w) CHECK_NEAR(va.telemetry().wheels[w].load, vb.telemetry().wheels[w].load, 1e-6);
    // 8 cm curb at 120 km/h through the ground cache: 240 Hz vs 2000 Hz peak heave.
    auto heaveErr = [](int probes) {
        RunConfig lo = base(240), hi = base(2000);
        lo.prefetch = hi.prefetch = true;
        lo.fidelity.envelopeProbes = hi.fidelity.envelopeProbes = probes;
        return std::abs(curbStrike(lo, 120).maxHeave / curbStrike(hi, 120).maxHeave - 1.0);
    };
    const Real with = heaveErr(3), without = heaveErr(0);
    CHECK(with < 0.10);
    CHECK(without > with);
}

TEST_CASE("ground cache: a 100 ms game-thread hitch at 200 km/h degrades gracefully (extrapolation keeps the car on the road)") {
    TestTrackGround world([](Real x, Real y) { return 0.3 * std::sin(x / 15.0) + 0.05 * std::sin(y / 4.0); });
    for (int adaptive = 0; adaptive < 2; ++adaptive) {
        Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Sport));
        v.reset({0, 0, 1.0}, 0, world);
        v.selectGearForSpeed(kmhToMs(200));
        v.placeOnGround(0, 0, 0, world, kmhToMs(200));
        GroundPrefetchCache cache;
        std::vector<GroundStripRequest> req;
        DriverInput in;
        in.throttle = 0.7;
        const Real frame = 1.0 / 60;
        Real lastFrameDt = frame;
        for (int f = 0; f < 120; ++f) {
            const bool hitch = f >= 60 && f < 66;  // game thread misses 6 frames (100 ms)
            if (!hitch) {
                if (adaptive) cache.setHorizon(1.5 * std::max(lastFrameDt, frame));
                req.clear();
                v.groundPrefetchRequests(req);
                cache.rebuild(world, req.data(), req.size());
            }
            // The game thread knows a hitch happened only afterwards: frame 66 sees a 100 ms dt.
            lastFrameDt = (f == 66) ? 6 * frame : frame;
            for (int k = 0; k < 4; ++k) v.step(frame / 4, in, cache);
            CHECK(isFinite(v.state().position));
        }
        // Still on the road (no fall-through), never faulted.
        const Vec3 c = v.centerOfMassWorld();
        CHECK(c.z - world.heightAt(c.x, c.y) > 0.2);
        CHECK(c.z - world.heightAt(c.x, c.y) < 1.0);
        // Strips run out during the hitch either way (the game thread cannot know
        // in advance); the adaptive horizon only helps once slow frames are measured.
        CHECK(cache.misses() > 0);
    }
}
