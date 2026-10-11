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
    RunConfig ref = base(1000);
    RunConfig h = base(240);
    h.hostHz = 60;
    h.prefetch = true;
    CHECK_NEAR(curbStrike(h, 60).rmsVerticalAccelG / curbStrike(ref, 60).rmsVerticalAccelG, 1.0, 0.10);
    CHECK_NEAR(pothole(h, 60).rmsVerticalAccelG / pothole(ref, 60).rmsVerticalAccelG, 1.0, 0.10);
    CHECK_NEAR(cobblestones(h, 40, true).rmsVerticalAccelG / cobblestones(ref, 40, true).rmsVerticalAccelG, 1.0, 0.05);
}
