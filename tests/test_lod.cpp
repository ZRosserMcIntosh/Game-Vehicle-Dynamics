// OpenRoad Dynamics — traffic, fidelity tiers, telemetry recorder and debug draw tests.
// SPDX-License-Identifier: MIT
#include <sstream>

#include "ord/debug_draw.hpp"
#include "ord/lod_manager.hpp"
#include "ord/presets.hpp"
#include "ord/telemetry_recorder.hpp"
#include "ord/traffic.hpp"
#include "ord_test.hpp"
#include "reference_vehicles.hpp"
#include "scenarios.hpp"

using namespace ord;
using namespace ord::scenarios;

TEST_CASE("lane: circle geometry, projection and curvature") {
    const LanePath c = LanePath::circle({0, 0, 0}, 100.0, 30.0, 512);
    CHECK_NEAR(c.length(), 2 * kPi * 100.0, 0.5);
    CHECK_NEAR(c.curvatureAt(50.0), 0.01, 0.0005);  // counter-clockwise = left turn
    const Vec3 p = c.pointAt(123.0);
    CHECK_NEAR(c.project(p + Vec3{0.3, 0.2, 0}, 120.0), 123.0, 1.0);
    CHECK_NEAR(c.wrap(c.length() + 5.0), 5.0, 1e-9);
}

TEST_CASE("traffic: IDM keeps safe gaps and never lets cars pass through each other") {
    std::vector<LanePath> lanes{LanePath::circle({0, 0, 0}, 150.0, 30.0)};
    TrafficSystem t(lanes, TrafficSystem::Settings{});
    t.addAgent(0, 50.0, 0.0, 0.0);        // stopped car
    t.addAgent(0, 0.0, 20.0, 20.0);       // fast follower
    Real minGap = 1e9;
    for (int i = 0; i < 30 * 30; ++i) {
        t.step(1.0 / 30.0, nullptr);
        Real d = t.agents()[0].s - t.agents()[1].s;
        if (d < 0) d += lanes[0].length();
        minGap = std::min(minGap, d - t.settings().idm.vehicleLength);
    }
    CHECK(minGap > 0.5);
    CHECK(t.agents()[1].speed < 0.5);  // came to rest behind the stopped car
}

TEST_CASE("traffic: curve speed respects the lateral-acceleration limit") {
    std::vector<LanePath> lanes{LanePath::circle({0, 0, 0}, 60.0, 40.0)};
    TrafficSystem t(lanes, TrafficSystem::Settings{});
    t.addAgent(0, 0.0, 30.0, 40.0);
    for (int i = 0; i < 30 * 20; ++i) t.step(1.0 / 30.0, nullptr);
    const Real v = t.agents()[0].speed;
    CHECK(v * v / 60.0 <= t.settings().idm.maxLateralAccel * 1.05);
}

TEST_CASE("lod: promotion and demotion are seamless (B24)") {
    const TransitionResult r = lodTransition(70.0);
    CHECK(r.speedJumpPromote < 0.5);
    CHECK(r.yawRateJumpPromote < 0.02);
    CHECK(r.maxVerticalAccelG < 0.3);   // no drop or pop when the physical car appears
    CHECK(r.maxLateralError < 0.6);     // AI holds the lane
    CHECK(r.speedJumpDemote < 0.5);
    CHECK(r.positionJumpDemote < 0.6);
}

TEST_CASE("lod: 1 + 16 + 300 vehicles within the 2 ms/frame budget, bounded rays (B25)") {
    const ScaleResult r = trafficScale(300, 10.0, 16);
    CHECK(r.finite);
    CHECK(r.maxLod1 <= 16);
    CHECK(r.promotions > 0);
#ifdef NDEBUG  // CPU budgets are only meaningful in optimised builds (not Debug/sanitizer runs)
    CHECK(r.totalMsPerFrame < 2.0);
#endif
    CHECK(r.lod1RaysPerFrame < 16 * 8);   // ~one coarse strip per LOD1 car
    CHECK(r.lod2RaysPerFrame < 300 / 4);  // staggered, every few steps
    CHECK(r.maxLod1LateralError < 1.0);
}

TEST_CASE("telemetry recorder: fixed-capacity ring keeps the newest frames") {
    TelemetryRecorder rec(3);
    for (int i = 0; i < 5; ++i) {
        TelemetryFrame f;
        f.time = i;
        f.wheelCount = 4;
        rec.push(f);
    }
    CHECK(rec.size() == 3);
    CHECK_NEAR(rec.frame(0).time, 2.0, 1e-12);
    CHECK_NEAR(rec.frame(2).time, 4.0, 1e-12);
    std::ostringstream os;
    rec.writeCsv(os);
    const std::string csv = os.str();
    CHECK(csv.find("time,") == 0);
    CHECK(std::count(csv.begin(), csv.end(), '\n') == 4);  // header + 3 rows
}

TEST_CASE("debug draw: vehicle overlay emits contact samples and force arrows (SVG)") {
    Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Sport));
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    DriverInput in;
    in.throttle = 0.6;
    in.steer = 0.3;
    for (int i = 0; i < 240; ++i) v.step(1.0 / 240, in, g);
    SvgDebugSink svg;
    DebugDrawOptions opt;
    opt.labels = true;
    drawVehicle(v, svg, opt);
    std::ostringstream os;
    svg.write(os);
    const std::string s = os.str();
    CHECK(s.find("<svg") == 0);
    CHECK(std::count(s.begin(), s.end(), '\n') > 4 * (5 + 3) + 4);  // samples + arrows per wheel
}
