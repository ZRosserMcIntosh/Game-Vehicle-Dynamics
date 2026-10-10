// OpenRoad Dynamics — benchmark scenarios as regression tests.
// SPDX-License-Identifier: MIT
//
// Tolerance bands are plausibility envelopes for the fictional reference
// vehicles (docs/benchmarks/benchmark-plan.md), not claims of matching any
// real car or game. Tighten them when real-world reference data is added.
#include "ord/presets.hpp"
#include "ord_test.hpp"
#include "reference_vehicles.hpp"
#include "scenarios.hpp"

using namespace ord;
using namespace ord::scenarios;

namespace {
RunConfig sim(const VehicleDefinition& d, Real hz = 360) {
    RunConfig c;
    c.vehicle = d;
    c.assists = makePreset(DrivingPreset::Simulation);
    c.hz = hz;
    return c;
}
}  // namespace

TEST_CASE("bench: 0-100 km/h within plausible envelopes") {
    RunConfig s = sim(vehicles::referenceSportsRwd());
    s.assists.tractionControl = InterventionLevel::Sport;
    const AccelResult sports = accelerate0to100(s);
    CHECK(sports.reached);
    CHECK_RANGE(sports.time0to100, 3.8, 6.0);
    const AccelResult hatch = accelerate0to100(sim(vehicles::referenceHatchFwd()));
    CHECK(hatch.reached);
    CHECK_RANGE(hatch.time0to100, 7.5, 12.0);
}

TEST_CASE("bench: 100-0 km/h braking - ABS near peak mu, locked wheels longer") {
    RunConfig c = sim(vehicles::referenceSportsRwd());
    c.assists.abs = true;
    const BrakeResult abs = brake100to0(c);
    CHECK(abs.stopped);
    CHECK_RANGE(abs.meanDecelG, 0.9, 1.15);
    CHECK(abs.maxAbsSideslip < 0.02);
    c.assists.abs = false;
    const BrakeResult locked = brake100to0(c);
    CHECK(locked.anyWheelLocked);
    CHECK(locked.distance > abs.distance * 1.1);
}

TEST_CASE("bench: wet braking is longer than dry") {
    RunConfig c = sim(vehicles::referenceSportsRwd());
    c.assists.abs = true;
    const BrakeResult dry = brake100to0(c);
    c.surface.frictionScale = 0.75;
    c.surface.wetSpeedFalloff = 0.004;
    const BrakeResult wet = brake100to0(c);
    CHECK(wet.distance > dry.distance * 1.2);
}

TEST_CASE("bench: skidpad lateral acceleration near tire mu") {
    const SkidpadResult r = skidpad(sim(vehicles::referenceSportsRwd()));
    CHECK_RANGE(r.maxLateralG, 0.85, 1.12);
}

TEST_CASE("bench: understeer gradient of the reference car is positive (stable)") {
    const StepSteerResult r = stepSteer(sim(vehicles::referenceSportsRwd()));
    const VehicleDefinition d = vehicles::referenceSportsRwd();
    const Real v = kmhToMs(80);
    const Real delta = 0.08 * d.wheels[0].maxSteerAngle;
    const Real ay = v * r.steadyYawRate;
    const Real K = (delta - d.wheelbase() * r.steadyYawRate / v) / ay;  // rad per m/s^2
    CHECK(K > 0);
    CHECK(K < 0.006);
    CHECK_RANGE(r.yawRateRiseTime, 0.05, 0.5);
}

TEST_CASE("bench: timestep sensitivity - 120 Hz within 2% of 1000 Hz") {
    const VehicleDefinition d = vehicles::referenceSportsRwd();
    const BrakeResult b120 = brake100to0(sim(d, 120)), b1000 = brake100to0(sim(d, 1000));
    CHECK_NEAR(b120.distance / b1000.distance, 1.0, 0.02);
    const StepSteerResult s120 = stepSteer(sim(d, 120)), s1000 = stepSteer(sim(d, 1000));
    CHECK_NEAR(s120.steadyYawRate / s1000.steadyYawRate, 1.0, 0.02);
    RunConfig a120 = sim(d, 120), a1000 = sim(d, 1000);
    a120.assists.tractionControl = a1000.assists.tractionControl = InterventionLevel::Sport;
    CHECK_NEAR(accelerate0to100(a120).time0to100 / accelerate0to100(a1000).time0to100, 1.0, 0.03);
}

TEST_CASE("bench: drop test dissipates energy and settles") {
    const DropResult r = dropTest(sim(vehicles::referenceSportsRwd()));
    CHECK(r.finite);
    CHECK(!r.energyIncreased);
}

TEST_CASE("bench: free-rolling car in neutral does not gain speed") {
    const CoastResult r = coastNoLosses(sim(vehicles::referenceSportsRwd()));
    CHECK(!r.speedEverIncreased);
    CHECK(r.speedLossPerSecond >= 0);
    CHECK(r.speedLossPerSecond < 0.2);
}

TEST_CASE("bench: standstill holding on a 20% grade") {
    const VehicleDefinition d = vehicles::referenceSportsRwd();
    CHECK(slopeHold(sim(d), 20, 10, true).drift < 0.02);
    CHECK(slopeHold(sim(d), 20, 3, false).drift > 1.0);  // rolls back without brakes
}

TEST_CASE("bench: speed bump traversal stays finite and grounded at 40 km/h") {
    const BumpResult r = speedBump(sim(vehicles::referenceSportsRwd()), 40);
    CHECK(r.finite);
    CHECK(r.peakCompression > 0.05);
    CHECK(r.airborneTime < 0.1);
}
