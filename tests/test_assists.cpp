// OpenRoad Dynamics — driver-assist and preset tests.
// SPDX-License-Identifier: MIT
#include "ord/presets.hpp"
#include "ord_test.hpp"
#include "reference_vehicles.hpp"
#include "scenarios.hpp"

using namespace ord;
using namespace ord::scenarios;

TEST_CASE("presets: SIMULATION has every electronic and non-physical aid off") {
    const AssistConfig c = makePreset(DrivingPreset::Simulation);
    CHECK(!c.abs);
    CHECK(c.tractionControl == InterventionLevel::Off);
    CHECK(c.stabilityControl == InterventionLevel::Off);
    CHECK(c.countersteerAssist == 0);
    CHECK(c.steeringMode == SteeringMode::Direct);
    CHECK(c.steerRate == 0);
    CHECK(c.throttleRate == 0);
}

TEST_CASE("presets: aids are individually configurable (SIM steering + ABS, no TCS/ESC)") {
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.assists = makePreset(DrivingPreset::Simulation);
    cfg.assists.abs = true;
    const BrakeResult withAbs = brake100to0(cfg);
    cfg.assists.abs = false;
    const BrakeResult without = brake100to0(cfg);
    CHECK(withAbs.absActiveFraction > 0.5);
    CHECK(without.absActiveFraction == 0);
    CHECK(withAbs.distance < without.distance);
}

TEST_CASE("assists: with all aids off, the assist layer reports no interventions") {
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.assists = makePreset(DrivingPreset::Simulation);
    Vehicle v(cfg.vehicle, cfg.assists);
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    DriverInput in;
    in.throttle = 1;
    bool any = false;
    for (int i = 0; i < 360 * 6; ++i) {
        in.steer = i > 360 * 3 ? 0.5 : 0.0;
        v.step(1.0 / 360, in, g);
        const AssistReport& r = v.telemetry().assists;
        any = any || r.tcsThrottleCut > 0 || r.countersteerAngle != 0;
        for (std::size_t w = 0; w < 4; ++w) any = any || r.absActive[w] || r.escBrakeTorque[w] > 0;
    }
    CHECK(!any);
}

TEST_CASE("assists: traction control limits wheelspin on a low-grip launch") {
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.surface.frictionScale = 0.5;
    cfg.assists = makePreset(DrivingPreset::Simulation);
    const AccelResult off = accelerate0to100(cfg);
    cfg.assists.tractionControl = InterventionLevel::Full;
    const AccelResult on = accelerate0to100(cfg);
    CHECK(on.peakDrivenSlip < 0.5 * off.peakDrivenSlip);
    CHECK(on.maxTcsCut > 0);
}

TEST_CASE("assists: ESC keeps an RWD car stable in sine-with-dwell where it is unstable without") {
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.assists = makePreset(DrivingPreset::Simulation);
    const Real amp = 9.0 * kPi / 180;
    const SineDwellResult off = sineWithDwell(cfg, amp);
    cfg.assists.stabilityControl = InterventionLevel::Full;
    const SineDwellResult on = sineWithDwell(cfg, amp);
    CHECK(!off.passesFmvss126Criteria());
    CHECK(on.passesFmvss126Criteria());
    CHECK(on.maxSideslip < 0.5 * off.maxSideslip);
}

TEST_CASE("assists: gamepad speed-sensitive steering limits full-stick angle at speed") {
    Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Fun, InputDevice::Gamepad));
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    v.setRollingSpeed(kmhToMs(150));
    DriverInput in;
    in.steer = 1.0;
    in.throttle = 0.3;
    for (int i = 0; i < 360; ++i) v.step(1.0 / 360, in, g);
    const Real limit = v.telemetry().assists.steerLimitAngle;
    CHECK(limit < 0.2 * v.definition().wheels[0].maxSteerAngle + 0.12);
    CHECK(limit > 0.0);
    // A steering wheel in SIMULATION gets the full, unfiltered angle.
    Vehicle w(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Simulation, InputDevice::SteeringWheel));
    w.reset({0, 0, 0.6}, 0, g);
    w.setRollingSpeed(kmhToMs(150));
    in.steer = 0.1;
    w.step(1.0 / 360, in, g);
    CHECK_NEAR(w.telemetry().steerAngle, 0.1 * w.definition().wheels[0].maxSteerAngle, 1e-9);
}

TEST_CASE("presets: the vehicle definition is identical under every preset") {
    // Presets are AssistConfig only; this guards against future coupling.
    const VehicleDefinition d = vehicles::referenceSportsRwd();
    for (DrivingPreset p : {DrivingPreset::Fun, DrivingPreset::Sport, DrivingPreset::Simulation}) {
        Vehicle v(d, makePreset(p));
        CHECK(v.definition().wheels[0].tire.muY == d.wheels[0].tire.muY);
        CHECK(v.definition().mass == d.mass);
        CHECK(v.definition().inertia.z == d.inertia.z);
    }
}
