// OpenRoad Dynamics — whole-vehicle tests.
// SPDX-License-Identifier: MIT
#include <cstring>
#include <random>

#include "ord/ground.hpp"
#include "ord/presets.hpp"
#include "ord/vehicle.hpp"
#include "ord_test.hpp"
#include "reference_vehicles.hpp"

using namespace ord;

TEST_CASE("vehicle: definition validation rejects bad data") {
    VehicleDefinition d = vehicles::referenceSportsRwd();
    CHECK(d.validate().empty());
    VehicleDefinition bad = d;
    bad.mass = -1;
    CHECK(!bad.validate().empty());
    bad = d;
    bad.wheels[0].tire.curvatureY = 1.5;
    CHECK(!bad.validate().empty());
    bad = d;
    bad.drivetrain.drivenAxles[0].torqueShare = 0.7;
    CHECK(!bad.validate().empty());
    bad = d;
    bad.wheels[2].suspension.springRate = std::nan("");
    CHECK(!bad.validate().empty());
    bad.mass = 0;
    Vehicle v(bad);  // no exceptions: the vehicle is inert and reports why
    CHECK(!v.isValid());
    CHECK(!v.error().empty());
}

TEST_CASE("vehicle: static corner loads sum to weight and follow the CoM position") {
    const VehicleDefinition d = vehicles::referenceHatchFwd();
    Vehicle v(d);
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    DriverInput hold;
    hold.brake = 1;
    for (int i = 0; i < 720; ++i) v.step(1.0 / 360, hold, g);
    const auto& t = v.telemetry();
    Real total = 0, front = 0;
    for (std::size_t w = 0; w < 4; ++w) total += t.wheels[w].load;
    front = t.wheels[0].load + t.wheels[1].load;
    CHECK_NEAR(total, d.mass * kGravity, 0.005 * d.mass * kGravity);
    CHECK_NEAR(front / total, 0.60, 0.01);  // 60/40 by construction
    CHECK_NEAR(t.speed, 0.0, 1e-3);
}

TEST_CASE("vehicle: simulation is bit-for-bit deterministic") {
    auto run = [] {
        Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Sport));
        FlatGround g;
        v.reset({0, 0, 0.6}, 0.3, g);
        DriverInput in;
        for (int i = 0; i < 3600; ++i) {
            in.throttle = (i % 700) < 400 ? 1.0 : 0.0;
            in.brake = (i % 700) >= 600 ? 0.6 : 0.0;
            in.steer = 0.4 * std::sin(i * 0.004);
            v.step(1.0 / 360, in, g);
        }
        return v.state();
    };
    const RigidBodyState a = run(), b = run();
    CHECK(std::memcmp(&a.position, &b.position, sizeof(Vec3)) == 0);
    CHECK(std::memcmp(&a.velocity, &b.velocity, sizeof(Vec3)) == 0);
    CHECK(std::memcmp(&a.orientation, &b.orientation, sizeof(Quat)) == 0);
}

TEST_CASE("vehicle: random driver inputs on bumpy ground never produce NaN/Inf (fuzz)") {
    for (const auto& def : {vehicles::referenceSportsRwd(), vehicles::referenceHatchFwd(), vehicles::referenceSaloonAwd()}) {
        for (Real hz : {60.0, 360.0}) {
            Vehicle v(def, makePreset(DrivingPreset::Simulation));
            TestTrackGround g([](Real x, Real y) { return 0.05 * std::sin(0.7 * x) * std::cos(0.9 * y) + 0.02 * std::sin(5.3 * x); });
            v.reset({0, 0, 0.8}, 0, g);
            std::mt19937 rng(1234);
            std::uniform_real_distribution<Real> u(0, 1);
            DriverInput in;
            bool finite = true;
            const int steps = static_cast<int>(40 * hz);
            for (int i = 0; i < steps && finite; ++i) {
                if (i % static_cast<int>(hz / 4) == 0) {
                    in.throttle = u(rng);
                    in.brake = u(rng) < 0.2 ? u(rng) : 0;
                    in.handbrake = u(rng) < 0.05 ? 1 : 0;
                    in.steer = 2 * u(rng) - 1;
                    in.shiftUp = u(rng) < 0.1;
                    in.shiftDown = u(rng) < 0.05;
                }
                v.step(1.0 / hz, in, g);
                const auto& s = v.state();
                finite = isFinite(s.position) && isFinite(s.velocity) && isFinite(s.angularVelocity) &&
                         std::isfinite(v.telemetry().drivetrain.engineRpm);
            }
            CHECK(finite);
            CHECK(!v.telemetry().drivetrain.solveFailed);
        }
    }
}

TEST_CASE("vehicle: top speed is drag-limited and stable (no oscillation at very high speed)") {
    Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Sport));
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    DriverInput in;
    in.throttle = 1;
    Real maxYaw = 0, maxRoll = 0;
    for (int i = 0; i < 360 * 90; ++i) {
        v.step(1.0 / 360, in, g);
        if (v.telemetry().speed > kmhToMs(250)) {
            maxYaw = std::max(maxYaw, std::abs(v.telemetry().yawRate));
            maxRoll = std::max(maxRoll, std::abs(v.telemetry().rollAngle));
        }
    }
    const Real vmax = msToKmh(v.telemetry().speed);
    CHECK_RANGE(vmax, 270.0, 330.0);   // ~300 kW, CdA 0.68: analytic estimate ~305 km/h
    CHECK(maxYaw < 0.01);              // straight-line stability
    CHECK(maxRoll < 0.01);
}
