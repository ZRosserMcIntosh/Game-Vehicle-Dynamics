// OpenRoad Dynamics — robustness, edge cases and independent physics checks.
// SPDX-License-Identifier: MIT
//
// Tests marked [analytic] compare against closed-form physics, not against the
// simulator's own previous output.
#include <memory>
#include <random>

#include "ord/lod_manager.hpp"
#include "ord/presets.hpp"
#include "ord/vehicle.hpp"
#include "ord_test.hpp"
#include "reference_vehicles.hpp"

using namespace ord;

namespace {

VehicleDefinition noLossCar() {
    VehicleDefinition d = vehicles::referenceSportsRwd();
    for (auto& w : d.wheels) w.tire.rollingResistance = 0;
    d.aero.frontDownforceArea = d.aero.rearDownforceArea = 0;
    return d;
}

std::unique_ptr<Vehicle> rolling(const VehicleDefinition& d, const AssistConfig& a, Real speed, const GroundQuery& g) {
    auto v = std::make_unique<Vehicle>(d, a);
    v->reset({0, 0, 0.6}, 0, g);
    v->selectGearForSpeed(speed);
    v->placeOnGround(0, 0, 0, g, speed);
    return v;
}

}  // namespace

// ---------------------------------------------------------------- analytic

TEST_CASE("[analytic] coast-down with pure quadratic drag matches v(t) = v0 / (1 + k v0 t / m)") {
    VehicleDefinition d = noLossCar();
    FlatGround g;
    auto v = rolling(d, makePreset(DrivingPreset::Simulation), 50.0, g);
    v->assists().setGear(0);
    v->setRollingSpeed(50.0);
    DriverInput in;
    const Real k = 0.5 * d.aero.airDensity * d.aero.dragArea;
    for (Real hz : {60.0, 240.0}) {
        auto car = rolling(d, makePreset(DrivingPreset::Simulation), 50.0, g);
        car->assists().setGear(0);
        car->setRollingSpeed(50.0);
        const Real T = 10.0;
        for (int i = 0; i < static_cast<int>(T * hz); ++i) car->step(1.0 / hz, in, g);
        const Real expected = 50.0 / (1.0 + k * 50.0 * T / d.mass);
        CHECK_NEAR(car->forwardSpeed() / expected, 1.0, 0.005);
    }
}

TEST_CASE("[analytic] free fall: system momentum is ballistic (body + unsprung wheels)") {
    FlatGround g;
    Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Simulation));
    v.reset({0, 0, 0.6}, 0, g);
    v.mutableState().position.z += 3.0;
    v.mutableState().velocity = {};
    DriverInput in;
    const Real dt = 1.0 / 1000;
    const int n = 400;  // 0.4 s: falls ~0.8 m, still airborne
    for (int i = 0; i < n; ++i) v.step(dt, in, g);
    for (std::size_t w = 0; w < 4; ++w) CHECK(!v.telemetry().wheels[w].contact);
    // As the wheels drop to full droop they exchange momentum with the body,
    // so only the *system* momentum is exactly ballistic:
    //   M v_body + sum m_u * xdot_rel = -M g t   (xdot_rel: wheel speed relative to body, +up)
    const VehicleDefinition& d = v.definition();
    Real p = d.mass * v.state().velocity.z;
    for (std::size_t w = 0; w < 4; ++w) p += d.wheels[w].unsprungMass * v.telemetry().wheels[w].compressionRate;
    const Real aero = 0.5 * d.aero.airDensity * d.aero.dragArea * 4.0 * 4.0 * 0.4;  // generous drag impulse bound
    CHECK_NEAR(p, -d.mass * kGravity * n * dt, aero + 1.0);
}

namespace {
Real measuredUndersteerGradient(const VehicleDefinition& d) {
    FlatGround g;
    const Real speed = kmhToMs(60);
    auto v = rolling(d, makePreset(DrivingPreset::Simulation), speed, g);
    DriverInput in;
    Real integ = 0;
    in.steer = 0.04;  // ~0.2 g: linear tire range
    for (int i = 0; i < 240 * 8; ++i) {
        const Real e = speed - v->forwardSpeed();
        integ += e / 240;
        in.throttle = clamp(0.2 + 0.4 * e + 0.3 * integ, 0.0, 1.0);
        v->step(1.0 / 240, in, g);
    }
    const Real L = d.wheelbase();
    const Real r = v->yawRate(), vx = v->forwardSpeed();
    const Real delta = 0.5 * (v->telemetry().wheels[0].steerAngle + v->telemetry().wheels[1].steerAngle);
    return (delta - L * r / vx) / (vx * r);
}
}  // namespace

TEST_CASE("[analytic] understeer gradient matches the linear bicycle model from the tire's own stiffness") {
    // Understeering FWD car, open differential, no ARBs, no Ackermann: the
    // linear single-track model applies (pneumatic trail moves the axle force
    // lines: a -> a - t, b -> b + t).
    VehicleDefinition d = vehicles::referenceHatchFwd();
    d.steeringAckermann = 0;
    for (auto& a : d.axles) a.antiRollBarRate = 0;
    const Real kMeasured = measuredUndersteerGradient(d);
    auto ky = [](const WheelDef& w, Real fz) {
        MagicFormulaTire t{w.tire};
        return t.pureLateral(fz, 1e-4, 1.0) / 1e-4;
    };
    const Real L = d.wheelbase(), W = d.mass * kGravity;
    const Real a = d.wheels[0].suspension.hardpoint.x - d.centerOfMass.x;
    const Real b = d.centerOfMass.x - d.wheels[2].suspension.hardpoint.x;
    const Real cf = 2 * ky(d.wheels[0], 0.5 * W * b / L), cr = 2 * ky(d.wheels[2], 0.5 * W * a / L);
    const Real t = d.wheels[0].tire.pneumaticTrail;
    const Real kBicycle = (d.mass / L) * ((b + t) / cf - (a - t) / cr);
    CHECK(kBicycle > 0.001);  // clearly understeering
    // Remaining differences: front drive force (combined slip), load transfer.
    CHECK_NEAR(kMeasured / kBicycle, 1.0, 0.20);
}

TEST_CASE("[physics] limited-slip preload adds steady-state understeer versus an open differential") {
    VehicleDefinition lsd = vehicles::referenceSportsRwd();
    VehicleDefinition open = lsd;
    open.drivetrain.drivenAxles[0].differential.type = DifferentialType::Open;
    CHECK(measuredUndersteerGradient(lsd) > 2.0 * measuredUndersteerGradient(open));
}

// ---------------------------------------------------------------- assists

TEST_CASE("assists: disabling ABS mid-stop removes its modulation immediately (no hidden residual aid)") {
    FlatGround g;
    auto v = rolling(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Sport), 30.0, g);
    DriverInput in;
    in.brake = 1;
    for (int i = 0; i < 240; ++i) v->step(1.0 / 240, in, g);
    CHECK(v->telemetry().assists.absActive[0] || v->telemetry().assists.absActive[1]);  // ABS was working
    AssistConfig c = v->assists().config();
    c.abs = false;
    v->assists().setConfig(c);
    v->step(1.0 / 240, in, g);
    for (std::size_t w = 0; w < 4; ++w) {
        CHECK_NEAR(v->telemetry().wheels[w].brakeTorque, v->definition().wheels[w].maxBrakeTorque, 1e-9);
        CHECK(!v->telemetry().assists.absActive[w]);
    }
}

TEST_CASE("assists: switching presets mid-corner blends steering and stays stable") {
    FlatGround g;
    auto v = rolling(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Fun), 25.0, g);
    DriverInput in;
    in.throttle = 0.5;
    in.steer = 0.5;
    Real maxStep = 0, prev = 0;
    for (int i = 0; i < 240 * 12; ++i) {
        if (i == 240 * 3) v->assists().setConfig(makePreset(DrivingPreset::Simulation));
        if (i == 240 * 6) v->assists().setConfig(makePreset(DrivingPreset::Sport));
        if (i == 240 * 9) v->assists().setConfig(makePreset(DrivingPreset::Fun));
        v->step(1.0 / 240, in, g);
        const Real a = v->telemetry().steerAngle;
        if (i > 0) maxStep = std::max(maxStep, std::abs(a - prev));
        prev = a;
    }
    // Largest per-step change in road-wheel angle: blended over 0.3 s, so well
    // under 1 degree per 1/240 s step even though the mappings differ by ~10 deg.
    CHECK(maxStep < 0.01);
    CHECK(isFinite(v->state().position));
    CHECK(std::abs(v->telemetry().sideslip) < 0.3);
}

TEST_CASE("assists: SIMULATION with a steering wheel passes the input through exactly, at any speed") {
    FlatGround g;
    for (Real speed : {5.0, 40.0, 80.0}) {
        auto v = rolling(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Simulation, InputDevice::SteeringWheel), speed, g);
        DriverInput in;
        in.steer = -0.37;
        in.throttle = 0.3;
        v->step(1.0 / 240, in, g);
        CHECK_NEAR(v->telemetry().steerAngle, -0.37 * v->definition().wheels[0].maxSteerAngle, 1e-12);
        CHECK_NEAR(v->telemetry().throttleOut, 0.3, 1e-12);
    }
}

TEST_CASE("assists: FUN countersteer aid recovers a handbrake slide that spins the car in SIMULATION") {
    FlatGround g;
    Real slip[2];
    Real finalSpeed[2];
    for (int p = 0; p < 2; ++p) {
        auto v = rolling(vehicles::referenceSportsRwd(), makePreset(p ? DrivingPreset::Fun : DrivingPreset::Simulation), 19.4, g);
        DriverInput in;
        Real maxB = 0;
        for (int i = 0; i < 240 * 6; ++i) {
            const Real t = i / 240.0;
            in.steer = t < 0.8 ? 0.6 : 0.0;
            in.handbrake = (t > 0.3 && t < 1.0) ? 1 : 0;
            in.throttle = t < 1.0 ? 0.6 : 0.3;
            v->step(1.0 / 240, in, g);
            maxB = std::max(maxB, std::abs(v->telemetry().sideslip));
        }
        slip[p] = maxB;
        finalSpeed[p] = v->forwardSpeed();
        CHECK(isFinite(v->state().position));
    }
    CHECK(slip[0] > 1.0);          // SIMULATION, hands off: genuine spin (> 57 deg)
    CHECK(slip[1] < 0.7);          // FUN: caught by the aid
    CHECK(finalSpeed[1] > 10.0);   // and driving on
}

// ---------------------------------------------------------------- edge cases

TEST_CASE("edge: reverse, stop and hold without standstill jitter") {
    FlatGround g;
    Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Fun));
    v.reset({0, 0, 0.6}, 0, g);
    DriverInput in;
    in.brake = 1;
    in.shiftDown = true;
    v.step(1.0 / 240, in, g);
    CHECK(v.telemetry().gear == -1);
    in.shiftDown = false;
    in.brake = 0;
    in.throttle = 0.4;
    Real vmin = 0;
    for (int i = 0; i < 240 * 4; ++i) {
        v.step(1.0 / 240, in, g);
        vmin = std::min(vmin, v.forwardSpeed());
    }
    CHECK(vmin < -5.0);
    in.throttle = 0;
    in.brake = 0.8;
    for (int i = 0; i < 240 * 4; ++i) v.step(1.0 / 240, in, g);
    Real jitter = 0;
    for (int i = 0; i < 240 * 3; ++i) {
        v.step(1.0 / 240, in, g);
        jitter = std::max(jitter, length(v.state().velocity));
    }
    CHECK(jitter < 1e-3);
}

TEST_CASE("edge: 450+ km/h straight-line stability (hypercar)") {
    FlatGround g;
    Vehicle v(vehicles::referenceHypercarAwd(), makePreset(DrivingPreset::Sport));
    CHECK(v.isValid());
    v.reset({0, 0, 0.6}, 0, g);
    DriverInput in;
    in.throttle = 1;
    Real maxYaw = 0, vmax = 0;
    const int seconds = ordtest::kOptimisedBuild ? 90 : 25;
    for (int i = 0; i < 240 * seconds; ++i) {
        v.step(1.0 / 240, in, g);
        vmax = std::max(vmax, msToKmh(v.forwardSpeed()));
        if (vmax > 300) maxYaw = std::max(maxYaw, std::abs(v.yawRate()));
    }
    CHECK(vmax > (ordtest::kOptimisedBuild ? 440 : 380));
    CHECK(maxYaw < 1e-3);
    CHECK(!v.faulted());
}

TEST_CASE("edge: rapid steering reversals at 120 km/h stay finite; FUN keeps the car under control") {
    FlatGround g;
    for (DrivingPreset p : {DrivingPreset::Simulation, DrivingPreset::Fun}) {
        auto v = rolling(vehicles::referenceSportsRwd(), makePreset(p), kmhToMs(120), g);
        DriverInput in;
        Real maxB = 0;
        for (int i = 0; i < 240 * 6; ++i) {
            in.steer = (i / 60) % 2 ? 0.8 : -0.8;  // full-ish reversal every 0.25 s
            in.throttle = (i / 120) % 2 ? 1.0 : 0.0;
            v->step(1.0 / 240, in, g);
            maxB = std::max(maxB, std::abs(v->telemetry().sideslip));
        }
        CHECK(isFinite(v->state().position));
        CHECK(!v->faulted());
        if (p == DrivingPreset::Fun) CHECK(maxB < 0.35);
    }
}

TEST_CASE("edge: cresting a hill at 200 km/h - contact loss, ballistic flight and landing") {
    TestTrackGround hill([](Real x, Real) { return 4.0 * std::exp(-((x - 200) / 25) * ((x - 200) / 25)); });
    auto v = rolling(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Sport), kmhToMs(200), hill);
    DriverInput in;
    in.throttle = 0.6;
    Real air = 0, maxLoad = 0;
    bool regained = false, wasAir = false;
    for (int i = 0; i < 240 * 8; ++i) {
        v->step(1.0 / 240, in, hill);
        bool all = true, any = false;
        for (std::size_t w = 0; w < 4; ++w) {
            all = all && !v->telemetry().wheels[w].contact;
            any = any || v->telemetry().wheels[w].contact;
            maxLoad = std::max(maxLoad, v->telemetry().wheels[w].load);
        }
        if (all) { air += 1.0 / 240; wasAir = true; }
        if (wasAir && any) regained = true;
    }
    CHECK(air > 0.5);       // genuinely airborne
    CHECK(regained);        // and lands
    CHECK(isFinite(v->state().position));
    CHECK(std::abs(v->telemetry().rollAngle) < 0.05);  // symmetric jump: no spurious roll
    CHECK(v->forwardSpeed() > kmhToMs(150));
}

TEST_CASE("edge: very high downforce raises cornering capacity with speed (race car)") {
    FlatGround g;
    const VehicleDefinition d = vehicles::referenceRaceDownforce();
    auto maxLat = [&](Real kmh) {
        Real best = 0;
        for (Real st = 0.04; st <= 0.40; st += 0.04) {
            auto v = rolling(d, makePreset(DrivingPreset::Simulation), kmh / 3.6, g);
            DriverInput in;
            in.steer = st;
            Real ay = 0, integ = 0;
            int n = 0;
            for (int i = 0; i < 240 * 4; ++i) {
                const Real e = kmh / 3.6 - v->forwardSpeed();
                integ += e / 240;
                in.throttle = clamp(0.3 + 0.3 * e + 0.2 * integ, 0.0, 1.0);
                v->step(1.0 / 240, in, g);
                if (i > 240 * 3) { ay += v->telemetry().accelBody.y; ++n; }
            }
            if (std::abs(v->telemetry().sideslip) < 0.15) best = std::max(best, ay / n);
        }
        return best / kGravity;
    };
    const Real g100 = maxLat(100), g250 = maxLat(250);
    CHECK(g100 > 1.3);
    CHECK(g250 > 1.3 * g100);
}

TEST_CASE("edge: long-duration stability - 30 simulated minutes of cruising (3 in Debug)") {
    FlatGround g;
    auto v = rolling(vehicles::referenceHatchFwd(), makePreset(DrivingPreset::Sport), 25.0, g);
    DriverInput in;
    Real integ = 0;
    const int minutes = ordtest::kOptimisedBuild ? 30 : 3;
    for (int i = 0; i < 240 * 60 * minutes; ++i) {
        const Real e = 25.0 - v->forwardSpeed();
        integ = clamp(integ + e / 240, -10.0, 10.0);
        in.throttle = clamp(0.2 + 0.2 * e + 0.05 * integ, 0.0, 1.0);
        in.steer = 0.02 * std::sin(i / 2400.0);
        v->step(1.0 / 240, in, g);
    }
    const Quat& q = v->state().orientation;
    CHECK_NEAR(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z, 1.0, 1e-9);
    CHECK_NEAR(v->forwardSpeed(), 25.0, 0.5);
    CHECK(std::abs(v->telemetry().rollAngle) < 0.02);
    CHECK(!v->faulted());
}

// ---------------------------------------------------------------- robustness

namespace {
class NaNGround final : public GroundQuery {
public:
    GroundHit raycast(const Vec3& o, const Vec3& dir, Real maxDist) const override {
        GroundHit h = FlatGround().raycast(o, dir, maxDist);
        if (++calls_ > trigger) h.normal = {std::nan(""), 0, 1};
        return h;
    }
    int trigger = 100000;
    mutable int calls_ = 0;
};
}  // namespace

TEST_CASE("robustness: corrupted ground data faults the vehicle instead of propagating NaN") {
    NaNGround g;
    Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Sport));
    v.reset({0, 0, 0.6}, 0, g);
    g.trigger = g.calls_ + 50;
    DriverInput in;
    in.throttle = 0.5;
    for (int i = 0; i < 100; ++i) v.step(1.0 / 240, in, g);
    CHECK(v.faulted());
    CHECK(isFinite(v.state().position));
    CHECK(isFinite(v.chassisForces().force));
    CHECK(isFinite(v.chassisForces().torque));
    // A placement clears the fault.
    g.trigger = 1 << 30;
    v.placeOnGround(0, 0, 0, g);
    CHECK(!v.faulted());
}

TEST_CASE("robustness: fuzzed vehicle definitions are either rejected or simulate without NaN or faults") {
    std::mt19937_64 rng(2026);
    std::uniform_real_distribution<Real> u(0, 1);
    int rejected = 0, ran = 0, broken = 0;
    for (int it = 0; it < 400; ++it) {
        VehicleDefinition d = it % 2 ? vehicles::referenceSportsRwd() : vehicles::referenceHatchFwd();
        Real* fields[] = {&d.mass, &d.inertia.z, &d.centerOfMass.z, &d.wheels[0].tire.radius, &d.wheels[1].tire.muY,
                          &d.wheels[2].tire.nominalLoad, &d.wheels[0].suspension.springRate, &d.wheels[3].suspension.travel,
                          &d.wheels[0].suspension.bumpDamping, &d.wheels[0].unsprungMass, &d.wheels[2].tire.verticalStiffness,
                          &d.wheels[0].tire.relaxationLengthY, &d.wheels[0].tire.wheelInertia, &d.drivetrain.engine.inertia,
                          &d.drivetrain.gearbox.forwardRatios[0], &d.drivetrain.drivenAxles[0].differential.finalDrive,
                          &d.aero.dragArea, &d.aero.frontDownforceArea, &d.wheels[0].maxSteerAngle, &d.wheels[0].maxBrakeTorque,
                          &d.drivetrain.clutch.maxTorque, &d.drivetrain.engine.torqueCurve[2].torque, &d.wheels[0].tire.patchHalfLength,
                          &d.axles[0].antiRollBarRate, &d.wheels[0].tire.shapeY, &d.wheels[0].tire.cornerStiffness,
                          &d.drivetrain.engine.limiterRpm, &d.wheels[1].suspension.hardpoint.z, &d.aero.airDensity,
                          &d.drivetrain.gearbox.efficiency};
        const int nm = 1 + static_cast<int>(u(rng) * 3);
        for (int m = 0; m < nm; ++m) {
            Real& x = *fields[static_cast<std::size_t>(u(rng) * (sizeof(fields) / sizeof(fields[0])))];
            const Real r = u(rng);
            if (r < 0.1) x = 0;
            else if (r < 0.2) x = -x;
            else if (r < 0.25) x = std::nan("");
            else if (r < 0.3) x = 1e30;
            else x *= std::pow(10.0, (u(rng) - 0.5) * 6);
        }
        Vehicle v(d, makePreset(DrivingPreset::Simulation));
        if (!v.isValid()) { ++rejected; continue; }
        TestTrackGround g([](Real x, Real y) { return 0.05 * std::sin(0.7 * x) * std::cos(0.9 * y); });
        v.reset({0, 0, 0.8}, 0, g);
        DriverInput in;
        in.throttle = 1;
        in.steer = 0.3;
        for (int i = 0; i < 240 * 2; ++i) {
            if (i == 240) in.brake = 1;
            v.step(1.0 / 240, in, g);
        }
        if (v.faulted() || !isFinite(v.state().position)) ++broken;
        else ++ran;
    }
    CHECK(broken == 0);
    CHECK(ran > 50);       // the fuzzer still exercises many valid variants
    CHECK(rejected > 50);  // and the validator rejects plenty
}

TEST_CASE("lifecycle: repeated vehicle construction/destruction and rapid LOD churn") {
    FlatGround g;
    for (int i = 0; i < 300; ++i) {
        auto v = std::make_unique<Vehicle>(i % 2 ? vehicles::referenceSportsRwd() : vehicles::referenceRaceDownforce(),
                                           makePreset(DrivingPreset::Fun));
        v->placeOnGround(static_cast<Real>(i), 0, 0, g, 10.0);
        DriverInput in;
        in.throttle = 0.5;
        for (int k = 0; k < 10; ++k) v->step(1.0 / 240, in, g);
        CHECK(isFinite(v->state().position));
    }
    std::vector<LanePath> lanes{LanePath::circle({0, 0, 0}, 120.0, 30.0)};
    TrafficSystem traffic(lanes, TrafficSystem::Settings{});
    for (int i = 0; i < 40; ++i) traffic.addAgent(0, i * 18.0, 15.0, 15.0, i % 2);
    LodSettings ls;
    ls.maxLod1 = 4;
    ls.promoteRadius = 40;
    ls.demoteRadius = 50;
    LodManager lod(traffic, {vehicles::referenceHatchFwd(), vehicles::referenceSportsRwd()}, ls);
    for (int f = 0; f < 600; ++f) {
        lod.update(1.0 / 60, {f % 2 ? -120.0 : 120.0, 0.0, 0}, g);  // focus teleports across the ring every frame
        for (const auto& c : lod.lod1())
            if (c.used) CHECK(isFinite(c.vehicle->state().position));
    }
    CHECK(lod.stats().promotions > 100);
    CHECK(lod.stats().demotions > 100);
    CHECK(lod.stats().lod1Active <= 4);
}
