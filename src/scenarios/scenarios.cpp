// OpenRoad Dynamics — reproducible benchmark scenarios.
// SPDX-License-Identifier: MIT
#include "scenarios.hpp"

#include <chrono>
#include <ostream>

#include "ord/ground.hpp"
#include "ord/telemetry.hpp"

namespace ord::scenarios {

namespace {

struct Sim {
    Vehicle vehicle;
    TestTrackGround ground;
    Real dt;
    std::ostream* csv;

    Sim(const RunConfig& cfg, TestTrackGround::HeightFn h = nullptr, Real yaw = 0, Vec3 start = {0, 0, 0.6})
        : vehicle(cfg.vehicle, cfg.assists), ground(std::move(h), cfg.surface), dt(1.0 / cfg.hz), csv(cfg.csv) {
        start.z += ground.heightAt(start.x, start.y);
        vehicle.reset(start, yaw, ground);
        if (csv) writeCsvHeader(*csv, cfg.vehicle.wheels.size());
    }
    void step(const DriverInput& in) {
        vehicle.step(dt, in, ground);
        if (csv) writeCsvRow(*csv, vehicle.telemetry());
    }
    Vec3 com() const {
        const auto& s = vehicle.state();
        return s.position + s.orientation.rotate(vehicle.definition().centerOfMass);
    }
};

Real maxSteerRef(const VehicleDefinition& d) {
    Real m = 0;
    for (const auto& w : d.wheels) m = std::max(m, w.maxSteerAngle);
    return m;
}

// Picks a gear giving a mid-range engine speed and sets the car rolling.
void prepareRolling(Vehicle& v, Real speed) {
    const auto& e = v.definition().drivetrain.engine;
    int best = 1;
    for (int g = 1; g <= v.drivetrain().forwardGearCount(); ++g) {
        v.assists().setGear(g);
        v.setRollingSpeed(speed);
        const Real rpm = radSToRpm(v.drivetrain().engineOmegaForGear(g));
        if (rpm >= 0.35 * e.limiterRpm && rpm <= 0.8 * e.limiterRpm) { best = g; break; }
        if (rpm >= 0.35 * e.limiterRpm) best = g;
    }
    v.assists().setGear(best);
    v.setRollingSpeed(speed);
}

// Proportional speed-holding throttle/brake.
void holdSpeed(DriverInput& in, Real speed, Real target) {
    const Real err = target - speed;
    in.throttle = clamp(0.25 + 0.4 * err, 0.0, 1.0);
    in.brake = err < -1.5 ? clamp(-0.1 * err, 0.0, 1.0) : 0.0;
}

RunConfig withAutoShift(const RunConfig& cfg) {
    RunConfig c = cfg;
    c.assists.shiftMode = ShiftMode::Automatic;  // scripted manoeuvres use automatic shifting
    c.assists.autoClutch = true;
    return c;
}

RunConfig withDirectSteering(const RunConfig& cfg) {
    RunConfig c = withAutoShift(cfg);
    // Path-following benchmarks drive the road-wheel angle directly so that
    // results measure the car, not the input filter.
    c.assists.steeringMode = SteeringMode::Direct;
    c.assists.steerRate = 0;
    c.assists.steeringExponent = 1.0;
    c.assists.countersteerAssist = 0;
    return c;
}

}  // namespace

AccelResult accelerate0to100(const RunConfig& cfgIn) {
    const RunConfig cfg = withAutoShift(cfgIn);
    Sim sim(cfg);
    AccelResult r;
    DriverInput in;
    in.throttle = 1.0;
    Real t = 0;
    while (t < 40.0) {
        sim.step(in);
        t += sim.dt;
        const auto& tel = sim.vehicle.telemetry();
        for (std::size_t w = 0; w < tel.wheelCount; ++w)
            if (tel.wheels[w].contact && tel.speed > 1.0)
                r.peakDrivenSlip = std::max(r.peakDrivenSlip, tel.wheels[w].slipRatio);
        r.maxTcsCut = std::max(r.maxTcsCut, tel.assists.tcsThrottleCut);
        if (r.time0to60 == 0 && tel.speed >= kmhToMs(60)) r.time0to60 = t;
        if (tel.speed >= kmhToMs(100)) {
            r.reached = true;
            r.time0to100 = t;
            break;
        }
    }
    return r;
}

BrakeResult brake100to0(const RunConfig& cfgIn, Real startKmh) {
    const RunConfig cfg = withAutoShift(cfgIn);
    Sim sim(cfg);
    const Real v0 = kmhToMs(startKmh);
    prepareRolling(sim.vehicle, v0);
    BrakeResult r;
    DriverInput in;
    in.brake = 1.0;
    Real t = 0, dist = 0;
    int absSteps = 0, steps = 0;
    const Vec3 p0 = sim.com();
    while (t < 20.0) {
        sim.step(in);
        t += sim.dt;
        ++steps;
        const auto& tel = sim.vehicle.telemetry();
        r.maxAbsSideslip = std::max(r.maxAbsSideslip, std::abs(tel.sideslip));
        bool abs = false;
        for (std::size_t w = 0; w < tel.wheelCount; ++w) {
            abs = abs || tel.assists.absActive[w];
            if (tel.speed > 2.0 && tel.wheels[w].contact && std::abs(tel.wheels[w].omega) < 0.05) r.anyWheelLocked = true;
        }
        absSteps += abs;
        if (tel.speed < 0.05) {
            r.stopped = true;
            break;
        }
    }
    const Vec3 d = sim.com() - p0;
    dist = std::hypot(d.x, d.y);
    r.distance = dist;
    r.time = t;
    r.meanDecelG = dist > 0 ? v0 * v0 / (2 * dist) / kGravity : 0;
    r.absActiveFraction = steps ? static_cast<Real>(absSteps) / steps : 0;
    return r;
}

SkidpadResult skidpad(const RunConfig& cfgIn, Real radius) {
    const RunConfig cfg = withDirectSteering(cfgIn);
    Sim sim(cfg);
    SkidpadResult res;
    res.radius = radius;
    const Real L = cfg.vehicle.wheelbase();
    const Real maxSteer = maxSteerRef(cfg.vehicle);
    const Vec3 c{0, radius, 0};  // counter-clockwise circle (left turn) starting at the origin heading +X
    Real target = 8.0;
    prepareRolling(sim.vehicle, target);
    Real ayFilt = 0, t = 0, offTrack = 0, rErrInt = 0;
    DriverInput in;
    while (t < 180.0) {
        const Vec3 p = sim.com();
        const auto& tel = sim.vehicle.telemetry();
        const Real speed = tel.speed;
        // Pure-pursuit steering toward a point on the circle.
        const Real ld = std::max(6.0, 0.5 * speed);
        const Real th = std::atan2(p.y - c.y, p.x - c.x) + ld / radius;
        const Vec3 tgt{c.x + radius * std::cos(th), c.y + radius * std::sin(th), 0};
        const Vec3 toT = sim.vehicle.state().orientation.inverseRotate(tgt - p);
        const Real eta = std::atan2(toT.y, toT.x);
        // Pure pursuit alone drifts outward as the car understeers; a PI term
        // on the radius error keeps the measurement on the intended circle.
        const Real rNow = std::hypot(p.x - c.x, p.y - c.y) - radius;
        rErrInt = clamp(rErrInt + rNow * sim.dt, -2.0, 2.0);
        const Real delta = std::atan(2.0 * L * std::sin(eta) / ld) + 0.01 * rNow + 0.01 * rErrInt;
        in.steer = clamp(delta / maxSteer, -1.0, 1.0);
        holdSpeed(in, speed, target);
        sim.step(in);
        t += sim.dt;
        target += 0.15 * sim.dt;  // slow ramp: quasi-steady-state

        const Real rErr = std::hypot(p.x - c.x, p.y - c.y) - radius;
        const Real a = std::min(1.0, sim.dt / 0.5);
        ayFilt += a * (tel.accelBody.y - ayFilt);
        if (std::abs(rErr) < 1.5 && t > 5.0 && ayFilt / kGravity > res.maxLateralG) {
            res.maxLateralG = ayFilt / kGravity;
            res.speedAtMaxKmh = msToKmh(speed);
            res.sideslipAtMax = tel.sideslip;
        }
        offTrack = std::abs(rErr) > 3.0 ? offTrack + sim.dt : 0.0;
        if (offTrack > 1.0) break;
    }
    return res;
}

StepSteerResult stepSteer(const RunConfig& cfgIn, Real speedKmh, Real steerInput) {
    const RunConfig cfg = withDirectSteering(cfgIn);
    Sim sim(cfg);
    const Real v0 = kmhToMs(speedKmh);
    prepareRolling(sim.vehicle, v0);
    DriverInput in;
    for (Real t = 0; t < 2.0; t += sim.dt) {
        holdSpeed(in, sim.vehicle.telemetry().speed, v0);
        sim.step(in);
    }
    in.brake = 0;
    in.steer = steerInput;
    std::vector<Real> yaw;
    for (Real t = 0; t < 4.0; t += sim.dt) {
        sim.step(in);
        yaw.push_back(sim.vehicle.telemetry().yawRate);
    }
    StepSteerResult r;
    const std::size_t lastSec = static_cast<std::size_t>(1.0 / sim.dt);
    Real sum = 0;
    for (std::size_t i = yaw.size() - lastSec; i < yaw.size(); ++i) sum += yaw[i];
    r.steadyYawRate = sum / static_cast<Real>(lastSec);
    r.steadyLateralG = sim.vehicle.telemetry().accelBody.y / kGravity;
    Real t10 = -1, t90 = -1, peak = 0;
    for (std::size_t i = 0; i < yaw.size(); ++i) {
        const Real f = yaw[i] / r.steadyYawRate;
        if (t10 < 0 && f >= 0.1) t10 = static_cast<Real>(i) * sim.dt;
        if (t90 < 0 && f >= 0.9) t90 = static_cast<Real>(i) * sim.dt;
        peak = std::max(peak, f);
    }
    r.yawRateRiseTime = (t10 >= 0 && t90 >= 0) ? t90 - t10 : -1;
    r.overshoot = peak - 1.0;
    return r;
}

SineDwellResult sineWithDwell(const RunConfig& cfgIn, Real amp, Real speedKmh) {
    const RunConfig cfg = withDirectSteering(cfgIn);
    Sim sim(cfg);
    const Real v0 = kmhToMs(speedKmh);
    prepareRolling(sim.vehicle, v0 + 1.0);
    const Real maxSteer = maxSteerRef(cfg.vehicle);
    DriverInput in;
    for (Real t = 0; t < 1.0; t += sim.dt) {
        holdSpeed(in, sim.vehicle.telemetry().speed, v0 + 1.0);
        sim.step(in);
    }
    in.throttle = 0;
    in.brake = 0;
    constexpr Real f = 0.7;
    const Real quarter = 0.25 / f;
    const Real tDwellStart = 3 * quarter;   // second (negative) peak
    const Real tDwellEnd = tDwellStart + 0.5;
    const Real tEnd = tDwellEnd + quarter;  // back to zero
    SineDwellResult r;
    Real t = 0;
    Real firstPeak = 0;
    bool firstPeakDone = false;
    while (t < tEnd + 2.0) {
        Real delta;
        if (t < tDwellStart) delta = amp * std::sin(2 * kPi * f * t);
        else if (t < tDwellEnd) delta = -amp;
        else if (t < tEnd) delta = amp * std::sin(2 * kPi * f * (t - tDwellEnd + tDwellStart));
        else delta = 0;
        in.steer = clamp(delta / maxSteer, -1.0, 1.0);
        sim.step(in);
        t += sim.dt;
        const auto& tel = sim.vehicle.telemetry();
        r.maxSideslip = std::max(r.maxSideslip, std::abs(tel.sideslip));
        if (std::abs(tel.sideslip) > kPi / 4) r.spun = true;
        if (!firstPeakDone) {
            if (tel.yawRate > firstPeak) firstPeak = tel.yawRate;
            else if (t > quarter && tel.yawRate < 0.8 * firstPeak) firstPeakDone = true;
        }
        if (t >= tEnd + 1.0 && r.ratioAt1s == 0) r.ratioAt1s = std::abs(tel.yawRate) / std::max(firstPeak, 1e-6);
        if (t >= tEnd + 1.75 && r.ratioAt175s == 0) r.ratioAt175s = std::abs(tel.yawRate) / std::max(firstPeak, 1e-6);
    }
    r.peakYawRate = firstPeak;
    return r;
}

HoldResult slopeHold(const RunConfig& cfgIn, Real gradePercent, Real seconds, bool brakes) {
    const RunConfig cfg = withAutoShift(cfgIn);
    const Real g = gradePercent / 100.0;
    Sim sim(cfg, [g](Real x, Real) { return g * x; });
    DriverInput in;
    in.brake = brakes ? 1.0 : 0.0;
    if (!brakes) sim.vehicle.assists().setGear(0);
    const Vec3 p0 = sim.com();
    for (Real t = 0; t < seconds; t += sim.dt) sim.step(in);
    return {length(sim.com() - p0)};
}

BumpResult speedBump(const RunConfig& cfgIn, Real speedKmh, Real h, Real len) {
    const RunConfig cfg = withAutoShift(cfgIn);
    constexpr Real x0 = 15.0;
    Sim sim(cfg, [h, len](Real x, Real) {
        if (x < x0 || x > x0 + len) return 0.0;
        const Real s = std::sin(kPi * (x - x0) / len);
        return h * s * s;
    });
    const Real v0 = kmhToMs(speedKmh);
    prepareRolling(sim.vehicle, v0);
    BumpResult r;
    DriverInput in;
    for (Real t = 0; t < 10.0 && sim.com().x < x0 + 20.0; t += sim.dt) {
        holdSpeed(in, sim.vehicle.telemetry().speed, v0);
        sim.step(in);
        const auto& tel = sim.vehicle.telemetry();
        if (!isFinite(tel.position) || !std::isfinite(tel.speed)) {
            r.finite = false;
            break;
        }
        if (sim.com().x < x0 - 3.0) continue;
        bool air = false;
        for (std::size_t w = 0; w < tel.wheelCount; ++w) {
            r.peakCompression = std::max(r.peakCompression, tel.wheels[w].compression);
            air = air || !tel.wheels[w].contact;
        }
        if (air) r.airborneTime += sim.dt;
        r.peakVerticalAccelG = std::max(r.peakVerticalAccelG, std::abs(tel.accelBody.z) / kGravity);
    }
    return r;
}

DropResult dropTest(const RunConfig& cfgIn, Real dropHeight) {
    const RunConfig cfg = withAutoShift(cfgIn);
    Sim sim(cfg);
    const VehicleDefinition& d = cfg.vehicle;
    sim.vehicle.mutableState().position.z += dropHeight;
    sim.vehicle.mutableState().velocity = {};
    DriverInput in;
    in.brake = 1.0;

    auto energy = [&]() {
        const auto& tel = sim.vehicle.telemetry();
        Real e = tel.kineticEnergy + d.mass * kGravity * sim.com().z;
        for (std::size_t w = 0; w < d.wheels.size(); ++w) {
            const auto& s = d.wheels[w].suspension;
            const Real x = tel.wheels[w].compression;
            const Real xs = std::min(x, s.travel);
            e += s.springPreload * xs + 0.5 * s.springRate * xs * xs;
            if (x > s.travel) e += 0.5 * s.bumpStopRate * (x - s.travel) * (x - s.travel);
        }
        for (const auto& ax : d.axles) {
            const Real dx = tel.wheels[ax.leftWheel].compression - tel.wheels[ax.rightWheel].compression;
            e += 0.5 * ax.antiRollBarRate * dx * dx;
        }
        return e;
    };

    DropResult r;
    sim.step(in);
    const Real e0 = energy();
    Real prevZ = sim.com().z, prevDz = 0;
    bool landed = false;
    for (Real t = 0; t < 4.0; t += sim.dt) {
        sim.step(in);
        const auto& tel = sim.vehicle.telemetry();
        if (!isFinite(tel.position)) {
            r.finite = false;
            break;
        }
        bool anyContact = false;
        for (std::size_t w = 0; w < tel.wheelCount; ++w) anyContact = anyContact || tel.wheels[w].contact;
        landed = landed || anyContact;
        if (energy() > e0 + 0.005 * std::abs(e0) + 50.0) r.energyIncreased = true;
        const Real z = sim.com().z;
        const Real dz = z - prevZ;
        if (landed && prevDz > 0 && dz <= 0) r.bouncePeaks.push_back(prevZ);
        prevDz = dz;
        prevZ = z;
    }
    return r;
}

CoastResult coastNoLosses(const RunConfig& cfgIn, Real startKmh) {
    RunConfig cfg = withAutoShift(cfgIn);
    cfg.vehicle.aero = {0, 0, 0, cfg.vehicle.aero.airDensity};
    for (auto& w : cfg.vehicle.wheels) w.tire.rollingResistance = 0;
    Sim sim(cfg);
    const Real v0 = kmhToMs(startKmh);
    sim.vehicle.assists().setGear(0);
    sim.vehicle.setRollingSpeed(v0);
    CoastResult r;
    DriverInput in;
    Real vmax = v0;
    const Real T = 10.0;
    for (Real t = 0; t < T; t += sim.dt) {
        sim.step(in);
        const Real v = std::hypot(sim.vehicle.telemetry().speed, sim.vehicle.telemetry().lateralVelocity);
        if (v > vmax + 0.01) r.speedEverIncreased = true;
        vmax = std::max(vmax, v);
    }
    r.speedLossPerSecond = msToKmh(v0 - sim.vehicle.telemetry().speed) / T;
    return r;
}

PerfResult measureStepCost(const RunConfig& cfgIn, int steps) {
    const RunConfig cfg = withAutoShift(cfgIn);
    RunConfig quiet = cfg;
    quiet.csv = nullptr;
    Sim sim(quiet);
    prepareRolling(sim.vehicle, 20.0);
    DriverInput in;
    in.throttle = 0.3;
    in.steer = 0.05;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < steps; ++i) sim.step(in);
    const auto t1 = std::chrono::steady_clock::now();
    PerfResult r;
    r.microsecondsPerStep = std::chrono::duration<Real, std::micro>(t1 - t0).count() / steps;
    return r;
}

}  // namespace ord::scenarios
