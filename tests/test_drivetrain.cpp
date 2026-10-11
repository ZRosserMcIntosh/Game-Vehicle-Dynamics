// OpenRoad Dynamics — drivetrain solver tests.
// SPDX-License-Identifier: MIT
#include "ord/drivetrain.hpp"
#include "ord_test.hpp"

using namespace ord;

namespace {

DrivetrainDef rwd(DifferentialType diff) {
    DrivetrainDef d;
    d.engine.torqueCurve = {{0, 300}, {8000, 300}};
    d.engine.frictionTorque = 0;
    d.engine.frictionPerRadS = 0;
    d.engine.idleRpm = 0;
    d.engine.limiterRpm = 1e6;
    d.gearbox.forwardRatios = {2.0};
    d.gearbox.efficiency = 1.0;
    DrivenAxle a;
    a.axleIndex = 1;
    a.differential.type = diff;
    a.differential.finalDrive = 3.0;
    d.drivenAxles = {a};
    d.clutch.maxTorque = 1e5;
    return d;
}

const std::array<std::array<int, 2>, kMaxAxles> kAxles = {{{0, 1}, {2, 3}}};

// Wheels lifted (no tire forces): free inertias.
std::array<WheelSolveInput, kMaxWheels> airborne() {
    std::array<WheelSolveInput, kMaxWheels> w{};
    for (auto& x : w) { x.inertia = 1.0; x.radius = 0.3; }
    return w;
}

}  // namespace

TEST_CASE("drivetrain: open diff splits torque equally; lighter wheel spins up faster") {
    Drivetrain dt(rwd(DifferentialType::Open), 4, kAxles);
    dt.setEngineOmega(0);
    auto in = airborne();
    in[3].inertia = 4.0;  // heavier right-rear
    std::array<WheelSolveOutput, kMaxWheels> out{};
    DrivetrainCommand cmd{1.0, 1.0, 1};
    for (int i = 0; i < 100; ++i) {
        for (int w = 0; w < 4; ++w) in[w].kappa0 = 0, in[w].contactVx = dt.wheelOmega(w) * 0.3;
        dt.step(1.0 / 360, cmd, in.data(), out.data());
    }
    // Equal torque, inertia 1 vs 4 -> angular acceleration ratio 4 (engine/gearbox inertia shared).
    const Real wl = dt.wheelOmega(2), wr = dt.wheelOmega(3);
    CHECK(wl > 0 && wr > 0);
    CHECK_NEAR(wl / wr, 4.0, 0.05);
    // Kinematics: engine = ratio * mean(wheels).
    CHECK_NEAR(dt.engineOmega(), 6.0 * 0.5 * (wl + wr), 1e-3 * dt.engineOmega());
    CHECK_NEAR(dt.wheelOmega(0), 0.0, 1e-9);  // undriven front untouched
}

TEST_CASE("drivetrain: locked diff keeps both wheels at the same speed") {
    Drivetrain dt(rwd(DifferentialType::Locked), 4, kAxles);
    dt.setEngineOmega(0);
    auto in = airborne();
    in[3].inertia = 4.0;
    std::array<WheelSolveOutput, kMaxWheels> out{};
    DrivetrainCommand cmd{1.0, 1.0, 1};
    for (int i = 0; i < 100; ++i) dt.step(1.0 / 360, cmd, in.data(), out.data());
    CHECK_NEAR(dt.wheelOmega(2), dt.wheelOmega(3), 1e-3);
    CHECK(dt.wheelOmega(2) > 1.0);
}

TEST_CASE("drivetrain: energy check - engine work equals rotational kinetic energy (lossless)") {
    DrivetrainDef def = rwd(DifferentialType::Open);
    Drivetrain dt(def, 4, kAxles);
    dt.setEngineOmega(0);
    auto in = airborne();
    std::array<WheelSolveOutput, kMaxWheels> out{};
    DrivetrainCommand cmd{1.0, 1.0, 1};
    Real work = 0;
    const Real h = 1.0 / 1000;
    for (int i = 0; i < 1000; ++i) {
        const Real w0 = dt.engineOmega();
        dt.step(h, cmd, in.data(), out.data());
        work += 300.0 * 0.5 * (w0 + dt.engineOmega()) * h;
    }
    const Real ke = 0.5 * def.engine.inertia * dt.engineOmega() * dt.engineOmega() +
                    0.5 * def.gearbox.inputInertia * dt.engineOmega() * dt.engineOmega() +
                    0.5 * (dt.wheelOmega(2) * dt.wheelOmega(2) + dt.wheelOmega(3) * dt.wheelOmega(3));
    // Backward Euler dissipates slightly; require agreement within 1%.
    CHECK_NEAR(ke / work, 1.0, 0.01);
}

TEST_CASE("drivetrain: brakes hold a wheel exactly at standstill against engine torque below capacity") {
    Drivetrain dt(rwd(DifferentialType::Open), 4, kAxles);
    dt.setEngineOmega(0);
    auto in = airborne();
    for (auto& w : in) w.brakeCapacity = 5000;  // > 300*6/2 per wheel
    std::array<WheelSolveOutput, kMaxWheels> out{};
    DrivetrainCommand cmd{1.0, 1.0, 1};
    for (int i = 0; i < 200; ++i) dt.step(1.0 / 360, cmd, in.data(), out.data());
    CHECK_NEAR(dt.wheelOmega(2), 0.0, 1e-3);
    CHECK_NEAR(dt.wheelOmega(3), 0.0, 1e-3);
    CHECK(out[2].brakeLocked);
}

TEST_CASE("drivetrain: clutch slips when torque exceeds capacity and locks when speeds match") {
    DrivetrainDef def = rwd(DifferentialType::Open);
    def.clutch.maxTorque = 100;
    Drivetrain dt(def, 4, kAxles);
    dt.setEngineOmega(0);
    auto in = airborne();
    // Heavy driven wheels: holding the clutch locked would need ~225 N*m (> 100 capacity).
    in[2].inertia = in[3].inertia = 10.0;
    std::array<WheelSolveOutput, kMaxWheels> out{};
    // 300 N*m engine vs 100 N*m clutch: engine runs away from the gearbox.
    DrivetrainCommand cmd{1.0, 1.0, 1};
    for (int i = 0; i < 50; ++i) dt.step(1.0 / 360, cmd, in.data(), out.data());
    CHECK(!dt.telemetry().clutchLocked);
    CHECK(dt.telemetry().clutchSlip > 1.0);
    // Close the throttle: the friction torque brings the shafts together and it locks.
    cmd.throttle = 0;
    for (int i = 0; i < 2000; ++i) dt.step(1.0 / 360, cmd, in.data(), out.data());
    CHECK(dt.telemetry().clutchLocked);
    CHECK_NEAR(dt.telemetry().clutchSlip, 0.0, 1e-2);
}

TEST_CASE("drivetrain: wheel-tire coupling stays stable at 60 Hz with very stiff slip") {
    Drivetrain dt(rwd(DifferentialType::Open), 4, kAxles);
    dt.setEngineOmega(0);
    std::array<WheelSolveOutput, kMaxWheels> out{};
    DrivetrainCommand cmd{1.0, 1.0, 1};
    auto in = airborne();
    Real kappa[4] = {0, 0, 0, 0};
    for (int i = 0; i < 600; ++i) {
        for (int w = 0; w < 4; ++w) {
            // Linear tire, Kx = 1e5 N, ground fixed at vx = 0.5 m/s (near standstill: stiffest case).
            in[w].contactVx = 0.5;
            in[w].kappa0 = kappa[w];
            in[w].fx0 = 1e5 * kappa[w];
            in[w].dFxdKappa = 1e5;
            in[w].relaxationLength = 0.15;
        }
        dt.step(1.0 / 60, cmd, in.data(), out.data());
        for (int w = 0; w < 4; ++w) kappa[w] = out[w].kappa;
    }
    // Steady state: torque at each rear wheel = 300*6/2 = 900 N*m -> Fx = 900/0.3 = 3000 N.
    CHECK_NEAR(out[2].fx, 3000.0, 30.0);
    CHECK_NEAR(out[3].fx, 3000.0, 30.0);
    CHECK(std::isfinite(dt.engineOmega()));
}
