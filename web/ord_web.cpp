// OpenRoad Dynamics — WebAssembly bindings for the browser proving ground.
// SPDX-License-Identifier: MIT
//
// A thin C ABI over ord::Vehicle so a web page can drive the real core. The
// proving-ground surface (heights and patches) is defined here once and also
// exported, so the page renders exactly the ground the physics sees.
#include <array>
#include <cmath>
#include <memory>

#include "ord/ground.hpp"
#include "ord/presets.hpp"
#include "ord/vehicle.hpp"
#include "reference_vehicles.hpp"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define ORD_EXPORT extern "C" EMSCRIPTEN_KEEPALIVE
#else
#define ORD_EXPORT extern "C"
#endif

using namespace ord;

namespace {

// ---- Proving ground (ISO 8855: x forward from the start line, y left, z up) ----
// Bump lane runs along y = kBumpLaneY: speed bumps, a rumble strip, a pothole,
// and an 8 cm kerb on its left edge. Everything else is flat asphalt.
constexpr Real kBumpLaneY = -110.0;

Real bumpLaneHeight(Real x, Real y) {
    Real h = 0;
    // Three 8 cm cosine speed bumps, 3.6 m long.
    for (Real c : {60.0, 95.0, 130.0}) {
        const Real d = std::abs(x - c);
        if (d < 1.8) h += 0.04 * (1 + std::cos(kPi * d / 1.8));
    }
    // Rumble strip: 2 cm sawtooth ridges every 0.5 m.
    if (x > 160 && x < 200) {
        const Real p = std::fmod(x - 160, 0.5) / 0.5;
        h += 0.02 * (p < 0.5 ? 2 * p : 2 * (1 - p));
    }
    // 6 cm deep pothole, 1.2 m across, under the left wheel track.
    {
        const Real dx = x - 235, dy = y - (kBumpLaneY + 0.8);
        const Real r = std::sqrt(dx * dx + dy * dy);
        if (r < 0.6) h -= 0.06 * 0.5 * (1 + std::cos(kPi * r / 0.6));
    }
    // 8 cm kerb along the left edge of the lane (rounded 10 cm shoulder).
    {
        const Real e = y - (kBumpLaneY + 3.0);
        if (e > 0.1 && e < 0.6 && x > 0 && x < 300) h += 0.08;
        else if (e > 0 && e <= 0.1 && x > 0 && x < 300) h += 0.08 * 0.5 * (1 - std::cos(kPi * e / 0.1));
    }
    return h;
}

Real provingGroundHeight(Real x, Real y) {
    if (y > kBumpLaneY - 4 && y < kBumpLaneY + 4 && x > 40 && x < 300) return bumpLaneHeight(x, y);
    return 0.0;
}

SurfaceProperties wetSurface() {
    SurfaceProperties s;
    s.frictionScale = 0.6;
    s.wetSpeedFalloff = 0.008;
    return s;
}

SurfaceProperties bumpSurface() {
    SurfaceProperties s;
    s.roughness = 1.0;
    return s;
}

struct Sim {
    std::unique_ptr<TestTrackGround> ground;
    std::unique_ptr<Vehicle> vehicle;
    DriverInput input;
    AssistConfig assists;
    Real accumulator = 0;
    int carIndex = 0;
};

Sim g;
constexpr Real kStep = 1.0 / 360.0;
// Frame state exported to JavaScript (see web/README.md for the layout).
constexpr int kHeader = 32, kPerWheel = 16;
std::array<double, kHeader + kPerWheel * kMaxWheels> g_out{};

VehicleDefinition carByIndex(int i) {
    switch (i) {
        case 1: return vehicles::referenceHatchFwd();
        case 2: return vehicles::referenceSaloonAwd();
        case 3: return vehicles::referenceHypercarAwd();
        case 4: return vehicles::referenceRaceDownforce();
        default: return vehicles::referenceSportsRwd();
    }
}

void ensureGround() {
    if (g.ground) return;
    g.ground = std::make_unique<TestTrackGround>(provingGroundHeight);
    g.ground->addPatch({250, 450, 25, 75, wetSurface(), 2});
    g.ground->addPatch({40, 300, kBumpLaneY - 4, kBumpLaneY + 4, bumpSurface(), 3});
}

void publish() {
    const Vehicle& v = *g.vehicle;
    const RigidBodyState& b = v.state();
    const TelemetryFrame& t = v.telemetry();
    const VehicleDefinition& d = v.definition();
    double* o = g_out.data();
    o[0] = b.position.x; o[1] = b.position.y; o[2] = b.position.z;
    o[3] = b.orientation.w; o[4] = b.orientation.x; o[5] = b.orientation.y; o[6] = b.orientation.z;
    o[7] = t.speed; o[8] = t.lateralVelocity; o[9] = t.yawRate; o[10] = t.sideslip;
    o[11] = t.accelBody.x; o[12] = t.accelBody.y; o[13] = t.accelBody.z;
    o[14] = t.drivetrain.engineRpm; o[15] = t.gear; o[16] = t.steerAngle; o[17] = t.throttleOut;
    o[18] = t.assists.tcsThrottleCut; o[19] = t.assists.countersteerAngle; o[20] = t.assists.escYawError;
    o[21] = t.time; o[22] = static_cast<double>(t.wheelCount); o[23] = v.faulted() ? 1 : 0;
    o[24] = t.assists.shifting ? 1 : 0; o[25] = d.wheels.empty() ? 0 : d.wheels[0].tire.radius;
    o[26] = t.clutchEngagement; o[27] = t.steerInput; o[28] = t.throttleInput; o[29] = t.brakeInput;
    o[30] = static_cast<double>(t.groundQueries); o[31] = 0;
    for (std::size_t w = 0; w < t.wheelCount; ++w) {
        const WheelTelemetry& wt = t.wheels[w];
        const WheelDef& wd = d.wheels[w];
        double* p = o + kHeader + kPerWheel * static_cast<int>(w);
        p[0] = wd.suspension.hardpoint.x; p[1] = wd.suspension.hardpoint.y;
        p[2] = wd.suspension.hardpoint.z - (wd.suspension.travel - wt.compression);  // wheel centre z, body frame
        p[3] = wd.tire.radius; p[4] = wt.steerAngle; p[5] = wt.omega;
        p[6] = wt.load; p[7] = wt.slipRatio; p[8] = wt.slipAngle; p[9] = wt.fx; p[10] = wt.fy;
        p[11] = wt.contact ? 1 : 0; p[12] = wt.brakeLocked ? 1 : 0;
        p[13] = t.assists.absActive[w] ? 1 : 0; p[14] = t.assists.escBrakeTorque[w]; p[15] = wt.surfaceId;
    }
}

}  // namespace

// Creates (or replaces) the car and places it at (x, y) heading `yaw`. Returns 1 on success.
ORD_EXPORT int ord_create(int carIndex, double x, double y, double yaw) {
    ensureGround();
    g.carIndex = carIndex;
    g.vehicle = std::make_unique<Vehicle>(carByIndex(carIndex), g.assists);
    if (!g.vehicle->isValid()) return 0;
    g.vehicle->reset({x, y, 1.0}, yaw, *g.ground);
    g.accumulator = 0;
    g.input = {};
    publish();
    return 1;
}

ORD_EXPORT void ord_reset(double x, double y, double yaw) {
    if (!g.vehicle) return;
    g.vehicle->reset({x, y, 1.0}, yaw, *g.ground);
    g.accumulator = 0;
    publish();
}

// preset: 0 FUN, 1 SPORT, 2 SIMULATION. Same physics; only the assists change.
ORD_EXPORT void ord_set_preset(int preset) {
    g.assists = makePreset(preset == 0 ? DrivingPreset::Fun : preset == 1 ? DrivingPreset::Sport : DrivingPreset::Simulation);
    if (g.vehicle) g.vehicle->assists().setConfig(g.assists);
}

// Individual overrides on top of the preset. level: 0 off, 1 sport, 2 full.
ORD_EXPORT void ord_set_aids(int abs, int tcsLevel, int escLevel, int automatic) {
    auto lvl = [](int l) { return l <= 0 ? InterventionLevel::Off : l == 1 ? InterventionLevel::Sport : InterventionLevel::Full; };
    g.assists.abs = abs != 0;
    g.assists.tractionControl = lvl(tcsLevel);
    g.assists.stabilityControl = lvl(escLevel);
    g.assists.shiftMode = automatic ? ShiftMode::Automatic : ShiftMode::Manual;
    if (g.vehicle) g.vehicle->assists().setConfig(g.assists);
}

ORD_EXPORT void ord_set_input(double steer, double throttle, double brake, double handbrake, int shiftUp, int shiftDown) {
    g.input.steer = steer;
    g.input.throttle = throttle;
    g.input.brake = brake;
    g.input.handbrake = handbrake;
    // Edge-triggered: latched until the next fixed step consumes them.
    g.input.shiftUp = g.input.shiftUp || shiftUp != 0;
    g.input.shiftDown = g.input.shiftDown || shiftDown != 0;
}

// Advances real time `dt` (s) in fixed 1/360 s steps. Returns the number of steps taken.
ORD_EXPORT int ord_advance(double dt) {
    if (!g.vehicle || g.vehicle->faulted()) return 0;
    g.accumulator += std::min(dt, 0.1);  // never try to catch up more than 100 ms
    int n = 0;
    while (g.accumulator >= kStep) {
        g.vehicle->step(kStep, g.input, *g.ground);
        g.input.shiftUp = g.input.shiftDown = false;
        g.accumulator -= kStep;
        ++n;
    }
    publish();
    return n;
}

ORD_EXPORT double* ord_state() { return g_out.data(); }
ORD_EXPORT int ord_state_header() { return kHeader; }
ORD_EXPORT int ord_state_per_wheel() { return kPerWheel; }
ORD_EXPORT double ord_ground_height(double x, double y) { return provingGroundHeight(x, y); }
ORD_EXPORT double ord_bump_lane_y() { return kBumpLaneY; }
