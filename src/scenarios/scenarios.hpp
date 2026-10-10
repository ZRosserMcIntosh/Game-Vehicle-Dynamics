// OpenRoad Dynamics — reproducible benchmark scenarios.
// SPDX-License-Identifier: MIT
//
// Each scenario is a deterministic, scripted manoeuvre on an analytic test
// track, returning objective metrics. The same code drives the unit tests
// (with tolerances) and the `ordbench` report tool.
// Specification: docs/benchmarks/benchmark-plan.md
#pragma once

#include <iosfwd>
#include <string>
#include <vector>

#include "ord/assists.hpp"
#include "ord/tire.hpp"
#include "ord/vehicle.hpp"
#include "ord/vehicle_def.hpp"

namespace ord::scenarios {

struct RunConfig {
    VehicleDefinition vehicle;
    AssistConfig assists;
    Real hz = 360.0;
    SurfaceProperties surface;   // whole test area
    std::ostream* csv = nullptr; // optional per-step telemetry log
};

struct AccelResult {
    bool reached = false;
    Real time0to100 = 0;      // s
    Real time0to60 = 0;       // s (km/h)
    Real peakDrivenSlip = 0;  // max driven-wheel slip ratio observed
    Real maxTcsCut = 0;
};
AccelResult accelerate0to100(const RunConfig& cfg);

struct BrakeResult {
    bool stopped = false;
    Real distance = 0;        // m from 100 km/h
    Real time = 0;            // s
    Real meanDecelG = 0;
    Real maxAbsSideslip = 0;  // rad, stability indicator
    Real absActiveFraction = 0;
    bool anyWheelLocked = false;
};
BrakeResult brake100to0(const RunConfig& cfg, Real startKmh = 100.0);

struct SkidpadResult {
    Real maxLateralG = 0;     // steady lateral acceleration while holding the radius
    Real speedAtMaxKmh = 0;
    Real radius = 0;
    Real sideslipAtMax = 0;   // rad
};
SkidpadResult skidpad(const RunConfig& cfg, Real radius = 40.0);

struct StepSteerResult {
    Real steadyYawRate = 0;   // rad/s
    Real yawRateRiseTime = 0; // s, 10% -> 90%
    Real overshoot = 0;       // fraction of steady value
    Real steadyLateralG = 0;
};
StepSteerResult stepSteer(const RunConfig& cfg, Real speedKmh = 80.0, Real steerInput = 0.08);

// Sine-with-dwell, after the US FMVSS No. 126 ESC test procedure (49 CFR
// 571.126): 0.7 Hz sine steer with a 500 ms dwell at the second peak, from
// 80 km/h with the throttle released. Pass criteria in the regulation: yaw
// rate 1.0 s after steering completion <= 35% of the first yaw-rate peak,
// and <= 20% at 1.75 s. Here the amplitude is given directly as a road-wheel
// angle instead of the regulation's steering-wheel-angle scaling.
struct SineDwellResult {
    Real peakYawRate = 0;       // rad/s, first peak (magnitude)
    Real ratioAt1s = 0;         // |yaw rate| / peak at 1.0 s after steer completion
    Real ratioAt175s = 0;       // at 1.75 s
    Real maxSideslip = 0;       // rad
    bool spun = false;          // |sideslip| exceeded 45 deg
    bool passesFmvss126Criteria() const { return !spun && ratioAt1s <= 0.35 && ratioAt175s <= 0.20; }
};
SineDwellResult sineWithDwell(const RunConfig& cfg, Real roadWheelAmplitude, Real speedKmh = 80.0);

struct HoldResult {
    Real drift = 0;           // m moved over the hold period
};
HoldResult slopeHold(const RunConfig& cfg, Real gradePercent, Real seconds, bool brakes);

struct BumpResult {
    Real peakCompression = 0; // m
    Real peakVerticalAccelG = 0;
    Real airborneTime = 0;    // s with any wheel off the ground
    bool finite = true;
};
BumpResult speedBump(const RunConfig& cfg, Real speedKmh, Real bumpHeight = 0.08, Real bumpLength = 0.5);

struct DropResult {
    std::vector<Real> bouncePeaks;  // successive body-height peaks after landing
    bool energyIncreased = false;
    bool finite = true;
};
DropResult dropTest(const RunConfig& cfg, Real dropHeight = 0.25);

struct CoastResult {
    Real speedLossPerSecond = 0;  // km/h per s, with drag and rolling resistance removed
    bool speedEverIncreased = false;
};
CoastResult coastNoLosses(const RunConfig& cfg, Real startKmh = 100.0);

struct PerfResult {
    Real microsecondsPerStep = 0;
};
PerfResult measureStepCost(const RunConfig& cfg, int steps = 20000);

}  // namespace ord::scenarios
