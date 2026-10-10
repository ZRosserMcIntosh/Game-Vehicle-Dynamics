// OpenRoad Dynamics — benchmark report tool.
// SPDX-License-Identifier: MIT
//
// Usage: ordbench [--csv-dir DIR] [--markdown]
// Runs the benchmark suite for every reference vehicle and preset and prints
// a Markdown report (the source of docs/benchmarks/baseline-results.md).
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "ord/presets.hpp"
#include "reference_vehicles.hpp"
#include "scenarios.hpp"

using namespace ord;
using namespace ord::scenarios;

namespace {

struct Car {
    const char* id;
    VehicleDefinition def;
};

void printVehicleSection(const Car& car) {
    std::cout << "\n## " << car.def.name << "\n\n";
    std::cout << "| Preset | 0-100 km/h (s) | 100-0 km/h (m) | Brake mean decel (g) | Skidpad R40 max lat (g) | Step steer 80 km/h yaw rise (s) | Peak driven slip | Max TCS cut |\n";
    std::cout << "|---|---|---|---|---|---|---|---|\n";
    for (DrivingPreset p : {DrivingPreset::Fun, DrivingPreset::Sport, DrivingPreset::Simulation}) {
        RunConfig cfg;
        cfg.vehicle = car.def;
        cfg.assists = makePreset(p);
        const AccelResult a = accelerate0to100(cfg);
        const BrakeResult b = brake100to0(cfg);
        const SkidpadResult s = skidpad(cfg);
        const StepSteerResult st = stepSteer(cfg);
        std::cout << std::fixed << std::setprecision(2) << "| " << presetName(p) << " | "
                  << (a.reached ? std::to_string(a.time0to100).substr(0, 5) : std::string("n/a")) << " | "
                  << b.distance << " | " << b.meanDecelG << " | " << s.maxLateralG << " | " << st.yawRateRiseTime
                  << " | " << a.peakDrivenSlip << " | " << a.maxTcsCut << " |\n";
    }
}

void printWet(const Car& car) {
    std::cout << "\n### Wet vs dry braking (" << car.def.name << ", ABS on vs off)\n\n";
    std::cout << "| Surface | ABS | 100-0 (m) | Mean decel (g) | Max sideslip (deg) | ABS active % |\n|---|---|---|---|---|---|\n";
    for (int wet = 0; wet < 2; ++wet) {
        for (int abs = 1; abs >= 0; --abs) {
            RunConfig cfg;
            cfg.vehicle = car.def;
            cfg.assists = makePreset(DrivingPreset::Simulation);
            cfg.assists.abs = abs != 0;
            if (wet) {
                cfg.surface.frictionScale = 0.75;
                cfg.surface.wetSpeedFalloff = 0.004;
            }
            const BrakeResult b = brake100to0(cfg);
            std::cout << std::fixed << std::setprecision(2) << "| " << (wet ? "wet (placeholder model)" : "dry") << " | "
                      << (abs ? "on" : "off") << " | " << b.distance << " | " << b.meanDecelG << " | "
                      << b.maxAbsSideslip * 180 / kPi << " | " << b.absActiveFraction * 100 << " |\n";
        }
    }
}

void printTimestep(const Car& car) {
    std::cout << "\n### Timestep sensitivity (" << car.def.name << ", SIMULATION preset)\n\n";
    std::cout << "| Rate (Hz) | 0-100 (s) | 100-0 (m) | Skidpad (g) | Step-steer yaw (rad/s) | Cost (us/step) |\n|---|---|---|---|---|---|\n";
    for (Real hz : {60.0, 120.0, 240.0, 360.0, 480.0, 1000.0}) {
        RunConfig cfg;
        cfg.vehicle = car.def;
        cfg.assists = makePreset(DrivingPreset::Simulation);
        cfg.hz = hz;
        const AccelResult a = accelerate0to100(cfg);
        const BrakeResult b = brake100to0(cfg);
        const SkidpadResult s = skidpad(cfg);
        const StepSteerResult st = stepSteer(cfg);
        const PerfResult perf = measureStepCost(cfg);
        std::cout << std::fixed << std::setprecision(3) << "| " << hz << " | " << a.time0to100 << " | " << b.distance
                  << " | " << s.maxLateralG << " | " << st.steadyYawRate << " | " << perf.microsecondsPerStep << " |\n";
    }
}

void printSineDwell(const Car& car) {
    std::cout << "\n### Sine-with-dwell, FMVSS 126-style (" << car.def.name << ", 80 km/h)\n\n";
    std::cout << "| Road-wheel amplitude (deg) | ESC | Peak yaw (rad/s) | Ratio @1.0 s | Ratio @1.75 s | Max sideslip (deg) | Meets 126 ratios |\n|---|---|---|---|---|---|---|\n";
    for (Real ampDeg : {3.0, 5.0, 7.0, 9.0}) {
        for (InterventionLevel esc : {InterventionLevel::Off, InterventionLevel::Full}) {
            RunConfig cfg;
            cfg.vehicle = car.def;
            cfg.assists = makePreset(DrivingPreset::Simulation);
            cfg.assists.stabilityControl = esc;
            const SineDwellResult r = sineWithDwell(cfg, ampDeg * kPi / 180);
            std::cout << std::fixed << std::setprecision(2) << "| " << ampDeg << " | "
                      << (esc == InterventionLevel::Off ? "off" : "full") << " | " << r.peakYawRate << " | "
                      << r.ratioAt1s << " | " << r.ratioAt175s << " | " << r.maxSideslip * 180 / kPi << " | "
                      << (r.passesFmvss126Criteria() ? "yes" : "NO") << " |\n";
        }
    }
}

void printStability(const Car& car) {
    RunConfig cfg;
    cfg.vehicle = car.def;
    cfg.assists = makePreset(DrivingPreset::Simulation);
    const DropResult d = dropTest(cfg);
    const CoastResult c = coastNoLosses(cfg);
    const BumpResult bump = speedBump(cfg, 40.0);
    const HoldResult hold = slopeHold(cfg, 20.0, 10.0, true);
    const HoldResult roll = slopeHold(cfg, 20.0, 3.0, false);
    std::cout << "\n### Stability & conservation (" << car.def.name << ")\n\n";
    std::cout << "| Check | Result |\n|---|---|\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "| Drop 0.25 m: energy ever increased | " << (d.energyIncreased ? "YES" : "no") << " |\n";
    std::cout << "| Drop 0.25 m: bounce peaks (m) | ";
    for (std::size_t i = 0; i < std::min<std::size_t>(d.bouncePeaks.size(), 4); ++i) std::cout << d.bouncePeaks[i] << " ";
    std::cout << "|\n";
    std::cout << "| Coast 100 km/h, no drag/rolling: loss (km/h per s) | " << c.speedLossPerSecond << " |\n";
    std::cout << "| Coast: speed ever increased | " << (c.speedEverIncreased ? "YES" : "no") << " |\n";
    std::cout << "| Speed bump 8 cm @ 40 km/h: peak compression (m) | " << bump.peakCompression << " |\n";
    std::cout << "| Speed bump: peak vertical accel (g) | " << bump.peakVerticalAccelG << " |\n";
    std::cout << "| Speed bump: airborne time (s) | " << bump.airborneTime << " |\n";
    std::cout << "| 20% grade, brakes held 10 s: drift (m) | " << hold.drift << " |\n";
    std::cout << "| 20% grade, neutral, no brakes 3 s: roll-back (m) | " << roll.drift << " |\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string csvDir;
    std::string only;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--csv-dir") && i + 1 < argc) csvDir = argv[++i];
        else if (!std::strcmp(argv[i], "--only") && i + 1 < argc) only = argv[++i];
    }
    const Car cars[] = {{"sports", vehicles::referenceSportsRwd()},
                        {"hatch", vehicles::referenceHatchFwd()},
                        {"awd", vehicles::referenceSaloonAwd()}};

    if (!csvDir.empty()) {
        // Telemetry traces for plotting: one file per scenario for the sports car.
        RunConfig cfg;
        cfg.vehicle = cars[0].def;
        cfg.assists = makePreset(DrivingPreset::Simulation);
        std::ofstream a(csvDir + "/accel_sports_sim.csv"); cfg.csv = &a; accelerate0to100(cfg);
        std::ofstream b(csvDir + "/brake_sports_sim.csv"); cfg.csv = &b; brake100to0(cfg);
        std::ofstream s(csvDir + "/skidpad_sports_sim.csv"); cfg.csv = &s; skidpad(cfg);
        std::cout << "CSV traces written to " << csvDir << "\n";
        return 0;
    }

    std::cout << "# OpenRoad Dynamics benchmark report\n\n";
    std::cout << "Fictional reference vehicles; flat dry asphalt (frictionScale 1.0) unless noted; 360 Hz unless noted.\n";
    for (const Car& c : cars) {
        if (!only.empty() && only != c.id) continue;
        printVehicleSection(c);
    }
    if (only.empty() || only == "sports") {
        printWet(cars[0]);
        printSineDwell(cars[0]);
        printTimestep(cars[0]);
    }
    for (const Car& c : cars) {
        if (!only.empty() && only != c.id) continue;
        printStability(c);
    }
    return 0;
}
