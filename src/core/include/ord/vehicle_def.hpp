// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Data-driven vehicle definition. A VehicleDefinition fully describes the
// *physical* car. Driver assists and input interpretation live elsewhere
// (assists.hpp) and can never modify these values: presets change how the
// driver's intent is turned into actuator commands, not the car itself.
#pragma once

#include <string>
#include <vector>

#include "ord/drivetrain.hpp"
#include "ord/math.hpp"
#include "ord/tire.hpp"

namespace ord {

struct SuspensionDef {
    // Wheel-centre position at full bump (maximum compression), body frame,
    // relative to the body origin. The wheel travels along -Z_body from here.
    Vec3 hardpoint;
    Real travel = 0.20;            // m, full droop to full bump
    Real springRate = 35000.0;     // N/m at the wheel
    Real springPreload = 0.0;      // N at full droop
    Real bumpDamping = 3000.0;     // N*s/m, compression
    Real reboundDamping = 4500.0;  // N*s/m, extension
    Real bumpStopRate = 300000.0;  // N/m beyond full bump
};

struct WheelDef {
    SuspensionDef suspension;
    TireParams tire;
    Real maxSteerAngle = 0.0;      // rad at full steering lock (0 = not steered)
    Real staticToe = 0.0;          // rad, + = toe-in on the left wheel convention handled per side
    Real maxBrakeTorque = 2000.0;  // N*m at 100% pedal
    Real handbrakeTorque = 0.0;    // N*m at 100% handbrake
    Real unsprungMass = 40.0;      // kg: wheel, tire, hub, brake, half the links. Part of VehicleDefinition::mass.
    bool isLeft = true;
};

struct AxleDef {
    int leftWheel = 0, rightWheel = 1;
    Real antiRollBarRate = 0.0;    // N/m at the wheel (per metre of travel difference)
};

struct AeroDef {
    Real dragArea = 0.70;          // Cd*A, m^2
    Real frontDownforceArea = 0.0; // Cl*A at the front axle, m^2 (positive = downforce)
    Real rearDownforceArea = 0.0;  // Cl*A at the rear axle
    Real airDensity = 1.225;       // kg/m^3 (sea level, 15 C)
};

struct VehicleDefinition {
    std::string name = "unnamed";
    Real mass = 1400.0;            // kg, total including unsprung masses
    Vec3 centerOfMass{0, 0, 0.0};  // body frame
    Vec3 inertia{500, 2200, 2400}; // kg*m^2 principal moments about the CoM (roll, pitch, yaw)
    std::vector<WheelDef> wheels;  // ordered; axles reference them
    std::vector<AxleDef> axles;    // front to rear
    Real steeringAckermann = 0.6;  // 0 = parallel, 1 = full Ackermann
    DrivetrainDef drivetrain;
    AeroDef aero;

    // Returns an empty string when valid, else a human-readable reason.
    std::string validate() const;

    Real wheelbase() const;
    Real totalUnsprungMass() const;
};

}  // namespace ord
