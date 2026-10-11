// OpenRoad Dynamics — reference vehicle definitions.
// SPDX-License-Identifier: MIT
#include "reference_vehicles.hpp"

#include "ord/tire_generator.hpp"

namespace ord::vehicles {

namespace {

struct Layout {
    Real frontAxleX, rearAxleX, halfTrackFront, halfTrackRear;
    Real hardpointZ;  // wheel-centre height at full bump relative to the CoM
};

void addFourWheels(VehicleDefinition& d, const Layout& l, const WheelDef& frontTemplate, const WheelDef& rearTemplate) {
    auto place = [&](WheelDef w, Real x, Real y, bool left) {
        w.suspension.hardpoint = {x, y, l.hardpointZ};
        w.isLeft = left;
        d.wheels.push_back(w);
    };
    place(frontTemplate, l.frontAxleX, l.halfTrackFront, true);    // 0 FL
    place(frontTemplate, l.frontAxleX, -l.halfTrackFront, false);  // 1 FR
    place(rearTemplate, l.rearAxleX, l.halfTrackRear, true);       // 2 RL
    place(rearTemplate, l.rearAxleX, -l.halfTrackRear, false);     // 3 RR
}

}  // namespace

VehicleDefinition referenceSportsRwd() {
    VehicleDefinition d;
    d.name = "ORD Reference Sports RWD";
    d.mass = 1450;
    d.inertia = {520, 2050, 2250};

    WheelDef f;
    f.tire.radius = 0.33;
    f.tire.nominalLoad = 3600;
    f.tire.muX = 1.15;
    f.tire.muY = 1.10;
    f.suspension.travel = 0.18;
    f.suspension.springRate = 45000;
    f.suspension.bumpDamping = 3200;
    f.suspension.reboundDamping = 4800;
    f.maxSteerAngle = 0.55;
    f.maxBrakeTorque = 2800;

    WheelDef r = f;
    r.tire.radius = 0.335;
    r.tire.muX = 1.18;
    r.tire.muY = 1.12;
    r.suspension.springRate = 50000;
    r.maxSteerAngle = 0;
    r.maxBrakeTorque = 1600;
    r.handbrakeTorque = 2500;

    addFourWheels(d, {1.25, -1.30, 0.80, 0.80, -0.05}, f, r);
    d.axles = {{0, 1, 22000}, {2, 3, 12000}};
    d.steeringAckermann = 0.5;

    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{800, 260}, {1000, 300}, {2000, 480}, {3000, 520}, {5000, 520}, {6500, 440}, {7200, 380}};
    e.idleRpm = 850;
    e.limiterRpm = 7200;
    e.inertia = 0.18;
    d.drivetrain.clutch.maxTorque = 800;
    d.drivetrain.gearbox.forwardRatios = {3.30, 2.20, 1.60, 1.25, 1.00, 0.82};
    d.drivetrain.gearbox.reverseRatio = 3.2;
    d.drivetrain.gearbox.shiftTime = 0.15;
    d.drivetrain.gearbox.kind = TransmissionKind::DualClutch;
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.torqueShare = 1.0;
    rear.differential.type = DifferentialType::ClutchLsd;
    rear.differential.finalDrive = 3.5;
    d.drivetrain.drivenAxles = {rear};

    d.aero.dragArea = 0.68;
    d.aero.frontDownforceArea = 0.10;
    d.aero.rearDownforceArea = 0.15;
    return d;
}

VehicleDefinition referenceHatchFwd() {
    VehicleDefinition d;
    d.name = "ORD Reference Hatch FWD";
    d.mass = 1250;
    d.inertia = {420, 1600, 1750};
    d.centerOfMass = {0.0, 0, 0};

    WheelDef f;
    f.tire.radius = 0.31;
    f.tire.nominalLoad = 3500;
    f.tire.muX = 1.05;
    f.tire.muY = 1.00;
    f.tire.cornerStiffness = 14.0;
    f.suspension.travel = 0.20;
    f.suspension.springRate = 30000;
    f.suspension.bumpDamping = 2400;
    f.suspension.reboundDamping = 3400;
    f.maxSteerAngle = 0.60;
    f.maxBrakeTorque = 2000;

    WheelDef r = f;
    r.suspension.springRate = 26000;
    r.maxSteerAngle = 0;
    r.maxBrakeTorque = 900;
    r.handbrakeTorque = 1500;

    // 60/40 static weight distribution: CoM 1.02 m behind the front axle (L = 2.55 m).
    addFourWheels(d, {1.02, -1.53, 0.76, 0.75, -0.07}, f, r);
    d.axles = {{0, 1, 15000}, {2, 3, 8000}};

    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{800, 100}, {1000, 120}, {2000, 170}, {4000, 180}, {5500, 175}, {6300, 160}, {6600, 140}};
    e.idleRpm = 800;
    e.limiterRpm = 6500;
    e.inertia = 0.14;
    d.drivetrain.clutch.maxTorque = 320;
    d.drivetrain.gearbox.forwardRatios = {3.50, 2.00, 1.35, 1.00, 0.80};
    d.drivetrain.gearbox.reverseRatio = 3.3;
    d.drivetrain.gearbox.shiftTime = 0.35;
    d.drivetrain.gearbox.kind = TransmissionKind::ManualHPattern;
    DrivenAxle front;
    front.axleIndex = 0;
    front.torqueShare = 1.0;
    front.differential.type = DifferentialType::Open;
    front.differential.finalDrive = 4.1;
    d.drivetrain.drivenAxles = {front};

    d.aero.dragArea = 0.65;
    return d;
}

VehicleDefinition referenceSaloonAwd() {
    VehicleDefinition d = referenceSportsRwd();
    d.name = "ORD Reference Saloon AWD";
    d.mass = 1600;
    d.inertia = {600, 2400, 2600};
    for (auto& w : d.wheels) {
        w.tire.nominalLoad = 4000;
        w.suspension.springRate *= 1.1;
    }
    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{800, 280}, {1000, 320}, {2000, 500}, {4500, 500}, {6000, 440}, {6800, 380}};
    e.limiterRpm = 6800;
    DrivenAxle front;
    front.axleIndex = 0;
    front.torqueShare = 0.4;
    front.differential.type = DifferentialType::Open;
    front.differential.finalDrive = 3.6;
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.torqueShare = 0.6;
    rear.differential.type = DifferentialType::ClutchLsd;
    rear.differential.finalDrive = 3.6;
    d.drivetrain.drivenAxles = {front, rear};
    d.drivetrain.center = CenterCoupling::Open;
    d.aero.frontDownforceArea = 0.0;
    d.aero.rearDownforceArea = 0.0;
    d.aero.dragArea = 0.72;
    return d;
}

VehicleDefinition referenceRaceDownforce() {
    VehicleDefinition d;
    d.name = "ORD Reference Race Downforce";
    d.mass = 750;
    d.inertia = {120, 900, 950};
    d.centerOfMass = {0, 0, 0};
    TireSpec fs{305, 40, 13, TireCategory::Slick, 2400};  // generic 13-inch slick sizes
    TireSpec rs{405, 40, 13, TireCategory::Slick, 3000};
    WheelDef f;
    f.tire = generateTireParams(fs);
    f.unsprungMass = 14;
    f.suspension.travel = 0.06;
    f.suspension.springRate = 160000;
    f.suspension.bumpDamping = 5500;
    f.suspension.reboundDamping = 8000;
    f.suspension.bumpStopRate = 900000;
    f.maxSteerAngle = 0.35;
    f.maxBrakeTorque = 3200;
    WheelDef r = f;
    r.tire = generateTireParams(rs);
    r.unsprungMass = 16;
    r.suspension.springRate = 180000;
    r.maxSteerAngle = 0;
    r.maxBrakeTorque = 2400;
    // Wheelbase 3.0 m, 45/55 static split; hardpoints low (ride height ~3 cm at the stops).
    addFourWheels(d, {1.65, -1.35, 0.80, 0.78, 0.02}, f, r);
    d.axles = {{0, 1, 90000}, {2, 3, 60000}};
    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{3000, 250}, {6000, 330}, {9000, 380}, {11500, 375}, {12500, 340}};
    e.idleRpm = 3000;
    e.limiterRpm = 12500;
    e.inertia = 0.06;
    d.drivetrain.clutch.maxTorque = 900;
    d.drivetrain.gearbox.forwardRatios = {2.9, 2.2, 1.8, 1.5, 1.28, 1.12, 1.0, 0.9};
    d.drivetrain.gearbox.shiftTime = 0.05;
    d.drivetrain.gearbox.kind = TransmissionKind::Sequential;
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.differential.type = DifferentialType::ClutchLsd;
    rear.differential.finalDrive = 3.4;
    d.drivetrain.drivenAxles = {rear};
    d.aero.dragArea = 1.10;
    d.aero.frontDownforceArea = 1.55;
    d.aero.rearDownforceArea = 2.05;
    return d;
}

VehicleDefinition referenceHypercarAwd() {
    VehicleDefinition d = referenceSportsRwd();
    d.name = "ORD Reference Hypercar AWD";
    d.mass = 1500;
    d.inertia = {550, 2100, 2300};
    TireSpec fs{285, 30, 20, TireCategory::UltraHighPerformance, 3500};
    TireSpec rs{355, 25, 21, TireCategory::UltraHighPerformance, 3900};
    for (std::size_t w = 0; w < 4; ++w) {
        d.wheels[w].tire = generateTireParams(w < 2 ? fs : rs);
        d.wheels[w].suspension.springRate = w < 2 ? 70000 : 80000;
        d.wheels[w].maxBrakeTorque = w < 2 ? 4500 : 3000;
    }
    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{1000, 700}, {2500, 1300}, {4000, 1500}, {6500, 1500}, {7800, 1250}, {8500, 1100}};
    e.idleRpm = 1000;
    e.limiterRpm = 8500;
    e.inertia = 0.25;
    d.drivetrain.clutch.maxTorque = 2600;
    d.drivetrain.gearbox.forwardRatios = {3.0, 2.15, 1.65, 1.32, 1.08, 0.9, 0.76};
    d.drivetrain.gearbox.shiftTime = 0.08;
    DrivenAxle front;
    front.axleIndex = 0;
    front.torqueShare = 0.35;
    front.differential.finalDrive = 2.9;
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.torqueShare = 0.65;
    rear.differential.type = DifferentialType::ClutchLsd;
    rear.differential.finalDrive = 2.9;
    d.drivetrain.drivenAxles = {front, rear};
    d.aero.dragArea = 0.62;
    d.aero.frontDownforceArea = 0.25;
    d.aero.rearDownforceArea = 0.35;
    return d;
}

}  // namespace ord::vehicles
