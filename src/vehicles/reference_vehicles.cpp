// OpenRoad Dynamics — reference vehicle definitions.
// SPDX-License-Identifier: MIT
#include "reference_vehicles.hpp"

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

}  // namespace ord::vehicles
