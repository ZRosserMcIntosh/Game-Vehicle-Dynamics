// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/vehicle.hpp"

#include <stdexcept>

namespace ord {

namespace {
constexpr Real kRayStartOffset = 0.15;  // m above the full-bump hardpoint, so deep penetrations still hit
}

Vehicle::Vehicle(const VehicleDefinition& def, const AssistConfig& assists) : def_(def), assists_(assists) {
    const std::string err = def_.validate();
    if (!err.empty()) throw std::invalid_argument("VehicleDefinition invalid: " + err);

    for (const WheelDef& w : def_.wheels) tires_.emplace_back(w.tire);

    std::array<std::array<int, 2>, kMaxAxles> axleWheels{};
    for (std::size_t a = 0; a < def_.axles.size(); ++a) {
        axleWheels[a] = {def_.axles[a].leftWheel, def_.axles[a].rightWheel};
        front_[static_cast<std::size_t>(def_.axles[a].leftWheel)] = a == 0;
        front_[static_cast<std::size_t>(def_.axles[a].rightWheel)] = a == 0;
    }
    for (const auto& da : def_.drivetrain.drivenAxles) {
        driven_[static_cast<std::size_t>(def_.axles[da.axleIndex].leftWheel)] = true;
        driven_[static_cast<std::size_t>(def_.axles[da.axleIndex].rightWheel)] = true;
    }
    drivetrain_ = Drivetrain(def_.drivetrain, def_.wheels.size(), axleWheels);

    inertiaBody_ = def_.inertia;
    invInertiaBody_ = Mat3::diagonal({1.0 / def_.inertia.x, 1.0 / def_.inertia.y, 1.0 / def_.inertia.z});
    telem_.wheelCount = def_.wheels.size();
}

Real Vehicle::yawRate() const { return body_.orientation.inverseRotate(body_.angularVelocity).z; }

Real Vehicle::wheelSteerAngle(std::size_t w, Real ref) const {
    Real maxRef = 0;
    for (const WheelDef& wd : def_.wheels) maxRef = std::max(maxRef, wd.maxSteerAngle);
    const WheelDef& wd = def_.wheels[w];
    if (maxRef <= 0 || wd.maxSteerAngle <= 0) return wd.isLeft ? wd.staticToe * -1.0 : wd.staticToe;
    Real delta = ref * (wd.maxSteerAngle / maxRef);
    if (std::abs(delta) > 1e-6 && front_[w] && def_.steeringAckermann > 0 && def_.axles.size() >= 2) {
        const Real L = def_.wheelbase();
        const Real halfTrack = std::abs(wd.suspension.hardpoint.y);
        const Real R = L / std::tan(std::abs(delta));
        const bool inner = (delta > 0) == wd.isLeft;
        const Real ideal = std::atan(L / std::max(R + (inner ? -halfTrack : halfTrack), 0.1));
        delta = sign(delta) * lerp(std::abs(delta), ideal, def_.steeringAckermann);
    }
    // Toe-in convention: positive static toe points both wheels toward the centreline.
    return delta + (wd.isLeft ? -wd.staticToe : wd.staticToe);
}

VehicleSensors Vehicle::buildSensors(Real dt) const {
    VehicleSensors s;
    s.dt = dt;
    const Vec3 vb = bodyVelocity();
    s.speed = vb.x;
    s.lateralVelocity = vb.y;
    s.yawRate = yawRate();
    s.wheelbase = def_.wheelbase();
    s.cgToFrontAxle = def_.wheels[def_.axles.front().leftWheel].suspension.hardpoint.x - def_.centerOfMass.x;
    s.maxSteerAngle = 0;
    for (const WheelDef& wd : def_.wheels) s.maxSteerAngle = std::max(s.maxSteerAngle, wd.maxSteerAngle);
    s.wheelCount = def_.wheels.size();
    Real muSum = 0;
    int muCount = 0;
    for (std::size_t w = 0; w < s.wheelCount; ++w) {
        const WheelState& ws = wheels_[w];
        const WheelDef& wd = def_.wheels[w];
        s.wheelOmega[w] = drivetrain_.wheelOmega(w);
        s.wheelRadius[w] = wd.tire.radius;
        s.wheelGroundSpeed[w] = ws.contactVx;
        s.wheelDriven[w] = driven_[w];
        s.wheelIsFront[w] = front_[w];
        s.wheelIsLeft[w] = wd.isLeft;
        s.wheelInContact[w] = ws.contact;
        s.maxBrakeTorque[w] = wd.maxBrakeTorque;
        s.handbrakeTorque[w] = wd.handbrakeTorque;
        if (ws.contact) {
            muSum += wd.tire.muY * ws.hit.surface.frictionScale;
            ++muCount;
        }
    }
    s.frictionEstimate = muCount ? muSum / muCount : 1.0;
    s.engineRpm = radSToRpm(drivetrain_.engineOmega());
    s.idleRpm = def_.drivetrain.engine.idleRpm;
    s.limiterRpm = def_.drivetrain.engine.limiterRpm;
    s.gear = assists_.currentGear();
    s.forwardGears = drivetrain_.forwardGearCount();
    s.shiftTime = def_.drivetrain.gearbox.shiftTime;
    return s;
}

void Vehicle::setRollingSpeed(Real v) {
    body_.velocity = body_.orientation.rotate({v, 0, 0});
    body_.angularVelocity = {};
    for (std::size_t w = 0; w < def_.wheels.size(); ++w) {
        drivetrain_.setWheelOmega(w, v / def_.wheels[w].tire.radius);
        wheels_[w].slip = {};
        wheels_[w].contactVx = v;
    }
    const Real idle = rpmToRadS(def_.drivetrain.engine.idleRpm);
    drivetrain_.setEngineOmega(std::max(idle, drivetrain_.engineOmegaForGear(assists_.currentGear())));
    lastVelocity_ = body_.velocity;
}

void Vehicle::reset(const Vec3& position, Real yaw, const GroundQuery& ground) {
    body_ = RigidBodyState{};
    body_.position = position;
    body_.orientation = Quat::fromAxisAngle({0, 0, 1}, yaw);
    for (auto& w : wheels_) w = WheelState{};
    assists_.setGear(1);
    setRollingSpeed(0);
    // Settle onto the suspension with the brakes held.
    DriverInput hold;
    hold.brake = 1.0;
    const Real dt = 1.0 / 240.0;
    for (int i = 0; i < 720; ++i) step(dt, hold, ground);
    body_.velocity = {};
    body_.angularVelocity = {};
    setRollingSpeed(0);
    time_ = 0;
}

void Vehicle::step(Real dt, const DriverInput& input, const GroundQuery& ground) {
    const std::size_t nw = def_.wheels.size();
    const ActuatorCommands cmd = assists_.process(input, buildSensors(dt));

    const Quat& q = body_.orientation;
    const Vec3 up = q.rotate({0, 0, 1});
    const Vec3 down = -up;
    const Vec3 comWorld = body_.position + q.rotate(def_.centerOfMass);

    // ---- 1. Suspension contact, spring/damper loads, contact kinematics ----
    for (std::size_t w = 0; w < nw; ++w) {
        WheelState& ws = wheels_[w];
        const WheelDef& wd = def_.wheels[w];
        const SuspensionDef& sd = wd.suspension;
        const Real r = wd.tire.radius;
        const Vec3 hp = body_.position + q.rotate(sd.hardpoint);
        const Vec3 origin = hp + up * kRayStartOffset;
        ws.prevCompression = ws.compression;
        ws.hit = ground.raycast(origin, down, kRayStartOffset + sd.travel + r);
        ws.contact = ws.hit.hit;
        ws.compression = ws.contact ? sd.travel - (ws.hit.distance - kRayStartOffset - r) : 0.0;
        if (!ws.contact) ws.compression = 0.0;

        ws.steer = wheelSteerAngle(w, cmd.steerAngle);
        const Vec3 headingWorld = q.rotate({std::cos(ws.steer), std::sin(ws.steer), 0});
        const Vec3 n = ws.contact ? ws.hit.normal : up;
        ws.longDir = normalized(headingWorld - n * dot(n, headingWorld));
        ws.latDir = cross(n, ws.longDir);
        const Vec3 p = ws.contact ? ws.hit.point : hp + down * (sd.travel + r);
        const Vec3 vContact = body_.velocity + cross(body_.angularVelocity, p - comWorld);
        ws.contactVx = dot(vContact, ws.longDir);
        ws.contactVy = dot(vContact, ws.latDir);

        const Real x = ws.compression;
        Real f = sd.springPreload + sd.springRate * std::min(x, sd.travel);
        if (x > sd.travel) f += sd.bumpStopRate * (x - sd.travel);
        const Real xdot = (x - ws.prevCompression) / dt;
        f += (xdot > 0 ? sd.bumpDamping : sd.reboundDamping) * xdot;
        ws.springForce = f;
    }
    for (const AxleDef& ax : def_.axles) {
        WheelState& l = wheels_[static_cast<std::size_t>(ax.leftWheel)];
        WheelState& rr = wheels_[static_cast<std::size_t>(ax.rightWheel)];
        const Real farb = ax.antiRollBarRate * (l.compression - rr.compression);
        l.springForce += farb;
        rr.springForce -= farb;
    }

    // ---- 2. Tire loads, lateral transient slip, linearised longitudinal force ----
    std::array<WheelSolveInput, kMaxWheels> solveIn{};
    std::array<WheelSolveOutput, kMaxWheels> solveOut{};
    for (std::size_t w = 0; w < nw; ++w) {
        WheelState& ws = wheels_[w];
        const WheelDef& wd = def_.wheels[w];
        const MagicFormulaTire& tire = tires_[w];
        const Real r = wd.tire.radius;
        ws.load = ws.contact ? std::max(0.0, ws.springForce) : 0.0;
        WheelSolveInput& si = solveIn[w];
        si.radius = r;
        si.inertia = wd.tire.wheelInertia;
        si.relaxationLength = wd.tire.relaxationLengthX;
        si.kappa0 = ws.slip.kappa;
        si.brakeCapacity = cmd.brakeTorque[w];
        const Real omega = drivetrain_.wheelOmega(w);
        if (ws.load > 0) {
            tire.integrateLateralSlip(ws.slip, ws.contactVx, ws.contactVy, dt);
            const Real speed = std::hypot(ws.contactVx, ws.contactVy);
            const TireForces f0 = tire.evaluate(ws.load, ws.slip.kappa, ws.slip.tanAlpha, ws.hit.surface, speed);
            si.contactVx = ws.contactVx;
            si.fx0 = f0.fx;
            si.dFxdKappa = std::max(0.0, f0.dFxdKappa);
            const Real fade = std::max(0.0, 1.0 - speed / wd.tire.lowSpeedDampingFadeSpeed);
            si.slipDamping = wd.tire.lowSpeedDampingPerLoad * ws.load * fade;
            si.externalTorque = -wd.tire.rollingResistance * ws.hit.surface.rollingResistanceScale * ws.load * r *
                                std::tanh(omega * r / 0.5);
        } else {
            // Airborne: slip relaxes toward zero (contact velocity equals rim speed).
            ws.slip.tanAlpha = 0;
            si.contactVx = omega * r;
        }
    }

    // ---- 3. Implicit drivetrain + wheel spin + longitudinal slip ----
    DrivetrainCommand dc;
    dc.throttle = cmd.throttle;
    dc.clutchEngagement = cmd.clutchEngagement;
    dc.gear = cmd.gear;
    drivetrain_.step(dt, dc, solveIn.data(), solveOut.data());

    // ---- 4. Forces on the chassis ----
    Vec3 force{0, 0, -def_.mass * kGravity};
    Vec3 torque{};
    for (std::size_t w = 0; w < nw; ++w) {
        WheelState& ws = wheels_[w];
        ws.omega = solveOut[w].omega;
        ws.slip.kappa = clamp(solveOut[w].kappa, -MagicFormulaTire::kSlipStateLimit, MagicFormulaTire::kSlipStateLimit);
        if (ws.load <= 0) {
            ws.forces = {};
            continue;
        }
        const Real speed = std::hypot(ws.contactVx, ws.contactVy);
        ws.forces = tires_[w].evaluate(ws.load, ws.slip.kappa, ws.slip.tanAlpha, ws.hit.surface, speed);
        ws.forces.fx = solveOut[w].fx;  // consistent with the implicit wheel solve
        ws.forces.fy -= solveIn[w].slipDamping * ws.contactVy;
        const Vec3 n = ws.hit.normal;
        const Vec3 fc = n * ws.load + ws.longDir * ws.forces.fx + ws.latDir * ws.forces.fy;
        force += fc;
        torque += cross(ws.hit.point - comWorld, fc) + n * ws.forces.mz;
    }

    // Aerodynamics (still air): drag at the CoM, downforce at each axle.
    const Real vmag = length(body_.velocity);
    const AeroDef& aero = def_.aero;
    force += body_.velocity * (-0.5 * aero.airDensity * aero.dragArea * vmag);
    const Real vxBody = bodyVelocity().x;
    const Real qDyn = 0.5 * aero.airDensity * vxBody * vxBody;
    if (def_.axles.size() >= 2 && (aero.frontDownforceArea != 0 || aero.rearDownforceArea != 0)) {
        const auto axlePoint = [&](const AxleDef& ax) {
            const Vec3 hp = (def_.wheels[ax.leftWheel].suspension.hardpoint + def_.wheels[ax.rightWheel].suspension.hardpoint) * 0.5;
            return body_.position + q.rotate(hp);
        };
        const Vec3 ff = down * (qDyn * aero.frontDownforceArea);
        const Vec3 fr = down * (qDyn * aero.rearDownforceArea);
        force += ff + fr;
        torque += cross(axlePoint(def_.axles.front()) - comWorld, ff) + cross(axlePoint(def_.axles.back()) - comWorld, fr);
    }

    // ---- 5. Integrate the chassis (semi-implicit Euler, gyroscopic term included) ----
    const Vec3 vOld = body_.velocity;
    body_.velocity += force * (dt / def_.mass);
    const Mat3 R = q.toMatrix();
    const Mat3 invIWorld = R * invInertiaBody_ * R.transposed();
    const Mat3 iWorld = R * Mat3::diagonal(inertiaBody_) * R.transposed();
    const Vec3 w0 = body_.angularVelocity;
    body_.angularVelocity += invIWorld * (torque - cross(w0, iWorld * w0)) * dt;

    const Vec3 newCom = comWorld + body_.velocity * dt;
    const Vec3 wv = body_.angularVelocity;
    Quat dq{0, wv.x, wv.y, wv.z};
    Quat qn = dq * q;
    body_.orientation = {q.w + 0.5 * dt * qn.w, q.x + 0.5 * dt * qn.x, q.y + 0.5 * dt * qn.y, q.z + 0.5 * dt * qn.z};
    body_.orientation.normalize();
    body_.position = newCom - body_.orientation.rotate(def_.centerOfMass);
    time_ += dt;

    // ---- 6. Telemetry ----
    TelemetryFrame& t = telem_;
    t.time = time_;
    t.position = body_.position;
    const Vec3 vb = bodyVelocity();
    t.speed = vb.x;
    t.lateralVelocity = vb.y;
    t.sideslip = std::abs(vb.x) > 0.5 ? std::atan2(vb.y, std::abs(vb.x)) : 0.0;
    t.yawRate = yawRate();
    const Vec3 fwd = body_.orientation.rotate({1, 0, 0});
    const Vec3 left = body_.orientation.rotate({0, 1, 0});
    t.pitchAngle = -std::asin(clamp(fwd.z, -1.0, 1.0));
    t.rollAngle = std::asin(clamp(left.z, -1.0, 1.0));
    t.accelBody = body_.orientation.inverseRotate((body_.velocity - vOld) / dt);
    t.kineticEnergy = 0.5 * def_.mass * dot(body_.velocity, body_.velocity) +
                      0.5 * dot(body_.angularVelocity, iWorld * body_.angularVelocity);
    t.steerInput = input.steer;
    t.throttleInput = input.throttle;
    t.brakeInput = input.brake;
    t.steerAngle = cmd.steerAngle;
    t.throttleOut = cmd.throttle;
    t.gear = cmd.gear;
    t.clutchEngagement = cmd.clutchEngagement;
    t.drivetrain = drivetrain_.telemetry();
    t.assists = assists_.report();
    t.wheelCount = nw;
    for (std::size_t w = 0; w < nw; ++w) {
        const WheelState& ws = wheels_[w];
        WheelTelemetry& wt = t.wheels[w];
        wt.contact = ws.contact;
        wt.compression = ws.compression;
        wt.load = ws.load;
        wt.omega = ws.omega;
        wt.slipRatio = ws.slip.kappa;
        wt.slipAngle = std::atan(ws.slip.tanAlpha);
        wt.fx = ws.forces.fx;
        wt.fy = ws.forces.fy;
        wt.steerAngle = ws.steer;
        wt.brakeTorque = cmd.brakeTorque[w];
        wt.brakeLocked = solveOut[w].brakeLocked;
        wt.surfaceId = ws.hit.surfaceId;
    }
    lastVelocity_ = body_.velocity;
}

}  // namespace ord
