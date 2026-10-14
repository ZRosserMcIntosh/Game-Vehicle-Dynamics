// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/assists.hpp"

namespace ord {

namespace {
Real smoothstep(Real x) {
    x = clamp(x, 0.0, 1.0);
    return x * x * (3 - 2 * x);
}
// Practical slip ratio with a floor on the reference speed to stay finite at standstill.
Real slipRatio(Real omega, Real radius, Real groundSpeed, Real floor) {
    return (omega * radius - groundSpeed) / std::max(std::abs(groundSpeed), floor);
}
// Drive slip per driven axle, from the axle's mean wheel speed vs its mean ground speed.
// Averaging left and right removes the inside/outside speed difference of a tight turn
// (which a locking differential turns into apparent slip on the inner wheel) so it is not
// mistaken for wheelspin. Returns the largest axle slip.
Real drivenAxleSlip(const VehicleSensors& s) {
    Real worst = 0;
    for (int front = 0; front < 2; ++front) {
        Real om = 0, vg = 0;
        int n = 0;
        for (std::size_t w = 0; w < s.wheelCount; ++w) {
            if (!s.wheelDriven[w] || !s.wheelInContact[w] || s.wheelIsFront[w] != (front == 1)) continue;
            om += s.wheelOmega[w] * s.wheelRadius[w];
            vg += s.wheelGroundSpeed[w];
            ++n;
        }
        if (n) worst = std::max(worst, slipRatio(om / n, 1.0, vg / n, 2.0));
    }
    return worst;
}
}  // namespace

Real AssistSystem::shapeSteer(const DriverInput& in, const VehicleSensors& s) {
    const Real x = clamp(in.steer, -1.0, 1.0);
    const Real shaped = sign(x) * std::pow(std::abs(x), std::max(cfg_.steeringExponent, 0.1));

    if (cfg_.steerRate > 0) {
        const bool returning = std::abs(shaped) < std::abs(steerState_) || sign(shaped) != sign(steerState_);
        const Real rate = (returning && cfg_.steerReturnRate > 0) ? cfg_.steerReturnRate : cfg_.steerRate;
        steerState_ = approach(steerState_, shaped, rate * s.dt);
    } else {
        steerState_ = shaped;
    }

    Real limit = s.maxSteerAngle;
    if (cfg_.steeringMode == SteeringMode::SpeedSensitive && s.speed > 1.0) {
        // Kinematic angle for the tightest radius the tires can sustain, plus the
        // front slip angle that produces peak lateral force. Scaling (not clamping)
        // keeps full stick resolution while stopping a full-lock flick at speed
        // from saturating the front tires.
        const Real rMin = s.speed * s.speed / std::max(s.frictionEstimate * kGravity, 0.5);
        limit = std::min(s.maxSteerAngle, std::atan(s.wheelbase / rMin) + cfg_.targetFrontSlipAngle);
    }
    report_.steerLimitAngle = limit;
    Real angle = steerState_ * limit;

    report_.countersteerAngle = 0;
    if (cfg_.countersteerAssist > 0 && s.speed > 3.0) {
        const Real b = s.wheelbase - s.cgToFrontAxle;
        const Real vyFront = s.lateralVelocity + s.cgToFrontAxle * s.yawRate;
        const Real vyRear = s.lateralVelocity - b * s.yawRate;
        const Real frontTravel = std::atan2(vyFront, s.speed);  // wheel angle with zero front slip
        const Real rearSlip = std::atan2(vyRear, s.speed);
        const Real w = cfg_.countersteerAssist * smoothstep((std::abs(rearSlip) - 0.06) / 0.15);
        if (w > 0) {
            const Real target = clamp(frontTravel, -s.maxSteerAngle, s.maxSteerAngle);
            const Real blended = angle + w * (target - angle);
            report_.countersteerAngle = blended - angle;
            angle = blended;
        }
    }
    return clamp(angle, -s.maxSteerAngle, s.maxSteerAngle);
}

void AssistSystem::updateTransmission(const DriverInput& in, const VehicleSensors& s, ActuatorCommands& cmd) {
    shiftCooldown_ = std::max(0.0, shiftCooldown_ - s.dt);
    auto requestShift = [&](int g, Real cooldown = 0.6) {
        if (g == gear_ || g < -1 || g > s.forwardGears) return;
        downshiftBlip_ = g >= 1 && g < gear_;
        gear_ = g;
        shiftTimer_ = cfg_.autoClutch ? s.shiftTime : 0.0;
        shiftCooldown_ = s.shiftTime + cooldown;
    };

    if (cfg_.shiftMode == ShiftMode::Manual) {
        if (in.shiftUp) requestShift(gear_ + 1);
        if (in.shiftDown) requestShift(gear_ - 1);
    } else {
        const Real thr = clamp(in.throttle, 0.0, 1.0);
        // Braking: drop a gear as soon as the lower gear fits under ~88 % of the limiter (with a
        // throttle blip, below), like a dual-clutch box in its sport mode. This keeps the engine in
        // its power band for the exit and adds engine braking through the shorter ratio.
        const bool braking = clamp(in.brake, 0.0, 1.0) > 0.15 && s.speed > 3.0;
        if (braking && gear_ > 1 && shiftTimer_ <= 0 && shiftCooldown_ <= 0) {
            const Real below = s.gearRpm[static_cast<std::size_t>(gear_ - 1)];
            if (below > 0 && below < 0.88 * s.limiterRpm) requestShift(gear_ - 1, 0.12);
        }
        if (!braking && gear_ >= 1 && shiftTimer_ <= 0 && shiftCooldown_ <= 0) {
            const Real up = s.limiterRpm * lerp(0.55, 0.985, thr);  // full throttle: shift right at the limiter
            const Real down = s.limiterRpm * lerp(0.28, 0.50, thr);
            // Hold the gear while the driven wheels are spinning (burnouts, power slides): an
            // upshift there would only bog the engine. The limiter catches the revs instead.
            const bool spinning = drivenAxleSlip(s) > 0.3;
            if (s.engineRpm > up && gear_ < s.forwardGears && !spinning) requestShift(gear_ + 1);
            else if (s.engineRpm < down && gear_ > 1) requestShift(gear_ - 1);
        }
        // Drive/reverse selection when (nearly) stopped.
        if (std::abs(s.speed) < 0.5) {
            if (in.shiftDown && gear_ >= 1) requestShift(-1);
            if (in.shiftUp && gear_ == -1) requestShift(1);
        }
    }

    cmd.gear = gear_;
    report_.shifting = shiftTimer_ > 0;
    if (cfg_.autoClutch) {
        if (shiftTimer_ > 0) {
            shiftTimer_ -= s.dt;
            cmd.clutchEngagement = 0;
            if (!downshiftBlip_ && gear_ >= 1 && in.throttle > 0.5) {
                // Upshift under power, clutch-to-clutch: the oncoming clutch slips and pulls the engine down
                // to the new gear's speed while drive continues, with a brief torque reduction
                // (ignition retard). No free-revving flare and no torque gap.
                cmd.clutchEngagement = 0.55;
                cmd.throttle *= 0.6;
            }
            // Rev-match on downshifts: blip the throttle up to the new gear's speed so the clutch
            // re-engages without a jolt (or a locked rear axle).
            if (downshiftBlip_ && gear_ >= 1 && gear_ < static_cast<int>(s.gearRpm.size())) {
                const Real target = std::min(s.gearRpm[static_cast<std::size_t>(gear_)], 0.98 * s.limiterRpm);
                cmd.throttle = s.engineRpm < target - 150 ? 1.0 : 0.0;
            }
        } else {
            downshiftBlip_ = false;
            // Engine-speed-based engagement (like a centrifugal / launch clutch):
            // disengages to prevent stalling, engages fully above launch rpm.
            const Real lo = s.idleRpm * 1.05;
            cmd.clutchEngagement = smoothstep((s.engineRpm - lo) / std::max(cfg_.launchRpm - lo, 100.0));
        }
    } else {
        cmd.clutchEngagement = 1.0 - clamp(in.clutch, 0.0, 1.0);
    }
}

void AssistSystem::applyAbs(const DriverInput& in, const VehicleSensors& s, ActuatorCommands& cmd) {
    const Real pedal = clamp(in.brake, 0.0, 1.0);
    for (std::size_t w = 0; w < s.wheelCount; ++w) {
        Real& p = absPressure_[w];
        report_.absActive[w] = false;
        if (!cfg_.abs) {
            p = 1.0;  // disabled means no modulation at all, immediately (no hidden residual aid)
        } else if (pedal <= 0 || !s.wheelInContact[w] || s.wheelGroundSpeed[w] < 2.0) {
            p = approach(p, 1.0, 10.0 * s.dt);
        } else {
            const Real slip = slipRatio(s.wheelOmega[w], s.wheelRadius[w], s.wheelGroundSpeed[w], 1.0);
            if (slip < -cfg_.absTargetSlip) p -= 8.0 * s.dt;              // release
            else if (slip > -0.6 * cfg_.absTargetSlip) p += 4.0 * s.dt;   // re-apply
            p = clamp(p, 0.0, 1.0);
            report_.absActive[w] = p < 0.999;
        }
        cmd.brakeTorque[w] = pedal * s.maxBrakeTorque[w] * p + clamp(in.handbrake, 0.0, 1.0) * s.handbrakeTorque[w];
    }
}

void AssistSystem::applyTcs(const VehicleSensors& s, ActuatorCommands& cmd) {
    report_.tcsThrottleCut = 0;
    if (cfg_.tractionControl == InterventionLevel::Off) {
        tcsCut_ = 0;
        return;
    }
    const bool full = cfg_.tractionControl == InterventionLevel::Full;
    const Real target = full ? 0.08 : 0.15;
    const Real maxCut = full ? 1.0 : 0.8;
    // Per driven axle: if only one wheel spins (the unloaded inner wheel in a tight turn), brake
    // that wheel - the differential then sends the torque to the wheel with grip - and leave the
    // throttle alone. The engine is only cut when the axle as a whole spins (the wheel with the
    // most grip is over the target too).
    Real maxSlip = 0;
    for (int front = 0; front < 2; ++front) {
        std::array<std::size_t, kMaxWheels> idx{};
        std::array<Real, kMaxWheels> sl{};
        std::size_t n = 0;
        for (std::size_t w = 0; w < s.wheelCount; ++w) {
            if (!s.wheelDriven[w] || !s.wheelInContact[w] || s.wheelIsFront[w] != (front == 1)) continue;
            idx[n] = w;
            sl[n] = slipRatio(s.wheelOmega[w], s.wheelRadius[w], s.wheelGroundSpeed[w], 2.0);
            ++n;
        }
        if (n == 0) continue;
        Real least = sl[0];
        for (std::size_t i = 1; i < n; ++i) least = std::min(least, sl[i]);
        maxSlip = std::max(maxSlip, least);
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t w = idx[i];
            const Real excess = sl[i] - std::max(least, target);
            const Real want = (n > 1 && excess > 0 && cmd.throttle > 0) ? std::min(1.0, excess / 0.3) * 0.35 * s.maxBrakeTorque[w] : 0.0;
            tcBrake_[w] = approach(tcBrake_[w], want, 6.0 * s.maxBrakeTorque[w] * s.dt);
            cmd.brakeTorque[w] += tcBrake_[w];
        }
    }
    if (maxSlip > target && cmd.throttle > 0) tcsCut_ += s.dt * 6.0 * (maxSlip - target) / target;
    else tcsCut_ -= s.dt * 2.0;
    tcsCut_ = clamp(tcsCut_, 0.0, maxCut);
    report_.tcsThrottleCut = tcsCut_;
    cmd.throttle *= (1.0 - tcsCut_);
}

void AssistSystem::applyEsc(const VehicleSensors& s, ActuatorCommands& cmd) {
    report_.escYawError = 0;
    for (auto& t : report_.escBrakeTorque) t = 0;
    if (cfg_.stabilityControl == InterventionLevel::Off || s.speed < 5.0) return;
    const bool full = cfg_.stabilityControl == InterventionLevel::Full;
    const Real deadband = full ? 0.05 : 0.12;
    const Real betaMax = full ? 0.08 : 0.20;

    // Linear single-track reference yaw rate with an understeer gradient,
    // bounded by what the road can deliver.
    constexpr Real kUndersteerGradient = 0.002;  // s^2/m
    const Real v = s.speed;
    Real rRef = v * cmd.steerAngle / (s.wheelbase + kUndersteerGradient * v * v);
    const Real rMax = 0.85 * s.frictionEstimate * kGravity / v;
    rRef = clamp(rRef, -rMax, rMax);
    const Real r = s.yawRate;
    const Real err = r - rRef;
    // Sideslip the steering itself asks for: at low speed the body slides sideways geometrically
    // (rear axle as the turn centre), so that part is not oversteer. Fades out with speed.
    const Real lr = s.wheelbase - s.cgToFrontAxle;
    const Real betaKin = std::atan(lr / std::max(s.wheelbase, 0.5) * std::tan(clamp(cmd.steerAngle, -1.2, 1.2)));
    const Real betaRef = betaKin / (1.0 + (v / 20.0) * (v / 20.0));
    const Real beta = std::atan2(s.lateralVelocity, v) - betaRef;
    report_.escYawError = err;

    // Oversteer: car yawing more than intended, or sideslip beyond the limit.
    const Real overYaw = (sign(err) == sign(r) || std::abs(rRef) < 1e-3) ? std::abs(err) - deadband : -1.0;
    const Real overBeta = std::abs(beta) - betaMax;
    const Real over = std::max(overYaw, overBeta * 4.0);
    // Understeer: car yawing less than intended in the same direction.
    const Real under = (sign(r) == sign(rRef) || std::abs(r) < 1e-3) ? std::abs(rRef) - std::abs(r) - deadband : -1.0;

    auto brakeWheel = [&](bool front, bool left, Real torque) {
        for (std::size_t w = 0; w < s.wheelCount; ++w) {
            if (s.wheelIsFront[w] != front || s.wheelIsLeft[w] != left) continue;
            const Real t = std::min(torque, 0.6 * s.maxBrakeTorque[w]);
            cmd.brakeTorque[w] += t;
            report_.escBrakeTorque[w] += t;
        }
    };
    const Real yawDir = std::abs(r) > 1e-3 ? r : rRef;
    if (over > 0) {
        // Brake the front wheel on the outside of the yaw to create a restoring moment.
        brakeWheel(true, yawDir < 0, 3000.0 * over);
        if (full) cmd.throttle *= 0.5;
    } else if (under > 0 && v > 10.0) {
        // Brake the inside rear wheel to add yaw moment toward the intended path. Not at parking
        // speeds: a car at full lock always "understeers" the reference model there, and trimming
        // power then just stops it pulling away.
        brakeWheel(false, rRef > 0, 2000.0 * under);
        if (full && v > 15.0) cmd.throttle *= 0.7;
    }
}

ActuatorCommands AssistSystem::process(const DriverInput& in, const VehicleSensors& s) {
    ActuatorCommands cmd;
    cmd.steerAngle = shapeSteer(in, s);
    if (steerBlend_ > 0) {
        cmd.steerAngle = lerp(cmd.steerAngle, steerBlendFrom_, steerBlend_);
        steerBlend_ = std::max(0.0, steerBlend_ - s.dt / kSteerBlendTime);
    }
    lastSteerAngle_ = cmd.steerAngle;

    const Real thr = clamp(in.throttle, 0.0, 1.0);
    throttleState_ = cfg_.throttleRate > 0 ? approach(throttleState_, thr, cfg_.throttleRate * s.dt) : thr;
    cmd.throttle = throttleState_;

    updateTransmission(in, s, cmd);
    applyAbs(in, s, cmd);
    applyTcs(s, cmd);
    applyEsc(s, cmd);
    return cmd;
}

}  // namespace ord
