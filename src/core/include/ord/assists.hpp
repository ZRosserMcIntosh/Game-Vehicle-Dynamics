// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Driver assists and input interpretation.
//
// Design rule (docs/decisions/one-simulation-many-experiences.md):
// assists act ONLY through the same actuators a real car has — steering
// angle, throttle/engine torque, per-wheel brake torque, clutch and gear
// selection. They never touch tire grip, mass, inertia or any force model.
// Every intervention is reported in AssistReport so it can be shown,
// logged and tested.
//
// Every aid is individually configurable. Presets (presets.hpp) are only
// named bundles of AssistConfig values.
#pragma once

#include <array>
#include <string>

#include "ord/drivetrain.hpp"
#include "ord/inputs.hpp"
#include "ord/math.hpp"

namespace ord {

enum class SteeringMode {
    Direct,          // input maps linearly to road-wheel angle (wheels; "simulation steering")
    SpeedSensitive,  // gamepad-friendly: range limited by speed and peak slip angle, rate limited
};

enum class InterventionLevel { Off, Sport, Full };

enum class ShiftMode { Automatic, Manual };

struct AssistConfig {
    // --- Input interpretation ---
    SteeringMode steeringMode = SteeringMode::Direct;
    Real steeringExponent = 1.0;        // input shaping: |x|^e * sign(x) (1 = linear)
    Real steerRate = 0.0;               // max change of steer input per second (0 = unlimited)
    Real steerReturnRate = 0.0;         // faster rate when returning toward centre (0 = same as steerRate)
    Real targetFrontSlipAngle = 0.12;   // rad, SpeedSensitive: front slip angle allowed at the steering limit
    Real throttleRate = 0.0;            // max throttle change per second (0 = unlimited)

    // --- Physically modelled electronic aids ---
    bool abs = false;
    Real absTargetSlip = 0.12;          // |slip ratio| ABS tries to hold under braking

    InterventionLevel tractionControl = InterventionLevel::Off;
    InterventionLevel stabilityControl = InterventionLevel::Off;

    // --- Driving aids that have no real-car equivalent (always labelled as such) ---
    Real countersteerAssist = 0.0;      // 0..1, blends front wheels toward the direction of travel in slides

    // --- Transmission ---
    ShiftMode shiftMode = ShiftMode::Automatic;
    bool autoClutch = true;             // false: driver must operate the clutch pedal
    Real launchRpm = 2500;              // auto-clutch engagement target from standstill
};

// What the assists can "sense". Mirrors sensors a production car has (wheel
// speeds, yaw-rate/accelerometer, steering angle); reference vehicle speed is
// taken from the simulation for now (an estimator is roadmap work).
struct VehicleSensors {
    Real dt = 0;
    Real speed = 0;              // m/s, longitudinal at CoM (body frame)
    Real lateralVelocity = 0;    // m/s, body frame
    Real yawRate = 0;            // rad/s, + = left
    Real wheelbase = 2.6;
    Real cgToFrontAxle = 1.3;
    Real maxSteerAngle = 0.6;    // rad, reference road-wheel angle at full lock
    Real frictionEstimate = 1.0; // nominal surface mu under the car
    std::size_t wheelCount = 4;
    std::array<Real, kMaxWheels> wheelOmega{};
    std::array<Real, kMaxWheels> wheelRadius{};
    std::array<Real, kMaxWheels> wheelGroundSpeed{};  // contact-point speed along the wheel heading
    std::array<bool, kMaxWheels> wheelDriven{};
    std::array<bool, kMaxWheels> wheelIsFront{};
    std::array<bool, kMaxWheels> wheelIsLeft{};
    std::array<bool, kMaxWheels> wheelInContact{};
    std::array<Real, kMaxWheels> maxBrakeTorque{};
    std::array<Real, kMaxWheels> handbrakeTorque{};
    Real engineRpm = 0;
    Real idleRpm = 900;
    Real limiterRpm = 7000;
    int gear = 1;
    int forwardGears = 6;
    Real shiftTime = 0.25;
};

struct AssistReport {
    std::array<bool, kMaxWheels> absActive{};
    Real tcsThrottleCut = 0;        // 0..1 fraction of driver throttle removed
    std::array<Real, kMaxWheels> escBrakeTorque{};
    Real escYawError = 0;           // rad/s, measured - reference
    Real steerLimitAngle = 0;       // rad, active SpeedSensitive limit
    Real countersteerAngle = 0;     // rad added by the countersteer aid
    bool shifting = false;
};

class AssistSystem {
public:
    explicit AssistSystem(const AssistConfig& cfg = {}) : cfg_(cfg) {}
    void setConfig(const AssistConfig& cfg) { cfg_ = cfg; }
    const AssistConfig& config() const { return cfg_; }

    // Turns raw driver input into actuator commands.
    ActuatorCommands process(const DriverInput& in, const VehicleSensors& s);
    const AssistReport& report() const { return report_; }
    int currentGear() const { return gear_; }
    void setGear(int g) { gear_ = g; }

private:
    Real shapeSteer(const DriverInput& in, const VehicleSensors& s);
    void updateTransmission(const DriverInput& in, const VehicleSensors& s, ActuatorCommands& cmd);
    void applyAbs(const DriverInput& in, const VehicleSensors& s, ActuatorCommands& cmd);
    void applyTcs(const VehicleSensors& s, ActuatorCommands& cmd);
    void applyEsc(const VehicleSensors& s, ActuatorCommands& cmd);

    AssistConfig cfg_;
    AssistReport report_;
    Real steerState_ = 0;
    Real throttleState_ = 0;
    std::array<Real, kMaxWheels> absPressure_{1, 1, 1, 1, 1, 1, 1, 1};
    Real tcsCut_ = 0;
    int gear_ = 1;
    Real shiftTimer_ = 0;
    Real shiftCooldown_ = 0;
};

}  // namespace ord
