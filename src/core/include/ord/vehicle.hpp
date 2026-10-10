// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Full-fidelity vehicle (LOD 0): 6-DOF rigid chassis, per-corner raycast
// suspension with anti-roll bars, Magic Formula tires with transient slip,
// implicit drivetrain/wheel solve, aero, and the assist layer.
//
// Step order (one fixed step):
//   1. assists turn DriverInput into ActuatorCommands (using last step's sensors)
//   2. suspension contact + loads, lateral transient slip
//   3. implicit drivetrain/wheel/longitudinal-slip solve
//   4. combined-slip tire forces, aero, gravity -> chassis integration
//   5. telemetry
#pragma once

#include <array>

#include "ord/assists.hpp"
#include "ord/drivetrain.hpp"
#include "ord/ground.hpp"
#include "ord/inputs.hpp"
#include "ord/telemetry.hpp"
#include "ord/tire.hpp"
#include "ord/vehicle_def.hpp"

namespace ord {

struct RigidBodyState {
    Vec3 position;           // world position of the body origin
    Quat orientation;        // body -> world
    Vec3 velocity;           // world, of the centre of mass
    Vec3 angularVelocity;    // world
};

class Vehicle {
public:
    explicit Vehicle(const VehicleDefinition& def, const AssistConfig& assists = {});

    // Places the car at rest with its body origin at `position`, heading `yaw`
    // (rad about +Z), and settles the suspension onto the ground.
    void reset(const Vec3& position, Real yaw, const GroundQuery& ground);

    void step(Real dt, const DriverInput& input, const GroundQuery& ground);

    const VehicleDefinition& definition() const { return def_; }
    const RigidBodyState& state() const { return body_; }
    RigidBodyState& mutableState() { return body_; }
    const TelemetryFrame& telemetry() const { return telem_; }
    AssistSystem& assists() { return assists_; }
    const Drivetrain& drivetrain() const { return drivetrain_; }

    // Convenience: velocity in the body frame.
    Vec3 bodyVelocity() const { return body_.orientation.inverseRotate(body_.velocity); }
    Real forwardSpeed() const { return bodyVelocity().x; }
    Real yawRate() const;

    // Places a wheel's spin and slip state consistent with a rolling car
    // (useful for tests that start at speed).
    void setRollingSpeed(Real forwardSpeed);

private:
    struct WheelState {
        Real compression = 0;
        Real prevCompression = 0;
        bool contact = false;
        GroundHit hit;
        Real load = 0;
        TireSlipState slip;
        Real omega = 0;
        Vec3 longDir, latDir;      // contact frame, world
        Real contactVx = 0, contactVy = 0;
        Real steer = 0;
        Real springForce = 0;
        TireForces forces;
    };

    VehicleSensors buildSensors(Real dt) const;
    Real wheelSteerAngle(std::size_t w, Real refAngle) const;

    VehicleDefinition def_;
    std::vector<MagicFormulaTire> tires_;
    AssistSystem assists_;
    Drivetrain drivetrain_;
    RigidBodyState body_;
    std::array<WheelState, kMaxWheels> wheels_{};
    std::array<bool, kMaxWheels> driven_{};
    std::array<bool, kMaxWheels> front_{};
    Mat3 invInertiaBody_;
    Vec3 inertiaBody_;
    Real time_ = 0;
    Vec3 lastVelocity_;
    TelemetryFrame telem_;
};

}  // namespace ord
