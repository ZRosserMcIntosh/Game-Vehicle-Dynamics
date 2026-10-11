// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#pragma once

#include <array>

#include "ord/drivetrain.hpp"
#include "ord/math.hpp"

namespace ord {

enum class InputDevice { Gamepad, SteeringWheel, Keyboard };

// Raw driver intent, as read from the device (after dead-zone handling by the host).
struct DriverInput {
    Real steer = 0;      // -1..1, + = left (ISO: positive yaw is to the left)
    Real throttle = 0;   // 0..1
    Real brake = 0;      // 0..1
    Real handbrake = 0;  // 0..1
    Real clutch = 0;     // 0..1 pedal position, 1 = fully pressed (disengaged)
    bool shiftUp = false;    // edge-triggered
    bool shiftDown = false;  // edge-triggered
    InputDevice device = InputDevice::Gamepad;
};

// What actually reaches the physical car this step.
struct ActuatorCommands {
    Real steerAngle = 0;                       // rad, reference road-wheel angle (Ackermann applied later)
    Real throttle = 0;                         // 0..1 after traction control
    std::array<Real, kMaxWheels> brakeTorque{};  // N*m per wheel after ABS / ESC
    Real clutchEngagement = 1;                 // 0..1
    int gear = 1;
};

}  // namespace ord
