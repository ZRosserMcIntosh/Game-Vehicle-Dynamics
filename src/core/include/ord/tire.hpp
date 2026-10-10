// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Tire force model: a reduced Magic Formula (Pacejka) with load sensitivity,
// combined-slip weighting functions and first-order transient slip
// (relaxation length).
//
// References (published mathematics; no third-party code):
//  - H.B. Pacejka, "Tire and Vehicle Dynamics", 3rd ed., Butterworth-Heinemann,
//    2012. Ch. 4 (Magic Formula, MF 5.2/6.x structure: B,C,D,E, load
//    sensitivity dfz, combined-slip weighting functions G_xa / G_yk) and
//    Ch. 7 (transient slip, relaxation length sigma).
//  - E. Bakker, L. Nyborg, H.B. Pacejka, "Tyre modelling for use in vehicle
//    dynamics studies", SAE 870421, 1987.
//
// This is deliberately a *reduced* parameter set (see TireParams). It keeps the
// MF structure so a full MF 6.x coefficient set can replace it later behind the
// same interface (docs/decisions/physics-foundation.md).
#pragma once

#include "ord/math.hpp"

namespace ord {

struct TireParams {
    Real radius = 0.33;           // m, loaded rolling radius (rigid-ring approximation)
    Real nominalLoad = 4000.0;    // N, Fz0

    // Peak friction coefficients at nominal load and their load sensitivity:
    //   mu(Fz) = mu0 * (1 + loadSensitivity * dfz),  dfz = (Fz - Fz0) / Fz0
    Real muX = 1.10, muXLoadSensitivity = -0.08;
    Real muY = 1.05, muYLoadSensitivity = -0.10;

    // Shape factors (C) and curvature factors (E, must be <= 1).
    Real shapeX = 1.65, curvatureX = 0.10;
    Real shapeY = 1.35, curvatureY = -0.40;

    // Longitudinal slip stiffness: Kx = longStiffnessPerLoad * Fz   [N per unit slip]
    Real longStiffnessPerLoad = 18.0;
    // Cornering stiffness (MF 6.x form): Ky = cornerStiffness * Fz0 * sin(2 atan(Fz / (cornerStiffnessLoad * Fz0)))
    Real cornerStiffness = 16.0;      // peak Ky in units of Fz0 per rad
    Real cornerStiffnessLoad = 1.9;   // load (in Fz0) at which Ky peaks

    // Combined-slip weighting functions (MF 5.2 / 6.x, without shifts):
    //   G_xa = cos(C_xa * atan(B_xa * alpha)),  B_xa = rBx1 * cos(atan(rBx2 * kappa))
    //   G_yk = cos(C_yk * atan(B_yk * kappa)),  B_yk = rBy1 * cos(atan(rBy2 * alpha))
    Real rBx1 = 13.0, rBx2 = 9.7, rCx1 = 1.0;
    Real rBy1 = 10.6, rBy2 = 7.8, rCy1 = 1.0;

    // Transient slip: relaxation lengths [m].
    Real relaxationLengthX = 0.15;
    Real relaxationLengthY = 0.45;
    // Tread/carcass hysteresis damping used at very low speed, where the
    // relaxation ODE alone is undamped (standstill holding). [N*s/m per N of load]
    Real lowSpeedDampingPerLoad = 3.0;
    Real lowSpeedDampingFadeSpeed = 3.0;  // m/s, damping fades out above this speed

    Real rollingResistance = 0.012;   // dimensionless coefficient
    Real pneumaticTrail = 0.035;      // m, at zero slip angle
    Real trailCollapseAngle = 0.20;   // rad, trail reaches zero at this slip angle

    Real wheelInertia = 1.2;          // kg*m^2, wheel + tire + hub + brake rotor
};

// Per-contact surface multipliers; supplied by the host's surface query.
struct SurfaceProperties {
    Real frictionScale = 1.0;           // multiplies muX / muY (asphalt dry = 1)
    Real rollingResistanceScale = 1.0;
    // Crude wet-road speed falloff: mu *= 1 / (1 + wetSpeedFalloff * v). 0 = dry.
    // Placeholder for a proper water-film / hydroplaning model (roadmap M3).
    Real wetSpeedFalloff = 0.0;
};

struct TireSlipState {
    // Transient ("deflection-based") slip quantities, integrated with the
    // relaxation-length ODEs. In steady state they equal the practical slip
    // ratio kappa = (omega*r - vx)/|vx| and tan(alpha) = -vy/|vx|.
    Real kappa = 0.0;
    Real tanAlpha = 0.0;
};

struct TireForces {
    Real fx = 0.0;          // N, longitudinal force in the wheel frame (+ forward)
    Real fy = 0.0;          // N, lateral force in the wheel frame (+ left)
    Real mz = 0.0;          // N*m, self-aligning torque (about wheel Z)
    Real dFxdKappa = 0.0;   // N per unit slip, local slope, used by the implicit wheel solve
    Real muX = 0.0;         // effective peak coefficients after load/surface scaling
    Real muY = 0.0;
};

class MagicFormulaTire {
public:
    explicit MagicFormulaTire(const TireParams& p) : p_(p) {}

    const TireParams& params() const { return p_; }

    // Steady-state combined-slip force for the given (transient) slip.
    // `speed` is the contact-point ground speed magnitude, used only for the wet falloff.
    TireForces evaluate(Real fz, Real kappa, Real tanAlpha, const SurfaceProperties& surf, Real speed) const;

    // Pure-slip helpers (exposed for tests and tooling).
    Real pureLongitudinal(Real fz, Real kappa, Real muScale) const;
    Real pureLateral(Real fz, Real alpha, Real muScale) const;

    // Advance the lateral transient slip state implicitly:
    //   sigma_y * d(tanAlpha)/dt + |vx| * tanAlpha = -vy
    // Stable for any dt; at vx = 0 the state integrates lateral displacement,
    // which gives the tire its static (standstill) stiffness.
    void integrateLateralSlip(TireSlipState& s, Real vx, Real vy, Real dt) const;

    static constexpr Real kSlipStateLimit = 10.0;  // keeps transient slip finite

private:
    TireParams p_;
};

}  // namespace ord
