// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/tire.hpp"

namespace ord {

namespace {

// Magic Formula core: y = D sin(C atan(B x - E (B x - atan(B x))))
Real magicFormula(Real x, Real B, Real C, Real D, Real E) {
    const Real bx = B * x;
    return D * std::sin(C * std::atan(bx - E * (bx - std::atan(bx))));
}

Real loadDelta(const TireParams& p, Real fz) { return (fz - p.nominalLoad) / p.nominalLoad; }

}  // namespace

Real MagicFormulaTire::pureLongitudinal(Real fz, Real kappa, Real muScale) const {
    if (fz <= 0) return 0;
    const Real dfz = loadDelta(p_, fz);
    const Real mu = std::max(0.0, p_.muX * (1 + p_.muXLoadSensitivity * dfz)) * muScale;
    const Real D = mu * fz;
    if (D <= 1e-9) return 0;
    const Real K = p_.longStiffnessPerLoad * fz;
    const Real C = p_.shapeX;
    const Real B = K / (C * D);
    return magicFormula(kappa, B, C, D, std::min(p_.curvatureX, 1.0));
}

Real MagicFormulaTire::pureLateral(Real fz, Real alpha, Real muScale) const {
    if (fz <= 0) return 0;
    const Real dfz = loadDelta(p_, fz);
    const Real mu = std::max(0.0, p_.muY * (1 + p_.muYLoadSensitivity * dfz)) * muScale;
    const Real D = mu * fz;
    if (D <= 1e-9) return 0;
    const Real Fz0 = p_.nominalLoad;
    const Real K = p_.cornerStiffness * Fz0 * std::sin(2.0 * std::atan(fz / (p_.cornerStiffnessLoad * Fz0)));
    const Real C = p_.shapeY;
    const Real B = K / (C * D);
    return magicFormula(alpha, B, C, D, std::min(p_.curvatureY, 1.0));
}

TireForces MagicFormulaTire::evaluate(Real fz, Real kappa, Real tanAlpha, const SurfaceProperties& surf,
                                      Real speed) const {
    TireForces out;
    if (fz <= 0) return out;

    const Real wet = 1.0 / (1.0 + std::max(0.0, surf.wetSpeedFalloff) * std::abs(speed));
    const Real muScale = std::max(0.0, surf.frictionScale) * wet;
    const Real alpha = std::atan(tanAlpha);

    auto combinedFx = [&](Real k) {
        const Real Bxa = p_.rBx1 * std::cos(std::atan(p_.rBx2 * k));
        const Real Gxa = std::cos(p_.rCx1 * std::atan(Bxa * alpha));
        return Gxa * pureLongitudinal(fz, k, muScale);
    };

    const Real Byk = p_.rBy1 * std::cos(std::atan(p_.rBy2 * alpha));
    const Real Gyk = std::cos(p_.rCy1 * std::atan(Byk * kappa));

    out.fx = combinedFx(kappa);
    out.fy = Gyk * pureLateral(fz, alpha, muScale);

    // Central-difference slope for the linearly implicit wheel-spin solve.
    constexpr Real h = 1e-4;
    out.dFxdKappa = (combinedFx(kappa + h) - combinedFx(kappa - h)) / (2 * h);

    // Self-aligning torque from a collapsing pneumatic trail (simplified; the
    // MF Mz formulation with residual torque is roadmap work for force feedback).
    const Real trail = p_.pneumaticTrail * std::max(0.0, 1.0 - std::abs(alpha) / p_.trailCollapseAngle);
    out.mz = -trail * out.fy;

    const Real dfz = loadDelta(p_, fz);
    out.muX = std::max(0.0, p_.muX * (1 + p_.muXLoadSensitivity * dfz)) * muScale;
    out.muY = std::max(0.0, p_.muY * (1 + p_.muYLoadSensitivity * dfz)) * muScale;
    return out;
}

void MagicFormulaTire::integrateLateralSlip(TireSlipState& s, Real vx, Real vy, Real dt) const {
    const Real sigma = std::max(p_.relaxationLengthY, 1e-3);
    s.tanAlpha = (s.tanAlpha - dt * vy / sigma) / (1.0 + dt * std::abs(vx) / sigma);
    s.tanAlpha = clamp(s.tanAlpha, -kSlipStateLimit, kSlipStateLimit);
}

}  // namespace ord
