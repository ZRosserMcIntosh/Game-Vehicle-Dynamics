// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/vehicle_def.hpp"

#include <cmath>

namespace ord {

namespace {
bool finitePositive(Real v) { return std::isfinite(v) && v > 0; }
}  // namespace

std::string VehicleDefinition::validate() const {
    if (!finitePositive(mass)) return "mass must be finite and > 0";
    if (!finitePositive(inertia.x) || !finitePositive(inertia.y) || !finitePositive(inertia.z))
        return "inertia components must be finite and > 0";
    if (!isFinite(centerOfMass)) return "centerOfMass must be finite";
    if (wheels.empty() || wheels.size() > kMaxWheels) return "wheel count must be 1..kMaxWheels";
    if (axles.empty() || axles.size() > kMaxAxles) return "axle count must be 1..kMaxAxles";
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const WheelDef& w = wheels[i];
        const std::string id = "wheel " + std::to_string(i) + ": ";
        if (!finitePositive(w.tire.radius)) return id + "tire radius must be > 0";
        if (!finitePositive(w.tire.nominalLoad)) return id + "tire nominalLoad must be > 0";
        if (!finitePositive(w.tire.wheelInertia)) return id + "wheelInertia must be > 0";
        if (w.tire.curvatureX > 1 || w.tire.curvatureY > 1) return id + "Magic Formula curvature E must be <= 1";
        if (!finitePositive(w.tire.shapeX) || !finitePositive(w.tire.shapeY)) return id + "shape factors must be > 0";
        if (!finitePositive(w.suspension.travel)) return id + "suspension travel must be > 0";
        if (!finitePositive(w.suspension.springRate)) return id + "springRate must be > 0";
        if (w.suspension.bumpDamping < 0 || w.suspension.reboundDamping < 0) return id + "damping must be >= 0";
        if (w.maxBrakeTorque < 0 || w.handbrakeTorque < 0) return id + "brake torque must be >= 0";
        if (!isFinite(w.suspension.hardpoint)) return id + "hardpoint must be finite";
        if (!(w.unsprungMass >= 0) || !std::isfinite(w.unsprungMass)) return id + "unsprungMass must be >= 0";
        if (!finitePositive(w.tire.verticalStiffness)) return id + "tire verticalStiffness must be > 0";
        if (w.tire.verticalDamping < 0) return id + "tire verticalDamping must be >= 0";
        if (w.tire.patchHalfLength < 0 || w.tire.patchHalfWidth < 0 || w.tire.patchHalfLength >= w.tire.radius)
            return id + "patch half extents must be >= 0 and smaller than the radius";
    }
    for (std::size_t a = 0; a < axles.size(); ++a) {
        const AxleDef& ax = axles[a];
        if (ax.leftWheel < 0 || ax.rightWheel < 0 || ax.leftWheel >= static_cast<int>(wheels.size()) ||
            ax.rightWheel >= static_cast<int>(wheels.size()) || ax.leftWheel == ax.rightWheel)
            return "axle " + std::to_string(a) + ": invalid wheel indices";
    }
    const DrivetrainDef& dt = drivetrain;
    if (dt.engine.torqueCurve.size() < 2) return "engine torque curve needs >= 2 points";
    for (std::size_t i = 1; i < dt.engine.torqueCurve.size(); ++i)
        if (dt.engine.torqueCurve[i].rpm <= dt.engine.torqueCurve[i - 1].rpm) return "torque curve rpm must increase";
    if (!finitePositive(dt.engine.inertia) || !finitePositive(dt.gearbox.inputInertia)) return "drivetrain inertia must be > 0";
    if (dt.gearbox.forwardRatios.empty()) return "gearbox needs at least one forward ratio";
    if (dt.drivenAxles.empty()) return "at least one driven axle required";
    Real share = 0;
    for (const auto& da : dt.drivenAxles) {
        if (da.axleIndex >= axles.size()) return "driven axle index out of range";
        share += da.torqueShare;
    }
    if (std::abs(share - 1.0) > 1e-6) return "driven axle torque shares must sum to 1";
    if (totalUnsprungMass() >= 0.5 * mass) return "unsprung masses must be well below half the total mass";
    return {};
}

Real VehicleDefinition::totalUnsprungMass() const {
    Real m = 0;
    for (const WheelDef& w : wheels) m += w.unsprungMass;
    return m;
}

Real VehicleDefinition::wheelbase() const {
    if (axles.size() < 2) return 1.0;
    const Real xf = wheels[axles.front().leftWheel].suspension.hardpoint.x;
    const Real xr = wheels[axles.back().leftWheel].suspension.hardpoint.x;
    return std::abs(xf - xr);
}

}  // namespace ord
