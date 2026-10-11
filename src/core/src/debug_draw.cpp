// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/debug_draw.hpp"

#include <cstdio>

#include "ord/vehicle.hpp"

namespace ord {

void drawVehicle(const Vehicle& v, DebugDrawSink& sink, const DebugDrawOptions& opt) {
    const std::size_t nw = v.definition().wheels.size();
    const TelemetryFrame& t = v.telemetry();
    for (std::size_t w = 0; w < nw; ++w) {
        const Vehicle::ContactDebug& d = v.contactDebug(w);
        if (opt.suspension) {
            sink.line(d.hardpoint, d.wheelCenter, colors::kSuspension);
            sink.point(d.wheelCenter, 0.04, colors::kSuspension);
        }
        if (opt.contactSamples) {
            for (int k = 0; k < d.samples; ++k)
                sink.point(d.point[k], 0.02, d.penetration[k] > 0 ? colors::kContact : colors::kAirborne);
        }
        if (opt.tireForces && d.contact) {
            const Vec3 p = d.patchCenter;
            sink.line(p, p + d.normal * (d.fz * opt.forceScale), colors::kLoad);
            sink.line(p, p + d.longDir * (d.fx * opt.forceScale), colors::kLongitudinal);
            sink.line(p, p + d.latDir * (d.fy * opt.forceScale), colors::kLateral);
        }
        if (opt.assists && w < kMaxWheels && (t.assists.absActive[w] || t.assists.escBrakeTorque[w] > 0))
            sink.point(d.wheelCenter, 0.08, colors::kAssist);
        if (opt.labels) {
            char buf[64];
            std::snprintf(buf, sizeof buf, "Fz %.0f k %.2f a %.1f", d.fz, t.wheels[w].slipRatio,
                          t.wheels[w].slipAngle * 180.0 / kPi);
            sink.text(d.wheelCenter, buf, colors::kVelocity);
        }
    }
    if (opt.velocity) {
        const Vec3 c = v.centerOfMassWorld();
        sink.line(c, c + v.state().velocity * 0.1, colors::kVelocity);
    }
}

void SvgDebugSink::line(const Vec3& a, const Vec3& b, Color c) { items_.push_back({0, a, b, 0, c, {}}); }
void SvgDebugSink::point(const Vec3& p, Real size, Color c) { items_.push_back({1, p, p, size, c, {}}); }
void SvgDebugSink::text(const Vec3& p, const std::string& s, Color c) { items_.push_back({2, p, p, 0, c, s}); }

void SvgDebugSink::write(std::ostream& os, Real ppm, Real margin) const {
    Real minX = 1e300, minY = 1e300, maxX = -1e300, maxY = -1e300;
    for (const Item& it : items_) {
        for (const Vec3& p : {it.a, it.b}) {
            minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
            minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
        }
    }
    if (items_.empty()) { minX = minY = 0; maxX = maxY = 1; }
    minX -= margin; minY -= margin; maxX += margin; maxY += margin;
    const Real w = (maxX - minX) * ppm, h = (maxY - minY) * ppm;
    // Page coordinates: X right, world +Y up (SVG y grows downward).
    auto px = [&](Real x) { return (x - minX) * ppm; };
    auto py = [&](Real y) { return (maxY - y) * ppm; };
    auto rgb = [](Color c) {
        char b[32];
        std::snprintf(b, sizeof b, "rgb(%d,%d,%d)", c.r, c.g, c.b);
        return std::string(b);
    };
    os << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << w << "\" height=\"" << h << "\" viewBox=\"0 0 "
       << w << ' ' << h << "\">\n<rect width=\"100%\" height=\"100%\" fill=\"rgb(24,26,30)\"/>\n";
    for (const Item& it : items_) {
        if (it.kind == 0) {
            os << "<line x1=\"" << px(it.a.x) << "\" y1=\"" << py(it.a.y) << "\" x2=\"" << px(it.b.x) << "\" y2=\""
               << py(it.b.y) << "\" stroke=\"" << rgb(it.c) << "\" stroke-width=\"2\"/>\n";
        } else if (it.kind == 1) {
            os << "<circle cx=\"" << px(it.a.x) << "\" cy=\"" << py(it.a.y) << "\" r=\"" << std::max(1.5, it.size * ppm)
               << "\" fill=\"" << rgb(it.c) << "\"/>\n";
        } else {
            os << "<text x=\"" << px(it.a.x) << "\" y=\"" << py(it.a.y) << "\" fill=\"" << rgb(it.c)
               << "\" font-family=\"monospace\" font-size=\"11\">" << it.s << "</text>\n";
        }
    }
    os << "</svg>\n";
}

}  // namespace ord
