// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Engine-agnostic debug drawing. The core emits primitives in world space
// (ISO axes, metres); hosts render them (Unreal: DrawDebugLine via the adapter;
// headless: SvgDebugSink for docs and bug reports).
#pragma once

#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

#include "ord/math.hpp"

namespace ord {

class Vehicle;

struct Color {
    std::uint8_t r = 255, g = 255, b = 255, a = 255;
};

namespace colors {
inline constexpr Color kContact{40, 200, 90, 255};     // contact samples touching
inline constexpr Color kAirborne{150, 150, 150, 255};  // samples not touching
inline constexpr Color kLoad{60, 120, 255, 255};       // tire normal load
inline constexpr Color kLongitudinal{255, 140, 0, 255};
inline constexpr Color kLateral{230, 50, 60, 255};
inline constexpr Color kSuspension{200, 200, 60, 255};
inline constexpr Color kVelocity{255, 255, 255, 255};
inline constexpr Color kAssist{200, 80, 255, 255};     // ABS/ESC active markers
}  // namespace colors

class DebugDrawSink {
public:
    virtual ~DebugDrawSink() = default;
    virtual void line(const Vec3& a, const Vec3& b, Color c) = 0;
    virtual void point(const Vec3& p, Real size, Color c) = 0;
    virtual void text(const Vec3& /*p*/, const std::string& /*s*/, Color /*c*/) {}
};

struct DebugDrawOptions {
    Real forceScale = 1.0 / 5000.0;  // metres of arrow per newton
    bool contactSamples = true;
    bool tireForces = true;
    bool suspension = true;
    bool velocity = true;
    bool assists = true;
    bool labels = false;
};

void drawVehicle(const Vehicle& v, DebugDrawSink& sink, const DebugDrawOptions& opt = {});

// Top-down (X right, Y up on the page) SVG rendering of the emitted primitives.
class SvgDebugSink final : public DebugDrawSink {
public:
    void line(const Vec3& a, const Vec3& b, Color c) override;
    void point(const Vec3& p, Real size, Color c) override;
    void text(const Vec3& p, const std::string& s, Color c) override;
    void clear() { items_.clear(); }
    // Writes a self-contained SVG; the view fits all primitives plus `margin` metres.
    void write(std::ostream& os, Real pixelsPerMetre = 120.0, Real margin = 0.5) const;

private:
    struct Item {
        int kind;  // 0 line, 1 point, 2 text
        Vec3 a, b;
        Real size;
        Color c;
        std::string s;
    };
    std::vector<Item> items_;
};

}  // namespace ord
