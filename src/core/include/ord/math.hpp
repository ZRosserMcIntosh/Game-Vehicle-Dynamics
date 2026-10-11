// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Minimal linear algebra for the physics core. Double precision throughout:
// vehicle state is simulated in a local frame that the host engine may rebase
// (see docs/architecture/overview.md, "Large worlds").
//
// Conventions (ISO 8855 vehicle axes): X forward, Y left, Z up, right-handed.
// SI units everywhere: metres, kilograms, seconds, newtons, radians.
#pragma once

#include <algorithm>
#include <cmath>

namespace ord {

using Real = double;

inline constexpr Real kPi = 3.14159265358979323846;
inline constexpr Real kGravity = 9.80665;  // m/s^2, standard gravity

struct Vec3 {
    Real x = 0, y = 0, z = 0;

    constexpr Vec3() = default;
    constexpr Vec3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(Real s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(Real s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(Real s) { x *= s; y *= s; z *= s; return *this; }
};

constexpr Vec3 operator*(Real s, const Vec3& v) { return v * s; }
constexpr Real dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Real length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalized(const Vec3& v) {
    const Real l = length(v);
    return l > 1e-12 ? v / l : Vec3{};
}
inline bool isFinite(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

struct Mat3 {
    // Row-major.
    Real m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    static Mat3 diagonal(const Vec3& d) {
        Mat3 r;
        r.m[0][0] = d.x; r.m[1][1] = d.y; r.m[2][2] = d.z;
        r.m[0][1] = r.m[0][2] = r.m[1][0] = r.m[1][2] = r.m[2][0] = r.m[2][1] = 0;
        return r;
    }
    Vec3 operator*(const Vec3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
    Mat3 transposed() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i];
        return r;
    }
    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                r.m[i][j] = 0;
                for (int k = 0; k < 3; ++k) r.m[i][j] += m[i][k] * o.m[k][j];
            }
        return r;
    }
};

struct Quat {
    Real w = 1, x = 0, y = 0, z = 0;

    static Quat fromAxisAngle(const Vec3& axis, Real angle) {
        const Vec3 a = normalized(axis);
        const Real s = std::sin(angle * 0.5);
        return {std::cos(angle * 0.5), a.x * s, a.y * s, a.z * s};
    }
    Quat operator*(const Quat& q) const {
        return {w * q.w - x * q.x - y * q.y - z * q.z,
                w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w};
    }
    Quat conjugate() const { return {w, -x, -y, -z}; }
    void normalize() {
        const Real n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n > 0) { w /= n; x /= n; y /= n; z /= n; }
    }
    // Rotate a body-frame vector into the world frame.
    Vec3 rotate(const Vec3& v) const {
        const Vec3 u{x, y, z};
        const Vec3 t = 2.0 * cross(u, v);
        return v + w * t + cross(u, t);
    }
    Vec3 inverseRotate(const Vec3& v) const { return conjugate().rotate(v); }
    Mat3 toMatrix() const {
        Mat3 r;
        r.m[0][0] = 1 - 2 * (y * y + z * z); r.m[0][1] = 2 * (x * y - w * z);     r.m[0][2] = 2 * (x * z + w * y);
        r.m[1][0] = 2 * (x * y + w * z);     r.m[1][1] = 1 - 2 * (x * x + z * z); r.m[1][2] = 2 * (y * z - w * x);
        r.m[2][0] = 2 * (x * z - w * y);     r.m[2][1] = 2 * (y * z + w * x);     r.m[2][2] = 1 - 2 * (x * x + y * y);
        return r;
    }
};

inline Real clamp(Real v, Real lo, Real hi) { return std::min(std::max(v, lo), hi); }
inline Real sign(Real v) { return (v > 0) - (v < 0); }
inline Real lerp(Real a, Real b, Real t) { return a + (b - a) * t; }
inline Real kmhToMs(Real kmh) { return kmh / 3.6; }
inline Real msToKmh(Real ms) { return ms * 3.6; }
inline Real rpmToRadS(Real rpm) { return rpm * (2.0 * kPi / 60.0); }
inline Real radSToRpm(Real w) { return w * (60.0 / (2.0 * kPi)); }

// Move `current` toward `target` by at most `maxDelta`.
inline Real approach(Real current, Real target, Real maxDelta) {
    if (current < target) return std::min(current + maxDelta, target);
    return std::max(current - maxDelta, target);
}

}  // namespace ord
