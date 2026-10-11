// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Conversions between the core frame (ISO 8855: X forward, Y left, Z up,
// right-handed, metres) and Unreal Engine's frame (X forward, Y right, Z up,
// left-handed, centimetres). Kept as plain arithmetic, free of engine types,
// so it is unit-tested headless; the UE adapter wraps these for
// FVector/FQuat.
//
// The frames differ by the mirror M = diag(1, -1, 1) (det = -1) and a scale of
// 100. Under a mirror:
//   polar vectors (position, velocity, force)        v' =  M v
//   axial vectors (angular velocity, torque)         w' = -M w = (-x, y, -z)
//   rotation quaternions (w, x, y, z)                q' = (w, -x, y, -z)
// Unreal's Chaos uses centimetre-based SI: force in kg*cm/s^2, torque in
// kg*cm^2/s^2, inertia in kg*cm^2.
#pragma once

#include "ord/math.hpp"

namespace ord::host {

inline constexpr Real kUeUnitsPerMetre = 100.0;

struct UeVec { Real x, y, z; };
struct UeQuat { Real x, y, z, w; };  // FQuat member order

inline Vec3 positionFromUe(const UeVec& v) { return {v.x / 100.0, -v.y / 100.0, v.z / 100.0}; }
inline UeVec positionToUe(const Vec3& v) { return {v.x * 100.0, -v.y * 100.0, v.z * 100.0}; }
inline Vec3 velocityFromUe(const UeVec& v) { return positionFromUe(v); }          // cm/s -> m/s
inline UeVec velocityToUe(const Vec3& v) { return positionToUe(v); }
inline Vec3 angularVelocityFromUe(const UeVec& w) { return {-w.x, w.y, -w.z}; }  // rad/s, axial
inline UeVec angularVelocityToUe(const Vec3& w) { return {-w.x, w.y, -w.z}; }
inline UeVec forceToUe(const Vec3& f) { return {f.x * 100.0, -f.y * 100.0, f.z * 100.0}; }   // N -> kg*cm/s^2
inline UeVec torqueToUe(const Vec3& t) { return {-t.x * 1e4, t.y * 1e4, -t.z * 1e4}; }       // N*m -> kg*cm^2/s^2
inline Quat rotationFromUe(const UeQuat& q) { return {q.w, -q.x, q.y, -q.z}; }
inline UeQuat rotationToUe(const Quat& q) { return {-q.x, q.y, -q.z, q.w}; }
inline Real inertiaToUe(Real kgm2) { return kgm2 * 1e4; }

}  // namespace ord::host
