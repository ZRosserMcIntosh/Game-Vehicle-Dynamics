// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// FVector/FQuat wrappers around ord::host (host_frame.hpp). All frame and unit
// rules live in the core header, which is unit-tested headless; this file only
// moves numbers between engine and core types.
//
//   core: ISO 8855, X fwd, Y left, Z up, right-handed, metres, N, N*m
//   UE  : X fwd, Y right, Z up, left-handed, centimetres, kg*cm/s^2, kg*cm^2/s^2
//
// Polar vectors (positions, velocities, forces, directions, normals) mirror Y.
// Axial vectors (angular velocity, torque) negate X and Z. Quaternions follow
// the axial rule on their vector part.
#pragma once

#include "CoreMinimal.h"

#include "ord/debug_draw.hpp"
#include "ord/host_frame.hpp"

namespace OrdFrame
{
inline ord::host::UeVec ToUeVec(const FVector& V) { return {V.X, V.Y, V.Z}; }
inline FVector FromUeVec(const ord::host::UeVec& V) { return FVector(V.x, V.y, V.z); }

/** cm -> m (also valid for body-local offsets). */
inline ord::Vec3 PositionFromUe(const FVector& V) { return ord::host::positionFromUe(ToUeVec(V)); }
/** m -> cm (also valid for body-local offsets). */
inline FVector PositionToUe(const ord::Vec3& V) { return FromUeVec(ord::host::positionToUe(V)); }

/** cm/s -> m/s. */
inline ord::Vec3 VelocityFromUe(const FVector& V) { return ord::host::velocityFromUe(ToUeVec(V)); }
inline FVector VelocityToUe(const ord::Vec3& V) { return FromUeVec(ord::host::velocityToUe(V)); }

/** rad/s, axial. */
inline ord::Vec3 AngularVelocityFromUe(const FVector& W) { return ord::host::angularVelocityFromUe(ToUeVec(W)); }
inline FVector AngularVelocityToUe(const ord::Vec3& W) { return FromUeVec(ord::host::angularVelocityToUe(W)); }

/** N -> kg*cm/s^2. */
inline FVector ForceToUe(const ord::Vec3& F) { return FromUeVec(ord::host::forceToUe(F)); }
/** N*m -> kg*cm^2/s^2 (axial). */
inline FVector TorqueToUe(const ord::Vec3& T) { return FromUeVec(ord::host::torqueToUe(T)); }

inline ord::Quat RotationFromUe(const FQuat& Q) { return ord::host::rotationFromUe({Q.X, Q.Y, Q.Z, Q.W}); }
inline FQuat RotationToUe(const ord::Quat& Q)
{
	const ord::host::UeQuat U = ord::host::rotationToUe(Q);
	return FQuat(U.x, U.y, U.z, U.w);
}

/** Unit directions and surface normals: polar, no scale. */
inline ord::Vec3 DirectionFromUe(const FVector& D) { return {D.X, -D.Y, D.Z}; }
inline FVector DirectionToUe(const ord::Vec3& D) { return FVector(D.x, -D.y, D.z); }

/** kg*m^2 -> kg*cm^2 per component (diagonal inertia is mirror-invariant). */
inline FVector InertiaToUe(const ord::Vec3& I)
{
	return FVector(ord::host::inertiaToUe(I.x), ord::host::inertiaToUe(I.y), ord::host::inertiaToUe(I.z));
}

inline FColor ColorToUe(const ord::Color& C) { return FColor(C.r, C.g, C.b, C.a); }
}  // namespace OrdFrame
