// OpenRoad Dynamics — headless stand-ins for the few Unreal types used by the
// adapter's physics-thread code (OrdVehicleSim, OrdFrame, OrdGroundHorizon).
// SPDX-License-Identifier: MIT
//
// NOT Unreal Engine. Just enough FVector/FQuat/TArray/FMath semantics (UE
// member layout, Hamilton product, RotateVector) to compile and exercise the
// adapter's math outside the engine. Behavioural differences from the real
// types are possible; this only checks the adapter's own logic.
#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

using int32 = std::int32_t;
using uint32 = std::uint32_t;
using int64 = std::int64_t;
using uint64 = std::uint64_t;
using uint8 = std::uint8_t;

#define TEXT(x) x
#define check(x) assert(x)
#define MAX_uint32 (std::numeric_limits<uint32>::max())
#define UE_LOG(Category, Verbosity, Format, ...) std::fprintf(stderr, "[" #Verbosity "] " Format "\n", ##__VA_ARGS__)

struct FVector
{
	double X = 0, Y = 0, Z = 0;
	FVector() = default;
	FVector(double InX, double InY, double InZ) : X(InX), Y(InY), Z(InZ) {}
	FVector operator+(const FVector& O) const { return {X + O.X, Y + O.Y, Z + O.Z}; }
	FVector operator-(const FVector& O) const { return {X - O.X, Y - O.Y, Z - O.Z}; }
	FVector operator-() const { return {-X, -Y, -Z}; }
	FVector operator*(double S) const { return {X * S, Y * S, Z * S}; }
	FVector operator/(double S) const { return {X / S, Y / S, Z / S}; }
	FVector operator*(const FVector& O) const { return {X * O.X, Y * O.Y, Z * O.Z}; }
	FVector operator/(const FVector& O) const { return {X / O.X, Y / O.Y, Z / O.Z}; }
	FVector& operator+=(const FVector& O) { X += O.X; Y += O.Y; Z += O.Z; return *this; }
	double Size() const { return std::sqrt(X * X + Y * Y + Z * Z); }
	static FVector CrossProduct(const FVector& A, const FVector& B)
	{
		return {A.Y * B.Z - A.Z * B.Y, A.Z * B.X - A.X * B.Z, A.X * B.Y - A.Y * B.X};
	}
	static const FVector ZeroVector;
};
inline const FVector FVector::ZeroVector{0, 0, 0};

struct FQuat
{
	double X = 0, Y = 0, Z = 0, W = 1;
	FQuat() = default;
	FQuat(double InX, double InY, double InZ, double InW) : X(InX), Y(InY), Z(InZ), W(InW) {}
	FQuat operator*(const FQuat& Q) const  // Hamilton product, like UE: (this * Q) applies Q first
	{
		return {W * Q.X + X * Q.W + Y * Q.Z - Z * Q.Y, W * Q.Y - X * Q.Z + Y * Q.W + Z * Q.X,
		        W * Q.Z + X * Q.Y - Y * Q.X + Z * Q.W, W * Q.W - X * Q.X - Y * Q.Y - Z * Q.Z};
	}
	FVector RotateVector(const FVector& V) const
	{
		const FVector Q(X, Y, Z);
		const FVector T = FVector::CrossProduct(Q, V) * 2.0;
		return V + T * W + FVector::CrossProduct(Q, T);
	}
	FQuat Inverse() const { return {-X, -Y, -Z, W}; }
	void Normalize()
	{
		const double N = std::sqrt(X * X + Y * Y + Z * Z + W * W);
		X /= N; Y /= N; Z /= N; W /= N;
	}
	double GetAngle() const { return 2.0 * std::acos(std::min(1.0, std::abs(W))); }
	static const FQuat Identity;
};
inline const FQuat FQuat::Identity{0, 0, 0, 1};

struct FColor
{
	uint8 R = 255, G = 255, B = 255, A = 255;
	FColor() = default;
	FColor(uint8 InR, uint8 InG, uint8 InB, uint8 InA = 255) : R(InR), G(InG), B(InB), A(InA) {}
	static const FColor White;
};
inline const FColor FColor::White{255, 255, 255, 255};

template <typename T>
class TArray
{
public:
	T& AddDefaulted_GetRef() { return Data.emplace_back(); }
	void Reset() { Data.clear(); }
	int32 Num() const { return static_cast<int32>(Data.size()); }
	auto begin() { return Data.begin(); }
	auto end() { return Data.end(); }
	auto begin() const { return Data.begin(); }
	auto end() const { return Data.end(); }

private:
	std::vector<T> Data;
};

struct FMath
{
	template <typename T> static T Min(T A, T B) { return A < B ? A : B; }
	template <typename T> static T Max(T A, T B) { return A > B ? A : B; }
	template <typename T> static T Max3(T A, T B, T C) { return Max(Max(A, B), C); }
	template <typename T> static T Clamp(T V, T Lo, T Hi) { return V < Lo ? Lo : (V > Hi ? Hi : V); }
	static double RadiansToDegrees(double R) { return R * (180.0 / 3.14159265358979323846); }
	static double DegreesToRadians(double D) { return D * (3.14159265358979323846 / 180.0); }
};
