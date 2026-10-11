// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// Adaptive ground-strip horizon. Plain C++ (no engine types) so it is tested
// headless (Extras/HeadlessCheck).
//
// The ground cache built in game frame k is read by every physics step until
// the cache of frame k+1 arrives. Its strips start at the wheel positions of
// the physics step whose snapshot carried the requests, so the strip must
// cover the car's travel over that whole interval ("cache age"). The physics
// thread measures the age exactly (FOrdSimSnapshot::GroundCacheAge); the game
// thread feeds it back here together with its own frame time:
//
//   horizon = clamp(max(MinHorizon, 1.5 * peakFrameDt, 1.25 * peakCacheAge + physicsDt),
//                   1/60 s, MaxHorizon)
//
// Peaks rise immediately and decay by kPeakDecayPerSecond (0.02 s per second of
// game time), so after one 100 ms hitch the strips stay long for ~5 s and a
// repeated hitch is covered (headless check D). The result is quantised up to 1/120 s
// and changed only when it rises or falls by more than 20 %, so pooled caches
// (whose settings are fixed per object) are rarely re-created.
//
// A first, unannounced hitch cannot be predicted: until the next cache arrives
// the core extrapolates the nearest strip (flat continuation) and counts
// misses. Note that with async physics the physics thread does not run ahead
// of a stalled game thread on its own; the game thread hands it at most
// MaxPhysicsDeltaTime of simulation per frame (UNVERIFIED, see docs/AUDIT.md).
#pragma once

#include <algorithm>
#include <cmath>

namespace OrdGround
{
struct FHorizonState
{
	double PeakFrameDt = 0.0;
	double PeakCacheAge = 0.0;
	double Horizon = 0.075;
};

/**
 * @param FrameDt            game-thread frame time, s
 * @param PhysicsTimeConsumed sum of the physics steps' dt whose snapshots arrived this frame, s
 * @param MaxCacheAge         largest FOrdSimSnapshot::GroundCacheAge among them, s
 * @param PhysicsDt           physics (world) step, s
 * @return the horizon to build this frame's cache with
 */
inline double UpdateHorizon(FHorizonState& S, double FrameDt, double PhysicsTimeConsumed, double MaxCacheAge, double PhysicsDt,
                            bool bAdaptive, double MinHorizon, double MaxHorizon)
{
	constexpr double kFloor = 1.0 / 60.0;
	constexpr double kPeakDecayPerSecond = 0.02;
	const double Decay = kPeakDecayPerSecond * std::max(FrameDt, 0.0);
	S.PeakFrameDt = std::max(std::max(FrameDt, PhysicsTimeConsumed), S.PeakFrameDt - Decay);
	S.PeakCacheAge = std::max(MaxCacheAge, S.PeakCacheAge - Decay);
	double Target = std::max(MinHorizon, kFloor);
	if (bAdaptive)
	{
		Target = std::max({MinHorizon, 1.5 * S.PeakFrameDt, 1.25 * S.PeakCacheAge + PhysicsDt});
		Target = std::clamp(Target, kFloor, std::max(MaxHorizon, kFloor));
	}
	Target = std::ceil(Target * 120.0 - 1e-9) / 120.0;
	if (Target > S.Horizon || Target < 0.8 * S.Horizon)
	{
		S.Horizon = Target;
	}
	return S.Horizon;
}
}  // namespace OrdGround
