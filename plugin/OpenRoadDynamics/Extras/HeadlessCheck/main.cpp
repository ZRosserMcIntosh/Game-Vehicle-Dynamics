// OpenRoad Dynamics — headless check of the UE adapter's physics-thread code.
// SPDX-License-Identifier: MIT
//
// NOT an Unreal test. Compiles Private/OrdVehicleSim.cpp (the physics-thread
// wrapper), OrdFrame.h and OrdGroundHorizon.h against small stand-ins for
// FVector/FQuat/TArray (ue_stubs/), and drives FOrdVehicleSim with a rigid-body
// integrator written in UE axes and units (cm, left-handed, kg*cm^2), the way
// Chaos would. The "game thread" rebuilds a GroundPrefetchCache once per 60 Hz
// frame from the snapshots' strip requests, with one frame of output latency.
//
// What this verifies (source level only):
//  * frame/unit conversions and CoM transport: host-sync error ~0 and the
//    trajectory matches a ChassisMode::Internal reference run;
//  * the velocity-reference diagnostic identifies a CoM-velocity host;
//  * per-vehicle miss accounting, cache-age measurement and the adaptive
//    horizon under emulated game-thread hitches.
// What it cannot verify: anything about Chaos itself (see docs/AUDIT.md).
//
// Build and run: ./run.sh   (needs a C++20 compiler; no Unreal Engine)

#include <cmath>
#include <cstdio>
#include <deque>
#include <memory>
#include <vector>

#include "OrdFrame.h"
#include "OrdGroundHorizon.h"
#include "OrdVehicleSim.h"
#include "ord/ground.hpp"
#include "ord/ground_cache.hpp"
#include "ord/presets.hpp"
#include "ord/vehicle.hpp"
#include "vehicles/reference_vehicles.hpp"

namespace
{
int Failures = 0;
void Expect(bool Ok, const char* What, double Value)
{
	std::printf("  [%s] %s (%.6g)\n", Ok ? "ok" : "FAIL", What, Value);
	Failures += Ok ? 0 : 1;
}

/** Rigid body integrated like a PBD/semi-implicit solver, entirely in UE axes and units. */
struct FUeBody
{
	FVector X;            // origin, cm
	FQuat R;              // body -> world
	FVector V;            // CoM velocity, cm/s
	FVector W;            // rad/s (UE axial convention)
	FVector ComLocal;     // cm
	double Mass = 1;      // kg
	FVector I;            // body-axis principal inertia, kg*cm^2
	double Gravity = -980.665;

	FVector Com() const { return X + R.RotateVector(ComLocal); }

	FOrdBodyState State() const
	{
		FOrdBodyState S;
		S.Position = X;
		S.Rotation = R;
		S.CenterOfMass = Com();
		S.LinearVelocity = V;
		S.AngularVelocity = W;
		S.MassKg = Mass;
		S.InertiaKgCm2 = I;
		return S;
	}

	void Integrate(double Dt, const FVector& F, const FVector& T)
	{
		const FVector C0 = Com();
		V += (F / Mass + FVector(0, 0, Gravity)) * Dt;
		// Angular: in body axes, with the gyroscopic term (the core's own integrator has it too).
		const FQuat Rinv = R.Inverse();
		const FVector Wb = Rinv.RotateVector(W);
		const FVector Tb = Rinv.RotateVector(T);
		const FVector Lb = I * Wb;
		const FVector Gyro = FVector::CrossProduct(Wb, Lb);
		const FVector Wb1 = Wb + (Tb - Gyro) / I * Dt;
		W = R.RotateVector(Wb1);
		const FVector C1 = C0 + V * Dt;
		const FQuat Qw(W.X, W.Y, W.Z, 0.0);
		const FQuat D = Qw * R;
		R = FQuat(R.X + 0.5 * Dt * D.X, R.Y + 0.5 * Dt * D.Y, R.Z + 0.5 * Dt * D.Z, R.W + 0.5 * Dt * D.W);
		R.Normalize();
		X = C1 - R.RotateVector(ComLocal);
	}
};

/**
 * The reference sports car with its origin moved to the ground (CoM 0.45 m above
 * it), like the UE asset defaults: the body origin and the CoM differ, which the
 * velocity-reference check needs.
 */
ord::VehicleDefinition TestDefinition()
{
	ord::VehicleDefinition Def = ord::vehicles::referenceSportsRwd();
	const ord::Vec3 Shift{0.0, 0.0, 0.45};
	Def.centerOfMass = Def.centerOfMass + Shift;
	for (ord::WheelDef& W : Def.wheels)
	{
		W.suspension.hardpoint = W.suspension.hardpoint + Shift;
	}
	return Def;
}

ord::DriverInput Script(double T)
{
	ord::DriverInput In;
	In.throttle = T < 5.0 ? 1.0 : 0.0;
	In.steer = (T > 2.0 && T < 5.0) ? 0.15 * std::sin(1.5 * (T - 2.0)) : 0.0;
	In.brake = T > 6.0 ? 0.6 : 0.0;
	return In;
}

struct FRunConfig
{
	double PhysicsHz = 240.0;
	double VehicleRateHz = 240.0;
	double Seconds = 8.0;
	FVector HostComError = FVector::ZeroVector;  // cm, Chaos CoM differs from the definition's
	std::vector<double> HitchTimes;              // game frames that take HitchSeconds
	double HitchSeconds = 0.1;
	bool bAdaptive = true;
	bool bFullThrottle = false;
};

struct FRunResult
{
	ord::Vec3 FinalPosition;
	double MaxSyncCm = 0, MaxSyncDeg = 0;
	double MaxVelCheckCom = 0, MaxVelCheckOrigin = 0;
	uint64 Misses = 0;
	uint64 MissesSecondHitch = 0;
	double MaxAge = 0, MaxHorizon = 0;
	double TimeTo100 = -1;  // s, from the snapshots' speed
	bool bFaulted = false;
};

FRunResult Run(const FRunConfig& Cfg)
{
	const ord::VehicleDefinition Def = TestDefinition();
	const ord::FlatGround Ground(0.0);
	ord::AssistConfig Assists = ord::makePreset(ord::DrivingPreset::Simulation);
	Assists.shiftMode = ord::ShiftMode::Automatic;

	// Rest pose (chassis and wheels) from the core, then hand the chassis to the "UE" host.
	auto Vehicle = std::make_unique<ord::Vehicle>(Def, Assists);
	Vehicle->reset({0, 0, 0}, 0.0, Ground);
	const ord::RigidBodyState Rest = Vehicle->state();
	Vehicle->setChassisMode(ord::ChassisMode::External, /*hostAppliesGravity*/ true);

	FUeBody Body;
	Body.X = OrdFrame::PositionToUe(Rest.position);
	Body.R = OrdFrame::RotationToUe(Rest.orientation);
	Body.ComLocal = OrdFrame::PositionToUe(Def.centerOfMass) + Cfg.HostComError;
	Body.Mass = Def.mass;
	Body.I = OrdFrame::InertiaToUe(Def.inertia);

	std::vector<ord::GroundStripRequest> Requests;
	Vehicle->groundPrefetchRequests(Requests);
	double RequestTime = 0.0;
	FOrdVehicleSim Sim(std::move(Vehicle));

	const double PhysicsDt = 1.0 / Cfg.PhysicsHz;
	const double FrameDt = 1.0 / 60.0;
	OrdGround::FHorizonState Horizon;
	Horizon.Horizon = 0.075;
	std::vector<std::shared_ptr<ord::GroundPrefetchCache>> Caches;
	std::deque<std::vector<FOrdSimSnapshot>> OutputQueue;  // one entry per frame: output latency
	FRunResult Res;
	double T = 0.0, PhysicsT = 0.0, LastFrameDt = FrameDt;
	uint32 ShiftCount = 0;
	std::size_t HitchIndex = 0;
	const uint64 SecondHitchStart = 0;
	uint64 MissesBeforeSecondHitch = 0;
	(void)SecondHitchStart;

	while (T < Cfg.Seconds)
	{
		// ---- game thread: consume outputs delayed by one frame ----
		double Consumed = 0.0, FrameAge = 0.0;
		if (OutputQueue.size() > 1)
		{
			for (const FOrdSimSnapshot& S : OutputQueue.front())
			{
				Consumed += S.DeltaTime;
				if (S.NumStripRequests > 0)
				{
					Requests.assign(S.StripRequests.begin(), S.StripRequests.begin() + S.NumStripRequests);
					RequestTime = S.StripRequestTime;
				}
				if (S.bValid)
				{
					FrameAge = std::max(FrameAge, S.GroundCacheAge);
				}
			}
			OutputQueue.pop_front();
		}
		const double H = OrdGround::UpdateHorizon(Horizon, LastFrameDt, Consumed, FrameAge, PhysicsDt, Cfg.bAdaptive, 0.075, 0.25);
		Res.MaxHorizon = std::max(Res.MaxHorizon, H);
		ord::GroundPrefetchCache::Settings Settings;
		Settings.horizon = H;
		auto Cache = std::make_shared<ord::GroundPrefetchCache>(Settings);
		Cache->rebuild(Ground, Requests.data(), Requests.size());
		Caches.push_back(Cache);

		FOrdGameInput Input;
		Input.Driver = Script(T);
		if (Cfg.bFullThrottle)
		{
			Input.Driver = ord::DriverInput();
			Input.Driver.throttle = 1.0;
		}
		Input.ShiftUpCount = ShiftCount;
		Input.Assists = Assists;
		Input.AssistsRevision = 1;
		Input.Ground = Cache;
		Input.GroundRequestTime = RequestTime;
		Input.VehicleRateHz = Cfg.VehicleRateHz;

		// ---- this frame's duration (hitches) ----
		double ThisFrame = FrameDt;
		if (HitchIndex < Cfg.HitchTimes.size() && T >= Cfg.HitchTimes[HitchIndex])
		{
			ThisFrame = Cfg.HitchSeconds;
			if (HitchIndex == 1)
			{
				MissesBeforeSecondHitch = Res.Misses;
			}
			++HitchIndex;
		}
		T += ThisFrame;
		LastFrameDt = ThisFrame;

		// ---- physics thread: every fixed step up to the new game time ----
		std::vector<FOrdSimSnapshot> Frame;
		while (PhysicsT + 0.5 * PhysicsDt < T)
		{
			FOrdSimSnapshot Snap;
			const FOrdBodyWrench Wrench = Sim.Step(PhysicsDt, Body.State(), Input, Snap);
			if (Wrench.bApply)
			{
				Body.Integrate(PhysicsDt, Wrench.Force, Wrench.Torque);
			}
			else
			{
				Body.Integrate(PhysicsDt, FVector::ZeroVector, FVector::ZeroVector);
			}
			PhysicsT += PhysicsDt;
			if (Snap.bValid && Snap.SimTime > 0.5)  // skip the initial settle
			{
				Res.MaxSyncCm = std::max(Res.MaxSyncCm, Snap.HostSyncPositionErrorCm);
				Res.MaxSyncDeg = std::max(Res.MaxSyncDeg, Snap.HostSyncAngleErrorDeg);
				Res.MaxVelCheckCom = std::max(Res.MaxVelCheckCom, Snap.VelocityCheckComCmS);
				Res.MaxVelCheckOrigin = std::max(Res.MaxVelCheckOrigin, Snap.VelocityCheckOriginCmS);
			}
			if (Res.TimeTo100 < 0 && Snap.bValid && Snap.SpeedKmh >= 100.0)
			{
				Res.TimeTo100 = PhysicsT;
			}
			Res.Misses = Snap.GroundCacheMisses;
			Res.MaxAge = std::max(Res.MaxAge, Snap.GroundCacheAge);
			Res.bFaulted |= Snap.bFaulted;
			Frame.push_back(Snap);
		}
		OutputQueue.push_back(std::move(Frame));
		// Release caches no longer referenced (the pool would recycle them).
		std::erase_if(Caches, [](const auto& C) { return C.use_count() == 1; });
	}
	if (Cfg.HitchTimes.size() > 1)
	{
		Res.MissesSecondHitch = Res.Misses - MissesBeforeSecondHitch;
	}
	Res.FinalPosition = OrdFrame::PositionFromUe(Body.X);
	return Res;
}

ord::Vec3 InternalReference(double Hz, double Seconds)
{
	const ord::VehicleDefinition Def = TestDefinition();
	const ord::FlatGround Ground(0.0);
	ord::AssistConfig Assists = ord::makePreset(ord::DrivingPreset::Simulation);
	Assists.shiftMode = ord::ShiftMode::Automatic;
	ord::Vehicle V(Def, Assists);
	V.reset({0, 0, 0}, 0.0, Ground);
	const double Dt = 1.0 / Hz;
	// Same time base as the hosted run: inputs are sampled per 60 Hz game frame.
	double T = 0.0;
	while (T < Seconds)
	{
		const ord::DriverInput In = Script(T);
		const double FrameEnd = T + 1.0 / 60.0;
		for (int K = 0; K < static_cast<int>(std::lround(Hz / 60.0)); ++K)
		{
			V.step(Dt, In, Ground);
		}
		T = FrameEnd;
	}
	return V.state().position;
}
}  // namespace

int main()
{
	std::printf("OpenRoad UE adapter headless check (stand-in UE types; NOT Unreal Engine)\n");

	// Conversion identities the adapter relies on (mirror M = diag(1,-1,1), x100).
	{
		const ord::Vec3 R{0.3, -1.2, 0.5}, F{120.0, 3400.0, -800.0};
		const FVector TorqueUe = FVector::CrossProduct(OrdFrame::PositionToUe(R), OrdFrame::ForceToUe(F));
		const FVector TorqueConv = OrdFrame::TorqueToUe(ord::cross(R, F));
		Expect((TorqueUe - TorqueConv).Size() < 1e-6, "UE r x F == TorqueToUe(core r x F)", (TorqueUe - TorqueConv).Size());
		const ord::Quat Q = ord::Quat::fromAxisAngle({0.2, 0.9, -0.4}, 0.7);
		const FVector A = OrdFrame::RotationToUe(Q).RotateVector(OrdFrame::PositionToUe(R));
		const FVector B = OrdFrame::PositionToUe(Q.rotate(R));
		Expect((A - B).Size() < 1e-9, "FQuat::RotateVector agrees with core Quat::rotate through the mirror", (A - B).Size());
		const ord::Vec3 W{0.4, -0.7, 1.1};
		const FVector VUe = FVector::CrossProduct(OrdFrame::AngularVelocityToUe(W), OrdFrame::PositionToUe(R));
		const FVector VConv = OrdFrame::VelocityToUe(ord::cross(W, R));
		Expect((VUe - VConv).Size() < 1e-9, "UE w x r == VelocityToUe(core w x r)", (VUe - VConv).Size());
	}

	std::printf("A. 240 Hz world, 1 vehicle substep\n");
	FRunConfig A;
	const FRunResult RA = Run(A);
	const ord::Vec3 Ref = InternalReference(240.0, A.Seconds);
	const double DevA = ord::length(RA.FinalPosition - Ref);
	Expect(DevA < 0.01, "final position vs ChassisMode::Internal 240 Hz reference, m", DevA);
	Expect(RA.MaxSyncCm < 0.01, "max host-sync position error, cm", RA.MaxSyncCm);
	Expect(RA.MaxVelCheckCom < 1e-3, "velocity check (CoM) stays ~0 for a CoM-velocity host, cm/s", RA.MaxVelCheckCom);
	Expect(RA.MaxVelCheckOrigin > 1.0, "velocity check (origin) is clearly non-zero while rotating, cm/s", RA.MaxVelCheckOrigin);
	Expect(RA.Misses == 0, "ground-cache misses", static_cast<double>(RA.Misses));
	Expect(!RA.bFaulted, "no faulted steps", 0.0);

	std::printf("B. 60 Hz world, 4 vehicle substeps\n");
	FRunConfig B;
	B.PhysicsHz = 60.0;
	const FRunResult RB = Run(B);
	const double DevB = ord::length(RB.FinalPosition - Ref);
	const double Travel = ord::length(Ref);
	// The host integrates one 1/60 s step with the impulse-equivalent average
	// wrench: velocity matches the substeps, position differs by O(a * dt^2)
	// (~1-2 mm per step). Over the 8 s steer/brake script that drifts the end
	// point by ~2 % of the distance; parity metrics are compared below instead.
	std::printf("  info: end point vs 240 Hz Internal reference: %.2f m of %.1f m (%.2f %%)\n", DevB, Travel, 100.0 * DevB / Travel);
	Expect(RB.MaxSyncCm < 0.5, "max host-sync position error per 60 Hz step, cm (O(a dt^2) by design)", RB.MaxSyncCm);
	Expect(RB.Misses == 0, "ground-cache misses", static_cast<double>(RB.Misses));
	FRunConfig A100 = A;
	A100.bFullThrottle = true;
	A100.Seconds = 9.0;
	FRunConfig B100 = B;
	B100.bFullThrottle = true;
	B100.Seconds = 9.0;
	const double TA = Run(A100).TimeTo100, TB = Run(B100).TimeTo100;
	std::printf("  info: 0-100 km/h: 240 Hz world %.3f s, 60 Hz world + 4 substeps %.3f s\n", TA, TB);
	Expect(TA > 0 && TB > 0 && std::abs(TB - TA) / TA < 0.01, "0-100 km/h time, 60 Hz+4 substeps vs 240 Hz, relative", std::abs(TB - TA) / TA);

	std::printf("C. Chaos CoM 3 cm off the definition (CoM transport)\n");
	FRunConfig C;
	C.HostComError = FVector(3.0, -2.0, 1.0);
	const FRunResult RC = Run(C);
	Expect(std::isfinite(RC.FinalPosition.x) && !RC.bFaulted, "stable with a CoM mismatch", ord::length(RC.FinalPosition - Ref));

	std::printf("D. Two 100 ms game-thread hitches at full throttle (fixed vs adaptive horizon)\n");
	FRunConfig D;
	D.bFullThrottle = true;
	D.Seconds = 9.0;
	D.HitchTimes = {6.0, 7.0};
	D.bAdaptive = false;
	const FRunResult RDFixed = Run(D);
	D.bAdaptive = true;
	const FRunResult RDAdaptive = Run(D);
	std::printf("  fixed 0.075 s: misses %llu (second hitch %llu), max cache age %.3f s\n",
	            static_cast<unsigned long long>(RDFixed.Misses), static_cast<unsigned long long>(RDFixed.MissesSecondHitch), RDFixed.MaxAge);
	std::printf("  adaptive     : misses %llu (second hitch %llu), max cache age %.3f s, max horizon %.3f s\n",
	            static_cast<unsigned long long>(RDAdaptive.Misses), static_cast<unsigned long long>(RDAdaptive.MissesSecondHitch),
	            RDAdaptive.MaxAge, RDAdaptive.MaxHorizon);
	Expect(RDFixed.MissesSecondHitch > 0, "fixed horizon runs past its strips in a 100 ms hitch", static_cast<double>(RDFixed.MissesSecondHitch));
	Expect(RDAdaptive.MissesSecondHitch == 0, "adaptive horizon covers a repeated hitch", static_cast<double>(RDAdaptive.MissesSecondHitch));
	Expect(RDAdaptive.MaxHorizon > RDAdaptive.MaxAge - 1e-9 || RDAdaptive.MissesSecondHitch == 0, "horizon >= measured age after adaptation",
	       RDAdaptive.MaxHorizon);
	Expect(!RDAdaptive.bFaulted && !RDFixed.bFaulted, "no faulted steps through hitches", 0.0);

	std::printf("%s (%d failure%s)\n", Failures ? "FAILED" : "PASSED", Failures, Failures == 1 ? "" : "s");
	return Failures ? 1 : 0;
}
