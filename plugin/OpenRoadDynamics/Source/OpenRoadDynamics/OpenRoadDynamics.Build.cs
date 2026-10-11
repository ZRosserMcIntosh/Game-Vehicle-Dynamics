// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT

using UnrealBuildTool;

public class OpenRoadDynamics : ModuleRules
{
	public OpenRoadDynamics(ReadOnlyTargetRules Target) : base(Target)
	{
		// No shared PCH and no unity build: OrdCoreAmalgamation.cpp must be compiled
		// without any engine header in front of it (the core is engine-independent),
		// and every adapter file includes what it uses.
		PCHUsage = PCHUsageMode.NoPCHs;
		bUseUnity = false;

		CppStandard = CppStandardVersion.Cpp20;
		bEnableExceptions = false; // the core never throws (Vehicle::isValid()/error())

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"PhysicsCore",
			"OpenRoadCore",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"Chaos",
			"ChaosSolverEngine",
			// Demo pawn only (AOpenRoadDemoVehicle): runtime-created Enhanced Input actions.
			"EnhancedInput",
			"InputCore",
		});

		// Adds the Chaos/physics defines and dependencies for this target.
		SetupModulePhysicsSupport(Target);
	}
}
