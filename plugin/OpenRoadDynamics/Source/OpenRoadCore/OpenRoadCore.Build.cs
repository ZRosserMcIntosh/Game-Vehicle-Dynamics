// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// OpenRoadCore: the engine-independent physics core (src/core), exposed to
// Unreal as an *External* (include-path) module.
//
// Why External and not a compiled Runtime module: the core carries no
// dllexport/visibility annotations (it must stay engine-free and unchanged), so
// a separately compiled OpenRoadCore DLL would export nothing in modular
// (editor) builds and OpenRoadDynamics would fail to link. Instead, this module
// only publishes the include paths and OpenRoadDynamics compiles the core into
// its own binary through Private/OrdCoreAmalgamation.cpp.
//
// Core location, in order of preference:
//   1. <ModuleDirectory>/Core            packaged copy (tools/package_plugin.py)
//   2. <repository>/src/core             when the plugin lives in the repository
//      (<ModuleDirectory>/../../../../src/core)

using System.IO;
using UnrealBuildTool;

public class OpenRoadCore : ModuleRules
{
	public OpenRoadCore(ReadOnlyTargetRules Target) : base(Target)
	{
		Type = ModuleType.External;

		string PackagedRoot = Path.Combine(ModuleDirectory, "Core");
		string RepositoryRoot = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", "..", "..", "..", "src", "core"));

		string CoreRoot = null;
		if (IsCoreRoot(PackagedRoot))
		{
			CoreRoot = PackagedRoot;
		}
		else if (IsCoreRoot(RepositoryRoot))
		{
			CoreRoot = RepositoryRoot;
		}
		else
		{
			throw new BuildException(
				"OpenRoadCore: engine-independent core not found. Looked in '" + PackagedRoot + "' and '" + RepositoryRoot +
				"'. Run 'python3 tools/package_plugin.py' in the OpenRoad Dynamics repository before copying the plugin into a project.");
		}

		// System include paths: the core is compiled with its own (stricter) warning
		// set in the repository CI; here it is treated like a third-party library.
		//  - <CoreRoot>/include  : public headers, included as "ord/vehicle.hpp"
		//  - <CoreRoot>          : lets OrdCoreAmalgamation.cpp include "src/<file>.cpp"
		PublicSystemIncludePaths.Add(Path.Combine(CoreRoot, "include"));
		PublicSystemIncludePaths.Add(CoreRoot);

		PublicDefinitions.Add("ORD_CORE_PACKAGED=" + (CoreRoot == PackagedRoot ? "1" : "0"));
	}

	private static bool IsCoreRoot(string Root)
	{
		return File.Exists(Path.Combine(Root, "include", "ord", "vehicle.hpp")) &&
			File.Exists(Path.Combine(Root, "src", "vehicle.cpp"));
	}
}
