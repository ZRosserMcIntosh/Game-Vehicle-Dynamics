# Vehicle-physics alternatives, references and licensing

**Project:** OpenRoad Dynamics (MIT, engine-independent C++ core with a thin UE5 plugin adapter)
**Status:** Research note, written 2026-10-10
**Question:** What existing implementations and published models can we **reuse**, **study**, or **only read about**, given that our code must stay MIT-licensed?

> **Verification method.** Licenses were checked by downloading the actual LICENSE/COPYING file from each repo (raw.githubusercontent.com) on 2026-10-10, or from the project's SourceForge page. Where a vendor page could not be reached (most vendor sites were blocked by the research proxy), the claim rests on search-engine extracts of the official page and is marked *(extract)*. "Public" ≠ "open source" ≠ "MIT-compatible".
>
> **Reuse categories.**
> - **Direct reuse:** code may be vendored or linked from our MIT repo, keeping its notices.
> - **Reference only:** may be read and used as a behavioural or validation oracle, but no code is copied into our repo.
> - **Study published math only:** implement independently from papers and books. Do not look at or port the implementation.
> - **None:** do not use beyond public marketing information.

---

## 1. Key findings

1. **Jolt Physics (MIT)** is the best *direct-reuse* candidate. Its `Jolt/Physics/Vehicle` module has wheeled, tracked and motorcycle controllers, an engine with a torque curve, a manual/auto transmission with clutch, differentials with limited-slip ratio, anti-roll bars, and ray, sphere-cast and cylinder-cast wheel collision. Its tire model is simple (piecewise-linear friction vs slip curves), but it has a user callback hook for the tire impulse.
2. **NVIDIA PhysX vehicle module: a license change.** As of **PhysX 5.11.0**, the changelog says "PhysX SDK public source code is now licensed under the **Apache License 2.0**", and `physx/LICENSE.md` plus per-file `SPDX-License-Identifier: Apache-2.0` headers confirm it. The repo-root `LICENSE.md` and README still show **BSD-3-Clause**, and older releases were BSD-3. **Both licenses are MIT-compatible** (Apache-2.0 adds NOTICE/attribution and patent terms). Also, **"vehicle2" no longer exists by that name.** Since PhysX 5.7 the headers moved from `include/vehicle2` to `include/vehicle`, the library is `PhysXVehicle`, and the `vehicle2` namespace was removed after the legacy vehicle API was deleted in 5.6.
3. **Project Chrono (BSD-3)** is the best *validation reference*. It has Pacejka 2002 (`ChPac02Tire`, reads ADAMS-style `.tir`), Pac89, TMeasy, TMsimple, Fiala, and rigid/ANCF/FEA tires, plus full multibody suspensions and published vehicle parameter sets (for example a BMW E90 "taken from SAE Paper 2007-01-0818").
4. **A UE5 plugin to avoid:** `Kouroshx99/VehiclePhysics` (MF 6.1 tires, hardpoint double wishbone) is **non-commercial academic use only**. It is *not* open source and not MIT-compatible. Treat it as "none" (do not read its code).
5. **A public re-host of Epic's Chaos Vehicles code**, `Nobatgeldi/ChaosVehiclesPlugin` ("license-Epic Games" badge, no LICENSE file), should not be used or forked. Epic engine code may only be shared with other licensees or through Epic's GitHub fork network.
6. **Bullet's `btRaycastVehicle`** carries its **own permissive notice** (Erwin Coumans 2005, an HPND-style "permission to use, copy, modify, distribute and sell") inside a repo that is otherwise zlib. Both are MIT-compatible.

---

## 2. Engine and middleware implementations

### 2.1 Unreal Engine Chaos Vehicles (ChaosVehicles plugin + ChaosVehiclesCore)
- **What:** UE5's built-in wheeled-vehicle system (`UChaosWheeledVehicleMovementComponent`, `UChaosVehicleWheel`, core structs `FSimpleTireConfig`, `FSimpleSuspensionConfig`, `FSimpleWheelConfig`). A separate **Chaos Modular Vehicle** plugin builds vehicles from Geometry Collections under a Cluster Union for construction and destruction at runtime. Epic says that plugin does not require the standard Chaos Vehicle plugin. *(extract)*
- **License:** Unreal Engine EULA. Source is available to licensees, but **Engine Code may be distributed only to other Engine Licensees**. Public distribution containing Engine Tools must go through Fab/Marketplace or a fork of Epic's GitHub UnrealEngine network. The EULA also bars combining Engine Code with copyleft (GPL/LGPL) code. *(extract: EULA v15 PDF, cdn2.unrealengine.com, and summaries)*
- **Capabilities (from Epic API docs, extract):** per-wheel suspension trace (ray trace by default, optional shape sweep; simple or complex collision). Suspension parameters: SpringRate, SpringPreload, CompressionDamping, ReboundDamping, MaxLength, RaycastSafetyMargin. Tires: static and kinetic friction coefficients, radius, width, `CorneringStiffness`, `FrictionForceMultiplier`, `LateralSlipGraph`, ABS and TCS flags. The 4.27 `FSimpleWheelConfig` had SlipThreshold/SkidThreshold and lateral/longitudinal friction multipliers. Chaos can run physics on a separate async thread at a fixed step.
- **Known limitations (community, low–medium confidence):** the tire model is a simple friction/cornering-stiffness approach, not Pacejka/TMeasy. Suspension can oscillate on frame-time spikes. One user reported that with Async Physics enabled, bodies "slow down" when the game-thread dt drops below the max physics dt (Epic forum, reproduced with the vehicle template). One blog claims UE 5.7 improved suspension stability under hitches above 20 ms; we could not confirm this in Epic release notes.
- **Reuse:** **Reference only.** Our UE5 adapter may *depend on* engine APIs (that is our own code calling Epic's), but we must **not copy any Chaos source into the MIT repo**. Use it as a behavioural A/B baseline inside UE.

### 2.2 Unity WheelCollider
- **What:** Unity's built-in wheel: "built-in collision detection, wheel physics, and a slip-based tire friction model". It uses the **PhysX 4.1 Vehicles SDK** for wheels, suspension and braking. The friction curve is a two-piece spline (extremum and asymptote points, plus a stiffness multiplier). Wheels ignore PhysicsMaterial, so surfaces are simulated by changing `forwardFriction`/`sidewaysFriction`. *(extract, Unity Manual 2021.3 → Unity 6)*
- **License:** Proprietary (Unity Terms of Service). The engine source is not open.
- **Reuse:** **None** (study the public docs only). Its behaviour is essentially PhysX 4 vehicle tire behaviour.

### 2.3 NVIDIA PhysX 5 vehicle module (formerly "vehicle2")
- **License (verified):** root `LICENSE.md` = **BSD-3-Clause** (© 2008-2025 NVIDIA). `physx/LICENSE.md` = **Apache License 2.0**. Vehicle headers carry `SPDX-License-Identifier: Apache-2.0` (© 2008-2026). The CHANGELOG for v5.11.0 says "PhysX SDK public source code is now licensed under the Apache License 2.0." The current version file reads `5.11.0`. The older `NVIDIAGameWorks/PhysX` (4.1) is BSD-3.
- **Capabilities (from headers in `physx/include/vehicle/`):**
  - Tire: `PxVehicleTireForceParams` with load-dependent lateral stiffness (`latStiffX/latStiffY`), `longStiff`, `camberStiff`, and a 3-point `frictionVsSlip` graph. A function pipeline: `PxVehicleTireDirsUpdate`, `…SlipSpeedsUpdate`, `…SlipsUpdate`, `…CamberAnglesUpdate`, `…GripUpdate`, `…StickyStateUpdate` (low-speed sticky tire), `…ForcesUpdate`.
  - Suspension: `PxVehicleSuspensionParams`, compliance (toe/camber vs travel), force params, anti-roll bars (`PxVehicleAntiRollForceParams`).
  - Drivetrain: engine, clutch (with accuracy modes), gearbox, autobox, **multi-wheel, four-wheel and tank differentials** (a legacy 4WD diff is deprecated), and direct-drive.
  - Road geometry: `eRAYCAST` or `eSWEEP` queries.
  - Architecture: a data-oriented "component" pipeline with explicit parameter/state structs. The vehicle code is largely separable from the rigid-body solver (a direct-integration mode is available).
- **Reuse:** **Direct reuse possible** (Apache-2.0: keep LICENSE and notices, mark modifications). Most valuable as a **reference architecture** for a modular, SI-unit, data-oriented vehicle pipeline. If we vendor any file, add `third_party/physx/LICENSE` and an Apache NOTICE section.
- **Quality notes:** production-grade and well documented (PhysX Guide vehicle chapter). The tire model is the classic PhysX "linear stiffness + friction vs slip graph", not Pacejka, but the pipeline lets you replace the tire step.

### 2.4 Jolt Physics: `VehicleConstraint`
- **License (verified):** **MIT** (© 2021 Jorrit Rouwe). Latest release in notes: v5.6.0, plus unreleased changes.
- **Capabilities (verified in `Jolt/Physics/Vehicle/*.h`):** `VehicleConstraint` (with max pitch/roll angle limiting), `WheeledVehicleController`, `TrackedVehicleController`, `MotorcycleController`. `VehicleEngine` has max torque, a normalized torque curve vs RPM, and inertia. `VehicleTransmission` has Auto/Manual modes, gear ratios, reverse ratios, clutch strength, and shift up/down RPM and timing. `VehicleDifferential` and a controller-level `mDifferentialLimitedSlipRatio`. `VehicleAntiRollBar`. Collision testers: `VehicleCollisionTesterRay`, `…CastSphere`, `…CastCylinder`. Wheel: suspension min/max length, preload, frequency/damping spring, suspension force point, steering axis, camber via wheel-up. Tire: `mLongitudinalFriction` (LinearCurve vs slip ratio) and `mLateralFriction` (LinearCurve vs slip angle in degrees), plus a **`TireMaxImpulseCallback`** to override the tire impulse computation.
- **Reuse:** **Direct reuse.** It is the best fit to vendor or link as an optional backend, or as a test oracle for drivetrain logic. Plugging in a Pacejka/TMeasy tire via the callback is straightforward.
- **Quality notes:** actively maintained, used in shipping titles, and the constraint-based solver is stable at low rates. The tire model is simplistic by design.

### 2.5 Bullet `btRaycastVehicle`
- **License (verified):** repo `LICENSE.txt` = **zlib** (except `Extras` and `examples/ThirdPartyLibs`). `btRaycastVehicle.h` carries its own permissive notice (© 2005 Erwin Coumans).
- **Capabilities:** classic raycast vehicle. Spring/damper suspension, a single `frictionSlip` clamp, roll influence, engine force and brake per wheel. No drivetrain or tire model.
- **Reuse:** **Direct reuse** is allowed, but there is little value beyond being a minimal baseline.

### 2.6 Godot `VehicleBody3D`
- **License (verified):** **MIT** (Godot Engine contributors).
- **Capabilities:** a raycast vehicle derived from the Bullet design (the source has a comment comparing a constant to "in bullet its 0.2"). Per-wheel `engine_force`, brake, steering and `wheel_friction_slip`. Godot's own class doc says it "has known issues and isn't designed to provide realistic 3D vehicle physics."
- **Reuse:** Direct reuse is allowed, but there is **little value**. Use it only as a simple baseline.

---

## 3. Open-source simulators (copyleft)

| Project | License (verified) | Notes | Reuse |
|---|---|---|---|
| **TORCS** | **GPLv2** (SourceForge project page). Secondary sources: GPL-2.0-or-later; artwork Free Art License | Classic open racing sim (simuv2 physics) | **None for code.** Concepts only, from published literature |
| **Speed Dreams** (TORCS fork) | SourceForge lists **GPLv2** and GPL. The project README (extract) says code is GPLv2-or-later, non-functional data is Free Art License, and some directories differ | simuv4/simuv5 physics with tire temperature and wear | **None for code** |
| **VDrift** | GitHub `LICENSE` contains **GPL-3.0** text. SourceForge lists **GPLv2** (discrepancy noted) | Pacejka '96-style tire, full drivetrain. Also `VDrift/pacejka` editor (no LICENSE file found) | **None for code** |
| **Rigs of Rods** | **GPL-3.0** (`COPYING`) | Soft-body node/beam physics | **None for code** |

GPL code **cannot** be incorporated into an MIT project, and contributors should avoid reading it closely when implementing the equivalent feature (clean-room hygiene).

---

## 4. Commercial / closed products

| Product | License | Relevance | Reuse |
|---|---|---|---|
| **Vehicle Physics Pro** (Edy, Unity) | Commercial. Asset Store "Community Edition" listing exists. A Professional license includes the SDK repository *(extract)*. Older Edy's Vehicle Physics terms: royalty-free use in projects, but no redistribution "as anything except a final product… (asset / library / engine / middleware)" *(extract, 2012 wiki)* | High-quality Unity vehicle SDK. Edy also writes public explanatory material (forum posts on force application points, for example) | **None for code.** Reading public articles for ideas is fine, but implement from first principles and cite them |
| **BeamNG.drive / BeamNG.tech** | Proprietary. The EULA bars modifying the source or integrating parts into another work except per modding guidelines. BeamNG.tech academic licenses are non-commercial. Generated databases are CC-BY-NC 4.0 *(extract)* | Soft-body benchmark | **None.** BeamNG.tech-generated data is NC, so **not usable** in our MIT repo |
| **Racer** (Ruud van Gaal) | Freeware, "source available" per Wikipedia, but "NOT an Open Source project" per AlternativeTo; non-commercial *(extract)* | Documented car/track/physics file formats | **None** |
| **Assetto Corsa** (Kunos) | Proprietary | Casillo (2012, second-hand interview quote) described a physics-based tire model "tweaked with very few parameters", moving away from a pure Pacejka model *(extract, low confidence)*. AC EVO 0.9 reworked its tire model for slicks | **None** |
| **rFactor 2** (ISI → Studio 397) | Proprietary | Physical tire model developed with Goodyear; "real-road" evolving grip *(extract)* | **None** |
| **Forza** | Proprietary | Telemetry only. See `forza-telemetry.md` | Behavioural benchmark only |
| **CarSim / Adams Car / Simcenter Tire (MF-Tyre, MF-Swift)** | Proprietary | Industry references. Siemens docs: "proprietary and confidential to Siemens". MF-Tyre has been offered as freeware for MATLAB/Simulink; **MF-Swift needs a Siemens license**; MF-Tyre/MF-Swift 2019.1 reads `.tir` versions 5.2/6.0/6.1/6.2 *(extract)* | **None** (implement MF from published equations) |

---

## 5. Published models, tutorials and books

| Reference | Type | Rights status | Use for us |
|---|---|---|---|
| **Marco Monster, "Car Physics for Games"** (v1.9, Nov 2003; mirrored at asawicki.info, rsms.me) | 2D tutorial: traction, drag, rolling resistance, weight transfer, low-speed and high-speed turning | Copyrighted text, no license | **Study math only.** Good onboarding. A 2D/linear-tire simplification |
| **Brian Beckman, "The Physics of Racing"** (1991+, No Bucks Racing Club; PDF on miata.net, reprints on GameDev.net) | Article series: weight transfer, traction circle, etc. | Copyrighted articles | **Study math only.** Intuition for docs and tests |
| **Pacejka, "Tire and Vehicle Dynamics"**, 3rd ed., Butterworth-Heinemann/Elsevier 2012 (ISBN 978-0-08-097016-5); includes §13.3 "The MF-Swift Tire Model" (Besselink) | Book: Magic Formula, brush theory, similarity method, transient models | Copyrighted book. **Equations (math) are not copyrightable**; text and figures are | **Implement MF from equations.** Cite the book |
| **Magic Formula versions** (MF 5.2 → 6.1/6.2) | Besselink, Schmeitz & Pacejka 2010, "An Improved Magic Formula/Swift Tyre Model that Can Handle Inflation Pressure Changes", *Vehicle System Dynamics* 48 Suppl., pp. 337-352 (extends MF-Tyre 5.2) | Paper is copyrighted, equations public. Siemens MF-Tyre/Swift software is proprietary | **Implement from papers.** Support `.tir` (TYDEX/ADAMS-style keyword file) **reading**. The format is a plain-text key/value convention; we should not bundle vendor-supplied `.tir` files without a license |
| **TMeasy** (Hirschberg, Rill, Weinfurter 2002, *VSD* 38(2):103-125; Rill, "Road Vehicle Dynamics") | Semi-empirical model with physically meaningful parameters, first-order dynamics | Commercial TMeasy software (tmeasy.de) is proprietary. The papers and book are public | **Implement from papers.** Chrono's BSD-3 implementation is a legitimate reference |
| **Brush model** | Physical contact-patch theory (adhesion/sliding regions) | Textbook theory | Implement freely |
| **Fiala** (E. Fiala 1954, "Seitenkräfte am rollenden Luftreifen", VDI-Z 96) | Brush-based analytical model. Rectangular patch, uniform pressure. Weak at combined slip, parking, transients | Theory | Implement freely. Chrono has a BSD-3 version |
| **Dugoff** (Dugoff, Fancher, Segel 1970, SAE 700377) | Analytical combined-slip model; few parameters; no friction drop-off past peak in the original | Paper (SAE paywall) | Implement from paper or secondary literature |
| **Similarity method** (Pacejka & Sharp 1991, "Shear force development by pneumatic tyres in steady state conditions: a review of modelling aspects", *VSD* 20(3-4):121-176; Pacejka book) | Combined slip derived from pure-slip curves | Theory | Implement from the book |
| **MF-Swift** | Rigid-ring, short-wavelength, enveloping extension of MF | Proprietary software, published theory | Study only. Out of scope for v1 |

---

## 6. Other GitHub vehicle projects (UE5 and adjacent): licenses verified

| Repo | What | License (verified file) | Reuse |
|---|---|---|---|
| **KinetiForge** ([myoozy/KinetiForge-Vehicle-System](https://github.com/myoozy/KinetiForge-Vehicle-System), mirror [peba-assistant/…](https://github.com/peba-assistant/KinetiForge-Vehicle-System)) | UE5 C++ vehicle system | MIT (© 2025 Zhengyi Miao) | **See `docs/research/kinetiforge-analysis.md`** (covered by a separate analyst) |
| [Kouroshx99/VehiclePhysics](https://github.com/Kouroshx99/VehiclePhysics) | UE4.27/5.x plugin from scratch: MF 6.1 (combined slip, relaxation, pressure, turn slip), hardpoint double wishbone, unsprung mass, impulse-based driveline, test bench | **Custom non-commercial academic licence** (`LICENCE`; commercial use and redistribution prohibited without LSU authorization) | **None.** Do not read its code |
| [MrRobinOfficial/Unreal-NebulousVehicle](https://github.com/MrRobinOfficial/Unreal-NebulousVehicle) | WIP extension of Chaos Vehicle (UE 5.2) | MIT (© 2023 MrRobin) | Reference. It subclasses Chaos, so it is engine-bound |
| [WimbleSoft/VehiclePhysics](https://github.com/WimbleSoft/VehiclePhysics) | Blueprint vehicle physics in UE | MIT (© 2020 Fatih Sağlam) | Reference, low value |
| [nicholas477/EngineSimulatorPlugin](https://github.com/nicholas477/EngineSimulatorPlugin) | Integrates AngeTheGreat's Engine Simulator with Chaos Vehicles | MIT (© 2022) for the plugin. The bundled engine-sim license was not checked | Reference for audio/engine coupling. Check the sub-licence before reuse |
| [Nobatgeldi/ChaosVehiclesPlugin](https://github.com/Nobatgeldi/ChaosVehiclesPlugin) | Appears to republish Epic's Chaos Vehicles plugin | **No LICENSE file**; badge says "Epic Games" | **Do not use** (possible EULA issue) |
| [TLabAltoh/TLabVehiclePhysics](https://github.com/TLabAltoh/TLabVehiclePhysics) | Unity open WheelCollider with Pacejka tires | MIT (© 2023) | Reference or direct reuse of ideas (C#) |
| [svenlr/car-physics-pacejka](https://github.com/svenlr/car-physics-pacejka) | Pacejka friction and drivetrain for multibody sims (Gazebo) | BSD-2-Clause (© 2022) | Direct reuse possible |
| DriveSim (Fab marketplace, forum thread) | Commercial UE C++ vehicle plugin | Commercial | None |

Searches run: "github Unreal Engine 5 vehicle physics plugin pacejka open source", "github UE5 raycast vehicle plugin custom suspension tire model MIT license", and "github Unreal Engine vehicle plugin Chaos alternative custom wheel physics drift". Apart from the entries above, the hits were Godot/Unity/three.js raycast vehicles (several MIT, not UE).

---

## 7. Capability matrix

Legend: ● full or strong, ◐ partial or simple, ○ absent, ? not verified.

| Implementation | Tire model | Combined slip | Wheel contact | Suspension | Drivetrain (engine/gearbox/clutch) | Differentials / LSD | Anti-roll | Multi-wheel / tracked | Async / fixed-step | Engine-independent C++ |
|---|---|---|---|---|---|---|---|---|---|---|
| Chaos Vehicles (UE5) | ◐ friction + cornering stiffness + slip graph | ◐ | ray (default) / sweep | ◐ spring/damper per wheel | ● | ◐ | ● | ◐ | ● (Chaos async) | ○ (UE-bound) |
| Unity WheelCollider | ◐ 2-piece spline | ◐ | ray (PhysX 4.1) | ◐ | ○ (user script) | ○ | ○ | ◐ | fixed step | ○ |
| PhysX 5 vehicle | ◐ linear stiffness + friction-vs-slip, camber, sticky tire | ◐ | ray / sweep | ● incl. compliance | ● incl. autobox | ● multi/4W/tank | ● | ● | fixed step | ● |
| Jolt VehicleConstraint | ◐ LinearCurve vs slip + callback | ◐ (callback) | ray / sphere / cylinder | ● | ● | ● (LSD ratio) | ● | ● wheeled/tracked/moto | fixed step | ● |
| Bullet btRaycastVehicle | ○ (friction clamp) | ○ | ray | ◐ | ○ | ○ | ◐ roll influence | ◐ | fixed step | ● |
| Godot VehicleBody3D | ○ | ○ | ray | ◐ | ○ | ○ | ○ | ◐ | fixed step | ○ |
| Project Chrono::Vehicle | ● Pac02, Pac89, TMeasy, TMsimple, Fiala, FEA/ANCF | ● | multibody contact | ● full multibody (DW, MacPherson, multilink, solid axle…) | ● | ● | ● | ● incl. tracked | offline / real-time capable | ● (heavy) |
| Vehicle Physics Pro | ? (commercial) | ? | ? | ? | ● | ● | ● | ? | ? | ○ |
| TORCS / Speed Dreams / VDrift | ◐–● (Pacejka-style) | ◐ | ray | ◐ | ● | ● | ● | ○ | fixed step | ● (GPL) |
| Kouroshx99 VehiclePhysics | ● MF 6.1 | ● | ? | ● hardpoints | ● | ● | ● | ? | ? | ○ (UE, NC licence) |
| KinetiForge (MIT) | ◐ artist curve LUTs + constraint cap (not Pacejka) | ◐ | ray / sweep, 1 per step | ● MacPherson / double wishbone / solid axle kinematics | ◐ sequentially coupled (not implicit) | ◐ | ● | ○ | ◐ (unsafe physics-thread access) | ○ (UObject-bound); see [kinetiforge-analysis.md](kinetiforge-analysis.md) |

---

## 8. Licensing compatibility (for inclusion in an MIT repository)

| Item | License | Compatible with MIT distribution? | Obligations if reused |
|---|---|---|---|
| Jolt Physics | MIT | ✅ Yes | Keep copyright and notice |
| PhysX 5.11 (`physx/`) | Apache-2.0 (root file still BSD-3; older releases BSD-3) | ✅ Yes | Keep LICENSE and NOTICE, state changes, patent clause applies |
| Project Chrono | BSD-3-Clause | ✅ Yes | Keep notice, no endorsement |
| Bullet | zlib (+ HPND-style notice on btRaycastVehicle) | ✅ Yes | Don't misrepresent origin, mark altered versions |
| Godot | MIT | ✅ Yes | Keep notice |
| svenlr/car-physics-pacejka | BSD-2-Clause | ✅ Yes | Keep notice |
| TLabVehiclePhysics, NebulousVehicle, WimbleSoft, EngineSimulatorPlugin, KinetiForge | MIT | ✅ Yes | Keep notice (NebulousVehicle depends on Chaos) |
| geeooff/forza-data-web | Apache-2.0 | ✅ Yes | as Apache |
| nettrom / austinbaccus / heat-cycle Forza parsers | MIT | ✅ Yes | Keep notice |
| TORCS, Speed Dreams | GPLv2(+) | ❌ No | — |
| VDrift, Rigs of Rods, richstokes/Forza-data-tools | GPL-3.0 (VDrift: SourceForge says GPLv2) | ❌ No | — |
| Unreal Engine / Chaos Vehicles | Unreal EULA | ❌ No (cannot redistribute in a public repo). ✅ Our adapter may *call* it | Don't copy engine code; no GPL in the UE adapter |
| Kouroshx99/VehiclePhysics | Non-commercial academic | ❌ No | — |
| Repos with no LICENSE (0x20F/forza-telemetry, theRTB/ForzaGUI, VDrift/pacejka, Nobatgeldi/ChaosVehiclesPlugin) | All rights reserved | ❌ No | — |
| Unity, VPP, BeamNG, Racer, Assetto Corsa, rFactor 2, CarSim, Simcenter Tire | Proprietary | ❌ No | — |
| Pacejka / TMeasy / Fiala / Dugoff / brush / similarity equations | Published math | ✅ Implement independently | Cite sources. Don't copy text or figures |

**Rule for the UE5 adapter:** the core library stays free of UE. The adapter links UE headers at build time and ships as source (our code only). Never paste Epic code, including "small snippets", without legal review.

---

## 9. Validation datasets and reference data we can use lawfully

| Source | What | Rights | Use |
|---|---|---|---|
| **Project Chrono model parameter sets** (`src/chrono_models/vehicle/*`: Sedan, HMMWV, BMW E90, FEDA, CityBus, …; `data/vehicle/**`) | Complete vehicle, suspension and tire parameter sets. BMW E90 "Vehicle Parameters taken from SAE Paper 2007-01-0818" | BSD-3 (repo) | Run identical manoeuvres in Chrono and in our sim. ✅ Best cross-sim oracle. **Caveat:** some `.tir` files say they were derived from vendor examples (HMMWV Pac02 `.tir`: "gained from ADAMS/Car truck example file … by scaling"). Prefer TMeasy/Fiala sets or our own fitted parameters for anything we redistribute |
| **Published tire parameters in papers and books** (Pacejka's book example coefficients, Bakker/Nyborg/Pacejka papers, Rill's TMeasy examples) | MF/TMeasy coefficient sets | Numbers/facts are generally not copyrightable, but tables in books are. Re-key the values with citations, don't scan | ✅ With citation |
| **CC BY 4.0 tire bench datasets**, e.g. the Universidad de Oviedo / UC3M dataset (2025, e-cienciaDatos): lateral-force measurements at 3 slip angles and several loads | Measured tire data | CC BY 4.0 | ✅ With attribution. Ideal for fitting our tire models |
| **MathWorks Vehicle Dynamics Blockset example tire data** (235/45R18, TYDEX, GCAPS) | Measured tire data | MathWorks licence, not open | ❌ Do not redistribute |
| **FSAE Tire Test Consortium (Calspan)** | High-quality FSAE tire data | Members only; "no commercial use… or posting on the open internet" | ❌ Not for public use |
| **BeamNG.tech generated data** | Sim outputs | CC-BY-NC 4.0 | ❌ NC is incompatible with MIT reuse |
| **CarSim / Adams Car datasets** | Vehicle and tire models | Proprietary | ❌ |
| **Public manufacturer specs and road tests** (mass, wheelbase, power curves, 0-100 times, braking distances, skidpad g) | Facts | Facts are fine; don't copy magazine text or tables wholesale | ✅ As vehicle-level targets |
| **Our own Forza Data Out recordings** | Behavioural benchmark | See `forza-telemetry.md` §5.3 | ◐ Derived metrics only |
| **ISO / SAE test procedures** (ISO 4138 steady-state circular, ISO 7401 transient, ISO 3888 lane change) | Test definitions | Standards are paid. Cite the procedure, don't reproduce the text | ✅ Implement manoeuvres |

---

## 10. Recommendations

1. **Architecture reference:** PhysX 5 vehicle (data-oriented parameter/state components, SI units, sticky-tire low-speed handling, compliance) and Jolt (constraint-based wheel solving, collision tester variants, tire callback).
2. **Optional backends:** Jolt as a first-class optional backend (MIT). PhysX vehicle as an optional, separately licensed (Apache-2.0) module if we want its drivetrain.
3. **Tire models:** implement MF 5.2/6.1 (from Pacejka and Besselink), TMeasy (from Rill's papers), Fiala and brush ourselves. Validate against Chrono's BSD-3 implementations and the CC BY 4.0 bench data.
4. **Clean-room:** no reading of GPL simulators or of the Kouroshx99 code while implementing equivalent features. Record sources in each file header.
5. **Third-party notices:** keep a `THIRD_PARTY_NOTICES.md`, and add Apache NOTICE content if any PhysX code is vendored.

---

## Sources

All accessed 2026-10-10. *(extract)* = seen via search-engine extract only (site blocked from the research environment).

**License files and source code (fetched directly):**
- PhysX root LICENSE.md (BSD-3) https://github.com/NVIDIA-Omniverse/PhysX/blob/main/LICENSE.md. physx/LICENSE.md (Apache-2.0) https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/LICENSE.md. CHANGELOG https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/CHANGELOG.md. Version https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/version.txt. Vehicle headers https://github.com/NVIDIA-Omniverse/PhysX/tree/main/physx/include/vehicle (tire/PxVehicleTireParams.h, tire/PxVehicleTireFunctions.h, drivetrain/PxVehicleDrivetrainParams.h, suspension/PxVehicleSuspensionParams.h, physxRoadGeometry/PxVehiclePhysXRoadGeometryParams.h). README https://github.com/NVIDIA-Omniverse/PhysX/blob/main/README.md
- PhysX 4.1 LICENSE https://github.com/NVIDIAGameWorks/PhysX/blob/master/LICENSE.md
- Jolt LICENSE (MIT) https://github.com/jrouwe/JoltPhysics/blob/master/LICENSE. Vehicle headers https://github.com/jrouwe/JoltPhysics/tree/master/Jolt/Physics/Vehicle. README, Docs/ReleaseNotes.md
- Project Chrono LICENSE (BSD-3) https://github.com/projectchrono/chrono/blob/main/LICENSE. Tire headers https://github.com/projectchrono/chrono/tree/main/src/chrono_vehicle/wheeled_vehicle/tire. BMW_E90.h https://github.com/projectchrono/chrono/blob/main/src/chrono_models/vehicle/bmw/BMW_E90.h. HMMWV_Pac02Tire.tir https://github.com/projectchrono/chrono/blob/main/data/vehicle/hmmwv/tire/HMMWV_Pac02Tire.tir
- Bullet LICENSE.txt https://github.com/bulletphysics/bullet3/blob/master/LICENSE.txt. btRaycastVehicle.h https://github.com/bulletphysics/bullet3/blob/master/src/BulletDynamics/Vehicle/btRaycastVehicle.h
- Godot LICENSE.txt https://github.com/godotengine/godot/blob/master/LICENSE.txt. vehicle_body_3d.cpp https://github.com/godotengine/godot/blob/master/scene/3d/physics/vehicle_body_3d.cpp. VehicleBody3D.xml https://github.com/godotengine/godot/blob/master/doc/classes/VehicleBody3D.xml
- VDrift LICENSE https://github.com/VDrift/vdrift/blob/master/LICENSE. SourceForge https://sourceforge.net/projects/vdrift/
- Rigs of Rods COPYING https://github.com/RigsOfRods/rigs-of-rods/blob/master/COPYING
- TORCS SourceForge https://sourceforge.net/projects/torcs/. Speed Dreams SourceForge https://sourceforge.net/projects/speed-dreams/
- Kouroshx99/VehiclePhysics LICENCE and README https://github.com/Kouroshx99/VehiclePhysics/blob/main/LICENCE
- KinetiForge LICENSE https://github.com/myoozy/KinetiForge-Vehicle-System/blob/main/LICENSE. https://github.com/peba-assistant/KinetiForge-Vehicle-System/blob/main/LICENSE
- MrRobinOfficial/Unreal-NebulousVehicle LICENSE.txt https://github.com/MrRobinOfficial/Unreal-NebulousVehicle/blob/main/LICENSE.txt
- WimbleSoft/VehiclePhysics LICENSE https://github.com/WimbleSoft/VehiclePhysics/blob/master/LICENSE
- nicholas477/EngineSimulatorPlugin LICENSE https://github.com/nicholas477/EngineSimulatorPlugin/blob/main/LICENSE
- Nobatgeldi/ChaosVehiclesPlugin README (no LICENSE) https://github.com/Nobatgeldi/ChaosVehiclesPlugin
- TLabAltoh/TLabVehiclePhysics LICENSE.md https://github.com/TLabAltoh/TLabVehiclePhysics/blob/master/LICENSE.md
- svenlr/car-physics-pacejka LICENSE https://github.com/svenlr/car-physics-pacejka/blob/main/LICENSE
- VDrift/pacejka (no LICENSE found) https://github.com/VDrift/pacejka

**Vendor and other pages (extracts):**
- Epic API: UChaosVehicleWheel https://dev.epicgames.com/documentation/unreal-engine/API/Plugins/ChaosVehicles/UChaosVehicleWheel. FSimpleSuspensionConfig https://dev.epicgames.com/documentation/unreal-engine/API/Runtime/ChaosVehiclesCore/FSimpleSuspensionConfig. FSimpleTireConfig https://dev.epicgames.com/documentation/unreal-engine/API/Runtime/ChaosVehiclesCore/FSimpleTireConfig. FWheelSettings https://dev.epicgames.com/documentation/unreal-engine/API/Runtime/ChaosVehiclesCore/FWheelSettings. 4.27 FSimpleWheelConfig https://docs.unrealengine.com/4.27/API/Runtime/ChaosVehiclesCore/FSimpleWheelConfig/index.html. Chaos Modular Vehicles overview https://dev.epicgames.com/documentation/unreal-engine/chaos-modular-vehicles-overview
- Unreal Engine EULA (Publishing v15 PDF) https://cdn2.unrealengine.com/Unreal+Engine%2Ffaq%2FUnrealEngineEULA_for_Publishing_v15-b9955ec2954f0e09fade11b2d9acecfc65cb6ab1.pdf. UE on GitHub https://www.unrealengine.com/ue-on-github
- Epic forum: async physics slowdown https://forums.unrealengine.com/t/async-physics-slows-down-at-low-game-thread-fps/2731575. Strayspark blog (UE 5.7 claim) https://www.strayspark.studio/blog/chaos-vehicles-masterclass-shipping-racing-game-2026
- Unity Manual wheel colliders https://docs.unity3d.com/Manual/wheel-colliders.html. WheelCollider API https://docs.unity3d.com/ScriptReference/WheelCollider.html. WheelFrictionCurve https://docs.unity3d.com/351/Documentation/ScriptReference/WheelFrictionCurve.html
- VPP Community Edition (Asset Store) https://assetstore-fallback.unity.com/packages/tools/physics/vehicle-physics-pro-community-edition-153556. Edy wiki https://edy.repositoryhosting.com/trac/edy_vehicle-physics-pro-professional/wiki/WikiStart?version=4. Old terms https://vehiclephysics.repositoryhosting.com/trac/vehiclephysics_edys-vehicle-physics/wiki/TermsAndConditions
- BeamNG EULA https://beamng.com/game/cn/support/policies/eula/. BeamNG.tech database licensing https://www.beamng.tech/policies/database-licensing/. Academia https://www.beamng.tech/blog/beamng-tech-for-academia
- Racer: Wikipedia https://en.wikipedia.org/wiki/Racer_(simulator). AlternativeTo https://alternativeto.net/software/racer/about
- Assetto Corsa: RaySim post https://ravsim.com/2012/05/09/assetto-corsa/ (2012). AC EVO tyre model https://boxthislap.org/assetto-corsa-evo-0-9-tyre-model/
- rFactor 2: Goodyear partnership https://boxthislap.org/rfactor2-realistic-tire-developments-in-partnership-with-goodyear-tyres/. Steam https://store.steampowered.com/app/365960/rFactor_2
- Marco Monster, Car Physics for Games (v1.9, Nov 2003) https://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html. https://rsms.me/etc/car-physics/
- Brian Beckman, The Physics of Racing (1991) https://miata.net/sport/Physics/phor.pdf. https://gamedev.net/tutorials/programming/math-and-physics/the-physics-of-racing-part-1-weight-transfer-r1610
- Pacejka, Tire and Vehicle Dynamics 3rd ed. https://shop.elsevier.com/books/tire-and-vehicle-dynamics/pacejka/978-0-08-097016-5. SAE listing https://saemobilus.sae.org/books/tire-vehicle-dynamics-third-edition-r-418
- Besselink, Schmeitz, Pacejka 2010 https://trid.trb.org/view/1086180
- Siemens Simcenter blog (.tir 5.2–6.2) https://blogs.sw.siemens.com/simcenter/more-tire-modelling-capabilities-in-real-time-environments/. Altair MF-Tyre/MF-Swift help https://help.altair.com/hwsolvers/ms/topics/motionview/MFTyre-MFSwift_Help.pdf. Ansys tire help https://ansyshelp.ansys.com/public/Views/Secured/corp/v251/en/motion_pre/Tire.html
- TMeasy paper (Hirschberg, Rill, Weinfurter 2002) https://graz.elsevierpure.com/files/1925147/tmeasy_prt.pdf. Chrono tire docs https://api.projectchrono.org/wheeled_tire.html
- Fiala references (Altair) https://help.altair.com/hwdesktop/hwx/topics/motionview/fiala_tire_r.htm
- Dugoff et al. 1970, SAE 700377 https://saemobilus.sae.org/content/700377
- Pacejka & Sharp 1991 https://trid.trb.org/View/339881
- Svendenius & Gäfvert, SAE 2004-01-1064 https://saemobilus.sae.org/content/2004-01-1064
- FSAE TTC https://millikenresearch.com/fsaettc.html. https://calspan.com/automotive/fsae-ttc. Forum statement https://fsae.com/forums/showthread.php?p=98383
- Oviedo/UC3M CC BY 4.0 tire dataset https://portalinvestigacion.uniovi.es/documentos/67a9c7d519544708f8c73efd. MathWorks example tire data https://www.mathworks.com/help/vdynblks/ug/example-tire-data.html
- Speed Dreams data README https://forge.a-lec.org/speed-dreams/speed-dreams-data/src/branch/main/README.md. TORCS man page (GPL v2 or later) https://manpages.debian.org/buster/torcs/torcs.6.en.html
