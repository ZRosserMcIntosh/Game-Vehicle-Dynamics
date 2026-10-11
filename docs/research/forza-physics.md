# Forza physics: what is publicly known

> **Scope and method.** This document collects only public developer statements, public support documentation, press coverage and player observation of the Forza Motorsport (FM, Turn 10) and Forza Horizon (FH, Playground Games) series. No leaked material, no reverse engineering, no proprietary code. It informs the design of OpenRoad Dynamics. It is **not** a description of how Forza is implemented.
>
> **Research date:** 2026-10-10.
>
> **Retrieval limitation (important).** During this research session the sandbox's egress policy blocked direct page fetches (`WebFetch`/`curl` to forza.net, support.forza.net, news.xbox.com, gtplanet.net, gamingbolt.com, autos.yahoo.com and others failed with DNS or proxy errors). Every finding below therefore rests on **search-engine result snippets and summaries** of the cited pages, not on a full read of the page. Quotes are marked "(snippet)" where the wording comes from a search excerpt. Before anything here is quoted externally, the cited page should be opened and the wording checked by hand.

## Evidence classes

| Code | Meaning |
|---|---|
| **C** | Public, confirmed statement by the developer (Turn 10 / Playground / Microsoft / Xbox / official Forza support docs) |
| **O** | Observation by players or press (hands-on, reviews, forum testing) |
| **I** | Plausible inference by us. Not stated by anyone. |
| **U** | Unknown or proprietary. |

Confidence: **High** = developer primary source, consistent across several reports. **Medium** = developer statement seen only through secondary reporting, or consistent press observation. **Low** = single secondary source, forum claim, or our own inference.

---

## 1. Deep dive: "8 contact points at 360 Hz" (Forza Motorsport, 2023)

### 1.1 What was said

| Finding | Class | Source | Date | Game | Conf. | Implication |
|---|---|---|---|---|---|---|
| "Our new tire physics has 8 points of contact with the track surface running at 360 cycles per second. This allows tires to find grip on any type of uneven surface." (snippet). Previous FM games had "only a single point of tire contact on the surface that moved at 60 cycles per second" (snippet). The stated result is "better drivability everywhere and realistic behavior on uneven surfaces like curbs" (snippet). | C | forza.net/news/forza-motorsport-drivatars-tire-physics (Chris Esaki, Creative Director) | 2023-07-17 | FM 2023 | High | Multi-point contact matters most on non-flat ground. Target curbs and rumble strips first. |
| Esaki said the eight points "really represent the outer edges of that contact patch" (snippet, via The Drive / Yahoo Autos interview reporting). | C (via press) | thedrive.com hands-on; autos.yahoo.com "How much… built from the ground up" | 2023 (exact day not verified) | FM 2023 | Medium | The points sample the **perimeter** of the patch. They are not a dense grid across it. |
| "Eight points of contact versus one, 360 hertz versus 60, it's a 48x fidelity jump in a single tire collision itself" (snippet, quoted by GameSpot and others). | C (via press) | gamespot.com/articles/…generational-leap…/1100-6492448/; purexbox.com (2021-06); techradar.com | 2021 (May/June; exact day not verified) | FM 2023 (pre-release) | High | 48 = 8 × 6. It is a ratio of *samples per second*, not a measure of accuracy. |
| One Gamereactor transcription gives "x40". One secondary outlet says "roughly 24 times". Both conflict with 8 × 6 = 48 and look like transcription or arithmetic errors. | O | gamereactor.eu; racinggames.gg (via search summary) | 2021 / 2023 | FM 2023 | Medium | Use "48x (8 points × 6× rate)" and attribute it to Turn 10. |
| In FM7, curbs had no real collision geometry and were "as flat as the rest of the track". An artificial noise calculation was used instead, and the new tire model made that unnecessary. Esaki showed an FM7 vs new-physics clip of a BAC Mono on curbs. | C (via press) | gtplanet.net/forza-motorsport-new-tire-model-20210528/ | 2021-05-28 | FM7 → FM 2023 | Medium | Curb feel used to be faked with noise. Our curbs should be real geometry sampled by the tire. |
| Turn 10's 2022 showcase cited "a 48 times improvement in the fidelity of our physics simulation" (snippet). | C | news.xbox.com/en-us/2022/06/12/the-all-new-forza-motorsport-is-the-most-technically-advanced-racing-game-ever-made/ | 2022-06-12 | FM 2023 | High | Same claim at a different scope ("physics simulation" rather than "tire collision"). Treat as marketing shorthand. |

### 1.2 Reconciling this with older "360 Hz" claims

Earlier Forza titles were also described as running physics at 360 Hz:

| Finding | Class | Source | Date | Game | Conf. |
|---|---|---|---|---|---|
| Dan Greenawalt (Turn 10): "Forza 2 had a 360 frames-per-second (FPS) physics update engine and 60 FPS graphics", and "In Forza 3 we modeled tire and sidewall flexing and the effect on driving dynamics" (snippet). | C (via press) | autoblog.com/2009/06/04/sneak-peek-forza-motorsport-3/ | 2009-06-04 | FM2, FM3 | Medium |
| "Forza 3 brings a 360 Hz physics engine which can deal with the way in which the tyres move as they grip the road" (snippet). | O (press) | techradar.com/news/gaming/consoles/forza-3-pushes-xbox-further-than-ever-before-627284 | ~2009 (not verified) | FM3 | Medium |
| A GamesRadar hands-on of an Xbox 360-era Forza says graphics update at 60 fps "while the game's physics engine is running at 360 frames per second" (snippet). The snippet does not name the title, so tying it to FM2 is an inference. | O | gamesradar.com/you-need-another-two-xbox-360s | ~2007 (not verified) | Probably FM2 | Low |

**Apparent contradiction:** FM2/FM3 "physics at 360 Hz" against FM 2023 "previous games: single contact point at 60 Hz".

**Plausible reconciliation (I, Medium):** the two statements describe different subsystems. The rigid-body / drivetrain / tire-force integration may have run at 360 Hz since FM2. The **tire–surface collision query** (finding the contact point, the surface normal and the surface type under each tire) may have been refreshed only at 60 Hz, once per rendered frame, with the cached contact reused across substeps. Esaki's own wording supports this: he speaks of the "tire collision model" and "a single tire collision". If so, the 2023 change was mainly to **run the ground query at the full physics rate and widen it from one point to eight**. Turn 10 has not confirmed this reading.

### 1.3 What it plausibly means for an implementation (all class I)

1. **Substepping.** 360 Hz against a 60 fps render target is exactly **6 physics substeps per frame** (Δt ≈ 2.78 ms). Against 30 fps it is 12. Whether FM 2023 uses a fixed 360 Hz step decoupled from frame rate (likely on PC, which has unlocked frame rates) is **not stated**.
2. **Perimeter sampling.** Eight points on "the outer edges" of the patch suggests something like a ring or rectangle of probes, for example 3 front + 3 rear + 2 sides, or 4 corners + 4 edge midpoints. The exact layout is **unknown**. Plausible uses include:
   - estimating an effective contact plane (height and normal) when the tire straddles a curb edge, a seam or a crown;
   - partial-contact load fraction (how much of the patch is actually touching), which gives a smooth transition onto curbs instead of a single-point step;
   - camber-dependent load shift across the patch width;
   - per-sample surface material (half on a painted curb, half on asphalt or grass).
3. **It is not stated to be a brush or FE carcass model.** Eight *collision* points do not imply eight *force-generating* bristles. Forces could still come from one lumped slip-based model fed by the aggregated contact. The Drive's preview described the result as a "point-based model" that "can function almost as well as a carcass model with far less development time" (snippet). That is the journalist's characterisation, not Turn 10's.
4. **Cost.** 4 wheels × 8 probes × 360 Hz = 11,520 ground queries per second per car. That is cheap for a player car and noticeable for 24 AI cars. Whether AI cars use the full model is **unknown**.

### 1.4 What is not stated (U)

- The tire force model family (Pacejka / Magic Formula, brush, empirical tables, hybrid).
- The geometry of the probe layout, and whether it moves with load, pressure or camber.
- Whether 360 Hz applies to the whole vehicle simulation or only to the tire collision.
- Whether the step is fixed and whether it is decoupled from frame rate on PC.
- How the eight samples are combined (averaged plane, weighted load, per-point force).
- Whether AI or remote cars run the same fidelity.
- Whether Forza Horizon 5/6 use the 2023 tire model. When asked whether FH5 used the new tire model, Esaki reportedly said he "actually doesn't know" (forum, Low).

### 1.5 Implications for OpenRoad Dynamics

- Make physics rate and render rate independent. Default to a fixed **360 Hz** step (6 substeps at 60 fps) and allow 240/500/1000 Hz for SIMULATION. *Do not claim* this matches Forza's behaviour.
- Implement a **multi-probe contact patch** (configurable N, default 8, laid out on the patch perimeter). Run the ground query **every substep**, not once per frame.
- Model curbs as real geometry and surface material. **Do not inject noise** to fake curb feel (Turn 10 says FM7 did this and moved away from it).
- Keep "contact sampling" and "force model" as separate modules.

---

## 2. Tire dynamics

| Finding | Class | Source | Date | Game | Conf. | Implication |
|---|---|---|---|---|---|---|
| FM4's tire model was built from "reams of tire data from the Pirelli R&D environment", and Turn 10 asked Pirelli "to employ custom tests" (snippet). | C | blogs.microsoft.com/next/2012/05/04/the-making-of-forza-motorsport-4-physics-pirelli-and-performance/ | 2012-05-04 | FM4 | High | AAA tire models are fitted to measured data. We need public or open tire datasets (e.g. TTC-style data where licensing allows) and a fitting toolchain. |
| FM5 partnered with Calspan (a transportation and aerospace R&D organisation that runs tire test rigs) to "collect data on tire variables that have never been isolated" for "complex heat, slip, and weight-transfer calculations". The work covered "tire physics, suspension geometry, and aerodynamics" (Greenawalt, snippet). | C | news.xbox.com/en-us/2013/09/20/games-forza-5-new-gen-racing/; gamingbolt.com (Gamescom 2013 report) | 2013-09-20 | FM5 | High | Turn 10 confirms the model has thermal, slip and load-transfer dependence. Our tire model must include temperature and load sensitivity from the start. |
| FM3 added "tire and sidewall flexing and the effect on driving dynamics" (Greenawalt). | C (via press) | autoblog.com 2009-06-04 | 2009-06-04 | FM3 | Medium | Carcass compliance (lateral/torsional flex) has been part of the model since FM3. This supports relaxation-length / carcass-stiffness modelling. |
| FM 2023 compounds: Street, Performance (also reported as "Sport"), Race, Drag and Wet. Race splits into Soft / Medium / Hard, with "different grip and wear characteristics". Soft has the most grip and wears fastest (snippet). | C | forza.net July 2023 dev post (reposted at forums.forza.net/t/tire-physics-in-forza-motorsport/611866) | 2023-07-17 | FM 2023 | High | Compounds should be data assets (grip curve, thermal window, wear rate). They should not be code paths. |
| "We've also dialed in the tire heat model. The biggest change is in the grip fall off at extreme tire [temperatures]" (snippet, truncated). Overtake.gg paraphrase: grip "will not fall off as abruptly as they used to when exposed to extreme temperatures". | C (partly via press) | forums.forza.net/t/tire-physics-in-forza-motorsport/611866; overtake.gg/news/comments/59546/ | 2023-07 | FM 2023 | Medium | Use a smooth thermal grip curve with no cliff. Predictability was an explicit design goal. |
| A "new tire model and pressure system" was first mentioned on Turn 10's December 2019 stream. | C (via press) | gamingbolt.com "Forza Motorsport 8 will feature new tire model, pressure system…" | 2019-12 (not verified) | FM 2023 | Medium | Pressure is a modelled state, not only a tuning label. |
| Players report that the FM 2023 tuning value is the **hot target pressure**, and that pressure rises little with temperature over a stint. | O | forums.forza.net threads (tire pressures; FM 2023 tuning guide /729034) | 2023–2024 | FM 2023 | Low | This is a UX choice. Showing hot pressure hides the cold-to-hot gas-law step. We should expose both. |
| Tire wear and fuel consumption are simulated in FM 2023. Grip degradation is felt after "four or five laps… depending on tire type" (review). | C (wear/fuel announced) + O (review) | thesixthaxis.com 2022-06-12; gamingbolt.com FM 2023 review | 2022-06 / 2023-10 | FM 2023 | High (exists) / Low (magnitude) | Wear must change grip, thermal behaviour and possibly stiffness. |
| Data Out telemetry exposes per-wheel `TireSlipRatio`, `TireSlipAngle` and `TireCombinedSlip`, each **normalized** so that 0 = full grip and \|value\| > 1.0 = loss of grip. It also exposes per-wheel tire temperature. | C | support.forza.net/hc/en-us/articles/21742934024211-Forza-Motorsport-Data-Out-Documentation | undated (doc current as of search) | FM 2023 (format identical to FM7) | High | Strong hint (I) that the model works with a **combined-slip formulation normalized to peak slip**, in the style of friction-ellipse or similarity models: σ = slip / slip_at_peak, and \|σ\| > 1 means past the peak. We should expose the same normalized quantities, both for assists and for telemetry. |
| Specific force-model family (Pacejka, brush, table) | U | none | — | all | — | Choose on our own merits. |
| Load sensitivity (μ falling with Fz) | I | inferred from "weight-transfer calculations" plus data-driven fitting | — | FM5+ | Medium | Must implement. |
| Relaxation length / transient slip | I | inferred from "sidewall flexing" (FM3) | — | FM3+ | Low–Medium | Implement as first-order lag on slip, scaled by carcass stiffness. This also stabilises low-speed behaviour. |

---

## 3. Surfaces, wet grip, aquaplaning, track evolution

| Finding | Class | Source | Date | Game | Conf. | Implication |
|---|---|---|---|---|---|---|
| FM6 researched **148 drivable surfaces** by how they absorb water and what traction they give when wet, with separate wet and dry values per surface. | C | news.xbox.com/en-us/?p=10141 ("Forza Motorsport 6: When It Rains, It Pours") | 2015 (not verified) | FM6 | Medium | Use a surface material table with dry μ, wet μ, porosity/drainage and roughness. |
| Greenawalt (to IGN, via TeamVVV): surface porosity determines how water pools and how it changes friction. Standing water is simulated, and tire size and angle of attack determine aquaplaning. "Physically simulated 3D puddles… with accurate drag properties" (drivelife, snippet). | C (via press) | teamvvv.com/?p=22561; drivelife.co.nz | 2015 | FM6 | Medium | Aquaplaning should depend on water depth, tire width, speed and contact geometry. It should not be a flat μ multiplier. Puddles should also add drag. |
| FM6 weather was static (puddles in fixed places). Players reported that puddles felt "like ice". | O | bsimracing.com/?p=57276; forums.forza.net/t/hydroplaning/26008 | 2015 | FM6 | Medium | Aquaplaning onset needs a ramp with partial grip. A binary switch to zero grip feels unfair. |
| FM7 had dynamic weather: puddles and slick tarmac, a shifting dry line, possible aquaplaning, air temperature cooling the track, and a drying line forming once rain stops (Bill Giese). | C (via press) | gtplanet.net/forza-motorsport-7s-dynamic-weather-system-offers-visceral-experience | 2017 | FM7 | Medium | Track wetness should be a spatial, time-varying field, with a racing line that dries faster. |
| FM 2023: time of day changes ambient temperature, which changes track surface temperature, which affects grip. **Rubbering-in** also affects grip, on every track. | C | news.xbox.com 2022-06-12; thesixthaxis.com 2022-06-12 | 2022-06-12 | FM 2023 | High | Track temperature and rubber level should be global (or per-segment) state that feeds tire μ and tire thermals. |
| FM 2023 telemetry has per-wheel `WheelInPuddleDepth` (0–1). FH6 telemetry has only a boolean `WheelInPuddle`. | C | Data Out docs (FM article 21742934024211; FH6 article 51744149102611) | undated | FM 2023, FH6 | High | Motorsport models puddle *depth*. Horizon may model only presence. This difference in fidelity is a reasonable split for FUN vs SIMULATION. |
| The first FM 2023 patch "improves the wet tire physics" (gameswelt report of release notes). | O (press re: C) | gameswelt.de/forza-motorsport-2023/news/das-erste-update-ist-endlich-da-322817 | late 2023 | FM 2023 | Medium | Wet behaviour was tuned after launch. Plan for wet tuning as an ongoing task. |
| One forum thread says puddles/aquaplaning were "not implemented, even in the heaviest wet"; the game is not clearly identified. | O | forums.forza.net/t/lets-discuss-wet-conditions/755254 | ~2024 | probably FM 2023 | Low | Unresolved. Do not cite as fact. |
| FH5 Rally Adventure expansion: "entirely new system of deformable terrain", deformable sand roads with pre-deformed tracks, physics-driven dust. | C (via press) | videogameschronicle.com (Rally Adventure reveal); wccftech.com | 2023-03 | FH5 | Medium | Off-road should support deformable or soft-surface state, at least as a height-offset plus resistance model. |
| Whether the FH5 base game had distinct dirt tire physics is disputed on forums. | O | forums.forza.net/t/physics-in-fh5/534470 | 2021 | FH5 | Low | Unknown. |

---

## 4. Suspension

| Finding | Class | Source | Date | Game | Conf. | Implication |
|---|---|---|---|---|---|---|
| FM 2023 suspension: Esaki said **dynamic force propagation** (how forces travel from the contact patch through the suspension into the chassis) is now "super realistic". He named **roll center, anti-dive, anti-squat** as areas he researched. The work also reproduces partners' CAD data more faithfully. | C (stream, via press) | gtplanet.net/forza-motorsport-suspension-model-20211208/; traxion.gg | 2021-12-08 | FM 2023 | Medium–High | Model geometric (kinematic) load paths: roll centre height, anti-dive/anti-squat percentages and jacking forces. A plain spring-at-wheel model is not enough. |
| Esaki framed the suspension changes as "all just about making our cars much easier for all of our players to drive", and said setup changes now match real-world effects (citing drift partners). | C (via press) | traxion.gg/forza-motorsport-will-include-a-new-suspension-model/ | 2021-12 | FM 2023 | Medium | **Turn 10 presents better physics as an accessibility gain.** A coherent model is easier to read than a hacked one. |
| FH5 "completely rebuilt its suspension model and braking for more natural handling". | C (via press) | jalopnik.com/forza-horizon-5-will-handle-better… | 2021 (not verified) | FH5 | Medium | Horizon does not just inherit Motorsport physics. Playground owns its own suspension and brake work. |
| Telemetry exposes per-wheel normalized suspension travel (0 = full extension, 1 = full compression). It also exposes rumble-strip flags. | C | Data Out docs | undated | FM7/FM 2023/FH | High | Model travel limits and bump stops explicitly. |
| Tuning exposes (per axle, not per wheel) springs, ride height, rebound and bump damping, ARBs, camber, toe and caster (FM6 list, FH3 FAQ). | C | support.forza.net/hc/en-us/articles/360005306674-FH3-Tuning-Settings | undated | FM6 / FH3 | High | See §8. |

## 5. Drivetrain and brakes

| Finding | Class | Source | Date | Game | Conf. | Implication |
|---|---|---|---|---|---|---|
| The differential tuning category regulates lock under acceleration and under braking (accel/decel lock). AWD cars also have a front/rear balance (community guides; not confirmed in the official snippet seen). | C (category) / O (center) | FH3 Tuning Settings support FAQ; community guides | undated | FM/FH | High / Low | Implement a clutch-pack LSD with separate accel/decel ramps and preload, plus an AWD centre split. |
| FH5 ABS was "remodeled to work in a much more modern way. Brake pressure will ease off rather than just modulating". Brake tuning matters more than before. | C (via press) | racinggames.gg/article/forza-horizon-5-major-handling-upgrades-revealed | 2021 | FH5 | Medium | ABS should work as a slip-target pressure controller acting on brake pressure. It should not be on/off pulsing or a grip multiplier. |
| Brake tuning = balance + pressure. | C | FH3 Tuning Settings FAQ | undated | FM6/FH | High | Expose bias and maximum line pressure. |
| Launch control appears in FH5 (re-introduced with Rally Adventure) as a tuning-type option, with anti-lag. | O (press) | sportskeeda.com FH5 Rally Adventures guide | 2023 | FH5 | Low | Launch control should be an input/ECU controller (rev limit plus TC slip target). |
| Engine, clutch and gearbox internals (inertia, torque curve resolution, clutch model) | U | — | — | — | — | — |

## 6. Aerodynamics

| Finding | Class | Source | Date | Game | Conf. | Implication |
|---|---|---|---|---|---|---|
| FM5 Calspan work covered aerodynamics as well as tires. | C | Xbox Wire 2013-09-20 | 2013 | FM5 | Medium | — |
| Tuning exposes front and rear downforce, which trades cornering grip against top speed. | C | FH3 Tuning FAQ | undated | FM/FH | High | At minimum: front/rear downforce and drag coefficients as functions of the setting. Ride-height/pitch sensitivity is not documented. |
| Ride-height, rake or yaw aero sensitivity; drafting model | U | — | — | — | — | Our own decision. |

## 7. Vehicle dynamics and AI (overall)

| Finding | Class | Source | Date | Game | Conf. | Implication |
|---|---|---|---|---|---|---|
| Esaki said the change from FM7 to FM 2023 exceeds FM4→FM7 combined: a "huge generational leap" in foundational driving. | C (via press) | gamespot.com 1100-6492448 | 2021 | FM 2023 | High | — |
| Commentators note that some code likely carried over and the game is not fully "from the ground up". | O | autos.yahoo.com "How much… built from the ground up" | 2023 | FM 2023 | Low | — |
| FM 2023 AI: the ML-based Drivatar was trained on car/track combinations including upgrades and weather. It drives each track ~26,000 times to find lines. Turn 10 says no rubber-banding or cheats (snippet). | C | forza.net 2023-07-17 | 2023-07-17 | FM 2023 | Medium (via summary) | If AI drives the same physics without cheats, assists and AI share one model. We should do the same (AI uses the same vehicle, optionally with assists). |
| Ralph Fulton (Playground): Horizon started from "the Turn Ten Forza Motorsport codebase… the physics system, the way the cars handle and the way they react to the road is exactly the same" (snippet). | C (via press) | whatculture.com/gaming/forza-horizon-interview-design-director-ralph-fulton | ~2012 (not verified) | FH1 | Medium | Origin of the "one simulation, different defaults" approach. Note this is from 2012. FH5 later rebuilt suspension and brakes. |

## 8. Tuning parameters exposed (what the documented menu tells us)

Per the official FH3 support FAQ, which lists the FM6 options, and consistent community documentation for FH4/FH5/FH6 and FM 2023:

| Category | Parameters (front/rear unless noted) | Model state implied (I) |
|---|---|---|
| Tires | Pressure (FM 2023: hot target) | Pressure-dependent stiffness, patch size, thermal behaviour |
| Gearing | Final drive, individual ratios | Discrete gearbox with ratios |
| Alignment | Camber, toe, caster (front) | Camber thrust / camber-dependent grip; caster → mechanical trail (confirmed in FFB docs, §9) |
| Anti-roll bars | Stiffness | Roll stiffness distribution |
| Springs | Rate, ride height | Wheel rate, static ride height, travel limits |
| Damping | Rebound, bump | Velocity-dependent damper (single- or multi-slope unknown) |
| Aero | Front/rear downforce | Downforce and drag |
| Brakes | Balance, pressure | Bias, maximum torque |
| Differential | Accel lock, decel lock (+ centre balance on AWD per community) | Clutch-type LSD |
| Wheel offset (FM6) | Track width | Geometry |

Parts gate the tuning: most production cars need race parts installed before a category can be adjusted (C, FH3 FAQ). **Not exposed** in any documented menu we found: per-wheel settings, bump-stop gap, packers, damper high/low-speed split, roll-centre or anti-geometry adjustment, LSD preload, tire compound thermal data. Fulton stated FH3 added no new tuning options.

## 9. Input, force feedback and assists (physics-relevant summary; design aspects in `forza-handling-philosophy.md`)

| Finding | Class | Source | Date | Game | Conf. | Implication |
|---|---|---|---|---|---|---|
| FM7 Dec 2018 update: a "completely physically based" FFB. Steering torque is computed from "the alignment torque generated at the contact patch", combining **mechanical and pneumatic trail**. More caster → more mechanical trail → more alignment force. Road Feel derives from individual wheel loads. | C (support text via GTPlanet/forums) | support.forza.net/hc/en-us/articles/360012861313-FM7-Wheel-Introduction; gtplanet.net "Setting your wheel up for FM7's new FFB system" | 2018-12 | FM7 (carried into FM 2023 sliders) | High | FFB = (Mz from the tire model, i.e. pneumatic trail × Fy) + (mechanical trail from geometry × Fy) through the steering rack. That needs a tire model that outputs pneumatic trail that falls with slip. This is the main physical "grip-loss" cue. |
| FM 2023 FFB sliders include Force Feedback Scale (per car in Tune menu), Self-Steering Alignment, Mechanical Trail Scale, Road Feel, Center Spring, invert. | O (community guides) | briankoponen.com; simracingcockpit.gg | 2023 | FM 2023 | Medium | Expose scale factors on physically separate components, not one "gain". |
| FH6 support: Normal steering = "multiple systems layered together… Some of these layers alter inputs where some of these layers alter the vehicle dynamics. Simulation Steering turns these systems off." Input layers are always off on a wheel. Normal "influences the yaw inertia of the car", making it "somewhat less agile… but also more stable" and making FFB feel sharper (snippet). | C | support.forza.net/hc/en-us/articles/51642814113427-Forza-Horizon-6-on-Wheel-Advanced-Wheel-Tuning | undated (FH6, 2026) | FH6 | High | **A Horizon assist alters vehicle dynamics (yaw inertia).** It is not input-only. See the philosophy doc for why we choose differently, or at least make this explicit. |
| FM accessibility support: steering modes from fully/partially assisted to Normal to Simulation. Simulation removes damping and steering-speed help. Traction control On / Sport / Off (Sport allows more slip). Assisted braking can brake for corners and imminent impacts. ABS on/off. | C (via snippet) | support.forza.net/hc/en-us/articles/20964254277267-Forza-Motorsport-Accessibility-Support | undated | FM 2023 | Medium–High | Assists are graded controllers with slip targets. |
| Controller telemetry steering value stops increasing past a speed-dependent limit on pad (not on wheel), in both Normal and Simulation (player test). Another player describes "a filter to limit the input based on tyre friction". | O | forums.forza.net/t/steering-difference-between-wheel-vs-controller/78043; …/98148 | ~2017–2018 | FM7/FH4 era | Low–Medium | Pad steering is limited to the slip angle near peak. This matches a "friction-aware steering limiter" (input layer). |

## 10. Simulation rate

| Finding | Class | Source | Date | Game | Conf. |
|---|---|---|---|---|---|
| FM2: 360 Hz physics, 60 fps render. | C (via press) | autoblog 2009-06-04 | 2009 | FM2 | Medium |
| FM3: 360 Hz physics engine. | O | techradar ~2009 | ~2009 | FM3 | Medium |
| FM1–FM7: tire collision single point at 60 Hz. | C | forza.net 2023-07-17 | 2023 | FM1–FM7 | High |
| FM 2023: 8 points at 360 Hz. | C | forza.net 2023-07-17 | 2023 | FM 2023 | High |
| Telemetry output: 60 packets/s UDP. | C | Data Out docs | undated | FM7/FM 2023/FH | High |
| FH4/FH5/FH6 internal physics rate | U | — | — | FH | — |

Note: 60 Hz telemetry does not mean 60 Hz physics. Telemetry is decimated output.

---

## 11. Unknowns (proprietary or unpublished)

1. Tire force model family and its parameterisation. How the Pirelli (FM4) and Calspan (FM5) data was fitted. Whether Calspan was involved after FM5 (no source found).
2. Combined-slip method (the normalized telemetry suggests a normalized-slip formulation, but the exact method is unknown).
3. Load-sensitivity curves, thermal model structure (surface/core/carcass layers?), pressure–temperature coupling, wear model.
4. Relaxation length or transient tire behaviour beyond "sidewall flex" (FM3).
5. Layout and use of the 8 contact points. Whether 360 Hz is the whole-vehicle rate in FM 2023.
6. Exact aquaplaning model in FM 2023 (FM6 and FM7 described it; 2023 status is contested by players).
7. Suspension kinematics representation (full multibody? parameterised roll centre / anti-geometry curves?).
8. Aero maps (ride-height or pitch dependence), drafting.
9. FH physics rate and the degree of code sharing with FM 2023 at present.
10. Exact implementation of Normal steering's "vehicle dynamics" layer (only "yaw inertia" is stated), stability-control logic, and the shape of the controller input filter.
11. Whether AI/remote cars run reduced physics fidelity.

## 12. Summary of implications for OpenRoad Dynamics

- Fixed-step 360 Hz default, decoupled from rendering (6 substeps at 60 fps). Ground query **every substep**.
- Multi-probe contact patch (default 8, on the perimeter). Real curb geometry, no fake noise.
- Data-driven tire model with load sensitivity, temperature (smooth falloff, no cliff), pressure state (show cold and hot), wear, compound assets, and transient (relaxation) behaviour.
- Normalized slip outputs (|σ| > 1 = past peak) for assists, FFB and telemetry.
- Surface table with wet/dry μ, porosity and water depth. Aquaplaning as a function of depth, speed and tire width, with puddle drag. Track temperature and rubber state.
- Suspension with roll centre, anti-dive/anti-squat and bump stops.
- FFB from physical aligning torque (pneumatic + mechanical trail) with separate scale factors.
- **No claim of Forza equivalence** without side-by-side measurements (see the philosophy doc).

---

## Sources

All URLs below appeared in search results during this session (2026-10-10). **None could be fetched directly** because of sandbox network policy. Content was taken from search-result excerpts and summaries. Dates are publication dates where the URL or snippet established them. "Not verified" means inferred from context.

| # | URL | Date | Notes |
|---|---|---|---|
| 1 | https://forza.net/news/forza-motorsport-drivatars-tire-physics | 2023-07-17 | Primary. Fetch failed (DNS). Date from search summary; repost dated 2023-07-18 at idcgames.com |
| 2 | https://forums.forza.net/t/tire-physics-in-forza-motorsport/611866 | 2023-07 | Repost of #1 tire section |
| 3 | https://www.gtplanet.net/forza-motorsport-new-tire-model-20210528/ | 2021-05-28 | Esaki, FM7 curbs flat / noise |
| 4 | https://gamespot.com/articles/forza-motorsports-handling-will-be-a-huge-generational-leap-for-the-franchise/1100-6492448/ | 2021 (not verified) | 48x quote |
| 5 | https://www.purexbox.com/news/2021/06/the_new_forza_motorsport_will_be_a_huge_generational_leap_says_dev | 2021-06 | — |
| 6 | https://www.techradar.com/news/forza-motorsport-is-a-huge-generational-leap-from-previous-games-heres-why | 2021 (not verified) | — |
| 7 | https://www.gamereactor.eu/expect-a-huge-generational-leap-for-next-forza-motorsport/ | 2021 (not verified) | "x40" transcription |
| 8 | https://www.thedrive.com/car-reviews/forza-motorsport-hands-on-preview-review-xbox-racing-game | 2023 (not verified) | "outer edges"; point-based vs carcass comment |
| 9 | https://autos.yahoo.com//much-forza-motorsport-actually-built-210000716.html | 2023 (not verified) | — |
| 10 | https://traxion.gg/forza-motorsport-promises-new-drivatar-system-that-actually-works-details-on-new-tire-physics/ | 2023-07 (not verified) | — |
| 11 | https://news.xbox.com/en-us/2022/06/12/the-all-new-forza-motorsport-is-the-most-technically-advanced-racing-game-ever-made/ | 2022-06-12 | 48x, track temp, rubber |
| 12 | https://www.thesixthaxis.com/2022/06/12/forza-motorsport-is-coming-spring-2023-ray-tracing-tyre-fuel-management-gameplay-reveal/ | 2022-06-12 | — |
| 13 | https://www.thesixthaxis.com/2022/06/18/forza-motorsport-8-preview-physics-race-weekend-multiplayer-ray-tracing/ | 2022-06-18 | — |
| 14 | https://www.overtake.gg/news/comments/59546/ | 2023 (not verified) | Heat model paraphrase |
| 15 | https://www.gtplanet.net/forza-motorsport-suspension-model-20211208/ | 2021-12-08 | Suspension |
| 16 | https://traxion.gg/forza-motorsport-will-include-a-new-suspension-model/ | 2021-12 (not verified) | — |
| 17 | https://gamingbolt.com/forza-motorsport-8-will-feature-new-tire-model-pressure-system-and-suspension-model | 2019-12 (not verified) | — |
| 18 | https://blogs.microsoft.com/next/2012/05/04/the-making-of-forza-motorsport-4-physics-pirelli-and-performance/ | 2012-05-04 | Pirelli |
| 19 | https://news.xbox.com/en-us/2013/09/20/games-forza-5-new-gen-racing/ | 2013-09-20 | Calspan |
| 20 | https://gamingbolt.com/forza-motorsport-5-features-cutting-edge-understanding-of-tire-physics-aerodynamics | 2013-08 (not verified) | — |
| 21 | https://www.autoblog.com/2009/06/04/sneak-peek-forza-motorsport-3/ | 2009-06-04 | FM2 360 FPS physics; FM3 sidewall flex |
| 22 | https://www.techradar.com/news/gaming/consoles/forza-3-pushes-xbox-further-than-ever-before-627284 | ~2009 (not verified) | — |
| 23 | https://gamesradar.com/you-need-another-two-xbox-360s | ~2007 (not verified) | Title not identified in snippet |
| 24 | https://news.xbox.com/en-us/?p=10141 | 2015 (not verified) | FM6 rain, 148 surfaces |
| 25 | https://www.teamvvv.com/?p=22561 | 2015 (not verified) | Greenawalt/IGN porosity, aquaplaning |
| 26 | https://www.drivelife.co.nz/2015/06/forza-motosport-6-gameplay-and-release-date-at-e3/amp/ | 2015-06 | 3D puddles, drag |
| 27 | https://www.bsimracing.com/?p=57276 | 2015 (not verified) | Static weather |
| 28 | https://forums.forza.net/t/hydroplaning/26008 | ~2015 | Player feedback |
| 29 | https://gtplanet.net/forza-motorsport-7s-dynamic-weather-system-offers-visceral-experience | 2017 | Giese, dynamic weather |
| 30 | https://forums.forza.net/t/lets-discuss-wet-conditions/755254 | ~2024 (not verified) | Low-confidence claim |
| 31 | https://www.gameswelt.de/forza-motorsport-2023/news/das-erste-update-ist-endlich-da-322817 | late 2023 | First patch: wet tire physics |
| 32 | https://support.forza.net/hc/en-us/articles/21742934024211-Forza-Motorsport-Data-Out-Documentation | undated | Telemetry fields |
| 33 | https://support.forza.net/hc/en-us/articles/51744149102611-Forza-Horizon-6-Data-Out-Documentation | undated | FH6 telemetry |
| 34 | https://support.forza.net/hc/en-us/articles/360005306674-FH3-Tuning-Settings | undated | Tuning categories (FM6 list) |
| 35 | https://support.forza.net/hc/en-us/articles/360012861313-FM7-Wheel-Introduction | 2018-12 (update date) | FFB physics |
| 36 | https://gtplanet.net/setting-your-wheel-up-for-forza-motorsport-7s-new-force-feedback-system | 2018-12 (not verified) | — |
| 37 | https://support.forza.net/hc/en-us/articles/51642814113427-Forza-Horizon-6-on-Wheel-Advanced-Wheel-Tuning | undated (FH6) | Normal vs Simulation steering |
| 38 | https://support.forza.net/hc/en-us/articles/20964254277267-Forza-Motorsport-Accessibility-Support | undated | FM assists |
| 39 | https://jalopnik.com/forza-horizon-5-will-handle-better-have-a-more-engagin-1847526912 | 2021 (not verified) | FH5 suspension/brakes rebuilt |
| 40 | https://racinggames.gg/article/forza-horizon-5-major-handling-upgrades-revealed | 2021 (not verified) | FH5 ABS |
| 41 | https://wccftech.com/forza-horizon-5-q-campaign-is-more-structured/amp/ | 2021-08-24 | Physics answers truncated in snippet |
| 42 | https://forums.forza.net/t/physics-in-fh5/534470 | 2021 | Esaki "don't know" re FH5 tire model |
| 43 | https://videogameschronicle.com/news/forza-horizon-5s-second-expansion-rally-adventure-has-been-revealed | 2023 | Deformable terrain |
| 44 | https://whatculture.com/gaming/forza-horizon-interview-design-director-ralph-fulton | ~2012 (not verified) | Shared codebase |
| 45 | https://gamingbolt.com/forza-motorsport-2023-review-racing-renaissance | 2023-10 (not verified) | Review observations |
| 46 | https://forums.forza.net/t/steering-difference-between-wheel-vs-controller/78043 | not verified | Pad steering limit |
| 47 | https://forums.forza.net/t/steering-normal-or-simulation/98148 | not verified | Input filter description |
| 48 | https://forums.forza.net/t/forza-motorsport-2023-tuning-guide-part-1-2-now-available/729034 | 2023–2024 | Hot pressure observations |
| 49 | https://www.sportskeeda.com/esports/forza-horizon-5-rally-adventures-guide-what-anti-lag-launch-control | 2023 | Launch control |
| 50 | https://idcgames.com/en/forza-motorsport/news/new-ai-and-physics-make-every-race-a-challenge-2023-07-18-15-58-7247 | 2023-07-18 | Repost of #1 |
