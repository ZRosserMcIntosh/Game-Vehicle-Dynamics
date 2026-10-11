# Forza handling philosophy and what it means for OpenRoad Dynamics

> **Companion to** [`forza-physics.md`](./forza-physics.md). That document covers *what* is simulated. This one covers *how* the Forza series makes one simulation serve casual and expert players, and what we take from that for our FUN / SPORT / SIMULATION presets.
>
> **Research date:** 2026-10-10. Same method and evidence classes as the physics document: **C** = confirmed developer/official statement, **O** = player/press observation, **I** = our inference, **U** = unknown.
>
> **Retrieval limitation.** Direct page fetches were blocked by the sandbox network policy during this session. All findings rest on search-result excerpts and summaries of the cited pages. Check quotes against the live pages before citing them externally.

---

## ⚠ Caveat: no claim of Forza equivalence

OpenRoad Dynamics **must not claim** to "feel like Forza", to "match Forza Horizon" or to be "Forza Motorsport-grade". Our reasons:

1. Forza's tire, suspension and assist implementations are proprietary and largely undocumented (see *Unknowns* in the physics document).
2. Every statement below is qualitative. Turn 10 and Playground publish no response curves, gains or filter constants.
3. Equivalence could only be claimed after **measured comparison**, for example matched-car tests of steady-state cornering, step-steer yaw response, braking distance, and assist intervention traces recorded from the public Data Out telemetry. Even then the claim should be limited to the measured manoeuvres. Telemetry would be captured only through the official UDP feature and only under the game's terms.

Acceptable phrasing: *"inspired by the accessibility approach of games such as Forza Horizon"*. Unacceptable: *"Forza-level physics"*. We should also avoid Forza trademarks in product naming.

---

## 1. One simulation, many players: the Forza approach

| Finding | Class | Source | Date | Game | Conf. | Implication |
|---|---|---|---|---|---|---|
| Ralph Fulton (Playground): Horizon began from "the Turn Ten Forza Motorsport codebase… the physics system, the way the cars handle and the way they react to the road is exactly the same" (snippet). | C (via press) | whatculture.com Ralph Fulton interview | ~2012 (not verified) | FH1 | Medium | Accessibility was added **on top of** a simulation core. The core was not replaced by an arcade model. |
| Press: both series run on ForzaTech, "watered down for arcade playability in the open-world titles". | O | jalopnik.com FH5 article | 2021 | FH5 | Low–Medium | "Watered down" is the journalist's word. Mechanisms are unspecified. |
| FH5 rebuilt suspension and braking "for more natural handling". ABS was remodelled so brake pressure eases off. | C (via press) | jalopnik.com; racinggames.gg | 2021 | FH5 | Medium | Horizon invests in *more* realism to improve feel, not less. |
| Esaki (Turn 10) on the FM 2023 suspension rework: it is "all just about making our cars much easier for all of our players to drive". | C (via press) | traxion.gg | 2021-12 | FM 2023 | Medium | **Fidelity serves accessibility.** A physically coherent car is more predictable and so easier to drive. |
| FM 2023 heat model: grip falls off "less abruptly" at extreme temperatures, aimed at "predictability". | C (via press) | overtake.gg / forza.net July 2023 | 2023-07 | FM 2023 | Medium | Smooth, readable limits are a deliberate design target even in the sim title. |
| Former Playground staff (Raeburn, Davis-Hunt, Conti) reflect on Horizon's origins; partly inspired by DiRT 2's festival framing. Their handling-design remarks were not visible in the snippets. | O | traxion.gg/how-dirt-2-inspired-forza-horizons-secret-ingredient/ | not verified | FH1 | Low | Could not verify a reported "not a simple fun slider" quote. Do not cite. |

## 2. How Forza Horizon makes it accessible

### 2.1 Steering: Normal vs Simulation (the most important public document)

From the official **FH6 Advanced Wheel Tuning** support article (snippets):

- "There are multiple systems layered together to create Normal steering. **Some of these layers alter inputs where some of these layers alter the vehicle dynamics.** Simulation Steering turns these systems off."
- "The input layers of these systems are turned off any time you use a wheel, regardless of the steering setting."
- Normal "influences the **yaw inertia** of the car to help maintain control during rapid lock-to-lock steering situations. This makes the car somewhat less agile than with Simulation steering, but also more stable", and it can make FFB feel sharper.

| Finding | Class | Source | Date | Game | Conf. | Implication |
|---|---|---|---|---|---|---|
| Normal steering = input layers + vehicle-dynamics layers. Simulation removes both. A wheel always removes the input layers. | C | support.forza.net/…/51642814113427 | undated (FH6, 2026) | FH6 | High | Assists split into two separate classes: **input shaping** (device-dependent) and **dynamics assistance**. |
| The Normal dynamics layer modifies yaw inertia, trading agility for stability. | C | same | same | FH6 | High | Playground openly uses a non-physical dynamics change. We should make any such assist **explicit, named and optional**, never hidden. |
| Players describe the pad input layers as a lock-to-lock rate limiter plus "a filter to limit the input based on tyre friction". Telemetry steering on pad stops increasing past a speed-dependent limit. | O | forums.forza.net threads 98148, 78043 | ~2017–2018 | FH3/FH4/FM7 era | Low–Medium | Pad steering is (I) a rate limiter plus a slip-angle-aware lock limiter. This is the core of controller accessibility. |
| Some players report that Simulation on pad still feels filtered or nonlinear, with few steering rate "stages". | O | forums.forza.net 45481, 98148 | various | FH | Low | The pad response curve matters even in Simulation. Keep a minimal, documented pad curve separate from assists. |
| A community guide claims Normal applies an "artificial background gyro that counter-steers for you". | O | itemlevel.net FH6 wheel guide | 2026 | FH6 | Low | Plausible as a description of the yaw-inertia layer. Not official wording. |

### 2.2 Other assists (Horizon and Motorsport menus)

| Assist | Documented behaviour | Class / Source | Conf. |
|---|---|---|---|
| **Braking** | FM: assisted braking (can brake for corners and imminent impacts), ABS on, ABS off. FH: ABS on/off. FH5 ABS eases pressure rather than simply modulating. | C: FM Accessibility Support; C via press: racinggames.gg (FH5) | Medium–High |
| **Steering** | FM: fully assisted / partially assisted / Normal / Simulation. Simulation removes damping and steering-speed help ("difficult with a controller"). FH: Normal / Simulation (some FH versions also have an auto/assisted option). | C: FM Accessibility Support; FH6 support | Medium–High |
| **Traction control** | FM 2023: On / Sport / Off. Sport allows slightly more slip before intervention. (An older difficulty page lists Super Easy / On / Off, so naming varies by title.) FH: On/Off. Guides describe it as limiting power to available grip. | C: FM Accessibility Support; support.forza.net 360005363194; O: guides | Medium |
| **Stability control** | On/Off. Community: applies individual brakes to stop sliding or spinning. No official mechanism description found. | O | Low–Medium |
| **Shifting** | Automatic / Manual / Manual with clutch. | O (guides; standard menu) | Medium |
| **Driving line** | Full / braking-only / off (standard menu). | O | Medium |
| **Damage & tire wear, fuel** | FM 2023 simulates wear and fuel; these are toggleable race settings. | C (wear/fuel exist) | Medium |
| **Rewards** | FH gives credit bonuses for disabling assists (e.g. TCS off) and raising difficulty. | O: gamesradar.com FH5 tips | Medium |
| **Launch control** | FH5 (Rally Adventure): reintroduced, presented with anti-lag. | O | Low |
| **Countersteer assist** | No official Horizon or Motorsport description found as a separate menu item. Partly covered by the Normal steering dynamics layer. | U | — |

### 2.3 Tuning as accessibility

- Both series expose the same tuning categories: tires, gearing, alignment, ARBs, springs, damping, aero, brakes, differential (C, FH3 Tuning FAQ, which lists FM6). Parts gate them: race parts unlock adjustment.
- Fulton said FH3 added no new tuning options. Horizon deliberately keeps the Motorsport tuning set rather than inventing arcade sliders (C, Medium).
- Community drift tunes (e.g. 100% diff lock) show that **drifting is achieved through physical setup**, not a "drift mode" (O, Medium). Implication: our FUN preset can ship drift-friendly *setups* rather than special physics.

### 2.4 Force feedback

- FM7 onwards: FFB is "completely physically based", built from contact-patch aligning torque (mechanical + pneumatic trail). Caster changes feel (C, High).
- FH6: Normal steering changes yaw inertia and so changes FFB character (C, High).
- Implication: FFB should come from the simulation, with assist effects visible (or switchable) in FFB, so wheel users can feel what the assist is doing.

## 3. How Forza Motorsport differs

| Aspect | Forza Horizon | Forza Motorsport | Class / Conf. |
|---|---|---|---|
| Physics base | Started from FM codebase (2012). FH5 rebuilt suspension/brakes itself. Whether it uses the FM 2023 tire model is **unknown**. | 8-point/360 Hz tire collision, new suspension, heat model (2023) | C, Medium |
| Steering assist levels | Normal / Simulation (+ assisted) | Fully assisted → partially → Normal → Simulation | C, Medium–High |
| TCS granularity | On/Off | On/Sport/Off | C, Medium |
| Wet modelling | Puddle presence (FH6 telemetry boolean) | Puddle depth 0–1 (FM telemetry); historic 148-surface wet table (FM6), dynamic dry line (FM7) | C, High (telemetry fields) |
| Track evolution | Not documented | Time-of-day track temperature, rubber-in | C, High |
| Wear / fuel / compounds | Not central | Wear, fuel, Soft/Medium/Hard race compounds | C, High |
| Terrain | Off-road; deformable sand/mud (Rally Adventure, 2023) | Circuit racing only at launch ("No Dirt, Drag, or Drift at launch") | C via press, Medium |
| AI | Drivatars (Horizon) | ML-trained AI, "no rubber-banding" (FM 2023) | C, Medium |

Design reading (I): Motorsport adds **state** (temperatures, rubber, wear, fuel, water depth) and finer assist **grades**. Horizon adds **surface variety** and stronger **dynamics-level stabilisation** for pad users. The rigid-body and tire core is conceptually shared.

## 4. Implications for FUN / SPORT / SIMULATION

All three presets run the **same vehicle simulation** (same tire, suspension and drivetrain models and the same car data). Presets change only:
(a) input shaping, (b) assist controllers, (c) optional explicit stability aids, (d) environmental-state features, (e) default setups.

| Layer | FUN | SPORT | SIMULATION |
|---|---|---|---|
| Pad steering rate limit | Strong, speed-scaled | Moderate | Minimal (documented pad curve only) |
| Friction-aware steering limiter (cap steer at ≈ peak slip angle, using normalized slip) | On, tight to peak | On, allows slight overshoot | Off |
| Countersteer assist (adds steering toward the yaw-rate target, input-side) | On | Light | Off |
| Yaw stabilisation (ESC-style per-wheel braking with a yaw-rate target) | On | Off or Sport mode | Off (car's real ESC only, if fitted) |
| "Yaw inertia" style dynamics assist | **Avoid by default.** If offered, label it explicitly ("Stability Damping") and keep it off in SPORT/SIM | Off | Off |
| TCS | On (low slip target) | Sport (higher slip target) | Off / car's real system |
| ABS | On | On | Car-dependent; off available |
| Braking assist (auto-brake for corners) | Optional | Off | Off |
| Shifting | Auto | Auto/Manual | Manual (+clutch) |
| Tire temperature/wear/pressure | Simulated, start in-window, slow wear | Simulated, normal | Simulated, cold start |
| Wet/aquaplaning | Simulated, gentler onset ramp *via environment settings* (shallower water), never via hidden grip | Normal | Full |
| Damage / fuel | Off/cosmetic | Optional | On |
| Default setup | Stable, understeer-biased tune shipped as a setup file | Neutral | Car's real baseline |

## 5. Design principles we adopt

1. **One physical model.** No preset changes μ, tire curves, mass, inertia or aero "for fun". Accessibility is a layer *around* the car, not a different car.
2. **Assist through inputs and controllers, not fake grip.** Assists act where a real driver or a real ECU would act: steering angle and rate, throttle, brake pressure per wheel, gear selection. This keeps the model honest, keeps SIMULATION meaningful, and keeps tuning knowledge transferable across presets.
3. **Name every non-physical aid.** Playground's Normal steering openly changes yaw inertia (C). If we ever ship an equivalent, it is a separately named, separately toggleable assist that shows up in telemetry. It is never folded silently into "Normal".
4. **Device-aware input shaping.** As in FH (C), input-shaping layers apply to pad/keyboard and are disabled for wheels by default. Dynamics assists are device-independent.
5. **Use normalized slip as the shared language.** Assists, FFB, audio and telemetry read |σ| relative to peak (as Forza telemetry does, C). Assists target "just below peak", and that target is tunable per preset.
6. **Predictable limits.** Smooth thermal falloff, smooth aquaplaning onset and smooth curb transitions (multi-probe contact). Turn 10 lists predictability as a goal (C).
7. **Fidelity is accessibility.** Better suspension and contact modelling make the car easier to read (Turn 10, C). We should not "simplify for casual players" by removing physics. We hide complexity in defaults and assists.
8. **Physical FFB.** Steering torque from pneumatic + mechanical trail. Separate user scale factors per component (C, FM7+ approach).
9. **Tuning parity.** The same tuning categories in all presets. FUN simply ships stable default setups.
10. **Observable assists.** Each assist writes its intervention (torque cut, brake delta, steer correction) to telemetry so we can measure, debug and later compare against reference data.
11. **No equivalence claims without measurement** (see caveat above).

## 6. Open questions (to resolve by our own testing, not by assumption)

- What friction-aware steering limit (fraction of peak slip angle) feels best on pad for FUN vs SPORT? Needs playtesting.
- Can input-only countersteer assist plus ESC reach FH-like pad stability without a yaw-inertia hack? This is our main hypothesis to validate.
- What onset ramp for aquaplaning avoids FM6's reported "puddles feel like ice"?
- How do we reward assist-off driving (FH credit multipliers) without implying assists are "cheating"?

---

## Sources

Seen in search results on 2026-10-10. **Direct fetching was blocked** (sandbox network policy), so content comes from search excerpts and summaries. "Not verified" = date inferred from context.

| # | URL | Date | Notes |
|---|---|---|---|
| 1 | https://support.forza.net/hc/en-us/articles/51642814113427-Forza-Horizon-6-on-Wheel-Advanced-Wheel-Tuning | undated (FH6) | Normal vs Simulation steering, yaw inertia |
| 2 | https://support.forza.net/hc/en-us/articles/4409761195923-FH5-Wheel-Setup-and-Tuning | undated | Not retrievable. Referenced only via a Steam link |
| 3 | https://support.forza.net/hc/en-us/articles/20964254277267-Forza-Motorsport-Accessibility-Support | undated | FM assists |
| 4 | https://support.forza.net/hc/en-us/articles/360005363194-Difficulty-Assists-and-Drivatars | undated | Older assist naming (TCS Super Easy/On/Off) |
| 5 | https://support.forza.net/hc/en-us/articles/360005306674-FH3-Tuning-Settings | undated | Tuning categories, Fulton "no new tuning options" |
| 6 | https://support.forza.net/hc/en-us/articles/360012861313-FM7-Wheel-Introduction | 2018-12 (update) | Physical FFB |
| 7 | https://gtplanet.net/setting-your-wheel-up-for-forza-motorsport-7s-new-force-feedback-system | 2018-12 (not verified) | Mechanical/pneumatic trail scales |
| 8 | https://support.forza.net/hc/en-us/articles/21742934024211-Forza-Motorsport-Data-Out-Documentation | undated | Normalized slip, puddle depth |
| 9 | https://support.forza.net/hc/en-us/articles/51744149102611-Forza-Horizon-6-Data-Out-Documentation | undated | FH6 WheelInPuddle boolean |
| 10 | https://whatculture.com/gaming/forza-horizon-interview-design-director-ralph-fulton | ~2012 (not verified) | Shared FM codebase |
| 11 | https://jalopnik.com/forza-horizon-5-will-handle-better-have-a-more-engagin-1847526912 | 2021 (not verified) | FH5 suspension/brakes rebuilt |
| 12 | https://racinggames.gg/article/forza-horizon-5-major-handling-upgrades-revealed | 2021 (not verified) | FH5 ABS |
| 13 | https://traxion.gg/forza-motorsport-will-include-a-new-suspension-model/ | 2021-12 (not verified) | "easier for all of our players to drive" |
| 14 | https://www.gtplanet.net/forza-motorsport-suspension-model-20211208/ | 2021-12-08 | Suspension stream |
| 15 | https://www.overtake.gg/news/comments/59546/ | 2023 (not verified) | Heat-model predictability |
| 16 | https://forza.net/news/forza-motorsport-drivatars-tire-physics | 2023-07-17 | AI, tire physics (fetch failed) |
| 17 | https://traxion.gg/how-dirt-2-inspired-forza-horizons-secret-ingredient/ | not verified | Playground origins |
| 18 | https://forums.forza.net/t/steering-normal-or-simulation/98148 | not verified | Input-filter description (player) |
| 19 | https://forums.forza.net/t/steering-difference-between-wheel-vs-controller/78043 | not verified | Pad steering limit test |
| 20 | https://forums.forza.net/t/suggestion-of-fixes-for-gamepad-steering-input-options-either-normal-or-simulation-steering/45481 | not verified | Pad sim steering complaints |
| 21 | https://itemlevel.net/best-wheel-settings-in-forza-horizon-6-guide-900-simulation-steering/ | 2026 (not verified) | "background gyro" claim (community) |
| 22 | https://www.gamesradar.com/forza-horizon-5-tips | not verified | Assist reward bonuses |
| 23 | https://simracingsetup.com/forza/forza-motorsport-assists-settings/ | not verified | Assist descriptions (third-party) |
| 24 | https://www.sportskeeda.com/esports/forza-horizon-5-rally-adventures-guide-what-anti-lag-launch-control | 2023 | Launch control |
| 25 | https://gtplanet.net/forza-motorsport-focus-modern-circuit-racing-20230718 | 2023-07-18 | No dirt/drag/drift at launch |
| 26 | https://videogameschronicle.com/news/forza-horizon-5s-second-expansion-rally-adventure-has-been-revealed | 2023 | Deformable terrain |
| 27 | https://drifted.com/forza-horizon-5-drift-tune | not verified | Drift via setup (community) |
| 28 | https://news.xbox.com/en-us/2022/06/12/the-all-new-forza-motorsport-is-the-most-technically-advanced-racing-game-ever-made/ | 2022-06-12 | FM 2023 feature set |
