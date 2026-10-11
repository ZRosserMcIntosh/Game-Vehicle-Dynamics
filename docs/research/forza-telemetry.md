# Forza "Data Out" UDP telemetry: what it is and how we can use it for validation

**Project:** OpenRoad Dynamics (MIT, engine-independent C++ core with a UE5 adapter)
**Status:** Research note, written 2026-10-10
**Scope:** Forza Motorsport 7 (FM7), Forza Horizon 4 (FH4), Forza Horizon 5 (FH5), Forza Motorsport (2023, "FM8"/FM2023). Forza Horizon 6 (FH6) is noted where its official page adds information.

> **Confidence key.** **High:** confirmed by at least two independent sources, or read directly from source code or a primary text. **Medium:** a single primary source seen only as a search-engine extract (the official pages could not be opened directly from this environment, see the method note), or one good secondary source. **Low:** inference, a community claim, or not verified.
>
> **Method note.** The official Forza support pages (support.forza.net, formerly support.forzamotorsport.net) and forums.forza.net were **blocked by the research environment's network proxy**. Statements attributed to them come from search-engine extracts of those pages. They have not been read in full. The byte layout itself was checked against source code in three independent open-source parsers, all fetched as raw files. Before we rely on any field semantics in a release, someone should open the official page in a normal browser and recheck them.

---

## 1. Summary

| Fact | Value | Confidence |
|---|---|---|
| Transport | One-way UDP to a user-configured IP and port, enabled in the game's settings | High |
| Formats | **Sled** (motion-platform subset) and **Dash** (Sled plus dashboard data). FM2023 says its formats are "identical to Forza Motorsport 7" | Medium (official extract) |
| Packet sizes | Sled **232 B**. FM7 Dash **311 B**. FH4/FH5 Dash **324 B**. FM2023 Dash **331 B** | High (code in 3 parsers) |
| FH4/FH5 quirk | **12 extra bytes at offsets 232–243**, inserted between the Sled block and the Dash block, plus **1 trailing byte** (offset 323). Horizon games always send Dash | High |
| FM2023 additions | `TireWear` FL/FR/RL/RR (f32, 0 = new, 1 = fully worn) and `TrackOrdinal` (s32), at offsets 311–330 | High (layout), Medium (semantics) |
| Rate | FM2023: **60 packets/s** (official). FH6: **"equal to the game's frame rate"** (official). FM7/FH4/FH5: no official figure seen; community tools report 60 Hz for FM | Medium |
| Endianness | Little-endian (all parsers use `<`/LE layouts) | High |
| Local axes | Car local space: **X = right, Y = up, Z = forward**. Angular velocity: X = pitch, Y = yaw, Z = roll | High (doc text reproduced in parsers) |
| Units documented | Speed m/s, power W, torque N·m, wheel speed rad/s, suspension travel m | High |
| Units **not** documented | Tire temperature (community values suggest °F), Yaw/Pitch/Roll (radians is likely), Boost, Fuel, DistanceTraveled, position frame | Low |

---

## 2. Official documentation

| Game | Official page | Notes | Confidence |
|---|---|---|---|
| FM2023 | "Forza Motorsport 'Data Out' Documentation", support.forza.net article 21742934024211 | States 60 packets/s, Sled and Dash formats identical to FM7, localhost supported. Settings live under SETTINGS > GAMEPLAY & HUD > "UDP RACE TELEMETRY" | Medium |
| FM7 | Forum thread "Forza Motorsport 7 'Data Out' feature details" (forums.forza.net/t/74013) | The original field list with comments. The richstokes `FM7_packetformat.dat` reproduces this list with the official comments | Medium |
| FH4 / FH5 | **No official FH4/FH5 field document was found.** There is a community thread, "Data Out Telemetry Variables and Structure" (FH5 Discussion, forums.forza.net/t/535984) | Parsers handle the 12-byte gap empirically. geeooff calls the Horizon extras "not documented" and decodes only a community-guessed `CarCategory` | Medium |
| FH6 | "Forza Horizon 6 'Data Out' Documentation", support.forza.net article 51744149102611 | Single packet format, **324 bytes**, sent at **the game's frame rate**. Warns: **avoid ports 5200–5300**, because the game binds its own outgoing socket in that range | Medium |

Other points from the forums:
- A SimHub developer noted that some fields the game actually sends (for example lap and driver counts) are **undocumented**. Treat the documentation as possibly lagging the real data. (Medium, forum extract.)
- One forum post said the FM2023 packet has "6 additional fields (4 floats tire wear + 2 × unknown 2-byte fields)". The parser code models offsets 311–330 as 4 × f32 plus 1 × s32 (`TrackOrdinal`). The byte count (20) matches both readings. We follow the official field list (s32 `TrackOrdinal`). (Medium.)
- heat-cycle (an MIT-licensed FM2023 companion app) observed packets arriving at 60/s, all 331 bytes, in its own recordings. It also saw the `TrackOrdinal`→track mapping vary (880 vs 882 for the same venue), so ordinal tables should not be trusted without our own capture. (Low–Medium, project issue tracker extract.)

---

## 3. Packet layout

Offsets and types are from the richstokes `FM7_packetformat.dat` (official comments reproduced), nettrom `fdp.py` (Python `struct` format strings), and geeooff `ForzaDataReader.cs` (explicit byte ranges). All three agree. Types: s = signed int, u = unsigned, f32 = IEEE float.

### 3.1 Sled block (offsets 0–231, all games)

| Off | Type | Field | Official comment / unit |
|---|---|---|---|
| 0 | s32 | IsRaceOn | 1 when race is on, 0 in menus or when the race is stopped |
| 4 | u32 | TimestampMS | ms. "Can overflow to 0 eventually" |
| 8 | f32 | EngineMaxRpm | rpm |
| 12 | f32 | EngineIdleRpm | rpm |
| 16 | f32 | CurrentEngineRpm | rpm |
| 20 | f32 ×3 | AccelerationX/Y/Z | car local space, X right, Y up, Z forward (unit not stated; m/s² assumed) |
| 32 | f32 ×3 | VelocityX/Y/Z | car local space (m/s assumed) |
| 44 | f32 ×3 | AngularVelocityX/Y/Z | car local: X = pitch, Y = yaw, Z = roll (rad/s assumed) |
| 56 | f32 ×3 | Yaw, Pitch, Roll | unit not stated (radians likely) |
| 68 | f32 ×4 | NormalizedSuspensionTravel FL/FR/RL/RR | 0.0 = max stretch, 1.0 = max compression |
| 84 | f32 ×4 | TireSlipRatio FL..RR | **normalized**: 0 = 100% grip, \|ratio\| > 1.0 = loss of grip |
| 100 | f32 ×4 | WheelRotationSpeed FL..RR | rad/s |
| 116 | s32 ×4 | WheelOnRumbleStrip FL..RR | 1 on, 0 off |
| 132 | f32 ×4 | WheelInPuddleDepth FL..RR | 0..1, 1 = deepest puddle |
| 148 | f32 ×4 | SurfaceRumble FL..RR | non-dimensional, used for controller force feedback |
| 164 | f32 ×4 | TireSlipAngle FL..RR | **normalized**: 0 = 100% grip, \|angle\| > 1.0 = loss of grip |
| 180 | f32 ×4 | TireCombinedSlip FL..RR | **normalized**: 0 = 100% grip, \|slip\| > 1.0 = loss of grip |
| 196 | f32 ×4 | SuspensionTravelMeters FL..RR | actual travel in metres |
| 212 | s32 | CarOrdinal | unique ID of make/model |
| 216 | s32 | CarClass | 0 (D) .. 7 (X) |
| 220 | s32 | CarPerformanceIndex | 100..999 |
| 224 | s32 | DrivetrainType | 0 FWD, 1 RWD, 2 AWD |
| 228 | s32 | NumCylinders | |

Sled total = **232 bytes**.

### 3.2 Horizon extras (FH4/FH5 only, offsets 232–243)

12 undocumented bytes. geeooff decodes them as `CarCategory` (s32, community guess) plus two unknown s32 values. nettrom simply skips them (`data[:232] + data[244:323]`). **Confidence High** that they exist, **Low** on meaning.

### 3.3 Dash block (FM7/FM2023 at offset 232; FH4/FH5 at 244)

The table gives the relative offset from the start of the Dash block (add 232 for FM, 244 for FH).

| Rel | Type | Field | Unit / comment |
|---|---|---|---|
| 0 | f32 ×3 | PositionX/Y/Z | world position (m assumed, frame undocumented) |
| 12 | f32 | Speed | m/s |
| 16 | f32 | Power | W |
| 20 | f32 | Torque | N·m |
| 24 | f32 ×4 | TireTemp FL..RR | unit undocumented (community values look like °F) |
| 40 | f32 | Boost | unit undocumented (psi likely) |
| 44 | f32 | Fuel | fraction likely |
| 48 | f32 | DistanceTraveled | m likely |
| 52 | f32 | BestLap | s |
| 56 | f32 | LastLap | s |
| 60 | f32 | CurrentLap | s |
| 64 | f32 | CurrentRaceTime | s |
| 68 | u16 | LapNumber | |
| 70 | u8 | RacePosition | |
| 71 | u8 | Accel | 0..255 |
| 72 | u8 | Brake | 0..255 |
| 73 | u8 | Clutch | 0..255 |
| 74 | u8 | HandBrake | 0..255 |
| 75 | u8 | Gear | encoding of reverse/neutral not verified |
| 76 | s8 | Steer | −127..127 |
| 77 | s8 | NormalizedDrivingLine | |
| 78 | s8 | NormalizedAIBrakeDifference | |

Dash block = 79 bytes, so FM7 = 232 + 79 = **311**. FH4/FH5 = 232 + 12 + 79 + 1 trailing = **324**.

### 3.4 FM2023 extension (offsets 311–330)

| Off | Type | Field | Comment |
|---|---|---|---|
| 311 | f32 ×4 | TireWear FL/FR/RL/RR | 0.0 new, 1.0 fully worn |
| 327 | s32 | TrackOrdinal | "ID for track" |

Total **331 bytes**. Note that the fields are **unaligned** (311 is not a multiple of 4). Parse with `memcpy` or byte-wise reads, never by casting to a struct pointer.

### 3.5 Format detection rule (recommended)

Detect the format by datagram length: 232 → Sled. 311 → FM7 Dash. 324 → FH4/FH5/FH6 Dash. 331 → FM2023 Dash. Reject any other length. This is what geeooff does. Also ignore every packet with `IsRaceOn == 0`.

---

## 4. Existing open-source parsers (licenses verified from LICENSE files on 2026-10-10)

| Repo | Language | Games | License (verified) | Reuse in MIT? |
|---|---|---|---|---|
| [geeooff/forza-data-web](https://github.com/geeooff/forza-data-web) | C# (.NET 8) | FM7, FH4, FH5, FM2023 | **Apache-2.0** | Yes, with Apache notices kept. Cleanest reference for byte ranges |
| [nettrom/forza_motorsport](https://github.com/nettrom/forza_motorsport) | Python | FM7 Sled/Dash, FH4 | **MIT** (© 2018 Morten Wang) | Yes |
| [austinbaccus/forza-telemetry](https://github.com/austinbaccus/forza-telemetry) | C# | FM2023, FM7, FH4, FH5 | **MIT** (© 2021 Austin Baccus) | Yes |
| [heat-cycle/heat-cycle](https://github.com/heat-cycle/heat-cycle) | Swift (early stage) | FM2023 | **MIT** code, CC BY 4.0 data | Yes. Useful empirical notes in its issues |
| [richstokes/Forza-data-tools](https://github.com/richstokes/Forza-data-tools) | Go | FM7, FH4/5, FM2023 | **GPL-3.0** | **No code reuse.** Reading its field list as documentation is fine (facts are not copyrightable, but write our own table) |
| [0x20F/forza-telemetry](https://github.com/0x20F/forza-telemetry) | Rust | multiple | **No LICENSE file found** = all rights reserved | No |
| [theRTB/ForzaGUI](https://github.com/theRTB/ForzaGUI) | Python | FH | **No LICENSE file found** | No |

**Recommendation:** write our own small C++ decoder (about 150 lines) from the field table in §3. It needs no third-party code. Use nettrom or austinbaccus (MIT) as cross-check test oracles.

---

## 5. Can player-recorded telemetry support controlled comparisons with our sim?

**Short answer: yes, for *behavioural* comparisons at vehicle level ("does our car feel and measure like a plausible car of this class"), but not as ground truth for tire-force models or real-car validation.** Forza is itself a simulation with unknown parameters. Agreement with it shows plausibility, not physical correctness. We should present such comparisons as "game-reference behavioural benchmarks". Real-world validation should rest on published data (see `alternatives.md` §6).

### 5.1 Feasible tests

| Test | Forza channels used | Our metric | Feasibility |
|---|---|---|---|
| 0–100 km/h / 0–60 mph, ¼-mile | Speed, TimestampMS, Gear, Accel, CurrentEngineRpm | time-to-speed, shift points, traction-limited launch phase (slip ratio) | **Good.** Speed and time are explicit SI |
| Top speed / drag-limited coast-down | Speed, Accel = 0 & Clutch, Power | deceleration vs v → fit F = a + c·v² (lumped drag + rolling). Gives a *ratio*, since mass is unknown | **Good** |
| Braking 100→0 | Speed, Brake, AccelerationZ, TireSlipRatio, NormalizedSuspensionTravel | stopping distance, peak decel (g), pitch/dive, ABS behaviour (slip oscillation) | **Good** |
| Steady-state skidpad (constant-radius) | Velocity, AngularVelocityY, AccelerationX, Steer, Position | lateral g at limit, understeer gradient (steer vs a_y), body sideslip β = atan(Vx/Vz) | **Good** if the driver holds a steady line. A fixed track circle is best (Horizon open world has flat areas) |
| Slip-angle vs lateral-accel curve | TireSlipAngle (normalized!), AccelerationX, Velocity, yaw rate | a_y vs **normalized** slip angle per axle. Physical axle slip angle can be *estimated* from β, yaw rate and steer, but only with an assumed wheelbase and CG | **Partial.** Forza's slip is normalized to the tire's own peak (≈1 at peak grip), so it shows *where* the peak is relative to grip, not radians |
| Step-steer / sine-steer (ISO 7401-style) | Steer, AngularVelocityY, AccelerationX, Roll | yaw-rate rise time, overshoot, roll gradient | **Medium.** Inputs come from a controller or wheel, so steer timing has jitter; Steer resolution is 8-bit |
| Suspension travel / roll / pitch gradients | SuspensionTravelMeters, NormalizedSuspensionTravel, Roll, Pitch, accelerations | travel per g, roll deg/g, bump-stop contact (normalized → 1), wheel hop over kerbs (WheelOnRumbleStrip) | **Good** for gradients, **weak** for damping (60 Hz Nyquist = 30 Hz, wheel hop is about 10–15 Hz so it is borderline) |
| Gear ratios / effective rolling radius | CurrentEngineRpm, WheelRotationSpeed, Speed, Gear | overall ratio per gear = ω_engine / ω_wheel. r_eff = Speed / ω_wheel at zero slip | **Good** |
| Power/torque curve | Power, Torque, CurrentEngineRpm | engine map (as Forza reports it, likely at the crank) | **Good** |
| Physical slip ratio | WheelRotationSpeed, Speed, r_eff | κ = (ω·r_eff − v)/v. Compare with Forza's normalized TireSlipRatio to infer its peak-slip normalization | **Good** (this is a nice derived check) |

### 5.2 Limitations

1. **No forces.** There are no tire forces, loads (Fz), aligning moments or brake torques. Loads can only be *inferred* from suspension travel × unknown spring rates.
2. **Unknown vehicle parameters.** Mass, inertia, CG height, spring and damper rates, aero maps, tire parameters and differential settings are all hidden. `CarOrdinal`/`PI` identify the car but do not give parameters. Tuned cars make it worse. **Mitigation:** compare *dimensionless or normalized* quantities (g-levels, gradients, ratios), and set our sim's car from *public real-car specs*, never from Forza data.
3. **Normalized slip channels.** TireSlipRatio, TireSlipAngle and TireCombinedSlip are normalized to Forza's internal peak. Their mapping to physical slip is unknown and may differ by tire compound.
4. **Unknown filtering and sampling.** It is unknown whether channels are instantaneous physics-step values or render-frame snapshots. Forza's physics is widely reported to run faster than 60 Hz, but we have **no official figure**, so treat that as unverified. FH6 sends at frame rate, so the sample interval varies. **Always resample on `TimestampMS`, not on packet count.** UDP can drop or reorder packets, so detect gaps.
5. **Assists and driver.** ABS, TCS, STM, auto-clutch, steering assists and the controller's input filtering all change the result. The packet does not report assist state. It has to be logged by hand.
6. **Undocumented units** (tire temp, boost, fuel, angles, position frame) and the **unknown sign conventions** must be calibrated empirically (§6.3).
7. **Gravity in acceleration.** It is unknown whether `Acceleration*` includes gravity (specific force) or is pure kinematic acceleration. Calibrate at rest.
8. **Game balance.** Forza tunes cars for playability and class balance (PI). It is not a ground-truth physics reference.

### 5.3 Legal / ToS considerations (not legal advice)

- **Use only our own recordings**, made by contributors through the game's officially provided Data Out feature on copies they own. Do not use third-party telemetry dumps of unknown provenance.
- **Telemetry is data the game sends out by design.** Capturing it needs no reverse engineering, memory reading or file extraction. Never extract game files, car models, tire tables or physics parameters. Microsoft's Game Content Usage Rules forbid reverse-engineering games to access assets. (Rules page at xbox.com/en-US/developers/rules. Seen only via search extract, page dated 2015. Medium.)
- **No game assets in the repo.** No screenshots, HUD imagery, car names in marketing, logos or track maps traced from the game. Licensed cars and brands belong to their makers.
- **Do not claim official or real car data.** Label all comparisons "compared against our own recording of Forza X, car ordinal N, our settings". Do not say "validated against Forza" or "official BMW data". Forza and the car brand names should be used only descriptively ("Forza Motorsport is a trademark of Microsoft; not affiliated").
- **Redistribution of recordings.** Raw recordings are numeric data we generate ourselves. The safest policy is to commit **derived metrics and summary curves** to the repo, and keep raw captures in contributor-held storage, or commit them only small, anonymized and labelled. Microsoft's rules grant only a "personal, noncommercial" licence to game content. Whether telemetry numbers count as "Game Content" is unclear, so be conservative. (Low confidence on legal interpretation. Get counsel review before any public dataset release.)
- **Privacy:** strip gamertags and IP addresses (they are not in the packet, but they may appear in capture-tool metadata).

---

## 6. Proposed data-capture protocol

### 6.1 Session setup (log in a `session.yaml` alongside each capture)

- Game and version/build, platform (Xbox/PC), packet format (Sled/Dash), configured port (**not** 5200–5300 for FH6).
- Car: name as displayed, `CarOrdinal`, class/PI, **stock vs tuned** (stock only for benchmark captures), tire compound if shown, fuel load (FM).
- Assists: ABS, TCS, STM, transmission (auto/manual/manual+clutch), steering (normal/simulation), launch control. **Benchmark default: ABS off, TCS off, STM off, manual transmission, simulation steering**, plus a second run with ABS on for braking tests.
- Input device (pad/wheel model, deadzones, linearity settings).
- Track/location, layout, weather and time of day, surface (dry), `TrackOrdinal` (FM2023).
- Capture host: OS, receiver tool and version, NIC (wired preferred), clock source.

### 6.2 Recording rules

1. Record **raw datagrams** (binary, with a host receive timestamp in ns) to a `.fdo` file. Decode afterwards. This preserves undocumented bytes.
2. Use a wired LAN. On a PC game, sending to localhost is supported per the FM2023 doc.
3. Discard `IsRaceOn == 0` packets. Detect gaps where ΔTimestampMS exceeds 2× the median.
4. Warm up tires for 2 laps before limit tests (Forza simulates tire temperature).
5. For each manoeuvre, record **≥ 3 repetitions in both directions** (left/right, uphill/downhill cancellation).

### 6.3 Calibration manoeuvres (run first, every session)

| Step | Purpose |
|---|---|
| Stationary 10 s on level ground | Gravity in `AccelerationY`? Sensor bias. Static suspension travel. TireTemp ambient (unit inference) |
| Straight constant speed 50 / 100 km/h, 10 s each | Compare `Speed` with \|Velocity\|. r_eff = Speed/ω. Check longitudinal axis sign |
| Steady left-hand circle | Sign of `AngularVelocityY` (yaw), `AccelerationX`, `Steer`, `Roll` for a **left** turn → fixes the axis mapping |
| Gentle straight-line brake | Sign of `AccelerationZ` under braking. Pitch sign |

### 6.4 Manoeuvres

- **Launch/accel:** standstill → full throttle to 200 km/h, auto or manual shifts.
- **Coast-down:** 150 → 30 km/h in neutral (or clutch in) on a flat straight.
- **Braking:** 100 → 0 at max effort, ABS off and on.
- **Skidpad:** constant radius (≈ 40–60 m) at increasing speed until the limit, 30 s at each of 5 speeds.
- **Ramp-steer:** constant speed (80 km/h), slow steer ramp to the limit (gives a_y vs steer and β).
- **Step-steer:** 80 km/h, quick steer to a fixed angle and hold.
- **Kerb strike / bump:** straight-line pass over a rumble strip at 60 and 120 km/h (suspension response).

### 6.5 Mirror test in OpenRoad Dynamics

Configure our sim car from **public specs of the real car** (mass, wheelbase, track, power curve, gear ratios, tire size). Run the same manoeuvres with the same inputs (replay the recorded `Accel/Brake/Steer/Gear` traces, or use a closed-loop path follower for skidpad). Compare the metrics in §5.1 with tolerance bands, not point-wise traces.

---

## 7. CSV schema: Forza → OpenRoad telemetry channels

Our canonical channels use **SI units** and the **ISO 8855 vehicle frame (x forward, y left, z up; right-handed)**. UE5's adapter converts to UE (x forward, y right, z up, cm) separately.

Forza's local frame is X right, Y up, Z forward. Mapping to ISO 8855: `x_iso = Z_f`, `y_iso = −X_f`, `z_iso = Y_f`. That is a reflection plus a permutation, so **angular-velocity signs must be confirmed by the §6.3 left-turn calibration**. The table gives the *expected* sign, to be verified.

| CSV column (ours) | Unit | Source (Forza) | Transform | Notes |
|---|---|---|---|---|
| `t_s` | s | TimestampMS | (ms − ms₀)/1000, unwrap u32 overflow | resample to uniform 100 Hz later |
| `host_rx_ns` | ns | receiver clock | — | jitter analysis |
| `src_game` | enum | packet length | FM7/FH4/FH5/FM2023/FH6 | |
| `race_on` | bool | IsRaceOn | | |
| `car_ordinal`, `car_class`, `car_pi`, `drivetrain`, `n_cyl`, `track_ordinal` | int | same | | metadata, constant per run |
| `engine_rpm`, `engine_rpm_max`, `engine_rpm_idle` | rpm | CurrentEngineRpm, EngineMaxRpm, EngineIdleRpm | | |
| `ax_mps2`, `ay_mps2`, `az_mps2` | m/s² | AccelerationZ, −AccelerationX, AccelerationY | ISO remap | subtract g if the calibration shows it is included |
| `vx_mps`, `vy_mps`, `vz_mps` | m/s | VelocityZ, −VelocityX, VelocityY | ISO remap | |
| `p_radps`, `q_radps`, `r_radps` (roll, pitch, yaw rates) | rad/s | AngularVelocityZ, AngularVelocityX, AngularVelocityY | **signs TBD by calibration** | |
| `yaw_rad`, `pitch_rad`, `roll_rad` | rad | Yaw, Pitch, Roll | unit assumed rad, verify | |
| `beta_rad` (derived) | rad | atan2(vy, vx) | derived | body sideslip |
| `speed_mps` | m/s | Speed | | Dash only |
| `pos_x_m`, `pos_y_m`, `pos_z_m` | m | PositionX/Y/Z | keep the game's world frame, store raw | Dash only |
| `susp_travel_m_{fl,fr,rl,rr}` | m | SuspensionTravelMeters* | | |
| `susp_travel_norm_{..}` | 0..1 | NormalizedSuspensionTravel* | | 1 = max compression |
| `wheel_omega_radps_{..}` | rad/s | WheelRotationSpeed* | | |
| `slip_ratio_norm_{..}` | – | TireSlipRatio* | | **normalized**, not κ |
| `slip_angle_norm_{..}` | – | TireSlipAngle* | | **normalized**, not α |
| `slip_combined_norm_{..}` | – | TireCombinedSlip* | | |
| `slip_ratio_est_{..}` (derived) | – | (ω·r_eff − v_x)/max(\|v_x\|, 1) | derived | uses calibrated r_eff |
| `on_rumble_{..}`, `puddle_depth_{..}`, `surface_rumble_{..}` | – | same | | surface context |
| `tire_temp_raw_{..}` | game unit | TireTemp* | **store raw**. Add `tire_temp_c_{..}` only after unit calibration | Dash only |
| `tire_wear_{..}` | 0..1 | TireWear* | | FM2023 only |
| `power_w`, `torque_nm`, `boost_raw`, `fuel_raw`, `distance_raw` | W, N·m, raw | Power, Torque, Boost, Fuel, DistanceTraveled | | |
| `throttle`, `brake`, `clutch`, `handbrake` | 0..1 | Accel, Brake, Clutch, HandBrake | /255 | |
| `steer` | −1..1 | Steer | /127 | sign TBD (left = +? per ISO) |
| `gear` | int | Gear | raw | encoding of R/N to verify |
| `lap`, `race_pos`, `lap_time_cur_s`, `lap_time_last_s`, `lap_time_best_s`, `race_time_s` | | LapNumber, RacePosition, CurrentLap, LastLap, BestLap, CurrentRaceTime | | |
| `driving_line_norm`, `ai_brake_diff_norm` | −1..1 | NormalizedDrivingLine, NormalizedAIBrakeDifference | /127 | |
| `horizon_extra_{0,1,2}` | raw | FH bytes 232–243 as 3 × s32 | | undocumented |

**File layout:** `captures/<date>_<game>_<car_ordinal>_<manoeuvre>_<rep>/{raw.fdo, decoded.csv, session.yaml}`. Comparisons with our sim use the same CSV columns, with the `src_game` value `openroad`.

---

## 8. Open questions / to verify in a browser

1. Recheck §3 against the full official FM2023 and FH6 pages (we only saw extracts).
2. Is there an official FH4/FH5 Data Out page? (None found.)
3. Units of TireTemp, Boost, Fuel, DistanceTraveled, and angles. Gear encoding.
4. Does `Acceleration*` include gravity?
5. Physics step versus send rate in each title.

---

## Sources

All accessed 2026-10-10 unless noted. "(extract)" means seen only as a search-engine extract, because the page was blocked from this environment.

- Forza Motorsport "Data Out" Documentation (official, Turn 10), https://support.forza.net/hc/en-us/articles/21742934024211-Forza-Motorsport-Data-Out-Documentation (extract). Older URL cited by parsers: https://support.forzamotorsport.net/hc/en-us/articles/21742934024211-Forza-Motorsport-Data-Out-Documentation
- Forza Horizon 6 "Data Out" Documentation (official), https://support.forza.net/hc/en-us/articles/51744149102611-Forza-Horizon-6-Data-Out-Documentation (extract)
- Forum: "Forza Motorsport 7 'Data Out' feature details", https://forums.forza.net/t/forza-motorsport-7-data-out-feature-details/74013 (extract)
- Forum: "Data Out feature in Forza Motorsport", https://forums.forza.net/t/data-out-feature-in-forza-motorsport/651333 (extract, includes the SimHub undocumented-fields remark)
- Forum: "UDP telemetry packet details", https://forums.forza.net/t/udp-telemetry-packet-details/629111 (extract, 331 vs 311 bytes)
- Forum: "Data Out Telemetry Variables and Structure" (FH5), https://forums.forza.net/t/data-out-telemetry-variables-and-structure/535984 (extract)
- richstokes/Forza-data-tools, `FM7_packetformat.dat` (raw file read) https://github.com/richstokes/Forza-data-tools/blob/master/FM7_packetformat.dat. LICENSE (GPL-3.0) https://github.com/richstokes/Forza-data-tools/blob/master/LICENSE
- geeooff/forza-data-web, README and `Core/ForzaDataReader.cs`, `Core/ForzaHorizonExtrasDataStruct.cs`, `Core/ForzaMotorsportExtrasDataStruct.cs` (raw files read) https://github.com/geeooff/forza-data-web. LICENSE (Apache-2.0) https://github.com/geeooff/forza-data-web/blob/main/LICENSE
- nettrom/forza_motorsport, `fdp.py`, README (raw files read) https://github.com/nettrom/forza_motorsport. LICENSE (MIT) https://github.com/nettrom/forza_motorsport/blob/master/LICENSE
- austinbaccus/forza-telemetry, README, LICENSE (MIT) https://github.com/austinbaccus/forza-telemetry/blob/main/LICENSE
- heat-cycle/heat-cycle, README, LICENSE (MIT) https://github.com/heat-cycle/heat-cycle/blob/main/LICENSE. Issues #21/#29 https://github.com/heat-cycle/heat-cycle/issues/29 (extract)
- 0x20F/forza-telemetry https://github.com/0x20F/forza-telemetry (no LICENSE file found at main/master)
- theRTB/ForzaGUI https://github.com/theRTB/ForzaGUI (no LICENSE file found at main/master)
- Microsoft/Xbox Game Content Usage Rules, https://www.xbox.com/en-US/developers/rules (extract; page reported as dated 2015)
- Tubefilter, "Microsoft Lets Content Creators Monetize Its Games On YouTube", 2015-01-12, https://www.tubefilter.com/2015/01/12/microsoft-content-creators-monetize/ (extract)
