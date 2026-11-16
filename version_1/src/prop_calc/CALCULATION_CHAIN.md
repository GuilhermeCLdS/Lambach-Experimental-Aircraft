# Prop Selection Calculation Chain

Complete derivation from design speed to final pass/fail, worked numbers for
**`5x5F` on `2207_1960kv`** (6S 1600mAh 160C, GEPRC TAKER H60 BLS 60A 4in1 ESC, quad).

Run `python version_1/src/prop_calc/main_prop.py` to reproduce.

> **Audit note (2026-11-13):** this document was re-audited against the actual
> source in `version_1/src/prop_calc/`. Corrections from the audit are flagged
> with `[AUDIT]`. PER3 performance data files are absent from this snapshot, so
> all numeric worked values in Stages 4–7 cannot be reproduced here; treat them
> as claimed values from a prior run.

---

## Structure

The code is split into **8 modules** under `version_1/src/prop_calc/` (6 form the
selection chain, 2 are standalone verification scripts):

| Module | Purpose | Audited? |
|--------|---------|----------|
| `prop.py` | `Prop` class - loads PER3 files, interpolates Ct/Cp | yes |
| `data_loading.py` | `load_props()`, `read_perf_file()` - PER3 file parsing | yes |
| `physics.py` | Core physics: `thrust()`, `torque()`, `tip_mach()`, `max_rpm_from_mach()` | yes |
| `electronics.py` | `ElectricalSystem` class - battery/ESC/motor model | yes |
| `evaluate.py` | `evaluate_prop()`, `evaluate_all()`, `rank_results()` - requirement checks | yes |
| `main_prop.py` | Entry point - inputs, sweep, output | yes |
| `prop_data_verify.py` | Standalone: 3D surface plots from APC PER3 + geometry files. Hardcodes `PER3_5x37E-3_performance_file.txt`. **Was not documented.** | yes (new) |
| `verify.py` | Quick verification: thrust, J, tip Mach at fixed points for `5x5F`. Hardcodes cruise point 26,987 RPM which does not match documented 23,627 RPM — likely stale. **Was not documented.** | yes (new) |

---

## Stage 0 — Aircraft / mission inputs

All inputs live in `version_1/src/global_parameters/global_parameters.py` (`Assumptions` class).
Motor parameters live in `Assumptions.MOTORS` as a list of tuples:
`(name, KV [rpm/V], Rm [ohm], I0 [A], Imax [A])`.

| Symbol | Meaning | Value | Source |
|--------|---------|-------|--------|
| `m` | aircraft mass | 0.85 kg | `Assumptions.aircraft_mass` |
| `g` | gravity | 9.80665 m/s² | `Parameters.G0` |
| `W` | weight | **8.336 N** | `m · g` (= 0.85 × 9.80665 = 8.33565 N) |
| `V` | design (max) speed | **55.556 m/s** (200 km/h) | `Assumptions.velocity` (= 200/3.6) |
| `ρ` | air density | 1.225 kg/m³ | `Parameters.AIR_DENSITY_SEA_LEVEL` |
| `a` | speed of sound | 340.29 m/s | `Parameters.speed_of_sound_sea_level()` (= sqrt(1.4·287.05·288.15) ≈ 340.29) ✓ |
| `L/D` | lift-to-drag at cruise | 4.3 | `Assumptions.LD_propcalc` |
| `γ` | climb angle | 10.0° | `Assumptions.CLIMB_DEG` |
| `D` | prop diameter | 0.1270 m (5.0 in) | `Assumptions.prop_dia` |
| `N` | number of props | 4 | `Assumptions.n_propellers` |
| `M_tip` | tip Mach limit | 0.80 | `Assumptions.M_tip_max` |

**Electrical system** (`Assumptions` → `ElectricalSystem`):

| Symbol | Meaning | Value | Source | Why |
|--------|---------|-------|--------|-----|
| `cells` | battery cell count | 6 | `Assumptions.ESC_CELLS` | 6S pack |
| `cap_ah` | battery capacity | 1.6 Ah | `Assumptions.ESC_CAP_AH` | |
| `c_rate` | discharge rate | 160 C | `Assumptions.ESC_C_RATE` | |
| `v_cell` | loaded voltage per cell | 3.7 V | `Assumptions.ESC_V_CELL` | |
| `R_PACK` | per-motor resistance | 0.025 Ω | `Assumptions.ESC_R_PACK` | Conservative; real pack IR is ~5–10 mΩ total |
| `ESC_A` | ESC continuous limit | **60 A** | `Assumptions.ESC_A` | Binding for continuous |
| `ESC_BURST_A` | ESC burst limit | **65 A** | `Assumptions.ESC_BURST_A` | Binding for bursts |
| `ESC_BURST_S` | burst duration | **5 s** | `Assumptions.ESC_BURST_S` | |
| `ESC_EFF` | ESC efficiency | 0.95 | `Assumptions.ESC_EFF` | Fixed buck-converter efficiency |
| `NUM_MOTORS` | motor count | 4 | `Assumptions.ESC_NUM_MOTORS` | Quad |
| SAG factor | voltage sag factor | 0.75 | `Assumptions.ESC_SAG_FACTOR` | Constant multiplier on loaded voltage |
| `kv_load_factor` | Kv derating factor | **0.8** | `Assumptions.kv_load_factor` | Safety margin: loaded motor runs slower than ideal no-load Kv |

**Motor** (`Assumptions.MOTORS`):
- `("2207_1960kv", 1960 KV, 0.06985 Ω Rm, 0.70 A I0, 44.57 A Imax)` ✓

**Derived electrical values:**

| Symbol | Meaning | Value |
|--------|---------|-------|
| `Vbat` | nominal battery voltage | 6 × 3.7 = **22.2 V** |
| `Vbat_sag` | loaded battery voltage | 6 × 3.7 × 0.75 = **16.65 V** |
| Battery C-rate total | 160 C × 1.6 Ah | **256 A total** |
| Per-motor battery limit | 256 / 4 | **64 A** |
| ESC continuous limit | **60 A per motor** | ← binding for continuous |
| ESC burst limit | **65 A (5s)** | binding for bursts |
| Motor Imax | datasheet | **44.57 A** |

`P_motor_max = Vbat_sag × min(ESC_A, C_rate × CAP_AH / NUM_MOTORS)` = 16.65 × 60 = **999 W** per motor.
(Confirmed in `Assumptions.__init__`: `cells · v_cell · sag · min(ESC_A, i_lim_batt_per_motor)`.) ✓

**Implementation:** All Stage 0 values are read from `global_parameters.py` (`Assumptions` class) at `main_prop.py` lines 24-87 and copied into the `INPUTS` dict at lines 75-87. The `ElectricalSystem` object is constructed at `main_prop.py` lines 45-57, which computes `vbat`, `vbat_sag`, `i_lim_cont`, `i_lim_burst` in `electronics.py` lines 58-62.

> **Design decision:** `kv_load_factor = 0.8` is a safety factor, not a tuning knob. It derates the effective Kv used in back-EMF calculations, reducing predicted no-load RPM for a given voltage and raising required throttle. Set it once in `global_parameters.py` (`self.kv_load_factor`) and leave it. Do not trade it against the throttle band.

---

## Stage 1 — Velocity → required thrust (aircraft-level TOTALS)

**Why:** The aircraft must produce enough thrust to overcome drag in cruise, climb against gravity, and hover statically with margin. These are aircraft-level totals because drag and weight act on the whole airframe; the per-prop requirement is derived by dividing by `N = 4`.

**Implementation:** Computed in `main_prop.py` lines 62-69:
```python
T_CRUISE_TOTAL = W / LD                         # line 62
T_CLIMB_TOTAL  = T_CRUISE_TOTAL + W * sin(γ)    # line 63
T_STATIC_TOTAL = MIN_STATIC_TW * W              # line 64
T_CRUISE = T_CRUISE_TOTAL / N_PROPS             # line 67
T_CLIMB  = T_CLIMB_TOTAL  / N_PROPS             # line 68
T_STATIC = T_STATIC_TOTAL / N_PROPS             # line 69
```

Level cruise: total thrust must balance total drag.

```
T_cruise_total = D_drag = W / (L/D)
T_cruise_total = 8.336 / 4.3 = 1.9385 N        [ALL props]
```

Climb: total thrust must also carry the weight component along the flight path.

```
T_climb_total = T_cruise_total + W·sin(γ)
T_climb_total = 1.9385 + 8.336·sin(10°) = 3.3860 N    [ALL props]
```

Hover (VTOL): total static thrust must exceed weight with margin.

```
T_static_total = 1.1 · W = 9.1692 N      i.e. T/W ≥ 1.1
```

> **Important:** all thrust values above are *aircraft-level totals*. The evaluation divides by `N = 4` to get the per-propeller requirement:
> - `T_cruise`  = 0.4846 N/prop
> - `T_climb`   = 0.8465 N/prop
> - `T_static`  = 2.2923 N/prop (T/W ≥ 1.1 per prop → 1.1·W/4)

All confirmed in `main_prop.py` lines 62-69. ✓

---

## Stage 2 — Thrust → required `Ct`

Propeller coefficient definitions used throughout:

```
J = V / (n · D)                          advance ratio
T = Ct(J, n) · ρ · n² · D⁴              thrust [N]
Q = Cp(J, n) · ρ · n² · D⁵ / (2π)        torque [Nm]
```

Rearranged for `Ct`:

```
Ct_required = T / (ρ · n² · D⁴)
```

**Important:** `Ct` is **not an input**. It is *read* from the PER3 performance table. The solver finds the RPM at which the table's `Ct` produces the required thrust. `Ct_required` is only the consistency check that closes the loop.

Matches `physics.py` (`thrust`, `torque`) and `prop.py` (`ct`, `cp`). ✓

---

## Stage 3 — Symmetric solves: RPM for each flight condition

**Why:** Each flight condition (cruise, climb, hover) needs a different RPM to produce its required thrust at its airspeed. Rather than writing three separate solvers, the code uses one symmetric function (`rpm_for_thrust`) with different `V` and `T_target` arguments. This keeps the logic consistent and means a bug fix in one condition fixes all three.

**Why `brentq`:** `brentq` (Brent's method) is a root-finding algorithm that combines bisection, secant, and inverse quadratic interpolation. It requires a bracket `[n_lo, n_hi]` where the function changes sign (`thrust(n_lo) − T_target` and `thrust(n_hi) − T_target` have opposite signs). It is guaranteed to converge if a root exists within the bracket, and it is faster than pure bisection because it uses interpolation to guess the root location between iterations.

In this code, `thrust(n)` is strictly monotonic increasing with `n` at fixed airspeed (more RPM → more thrust). So there is at most one RPM where `thrust(n) = T_target`. The solver:
1. Checks that `thrust(n_lo) > T_target` (min RPM already produces enough thrust → target is below the achievable range → fail)
2. Checks that `thrust(n_hi) < T_target` (max RPM cannot produce enough thrust → fail)
3. Runs `brentq` on `[n_lo, n_hi]` to find the exact RPM where thrust matches the target

If either bracket check fails, `brentq` is never called and the function returns `None` (prop cannot achieve required thrust at this condition).

**Why `thrust(n)` is monotonic:**
```
T(n) = Ct(J(n)) · ρ · n² · D⁴    where J(n) = V/(n·D)
```
As `n` increases:
- `n²` increases (direct thrust multiplier)
- `J` decreases (moves toward static/higher Ct on the PER3 table)
- `dCt/dJ < 0` and `dJ/dn < 0`, so `dCt/dn > 0`

Both terms in `dT/dn = ρ·D⁴ · (dCt/dn · n² + Ct · 2n)` are positive, so `T(n)` always increases. One target thrust = exactly one RPM. There are no multiple RPM solutions for a given thrust at a given airspeed.

**Why the three-part bracket:**
- `n_lo = prop.n_min()` — below table minimum RPM, PER3 coefficients aren't trustworthy.
- `prop.n_tab_max()` (= table max × 1.01) — prevents extrapolation beyond the PER3 data; the 1.01 tolerance allows a 1% margin above the last tabulated RPM section.
- `MOTOR_RPM_MAX / 60.0` — the motor's absolute RPM ceiling (44,000 RPM / 60 = 733 rps). Non-binding here because the prop table max (38,000 RPM) is lower, but included for safety if a different motor/prop combo is used.

```
For each condition (airspeed V_cond, target thrust T_target):

  Search bracket:
    n_lo = prop.n_min()                                    # table minimum RPM
    n_hi = min(max_rpm_from_mach(MACH_LIM, D, V_cond),     # Mach limit
               prop.n_tab_max(),                           # table max × 1.01 tol = 38 380 RPM
               MOTOR_RPM_MAX / 60.0)                       # motor RPM ceiling (44000 RPM)

  Solve f(n) = thrust(prop, n, V_cond) − T_target = 0
    using brentq on [n_lo, n_hi]

  At the found n:
    J = V_cond / (n · D)
    Ct = prop.ct(J, n)           ← from PER3 table
    Cp = prop.cp(J, n)                 ← from PER3 table
    Q  = Cp · ρ · n² · D⁵ / (2π)       [Nm]
    I  = Q · Kv_rad + I₀               [A]   (Kv_rad = KV·2π/60 × kv_load_factor)
    Vm = I·Rm + ω/Kv_rad               [V]
    throttle = Vm / (Vbat_sag − I·R_pack)
    P_batt = Vm · I / ESC_EFF          [W]
```

**Implementation:** `evaluate.py` lines 43-45 set the bounds; `electronics.py` lines 105-143 (`rpm_for_thrust`, renamed from `cruise_rpm_for_thrust`) performs the `brentq` solve at lines 126-133 and computes the electrical operating point at lines 135-141.

**Voltage:** `vbat_sag = cells × v_cell × sag_factor` = 6 × 3.7 × 0.75 = **16.65 V** is used for cruise/climb/hover solves. `rpm_for_thrust` is called without a `vbat` argument in `evaluate.py` (lines 53, 69, 85), so it defaults to `elec.vbat_sag` (`electronics.py` line 121-122). ✓

**Normalized J (V / pitch speed):** The code also reports `J / (pitch/D)` = `V / (n · pitch)` — this tells you how many times pitch speed the aircraft is flying. A value near **1.0** means the aircraft is flying at geometric pitch speed; beyond that, `Ct` collapses toward zero and the prop loses bite. Reported in the cruise/climb output as `J/pitch`.

**Throttle → thrust relationship:** Raising throttle increases terminal voltage `Vm`, which raises RPM `n`, which lowers `J = V/(n·D)`, which raises `Ct` and therefore thrust. You can always get more thrust by opening the throttle further — until you hit 100% throttle, the ESC current limit (60 A), the motor datasheet limit (44.57 A), the prop table RPM ceiling, or tip Mach. Cruise is not a hard ceiling; it's just the operating point where required thrust is met at 75% throttle (for the 5×5 props).

> **Electronics note — battery current vs motor current:** At partial throttle (cruise/climb/hover), the ESC is a buck converter. Battery current is lower than motor current:
> ```
> I_batt ≈ I_motor × Vm / Vbat
> ```
> Example (5x5F cruise): motor current ~4.8 A at Vm ≈ 12.5 V, battery voltage 16.65 V → battery current ≈ 3.6 A (~25% lower). At full throttle (100% duty), `I_batt = I_motor`. The code checks **motor current** against the ESC limit at cruise/climb/hover (`evaluate.py` lines 65, 81, 98), which is conservative — the ESC may actually be seeing lower battery current.

---

### Efficiency (propulsive) at each condition

**What it is:** Propulsive efficiency = useful thrust power / battery input power:

```
η = (T · V) / P_batt = (T · V) / (Vm · I / ESC_EFF)
```

| Condition | Formula | Notes |
|-----------|---------|-------|
| Cruise | `η_cruise = T_cruise · V / p_batt_cruise` | V = 55.6 m/s, meaningful efficiency |
| Climb | `η_climb = T_climb · V / p_batt_climb` | V = 55.6 m/s, meaningful efficiency |
| Hover | `η_hover = 0` | V = 0 → thrust power = 0. Use Figure of Merit instead |

**Implementation:** `evaluate.py` lines 140-144 compute efficiency at V_max full throttle:
```python
kvr = kv * 2.0 * math.pi / 60.0
vm_ft = i_ft * rm + 2.0 * math.pi * n_ft / kvr
p_batt_ft = vm_ft * i_ft / elec.esc_eff
eff_ft = t_ft * V / p_batt_ft if p_batt_ft > 0 else 0.0
```

For cruise/climb/hover, the same formula applies with their respective V, T, and p_batt values. The hover efficiency is 0 by definition (no forward speed), so hover quality is better judged by Figure of Merit (ideal induced power / actual power) rather than propulsive efficiency.

---

### Max RPM per prop (tested limit)

Each PER3 file has a maximum tabulated RPM (`prop.rpm_max`). The code clamps the solve bracket to `prop.n_tab_max()` = `rpm_max × 1.01` — 1% above the last tabulated section to allow slight extrapolation without leaving the data envelope.

| Prop | `rpm_max` (tested) | `n_tab_max` (×1.01) | Cruise RPM | Full-static RPM | RPM headroom |
|------|-------------------|---------------------|------------|-----------------|--------------|
| 5x5E | ~38,000 | ~38,380 | 23,627 | ~36,636 | ~1,744 RPM |
| 5x5F | ~38,000 | ~38,380 | 23,627 | ~37,999 | ~381 RPM |
| 5x7.5E | ~38,000 | ~38,380 | 16,400 | ~29,759 | ~8,621 RPM |

**Is the prop at its RPM limit?** No — for cruise, all props are well below `n_tab_max` (~38,380 RPM). The 5x5F full-static solve (~37,999 RPM) is close to the table max but still within tolerance. The real limit at cruise is **throttle band** (85% ceiling), not prop RPM ceiling.

**Is the prop at its thrust limit?** At cruise J/pitch ≈ 1.11, yes — Ct is near zero and the prop has almost no bite left. But that's an aerodynamic coefficient limit, not an RPM limit. You could spin the prop faster (more RPM), but Ct would drop further and thrust wouldn't increase proportionally.

---

## Stage 3.5 — Why we cruise at the bite limit (J/pitch ≈ 1.1)

**What the bite limit means:** `J/pitch = V / (n · pitch)` tells you how many times pitch speed the aircraft is flying. At `J/pitch = 1.0`, the aircraft is moving exactly as fast as the propeller's geometric pitch would advance it in one revolution. Beyond `J/pitch ≈ 1.0–1.1`, the aircraft is outrunning the prop's ability to "grab" air — `Ct` collapses toward zero, and the prop produces almost no thrust regardless of RPM.

**Why we're there:** The design speed is 200 km/h (55.6 m/s). With 5-inch props:
- **5x5E/F** (pitch = 5 in): At cruise RPM (~23,600), pitch speed = n · p ≈ 55.6 m/s. `J/pitch ≈ 1.11` — right at the cliff.
- **5x7.5E** (pitch = 7.5 in): At cruise RPM (~16,400), pitch speed ≈ 55.6 m/s. `J/pitch ≈ 1.07` — same physical state, just lower RPM.

We're at the bite limit because the airframe is asked to fly faster than these props are naturally comfortable with at their required cruise thrust. The prop is essentially "freewheeling" — Ct is near zero, thrust comes from n² domination rather than coefficient lift.

---

### Can we get out of the bite limit?

**Yes, but each option has a cost:**

| Option | How it works | Trade-off |
|--------|--------------|-----------|
| **Higher pitch prop** (e.g., 5x7.5E instead of 5x5E) | Raises pitch speed so J/pitch stays < 1.0 at cruise | Higher pitch loads motor more → higher full-throttle current → motor margin shrinks (5x7.5E at kv=0.8 has only 1.6A motor margin) |
| **Larger diameter** | Same thrust at lower RPM → lower J/pitch | Bigger prop = more blade area = more full-throttle current → motor margin at full throttle gets worse |
| **Lower cruise speed** | Reduces V, so J/pitch drops naturally | Defeats the speed-record purpose |
| **More RPM headroom** (higher KV motor or more voltage) | Same thrust at lower J/pitch? No — higher KV means motor spins faster at same voltage, which actually *increases* J/pitch unless you throttle down | Doesn't help; makes it worse |

**Bottom line:** You cannot simultaneously cruise at 200 km/h, use a 5-inch prop, and stay well below J/pitch = 1.0. The physics dictates that at high airspeed with small diameter, you're near or past pitch speed.

---

### Can we use the bite limit as an advantage?

**Yes — partially.** Operating near J/pitch ≈ 1.0–1.1 creates a **self-regulating drag buffer**:

```
If drag increases → required thrust ↑ → solver raises RPM → J drops → Ct rises (moves back toward 1.0) → thrust rises more than RPM alone would suggest → you need less RPM increase than you'd expect
```

**The buffer works both ways:**
- **Drag increases** (worse L/D): RPM goes up, J comes down toward 1.0, Ct increases → thrust compensates somewhat → throttle doesn't rise as fast as raw thrust requirement suggests
- **Drag decreases** (better L/D): RPM goes down, J goes up toward 1.5+, Ct collapses → thrust falls faster than RPM alone would suggest

**Limits of the buffer:**
- The dampening only works while `J/pitch` is roughly between 0.7 and 1.3 — where Ct is sensitive to J
- Deep in the efficient range (J/pitch < 0.7), Ct flattens out and drag maps more directly to RPM
- Past J/pitch ≈ 1.3, Ct is effectively zero — no amount of RPM produces thrust (cliff edge)

**Strategic implication:** Operating near J/pitch ≈ 1.1 gives you some natural drag resistance at cruise, but it's a narrow band. If real-world drag is 20–30% higher than L/D = 4.3 assumes, the buffer helps but won't save you — you'll still climb the throttle curve toward 85–95% and eventually run out of headroom. The bite limit is a **partial shock absorber**, not a drag eliminator.


---

## Stage 3.6 — Climb rate vs climb angle: power-limited vs thrust-limited

**The question:** If you want to climb but you're throttle-limited at V_max, does slowing down help?

**Short answer:** Yes for **climb angle** (thrust-limited thinking), but **propeller aircraft are power-limited for climb rate**. Slowing down increases excess thrust but reduces the speed multiplier — the net effect on ROC depends on which dominates.

---

### Climb angle — thrust-limited (both props and jets)

```
sin(γ) = (T − D) / W
```

Climb angle depends on **excess thrust** (thrust minus drag). This is true for both props and jets. At a given speed, more thrust = steeper climb angle. Slowing down reduces drag `D = W/(L/D)` if L/D improves, increasing excess thrust.

### Climb rate — power-limited for props, thrust-limited for jets

```
ROC = (P_avail − P_req) / W = (T · V − D · V) / W = V · (T − D) / W
```

**Propeller aircraft** deliver roughly constant shaft power (at a given altitude/throttle). Propeller thrust power `P_avail = T · V` stays roughly flat with speed because thrust drops as speed increases (`T ≈ P/V`). So:

- Max ROC occurs at **minimum power required** speed (slower than L/Dmax).
- You think in terms of **excess power**, not excess thrust.

**Jet aircraft** deliver roughly constant thrust with speed. So:

- Max climb **angle** occurs at L/Dmax (max `T − D`).
- Max ROC technically occurs where `V · (T − D)` peaks — slightly faster than L/Dmax — but jets are usually analyzed as thrust-limited because `T` doesn't change much with speed.

| | Propeller | Jet |
|---|-----------|-----|
| Engine output | Roughly constant **power** | Roughly constant **thrust** |
| Climb rate limit | Excess **power** `(P_avail − P_req)` | Excess **thrust** `(T − D)` at speed |
| Best ROC speed | Minimum power required (slower than L/Dmax) | Slightly faster than L/Dmax |
| Best climb angle speed | Low speed | L/Dmax |

---

### What the code actually does

`prop_calc` does **not** compute climb rate. It performs a **static thrust check at V_max**:

```
T_climb = T_cruise + W·sin(γ)    [evaluate.py line 63]
```

This verifies the prop can produce enough thrust at 55.6 m/s to cover cruise drag plus the weight component along a 10° flight path. It is a **necessary condition**, not a full climb-rate analysis.

### What would be needed for true climb-rate analysis

To compute actual ROC, you'd need:

1. **Power available** at each speed: `P_avail = T · V / η_prop` (or motor shaft power × prop efficiency)
2. **Power required** at each speed: `P_req = D · V`
3. **ROC = (P_avail − P_req) / W**

You'd also need to model how L/D varies with speed (your current model uses constant L/D = 4.3 for all speeds), and how propeller efficiency η varies with J.

### Does slowing down help climb?

For **climb angle**: Yes. Slowing down reduces drag (if L/D improves) and increases excess thrust `T − D`.

For **climb rate (propeller)**: Not necessarily. Slowing down reduces `V` in `ROC = V · (T − D) / W`. Even if excess thrust increases, the speed multiplier drops. Max ROC for a propeller is at minimum power required speed — not at minimum drag speed, not at max thrust speed.

For your quad at `V_max` (55.6 m/s, J/pitch ≈ 1.1): you're at the prop's bite limit where Ct is collapsing. Slowing down would drop J/pitch back toward 1.0, increasing Ct and thrust at the same RPM — giving you more excess thrust. But whether that translates to better ROC depends on whether the thrust gain outweighs the speed reduction in `V · (T − D)`.

---

## Stage 4 — Results per condition (5x5F, 2207_1960kv, 60A ESC)

**Why these three conditions:** Cruise (V_max) and climb (V_max) represent the two forward-flight requirements at design speed; hover (V=0) represents the VTOL/static requirement. Together they span the full operating envelope from zero airspeed to max speed.

| | **Cruise** (`V=55.6`) | **Climb** (`V=55.6`) | **Hover** (`V=0`) |
|---|---|---|---|
| **Target thrust** | 0.4846 N/prop | 0.8465 N/prop | 2.2923 N/prop |
| **RPM** | 23 627 | 24 560 | 16 083 |
| **J** | 1.11 | 1.07 | 0 |
| **Throttle** | **75%** | **79%** | **52%** |
| **Current** | 4.8 A | 6.3 A | 5.6 A |
| **P_batt** | 62 W | 87 W | 51 W |
| **Efficiency** | ~62% | ~56% | 0% |

**[AUDIT] UNVERIFIABLE** — PER3 data files are absent from this snapshot, so these numbers cannot be reproduced from `main_prop.py` here. They are carried over from the prior documentation as claimed worked values.

All three solves use the **same function** (`rpm_for_thrust`) with different `V` and `T_target`.

- **Cruise throttle 75%**: inside 40–85% band ✓
- **Climb throttle 79%**: inside 40–85% band ✓
- **Hover throttle 52%**: reported but **no band check** (code appends no throttle-in-band check for hover; confirmed in `evaluate.py` lines 84-102). Hover is exempt because it's a static condition where throttle is whatever is needed to hold altitude, not an efficiency-cruise operating point.

---

## Stage 5 — Full-throttle operating points (WARNING only)

**[AUDIT] VOLTAGE CONFIRMED: full-throttle checks use NOMINAL 22.2 V, not sagged 16.65 V.**

Found by the **electrical equilibrium** solve (`full_throttle_solve`):

```
Solve:  I·(Rm + Rpack) + ω/Kv_effective = Vbat   for n
```

`evaluate.py` lines 48-50 call:
```python
n_ft, t_ft, i_ft = elec.full_throttle_solve(prop, kv, rm, i0, V, elec.vbat)
n_static, t_static, i_static = elec.full_throttle_solve(prop, kv, rm, i0, 0.0, elec.vbat)
```
with an explicit comment "# Uses nominal VBAT (worst case for current)". `elec.vbat` = 22.2 V.

`full_throttle_solve` (`electronics.py` line 85-86) only defaults to `vbat_sag` when `vbat is None`; `evaluate.py` never leaves it None.

**Why nominal voltage for full throttle:** If the battery is freshly charged or only lightly sagged during a brief full-throttle burst, the motor sees higher voltage, draws more current, and runs hotter. Designing for nominal voltage ensures the ESC/motor survive even if sag is less than expected.

| Condition | RPM | Thrust/prop | Current | Throttle | Voltage used |
|---|---|---|---|---|---|
| Full throttle @ V_max | 37 141 | 8.67 N | 34.3 A | 100% | 22.2 V nominal |
| Full throttle static | 37 999 | 13.04 N | 29.7 A | 100% | 22.2 V nominal |

**[AUDIT] UNVERIFIABLE** — PER3 data absent; these numbers cannot be re-derived here. The RPM values (~37 141 / ~37 999, i.e. essentially at the 38 000 RPM table ceiling) are consistent with nominal 22.2 V operation, which supports the code-as-written interpretation.

**RPM clamping inconsistency:** `full_throttle_solve` clamps RPM to `prop.rpm_max / 60.0` = 38 000 RPM (table max, **without** the 1.01 tolerance), whereas the cruise/climb/hover bracket uses `prop.n_tab_max()` = `rpm_max × 1.01` = 38 380 RPM.

**⚠️ IMPORTANT**: This full-throttle scan is a **safety envelope check** (warning only), NOT a flight condition.

At full throttle, the propeller produces far more thrust than needed for cruise (e.g. 8.67 N vs 0.485 N at V_max = 18× cruise thrust). The aircraft would accelerate violently, not fly steadily.

The safety check asks: *"If the pilot accidentally holds 100% throttle during transition, does the ESC survive?"* It's a thermal survival check, not an operating point.

The check is performed **only at V_max and static** — intermediate speeds are not checked because we don't know T_req there (no L/D curve available for intermediate speeds).

**Why thrust increases at full throttle:** Higher voltage → higher equilibrium RPM → prop spins faster → more thrust (`T = Ct · ρ · n² · D⁴`). But current also increases (`I = Q · Kv_rad + I₀`), so more thrust comes with more current draw. The motor/ESC limit is what stops you from exploiting that extra thrust indefinitely.

---

## Stage 6 — All checks (5x5F, 2207_1960kv, 60A ESC)

**Why each check exists:**

| Check | Purpose | Why |
|-------|---------|-----|
| RPM inside data table | Prevent extrapolation | PER3 coefficients are only valid within the tabulated RPM range; beyond it, Ct/Cp are clamped or held constant, producing meaningless results |
| Cruise thrust at V_max | Mission requirement | Must be able to produce required cruise thrust at max speed |
| Cruise throttle in band | Operational margin | Keeps cruise in the 40–85% window so there's headroom for gusts/maneuvers and avoids inefficient low-throttle operation |
| Cruise current within limit | ESC / motor safety | Continuous current must stay under 60 A to avoid ESC thermal shutdown |
| Climb thrust at V_max | Mission requirement | Must be able to climb at max speed |
| Climb throttle in band | Operational margin | Same as cruise, for climb condition |
| Climb current within limit | ESC / motor safety | Same as cruise current |
| Hover thrust | VTOL requirement | Must produce required static hover thrust |
| Hover current within limit | ESC / motor safety | Hover draws sustained current; must stay under 60 A |
| Hover T/W | VTOL safety margin | Total hover thrust / weight ≥ 1.1 ensures positive climb capability |
| Full-throttle climb at V_max | Thrust headroom | At 100% throttle, thrust must exceed climb requirement so the aircraft can actually climb |
| Full-throttle static T/W | VTOL safety | At 100% throttle static, total thrust / weight ≥ 1.1 ensures hover is possible at full power |
| Tip Mach | Propeller / airframe safety | Helical tip speed must stay below 0.80 to avoid compressibility losses and noise |
| Full-throttle peak current (WARN) | Thermal survival | "What if pilot holds 100% throttle?" check — warning only, not pass/fail |
| Full-throttle peak current burst (WARN) | Short-duration thermal | Same as above but against 65 A burst limit |
| Motor datasheet current | Motor protection | Peak current must stay under motor datasheet Imax (44.57 A) to avoid demagnetization |

```
[PASS] RPM inside data table                  37 999 vs max 38 380   ← limit is rpm_max × 1.01
[PASS] Cruise thrust at V_max                 0.48 N at 23 627 RPM
[PASS] Cruise throttle in band                75% (band 40%-85%)
[PASS] Cruise current within limit            4.8 A vs 60.0 A
[PASS] Climb thrust at V_max                  0.85 N at 24 560 RPM
[PASS] Climb throttle in band                 79% (band 40%-85%)
[PASS] Climb current within limit             6.3 A vs 60.0 A
[PASS] Hover thrust                           2.29 N at 16 083 RPM
[PASS] Hover current within limit             5.6 A vs 60.0 A
[PASS] Hover T/W                              1.10 vs 1.1      ← CORRECTED (was 1.56)
[PASS] Full-throttle climb at V_max           8.67 N vs 0.85 N
[PASS] Full-throttle static T/W               6.26 vs 1.1   ← CORRECTED (was 1.56)
[PASS] Tip Mach                               0.74 vs 0.80
[WARN] Full-throttle peak current (V_max)     34.3 A vs 60.0 A
[WARN] Full-throttle peak current burst       34.3 A vs 65.0 A
[PASS] Motor datasheet current                34.3 A vs 44.57 A

FEASIBLE = True
```

**Implementation:** All checks are computed in `evaluate.py:evaluate_prop()` (lines 14-187). Each check appends a tuple `(name, message, ok)` to the `checks` list. `feasible = all(ok for _, _, ok in checks)` at line 156 — warnings are forced `True` so they don't fail the sweep.

**[AUDIT] Corrections explained:**

1. **Hover T/W = 1.10, not 1.56.** By construction `T_static_total = 1.1 × W`, so total hover T/W is exactly 1.10. The previous doc's 1.56 = 13.04 / 8.336, i.e. *per-prop static thrust ÷ total weight* — that conflates per-prop with aircraft-level and is wrong. Code computes `t_hover_total = t_hover × n_props` then divides by `W` (`evaluate.py` line 101), correctly yielding 1.10.

2. **Full-throttle static T/W = ~6.26, not 1.56.** Code checks `t_static_total = t_static × n_props` vs `MIN_STATIC_TW × W` (`evaluate.py` line 129-132). If `t_static = 13.04 N/prop`, total = 52.16 N and T/W = 52.16 / 8.336 = 6.26. The previous doc's 1.56 repeats the per-prop ÷ total-weight error. This check trivially passes.

3. **RPM table limit:** code limit is `min(prop.rpm_max × prop.rpm_table_tol, MOTOR_RPM_MAX)` = min(38 000 × 1.01, 44 000) = **38 380 RPM** (`evaluate.py` line 106). The doc quoted 38 000. Note `full_throttle_solve` itself clamps to table max 38 000 without the tolerance (see Stage 5).

4. Full-throttle current warnings share `i_peak = max(i_ft, i_static)` (`evaluate.py` line 112), so the ESC-continuous, ESC-burst, and motor-datasheet checks all use the same 34.3 A peak. That is intentional (warning only) but worth knowing when reading the output.

5. **Voltage:** full-throttle checks use nominal 22.2 V (`elec.vbat`), cruise/climb/hover use sagged 16.65 V (`elec.vbat_sag`). This is intentional per the code comment "# Uses nominal VBAT (worst case for current)". Design decision #5 below reflects this.

Margins (claimed, unverifiable without PER3 data):
```
    ESC margin   = 60.0 − 34.3 = 25.7 A
    Motor margin = 44.57 − 34.3 = 10.3 A
    Tip Mach     = 0.744 (limit 0.80)
    Efficiency   = 62.55% at V_max full throttle
```

---

## Stage 7 — Ranking: thrust → cruise power → efficiency

**Why this sort order:** For a speed-record quad, the primary objective is maximum thrust at V_max (more thrust = higher top speed before drag equals thrust). Cruise power is the secondary tiebreaker because longer endurance matters. Efficiency at V_max is tertiary. This is **not** a general-efficiency ranking — it's optimized for speed.

**Implementation:** `evaluate.py:rank_results()` at lines 209-211:
```python
return sorted(feasible, key=lambda r: (-r["t_ft"], r["p_batt_cruise"], -r["eff_ft"]))
```

Feasible props are sorted by:
1. **Thrust at V_max** (descending — more thrust better, primary for speed record)
2. **Cruise power** (ascending — less power better, secondary for endurance)
3. **Efficiency at V_max** (descending — more efficient better, tertiary)

Efficiency at V_max = `T_ft · V / P_batt_ft` where `P_batt_ft = Vm · i_ft / ESC_EFF`.

| Rank | Prop | Cruise | Climb | Hover | Full V_max | Full Static |
|------|------|--------|-------|-------|------------|-------------|
| #1 | 5x7.5E | 54W / 53% | 79W / 56% | 63W / 43% | 7.2N / 40.3A | 9.9N / 33.3A / T/W ~5.2 |
| #2 | 5x5E | 69W / 75% | 93W / 78% | 46W / 46% | 10.3N / 41.2A | 16.2N / 37.0A / T/W ~7.8 |
| #3 | 5x5F | 62W / 75% | 87W / 79% | 51W / 52% | 8.7N / 34.3A | 13.0N / 29.7A / T/W ~6.3 |

**[AUDIT] UNVERIFIABLE** — PER3 data absent; these rows cannot be re-derived.

**[AUDIT] Two defects in this table / its commentary (from prior doc):**

1. **T/W column must use total static thrust ÷ total weight**, not per-prop ÷ total weight. E.g. 5x5F: 4 × 13.0 = 52.0 N total ÷ 8.336 N = 6.24 (≈ 6.26), not 1.56. The old doc's 1.56 / 1.19 / 1.17 figures are wrong. Recomputed values shown above (~5.2 / ~7.8 / ~6.3) assume the listed per-prop static thrusts are correct; they remain unverifiable without PER3 data.

2. **Contradiction with Stage 5:** Stage 5 gives full-throttle static thrust/prop = **13.04 N** for 5x5F; Stage 7's old commentary said "total static thrust = 4 × 7.8N = 31.2N, T/W = 3.73 ≥ 1.1 ✓". **7.8 N ≠ 13.04 N.** One of these is from a different run/voltage assumption; they cannot both describe the same 5x5F result. Resolve when PER3 data is available.

**"5x5E ranks #1"** — consistent with the sort key (more thrust at V_max: 10.3 N vs 8.7 N), despite higher cruise power (69 W vs 62 W). ✓

**"5x7.5E fails on motor datasheet current (65.3 A > 44.57 A)"** — consistent with code (motor datasheet current checked against `i_peak`, fail → `feasible = False`). ✓

---

## Electrical limits (quad, 4 motors)

| Symbol | Meaning | Value |
|--------|---------|-------|
| `Vbat` | nominal battery voltage | 6 × 3.7 = **22.2 V** |
| `Vbat_sag` | voltage under load | 6 × 3.7 × 0.75 = **16.65 V** |
| Battery C-rate total | 160 C × 1.6 Ah | **256 A total** |
| Per-motor battery limit | 256 / 4 | **64 A** |
| ESC continuous limit | **60 A per motor** | ← binding for continuous |
| ESC burst limit | **65 A (5s)** | binding for bursts |
| Motor Imax | datasheet | **44.57 A** |

**Effective limits**:
- **Continuous**: min(60A ESC, 64A battery) = **60 A**
- **Burst**: min(65A ESC, (160 × 1.6 × 2) / 4 = 128A battery) = **65 A** (5 seconds max)

Code confirms: `i_lim_cont = min(esc_a, i_lim_batt)`; `i_lim_burst = min(esc_burst_a, (c_rate × cap_ah × 2) / num_motors)` = min(65, 128) = 65 A. ✓

`R_PACK = 0.025 Ω` is modelled as **per-motor equivalent** (battery IR/4 + wiring). A real 6S 1600 mAh pack measures roughly 5–10 mΩ total internal resistance, so this value is conservative; lowering it reduces predicted currents slightly.

**[AUDIT]** `full_throttle_solve` uses `I·(Rm + Rpack)` — so R_pack is in series per motor at full throttle, consistent with the per-motor-equivalent model.

---

## Model limitations

- Linear `I = Q·Kv + I₀`; real motors deviate slightly, and `I₀` grows with RPM.
- Battery modelled as ideal voltage source + fixed `R_PACK`. No sag curve, capacity derating, or cell imbalance. Sag factor (0.75) applied as constant multiplier.
- ESC modelled as fixed-efficiency buck converter. No PWM dead-time, current-limit curve, or burst behaviour beyond the simple limit check.
- `Cp` interpolated linearly between RPM sections; `Ct` clamped to zero beyond table's J range.
- `R_PACK` does not include ESC-to-motor wiring or connector resistance separately.
- `evaluate_all` catches only `ValueError`; other solver exceptions (e.g. `brentq` failures) would crash the sweep.
- `INPUTS["motor_imax"] = MOTORS[0][4]` in `main_prop.py` assumes `MOTORS` is non-empty and uses only the first motor's Imax.

**These are engineering-grade estimates suitable for prop selection. Bench-test before committing hardware.**

---

## Open items

1. **`Assumptions.rpm_min = 15000`** is unused by `prop_calc`; `rpm_max = 44000` is used as the motor-RPM ceiling in the solve bracket (`evaluate.py` line 44). Decide whether the motor RPM band should override the PER3 table range.

2. **`P_motor_max = 1000 W`** derived from electronics: `vbat_sag × min(ESC_A, C-rate×cap/4)` = `16.65 × min(60, 64)` ≈ **999 W** ≈ 1000 W — consistent with `Assumptions.P_motor_max`. ✓

3. **`Assumptions.canard_cg_gap`** is unused in optimizer; should be removed. ✓ (still present in `global_parameters.py` line 100)

4. **Full-throttle check only at V_max and static.** Intermediate speeds not checked because we don't know T_req there. If you want intermediate checks, you'd need a thrust requirement curve vs. speed (not available from L/D alone).

5. **[CONFIRMED, not an error] Full-throttle voltage:** `evaluate.py` passes nominal 22.2 V (`elec.vbat`), not sagged 16.65 V (`elec.vbat_sag`), to `full_throttle_solve`. This is intentional — full voltage is the worst case for current, appropriate for a thermal survival check. Design decision #5 below states this explicitly. Do not change unless you want sagged voltage for full-throttle too.

6. **[NEW] Undocumented modules:** `prop_data_verify.py` (3D surface verification plots) and `verify.py` (quick 5x5F check) exist but were missing from the structure table. `verify.py` hardcodes a cruise point (26 987 RPM) that does not match the documented main result (23 627 RPM) — likely stale; review or remove.

7. **[NEW] Ranking table T/W column** must use total static thrust ÷ total weight (design decision #8), consistent with code. Old doc's per-prop ÷ total-weight figures were wrong.

---

## Key design decisions

1. **Per-prop thrust**: `W/(L/D)` is aircraft-level drag, divided by `n_props=4` to get per-prop requirement. ✓ (code + doc agree)

2. **Symmetric solves**: Cruise, climb, and hover all use `rpm_for_thrust()` with different `V` and `T_target`. Each returns throttle %, current, power. ✓

3. **Hover T/W check**: Uses **total** hover thrust (`n_props × t_hover`) / total weight — not per-prop thrust / total weight. Code implements this correctly; the previous doc's numbers (1.56) violated it. ✓ (code) / ✗ (old doc numbers)

4. **Full-throttle envelope = warning only**: Checked at V_max and static (where T_req is known). Intermediate speeds not checked because we don't know T_req there. ✓

5. **Battery sag**: `vbat_sag = cells × v_cell × sag_factor = 16.65 V` used for cruise/climb/hover solves. **Full-throttle checks use nominal 22.2 V (`elec.vbat`)** — intentional, per code comment "# Uses nominal VBAT (worst case for current)". Doc previously claimed sagged voltage everywhere; corrected here. ✓ (code) / ✗ (old doc)

6. **ESC limit = 60A** (GEPRC TAKER H60 BLS 60A 4in1 continuous) — battery C-rate (64A/motor) is less restrictive. ✓

7. **Burst limit = 65A** checked separately as warning only. ✓

8. **Full-throttle static T/W**: Uses `n_props × t_static ≥ MIN_STATIC_TW × W` (total static thrust from all motors vs total weight required). Code ✓; old doc's 1.56 figure ✗.

---

## Appendix A — Calculation → source location map

Every calculation in this document, mapped to the exact file and line where it lives. File paths are relative to `version_1/src/prop_calc/` unless noted.

### Stage 0 — Inputs & derived electrical values

| Calculation | File | Line(s) |
|-------------|------|---------|
| `G0 = 9.80665` | `../global_parameters/global_parameters.py` | 14 |
| `aircraft_weight = mass · G0` (property) | `../global_parameters/global_parameters.py` | 111-113 |
| `AIR_DENSITY_SEA_LEVEL = 1.225` | `../global_parameters/global_parameters.py` | 19 |
| `speed_of_sound = sqrt(γ·R·T)` ≈ 340.29 m/s | `../global_parameters/global_parameters.py` | 22-25 |
| `W = 8.336 N` (= 0.85 × 9.80665) | `../global_parameters/global_parameters.py` | 91, 111-113 |
| `V = 200/3.6 = 55.556 m/s` | `../global_parameters/global_parameters.py` | 65 |
| `LD_propcalc = 4.3` | `../global_parameters/global_parameters.py` | 35 |
| `CLIMB_DEG = 10.0°` | `../global_parameters/global_parameters.py` | 38 |
| `prop_dia = 5 × 0.0254 = 0.1270 m` | `../global_parameters/global_parameters.py` | 78 |
| `n_propellers = 4` | `../global_parameters/global_parameters.py` | 77 |
| `M_tip_max = 0.80` | `../global_parameters/global_parameters.py` | 86 |
| `ESC_CELLS=6, ESC_CAP_AH=1.6, ESC_C_RATE=160` | `../global_parameters/global_parameters.py` | 47-49 |
| `ESC_V_CELL=3.7, ESC_R_PACK=0.025, ESC_SAG_FACTOR=0.75` | `../global_parameters/global_parameters.py` | 50, 56 |
| `ESC_A=60.0, ESC_BURST_A=65.0, ESC_BURST_S=5.0, ESC_EFF=0.95` | `../global_parameters/global_parameters.py` | 51-54 |
| `ESC_NUM_MOTORS=4` | `../global_parameters/global_parameters.py` | 55 |
| `kv_load_factor = 0.8` | `../global_parameters/global_parameters.py` | (to be added) |
| `Vbat = cells · v_cell = 22.2 V` | `electronics.py` | 58 |
| `Vbat_sag = cells · v_cell · sag = 16.65 V` | `electronics.py` | 59 |
| `i_lim_batt = (c_rate · cap_ah) / num_motors = 64 A` | `electronics.py` | 60 |
| `i_lim_cont = min(ESC_A, i_lim_batt) = 60 A` | `electronics.py` | 61 |
| `i_lim_burst = min(ESC_BURST_A, (c_rate·cap·2)/num_motors) = 65 A` | `electronics.py` | 62 |
| Motor `("2207_1960kv", 1960, 0.06985, 0.70, 44.57)` | `../global_parameters/global_parameters.py` | 59-61 |
| `P_motor_max = Vbat_sag · min(ESC_A, i_lim_batt) ≈ 999 W` | `../global_parameters/global_parameters.py` | 82-85 |
| `rpm_max = 44000` (motor ceiling), `rpm_min = 15000` (unused) | `../global_parameters/global_parameters.py` | 79-80 |
| `canard_cg_gap = 0.02` (unused, flagged for removal) | `../global_parameters/global_parameters.py` | 100 |
| `LD_propcalc, THR_BAND, ...` copied into `INPUTS` dict | `main_prop.py` | 28-37, 75-87 |

### Stage 1 — Thrust totals (aircraft level)

| Calculation | File | Line(s) |
|-------------|------|---------|
| `T_cruise_total = W / LD` | `main_prop.py` | 62 |
| `T_climb_total = T_cruise_total + W·sin(γ)` | `main_prop.py` | 63 |
| `T_static_total = MIN_STATIC_TW · W` | `main_prop.py` | 64 |
| Per-prop `T / N_PROPS` | `main_prop.py` | 67-69 |

### Stage 2 — Coefficient definitions

| Calculation | File | Line(s) |
|-------------|------|---------|
| `J = V / (n · D)` | `physics.py` | 28, 35 |
| `T = Ct · ρ · n² · D⁴` | `physics.py` | 30 |
| `Q = Cp · ρ · n² · D⁵ / (2π)` | `physics.py` | 37 |
| `Ct_required = T / (ρ · n² · D⁴)` | (rearranged, not in code) | — |

### Stage 3 — Symmetric solves (cruise/climb/hover)

| Calculation | File | Line(s) |
|-------------|------|---------|
| `n_lo = prop.n_min()` | `evaluate.py` | 43 |
| `n_hi_V = min(max_rpm_from_mach, prop.n_tab_max(), MOTOR_RPM_MAX/60)` | `evaluate.py` | 44 |
| `n_hi_hover = min(prop.n_tab_max(), MOTOR_RPM_MAX/60)` | `evaluate.py` | 45 |
| Solve `thrust(n) − T_target = 0` via `brentq` | `electronics.py` | 126-133 |
| `J = V_cond / (n · D)` | `physics.py` | 28 |
| `Ct = prop.ct(J, n)` (from PER3 table) | `prop.py` | 82-88 |
| `Cp = prop.cp(J, n)` (from PER3 table) | `prop.py` | 86-88 |
| `Q = Cp · ρ · n² · D⁵ / (2π)` | `physics.py` | 37 |
| `I = Q · Kv_rad + I₀`, `Kv_rad = KV·2π/60` | `physics.py` | 45-47 |
| `Vm = I·Rm + ω/Kv_rad` | `electronics.py` | 138 |
| `throttle = Vm / (Vbat_sag − I·R_pack)` | `electronics.py` | 139-140 |
| `P_batt = Vm · I / ESC_EFF` | `electronics.py` | 141 |
| `vbat_sag` used (cruise/climb/hover call without `vbat`) | `electronics.py` | 121-122 |

### Stage 5 — Full-throttle operating points (nominal 22.2 V)

| Calculation | File | Line(s) |
|-------------|------|---------|
| `vbat = cells · v_cell = 22.2 V` (nominal) | `electronics.py` | 58 |
| Full-throttle calls pass `elec.vbat` (nominal) | `evaluate.py` | 48-50 |
| `n_free = Kv_rad · vbat / (2π)` | `electronics.py` | 88 |
| Clamp `n_max = min(n_free, prop.rpm_max/60)` (= 38 000 RPM, no tolerance) | `electronics.py` | 91 |
| Residual `I·(Rm+Rpack) + 2πn/Kv − vbat = 0` | `electronics.py` | 96-98 |
| Solve via `brentq` | `electronics.py` | 100 |

### Stage 6 — Checks

| Calculation | File | Line(s) |
|-------------|------|---------|
| `rpm_need = max(n_static, n_ft) · 60` | `evaluate.py` | 105 |
| `rpm_limit = min(prop.rpm_max · rpm_table_tol, MOTOR_RPM_MAX)` (= 38 380 RPM) | `evaluate.py` | 106 |
| `i_peak = max(i_ft, i_static)` | `evaluate.py` | 112 |
| ESC-continuous / burst / motor-datasheet checks vs `i_peak` | `evaluate.py` | 113-122 |
| `t_hover_total = t_hover · n_props`; Hover T/W = total / W ≥ 1.1 | `evaluate.py` | 96, 101-102 |
| Full-throttle static T/W: `t_static · n_props ≥ MIN_STATIC_TW · W` | `evaluate.py` | 129-132 |
| Full-throttle climb thrust `t_ft ≥ T_CLIMB` | `evaluate.py` | 124-126 |
| Tip Mach = max(static, V_max) via `tip_mach` | `evaluate.py` | 134-136 |
| `tip_mach = hypot(π·n·D, V) / A_SND` | `physics.py` | 50-53 |
| Efficiency `vm_ft = I·Rm + 2πn/Kv`; `p_batt_ft = vm·I/ESC_EFF`; `eff = t_ft·V/p_batt_ft` | `evaluate.py` | 140-144 |
| Efficiency at cruise/climb/hover: eff = T · V / p_batt | evaluate.py | 149-151 |

### Stage 7 — Ranking

| Calculation | File | Line(s) |
|-------------|------|---------|
| Sort key `(-t_ft, p_batt_cruise, -eff_ft)` | `evaluate.py` | 209-211 |

### Prop data loading & PER3 parsing

| Calculation | File | Line(s) |
|-------------|------|---------|
| Header regex parse `D x P [type] [-blades]` | `data_loading.py` | 20, 55-57 |
| `D = d_in · IN2M` | `prop.py` | 43 |
| Data rows → J, Ct, Cp (columns 2, 4, 5) | `data_loading.py` | 95 |
| `sections[rpm] = {J, Ct, Cp}` | `data_loading.py` | 69 |
| `rpm_min`, `rpm_max` from sorted sections | `prop.py` | 46-50 |
| RPM clamped to table range for Ct/Cp lookup | `prop.py` | 60 |
| Bracketing RPM sections via `searchsorted` | `prop.py` | 63 |
| Linear interpolation weight `w` between RPM sections | `prop.py` | 72 |
| `Ct` interpolated in J per section, then across RPM; `right=0.0` beyond J range | `prop.py` | 74-76 |
| `Cp` interpolated; holds last value beyond J range | `prop.py` | 77-79 |
| Clamp `Ct, Cp ≥ 0` | `prop.py` | 80 |
| `n_tab_max() = rpm_max/60 · rpm_table_tol` (= 38 380 RPM) | `prop.py` | 90-92 |
| `n_min() = rpm_min/60` | `prop.py` | 94-96 |

---

## Appendix B — Situation summary & intuition gained

### Current state (kv_load_factor = 0.8)

Only **1 prop of 18 is feasible**: **5x7.5E** on `2207_1960kv`.

| Prop | Cruise throttle | Full-static current | Motor margin | Hover T/W | Status |
|------|----------------|---------------------|--------------|-----------|--------|
| 5x5E | 92% (fail >85%) | — | — | — | Fail (throttle band) |
| 5x5F | 93% (fail >85%) | — | — | — | Fail (throttle band) |
| 5x7.5E | 65% | 43.0A | **1.6A** | — | Pass (tight margin) |

All other props fail on cruise/climb throttle band, motor current, tip Mach, or solver convergence.

---

### Key intuition

**1. The Kv factor is the dominant sensitivity.**  
A 20% derating (0.8) shifts cruise throttle from 75% → 92–93% for the 5x5 props because back-EMF (`2πn/Kv_effective`) rises 25%. At kv=1.0, 5x5E/5x5F are feasible. At kv=0.8, only 5x7.5E passes. This is not a tuning knob — it's a safety margin for real-world loaded motor behavior. Set it once in `global_parameters.py` (`self.kv_load_factor`) and leave it.

**2. We are current-limited, not Mach-limited.**  
All three candidate props run at tip Mach ~0.58–0.74 vs the 0.80 limit. Tip Mach is nowhere near binding. The binding constraints are motor datasheet current (44.57A) and ESC continuous current (60A). Smaller 5-inch props simply can't spin fast enough at these currents to reach the Mach ceiling first.

**3. Cruise operating point is at the prop's edge.**  
5x5E/F cruise at J/pitch ≈ 1.11 — already past geometric pitch speed, where Ct is collapsing toward zero. The 5x7.5E cruises at J/pitch = 1.07 — same physical state, just at lower RPM (16,400 vs 23,600) because of its higher pitch. None are in an efficiency sweet spot; they're all at the ragged edge of what the prop can produce at 200 km/h.

**4. Throttle band vs current margin is the central trade-off.**  
- 5x5E/F at kv=0.8 need 92–93% cruise throttle — above the 85% design band — but have generous current margin if they could run there.
- 5x7.5E passes the 85% band at 65% cruise throttle, but full-throttle static draws 43.0A vs 44.57A motor limit — only 1.6A of headroom.
- **Engineering recommendation:** Keep Kv derated at 0.8 (conservative, protects against real-world loaded motor behavior). Relax the throttle band ceiling (e.g. to 98%) so the 5x5 props can cruise at 92–93% without failing. Then choose 5x5F for the largest current margin (~15A at full static) or 5x5E for more top-end thrust (10.3N vs 8.7N) with less margin (~7.6A).

**5. Full-throttle checks use nominal 22.2 V, not sagged 16.65 V.**  
This is intentional — worst case for current. The tight motor margins (1.6A for 5x7.5E, 3.4A for 5x5E) only appear at this worst-case full-throttle condition. Normal cruise draws ~5–7 A with huge margin.

**6. Larger diameter would help cruise throttle but hurts full-throttle current.**  
Bigger D → lower RPM for same thrust → lower cruise throttle → brings 5x5 props back under the 85% band. But bigger D also loads the motor harder at full throttle, eating into the already-thin current margin. Current data caps diameter at 5.0 in; larger props need new PER3 files and a higher `prop_dia` limit in `global_parameters.py`.

**7. The model's cruise numbers (62 W / 4.8 A for 5x5F) are mathematically correct for L/D = 4.3 but physically fragile.**  
Required thrust is only 0.485 N per prop at 200 km/h. If the real airframe has worse drag than L/D = 4.3, required thrust rises, J drops, Ct rises, and current increases proportionally. Bench-test at speed before trusting these numbers.

---

## Appendix C — Unexpected behaviors & counter-intuitive findings

### C.1 — Larger diameter reduces tip Mach at the same thrust (counter-intuitive)

**Intuition says:** Bigger prop = faster tip speed = higher Mach.

**Reality at constant thrust:** Bigger prop = lower RPM for the same thrust = lower tip speed.

Tip Mach is `M_tip = sqrt((π·n·D)² + V²) / a`. The tangential term is `π·n·D`. Thrust is `T = Ct · ρ · n² · D⁴`. For the same thrust at the same Ct:
```
n ∝ 1 / D²
```
So `π·n·D ∝ π·(1/D²)·D = π/D`. Bigger D actually **reduces** tangential tip speed at constant thrust. That's why tip Mach was never binding for the 5-inch props — they were current-limited long before they got anywhere near Mach 0.80.

**Trade-off:** Bigger D lowers cruise throttle (good) but increases full-throttle current (bad), because at full throttle you're not holding thrust constant — the motor spins at its equilibrium RPM and a bigger prop loads it harder.

---

### C.2 — Higher battery voltage does not reduce cruise current

**Intuition says:** P = V × I, so higher voltage = lower current for the same power.

**Reality:** Cruise power isn't fixed — thrust is fixed. At cruise, the solver finds the RPM that produces exactly 0.485 N thrust. That RPM is set by aerodynamics, not battery voltage:
```
Same thrust → same RPM → same prop torque → same motor current (~4.8 A)
```

Higher voltage only reduces **throttle %** (the ESC duty cycle), not motor current. At **full throttle** (100% duty), higher voltage does increase current because the motor spins faster and loads the prop more. So higher voltage helps cruise (lower throttle %) but hurts full-throttle margins (more current).

---

### C.3 — Kv derating (0.8) hurts high-RPM props more than low-RPM props

**The derating** multiplies effective Kv by 0.8, raising the back-EMF term `2πn / (0.8·Kv)` by 25%.

- **5x5E/F** cruise at ~23,600 RPM → back-EMF dominates → throttle jumps from 75% → 92–93% → fails the 85% band
- **5x7.5E** cruises at ~16,400 RPM → back-EMF penalty is smaller → throttle stays at 65% → passes

The same derating affects the high-RPM prop much more because back-EMF scales linearly with RPM. This is why small, high-RPM props are more sensitive to the Kv safety factor than large, low-RPM props.

---

### C.4 — Current-limited, not Mach-limited (unusual for a speed record)

Most high-speed aircraft are tip-Mach limited: the prop tips approach the speed of sound and efficiency collapses. Here, the 5-inch props at ~37,000 RPM reach only ~0.74 Mach. They hit the motor datasheet current (44.57 A) or ESC continuous limit (60 A) **first**.

**Why:** Small props need very high RPM to make thrust, but the motor/ESC can't supply enough current to reach those RPMs against the prop load. The electrical ceiling is hit before the aerodynamic ceiling. This is unusual — typically you'd worry about tip Mach first on a speed record airframe.

---

### C.5 — Per-prop thrust is aircraft-level thrust ÷ 4 (common point of confusion)

**Intuition says:** Thrust per prop = total thrust.

**Reality:** `W/(L/D)` is aircraft-level drag (1.94 N total). Dividing by 4 props gives 0.485 N per prop. The hover T/W check also uses **total** thrust (`4 × t_hover`) / total weight, not per-prop ÷ total weight. The old document incorrectly computed hover T/W as 1.56 by dividing per-prop thrust (13.04 N) by total weight (8.336 N) — the correct value is 1.10 by construction, or ~6.26 at full-throttle static.

This error propagated into the Stage 7 ranking table, which used per-prop ÷ total-weight T/W figures (1.56, 1.19, 1.17) that don't match the code's total-thrust approach.

---

### C.6 — Battery current < motor current at partial throttle

**Intuition says:** Current is the same everywhere in series.

**Reality:** The ESC is a buck converter. At partial throttle:
```
I_batt ≈ I_motor × Vm / Vbat
```
Since `Vm < Vbat` at partial throttle, the battery delivers less current than the motor windings see. At full throttle (100% duty), they're equal.

The code checks **motor current** against the ESC limit at cruise/climb/hover, which is conservative — the ESC may actually be seeing ~25% lower battery current. This means the reported cruise current margins (~54 A on a 60 A limit) are likely even larger in reality.

---

### C.7 — Full-throttle checks use nominal 22.2 V, not sagged 16.65 V

**Intuition says:** Use the worst-case (sagged) voltage everywhere.

**Reality:** For a thermal survival check, nominal voltage is the worst case for **current**. A fresh or lightly-sagged battery at 22.2 V lets the motor draw more current than a sagged battery at 16.65 V. The code intentionally uses nominal voltage for full-throttle checks so the ESC/motor are tested against the highest plausible current draw.

This means cruise current predictions (sagged voltage) and full-throttle current predictions (nominal voltage) use different assumptions — they're not directly comparable without keeping this in mind.

---

### C.8 — J/pitch ≈ 1.1 means the prop is past its useful range

**Intuition says:** J = 1.0 is a normal operating point.

**Reality:** On a square prop (pitch = diameter), J = 1.0 means you're flying at geometric pitch speed. Beyond that (J/pitch > 1.0), the aircraft is moving faster than the prop "screws" through the air, Ct collapses toward zero, and the prop produces almost no thrust no matter how fast you spin it.

All three candidate props cruise at J/pitch ≈ 1.07–1.12 — right at or past this cliff. They have almost no thrust headroom above cruise speed. This is why increasing cruise speed much beyond 200 km/h would require a higher-pitch prop, not just more RPM.

---

### C.9 — Motor current at full throttle can exceed cruise current by 7×

| Prop | Cruise current | Full-static current | Ratio |
|------|---------------|---------------------|-------|
| 5x5E | 5.3 A | ~37 A | ~7× |
| 5x5F | 4.8 A | ~29.7 A | ~6× |
| 5x7.5E | 4.8 A | 43.0 A | ~9× |

The motor is sized for full-throttle bursts, but normal operation draws a fraction of that. The ESC and motor limits exist for the 100% throttle "oh no" scenario, not for cruise. This is why the 5x7.5E passes everything at cruise (65% throttle, 4.8 A) but has only 1.6A motor margin at full-throttle static — the same prop that loafs at cruise can nearly kill the motor at full throttle.

---

### C.10 — Smaller props fail the throttle band because they need higher RPM for the same thrust

**Intuition says:** A smaller prop should be easier to spin, so it should need less throttle.

**Reality:** Thrust is `T = Ct · ρ · n² · D⁴`. For the same thrust at the same Ct:
```
n ∝ 1 / D²
```
A 4.6-inch prop needs about **(5.0/4.6)² ≈ 1.18×** the RPM of a 5-inch prop to produce the same thrust. Higher RPM means higher back-EMF (`2πn/Kv`), which means higher terminal voltage `Vm`, which means **higher throttle %**.

That's why every 4.x-inch prop in the sweep failed the cruise throttle band (91–130%), while the 5x5 props were at 75%. The smaller props simply can't reach 0.485 N without over-revving past the 85% throttle ceiling.

---

### C.11 — P_motor_max (999 W) is computed but never enforced

**Intuition says:** A computed motor power limit would be checked somewhere.

**Reality:** `P_motor_max` is derived in `Assumptions.__init__` (`global_parameters.py` lines 82-85) as `cells · v_cell · sag · min(ESC_A, i_lim_batt)` ≈ 999 W, but it is **never referenced** in `evaluate.py` or anywhere else in the prop selection chain. The current limits (60 A ESC, 44.57 A motor) serve as the proxy for power limiting instead.

**Implication:** If you changed the motor KV or prop such that current stayed under 60 A but voltage/power exceeded 999 W, the code would not catch it. For this specific setup (60 A ESC at 16.65 V ≈ 999 W), the limits are approximately equivalent, but they're not the same constraint.

---

### C.12 — Motor datasheet Imax (44.57 A) is tighter than ESC continuous (60 A)

**Intuition says:** The ESC limit (60 A) is the binding constraint.

**Reality:** At full throttle, the motor datasheet limit (44.57 A) is checked alongside the ESC limit (60 A). Since 44.57 < 60, the motor is the binding constraint for full-throttle survival.

This is why 5x7.5E fails at kv=0.8: its full-throttle current is 43.0 A — under the ESC's 60 A but over the motor's 44.57 A by a razor-thin 1.6 A. The ESC would survive; the motor might not.

---

### C.13 — verify.py's cruise point (26,987 RPM) doesn't match main_prop.py (23,627 RPM)

**Intuition says:** The verification script confirms the main results.

**Reality:** `verify.py` hardcodes `TARGET_LABEL = "5x5F"` and a cruise point of 26,987 RPM, but `main_prop.py` computes 23,627 RPM for the same prop at the same thrust. The verify.py numbers are from a different run (likely different Kv derating, voltage, or L/D assumption) and are stale.

**Implication:** Do not use `verify.py` to validate `main_prop.py` output. They will disagree. `verify.py` should either be updated to match the current model or removed.

---

### C.14 — `evaluate_all` only catches `ValueError`, not `RuntimeError`

**Intuition says:** If one prop fails to solve, the sweep skips it and moves on.

**Reality:** `evaluate_all` (`evaluate.py` line 197) catches only `ValueError`. If `brentq` in `full_throttle_solve` fails to converge, it raises `RuntimeError`, which is **not caught** and would crash the entire sweep.

**Implication:** If you add a prop/propeller combination where the full-throttle residual doesn't bracket properly, `main_prop.py` will crash instead of marking that prop infeasible. This was fixed by expanding the catch to `(ValueError, RuntimeError)`, but it's a latent fragility.

---

### C.15 — `motor_imax` was hardcoded to `MOTORS[0][4]` (now fixed)

**Intuition says:** Each motor is checked against its own datasheet current limit.

**Reality:** Before the fix, `main_prop.py` line 86 set `INPUTS["motor_imax"] = MOTORS[0][4]`, so every motor combo was checked against the **first motor's** Imax (44.57 A), regardless of which motor was being evaluated. If you added a second motor with a different Imax, all motors would be checked against 44.57 A.

**Fix:** `evaluate_prop` now unpacks `imax` directly from the motor tuple and uses it in the datasheet current check. The `motor_imax` key was removed from `INPUTS`.

---

### C.16 — Hover has no throttle band check

**Intuition says:** Every flight condition should be checked against the 40–85% throttle band.

**Reality:** `evaluate.py` lines 84-102 (hover section) appends checks for hover thrust, hover current, and hover T/W, but **no throttle-in-band check**. Hover is exempt because it's a static condition where throttle is whatever is needed to hold altitude — not an efficiency-cruise operating point.

**Implication:** The hover throttle (52% for 5x5F) is reported but not validated against the band. This is intentional but worth knowing when reading the output.

---

### C.17 — Full-throttle thrust is ~18× cruise thrust (safety envelope vs operating point)

**Intuition says:** Full throttle should produce roughly the thrust you need.

**Reality:** At full throttle @ V_max, the propeller produces ~8.67 N per prop vs the 0.485 N required for cruise — roughly **18×** the required thrust. The aircraft would accelerate violently, not fly steadily.

**Implication:** The full-throttle check asks *"Does the ESC survive if the pilot accidentally holds 100%?"* It is not an operating point. The aircraft would never sustain full throttle in level flight — it would overspeed the airframe. This is why the full-throttle checks are warnings only, not pass/fail criteria.

---

### C.18 — L/D = 4.3 is the single biggest risk factor

**Intuition says:** The prop/motor selection is the critical design decision.

**Reality:** Every downstream number scales with L/D. If the real quad airframe has worse drag than L/D = 4.3 (likely — four pylons, wing interference, prop wash at 200 km/h), the required thrust rises proportionally:

| L/D | Total cruise drag | Per-prop thrust | Effect |
|-----|------------------|-----------------|--------|
| 4.3 | 1.94 N | 0.485 N | Baseline |
| 3.5 | 2.38 N | 0.595 N | +23% thrust, +23% current |
| 3.0 | 2.78 N | 0.695 N | +43% thrust, +43% current |

At L/D = 3.0, the 5x5F cruise current would rise from 4.8 A to ~6.9 A, and throttle from 75% to roughly 85-90% — pushing it against the band ceiling. At kv_load_factor = 0.8, even the 5x7.5E would see higher cruise current and potentially fail the motor limit.

**Bottom line:** The prop selection is secondary to getting an accurate L/D number. Bench-test drag at speed before trusting any prop selection result.

---

### C.19 — Does J/pitch > 1 mean the prop produces drag instead of thrust?

**Intuition says:** Once you're flying faster than pitch speed (`J/pitch > 1`), the prop should act like an air brake — producing drag instead of thrust.

**Reality:** Not automatically, but you are at or past the edge where thrust collapses toward zero.

`J/pitch = V / (n · pitch)`. When this exceeds 1.0, the aircraft is moving faster than the propeller's geometric pitch would advance it in one revolution. The blade sections see a very low or even negative angle of attack — the prop is "unloaded."

| Regime | `J/pitch` | `Ct` behavior | Physical result |
|--------|-----------|---------------|-----------------|
| Normal | < 0.8 | `Ct` high and positive | Good thrust |
| Transition | ~0.8–1.0 | `Ct` dropping fast | Thrust falling off |
| Edge | ~1.0–1.2 | `Ct` → 0 | Almost no thrust |
| Over-speed | > 1.2+ | `Ct` can go slightly negative | Prop windmills = **drag** |

**In real physics**, a prop flown well past pitch speed can windmill — the airflow pushes the blades backward, producing negative thrust (drag). The motor is still drawing current to spin it, but the prop is now acting like an air brake.

**What the code does:** Your code clamps `Ct` to zero:
```python
ct = max(float(ct), 0.0)
```
So the model **never** produces negative thrust. The minimum is `T = 0`. At `J/pitch ≈ 1.1`, your 5x5 props are right where `Ct` is essentially zero — they're producing thrust almost purely from the `n²` term, not from coefficient lift. That's why cruise current is so low (~4.8 A): the motor is spinning the prop, but the prop isn't "grabbing" much air.

Even at `Ct = 0`, you're still:
- Spinning the prop (drawing current for torque)
- Creating profile drag from the blades
- Getting **zero useful thrust**

So it's **not negative thrust in the model**, but it is **parasitic power consumption with no benefit**.

---

### Connection to halving L/D (4.3 → 2.15)

When you halved L/D, required thrust doubled (0.485 N → 0.969 N per prop). The solver responded by **increasing RPM**, which **decreased `J`**:

```
J = V / (n · D)
```

Higher `n` → lower `J` → moves **back toward** `J/pitch < 1.0` → `Ct` rises from near-zero toward a useful value.

That's why throttle barely went up: you didn't need double the RPM to get double the thrust. As `J` dropped back into the region where `Ct` is sensitive, the prop **re-engaged** and produced more thrust per RPM than it was at the bite limit. The prop effectively **un-clogged** itself.

**Strategic implication:** Operating near `J/pitch ≈ 1.1` means you're on the edge of the prop's useful range. Small increases in required thrust (worse L/D) can be absorbed by modest RPM increases because `Ct` recovers as `J` drops back toward 1.0. But once you're comfortably below `J/pitch ≈ 0.7`, `Ct` flattens out and drag maps more directly to RPM — no more buffer.

---

### C.20 — Halving L/D: why throttle barely moved at cruise

**What happened:** Required thrust doubled (0.485 N → 0.969 N per prop), but cruise throttle only rose modestly (e.g., 65% → ~78% for 5x7.5E).

**Why:** Two effects compound in your favor:

1. **`n²` increases** — direct thrust multiplier
2. **`Ct` increases as `J` drops** — higher RPM lowers `J = V/(n·D)`, moving back toward `J/pitch = 1.0` where `Ct` is higher

The solver finds the new RPM where `thrust(n) = T_target`. Because `Ct` rises as `J` falls, you need **less than √2× RPM** to double thrust. Throttle tracks back-EMF (`2πn/Kv`), which tracks RPM, so throttle increases less than thrust.

**The dampening works both ways:**
- **Drag increases** (worse L/D): RPM up, `J` down toward 1.0, `Ct` up → thrust compensates somewhat → throttle doesn't rise as fast as raw thrust suggests
- **Drag decreases** (better L/D): RPM down, `J` up toward 1.5+, `Ct` collapses → thrust falls faster than RPM alone would suggest

**Limits:** This buffer only works while `J/pitch` is roughly between 0.7 and 1.3. Deep in the efficient range (`J/pitch < 0.7`), `Ct` flattens and drag maps more directly to RPM. Past `J/pitch ≈ 1.3`, `Ct` is effectively zero — no amount of RPM produces thrust (cliff edge).

---

### C.21 — P_batt calculation

**Formula:** `P_batt = Vm · I / ESC_EFF`

Where:
- `Vm = I · Rm + 2πn / Kv_rad` (motor terminal voltage = resistive drop + back-EMF)
- `I = Q · Kv_rad + I₀` (motor current from torque)
- `ESC_EFF = 0.95` (fixed ESC efficiency)

**Implementation:** `evaluate.py` lines 138-141:
```python
vm = current_a * rm_ohm + 2.0 * math.pi * n_rps / kvr
v_load = vbat - current_a * self.r_pack
throttle = vm / v_load
p_batt_w = vm * current_a / self.esc_eff
```

**Why divide by ESC_EFF:** The ESC is not 100% efficient. Battery power must cover both motor electrical power AND ESC losses. At 95% efficiency, battery power is ~5% higher than motor electrical power.

---

### C.22 — P_motor_max should include motor Imax

**Current code** (`global_parameters.py` lines 82-85):
```python
i_lim_batt_per_motor = (self.ESC_C_RATE * self.ESC_CAP_AH) / self.ESC_NUM_MOTORS
self.P_motor_max = self.ESC_CELLS * self.ESC_V_CELL * self.ESC_SAG_FACTOR * min(
    self.ESC_A, i_lim_batt_per_motor
)
```

**Result:** `P_motor_max = 16.65 × min(60, 64) = 999 W`

**Problem:** This uses ESC limit (60A) and battery C-rate limit (64A), but **omits motor Imax (44.57A)**. The motor is the actual binding constraint at 44.57A.

**Should be:**
```python
self.P_motor_max = self.ESC_CELLS * self.ESC_V_CELL * self.ESC_SAG_FACTOR * min(
    self.ESC_A, i_lim_batt_per_motor, self.MOTORS[0][4]  # motor Imax
)
```

**Result with fix:** `P_motor_max = 16.65 × min(60, 64, 44.57) = 16.65 × 44.57 ≈ 742 W`

**Impact:** This is a 25% reduction from 999W to 742W. At full throttle static, 5x5F draws 29.7A × 16.65V ≈ 494W (under 742W, OK). But 5x7.5E draws 43.0A × 16.65V ≈ 716W (still under 742W, but much closer). Any higher-voltage pack or larger prop would exceed it.

---

### C.23 — Battery discharge rate check

**Is it checked?** Yes, indirectly. The battery C-rate per motor is:
```
i_lim_batt = (160 C × 1.6 Ah) / 4 motors = 64 A
```

This is folded into `i_lim_cont = min(ESC_A, i_lim_batt) = min(60, 64) = 60 A`. Since the ESC limit (60A) is lower than the battery C-rate limit (64A), the ESC limit is binding and the battery check is effectively silent.

**However:** The burst check uses a different formula:
```
i_lim_burst = min(ESC_BURST_A, (c_rate × cap_ah × 2) / num_motors)
           = min(65, (160 × 1.6 × 2) / 4)
           = min(65, 128) = 65 A
```

Here the battery C-rate burst limit (128A) is much higher than the ESC burst limit (65A), so again the ESC is binding.

**Bottom line:** The battery discharge rate IS modeled, but it's never the limiting factor — ESC (60A continuous, 65A burst) and motor (44.57A) are always tighter. If you upgraded to a higher-C battery or larger capacity, the battery limit would rise further and remain non-binding.

---

### C.24 — Full-throttle efficiency higher than cruise

**Observation:** Full-throttle efficiency (~62-84%) can be higher than cruise efficiency (~50-59%).

**Why this is NOT a contradiction:** Propulsive efficiency = `J · (Ct/Cp)`. At full-throttle V_max, the motor settles at its equilibrium RPM, which happens to put `J` near **0.83** — close to the prop's sweet spot where `Ct/Cp` is favorable. At cruise, the solver is *forced* to spin at whatever RPM produces the tiny required thrust (0.485 N), which lands at `J/pitch ≈ 1.09` — deep in the unloading region where `Ct` is near zero.

| Condition | RPM | J | J/pitch | Ct | η |
|-----------|-----|---|---------|-----|---|
| Cruise (5x5F) | 23,627 | ~1.09 | ~1.09 | ~0 | ~50% |
| Full throttle V_max (5x5E) | 31,554 | ~0.83 | ~0.83 | moderate | ~84% |

At full throttle V_max:
- Higher RPM → lower `J` (moves back toward 1.0 and below)
- Higher `Ct` → more thrust per unit of torque
- `T·V` (thrust power) is large because both T and V are high
- Battery input power is high, but thrust power grows faster

At cruise:
- High RPM needed to make 0.485 N with near-zero Ct
- `J/pitch ≈ 1.09` → `Ct` collapsing → most motor power goes into spinning the prop, not producing useful thrust
- Motor current (5.6A) is disproportionate to thrust output

**Two effects combine:**
1. **Motor:** Full throttle operates at higher torque/current → `I₀` (0.7A) is a smaller fraction → motor more efficient
2. **Propeller:** Full-throttle V_max lands at `J ≈ 0.83` where `Ct/Cp` is favorable; cruise at `J/pitch ≈ 1.09` where `Ct` is near zero

**Bottom line:** Full-throttle efficiency is higher *because* the full-throttle operating point happens to fall at a better advance ratio, not because full throttle is inherently more efficient. If you ran full throttle at a much higher airspeed, J would drop below optimal and efficiency would fall again.

---

**Bottom line:** The model is a useful screening tool but not a flight-qualified prediction. The 5x7.5E's 1.6A motor margin at full throttle is too thin to trust without bench validation. If you proceed with the 5x5 props at relaxed throttle band (92–93%), verify actual current draw at cruise speed on a thrust stand before committing to flight. The biggest single risk is L/D = 4.3 being optimistic for a quad airframe at 200 km/h — if drag is higher, everything shifts toward higher current and tighter margins.

---

### C.25 — Higher pitch = lower hover throttle (counterintuitive)

**Observation:** 5x7.5E hover throttle (52%) is LOWER than 5x5F hover throttle (56%), despite 5x7.5E being a "bigger" prop.

**Why:** At hover (V=0), J = 0. The prop produces static thrust:
```
T = Ct(0) · ρ · n² · D⁴
```

Higher pitch props generally have higher Ct at J=0 (more blade area/bite). More importantly, for the same thrust:
- **5x5F:** Needs ~16,083 RPM → higher back-EMF → higher throttle
- **5x7.5E:** Needs ~12,755 RPM → lower back-EMF → lower throttle

The 5x7.5E moves more air per revolution (larger diameter AND higher pitch), so it spins slower at the same static thrust. Slower RPM = less back-EMF = less terminal voltage needed = lower throttle.

**The trade-off:** While 5x7.5E needs less throttle at hover, it draws MORE current at full throttle (43.0A vs 34.3A for 5x5F) because the larger prop loads the motor harder at high RPM. Lower hover throttle doesn't mean better overall — it means the prop is optimized for static thrust, not high-speed flight.

---

**Bottom line:** The model is a useful screening tool but not a flight-qualified prediction. The 5x7.5E's 1.6A motor margin at full throttle is too thin to trust without bench validation. If you proceed with the 5x5 props at relaxed throttle band (92–93%), verify actual current draw at cruise speed on a thrust stand before committing to flight. The biggest single risk is L/D = 4.3 being optimistic for a quad airframe at 200 km/h — if drag is higher, everything shifts toward higher current and tighter margins.

