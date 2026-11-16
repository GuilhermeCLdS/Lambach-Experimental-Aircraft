"""Propeller + motor selection for fixed-wing aircraft (quad setup).

Run: python version_1/src/prop_calc/main_prop.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from global_parameters.global_parameters import Assumptions, Parameters

from data_loading import load_props
from electronics import ElectricalSystem
from evaluate import evaluate_all, rank_results


# ---------------------------------------------------------------------------
# All inputs come from global_parameters
# ---------------------------------------------------------------------------
assumptions = Assumptions()
parameters = Parameters()

# Airframe / mission
LD = assumptions.LD_propcalc
MIN_STATIC_TW = assumptions.MIN_STATIC_TW
CLIMB_DEG = assumptions.CLIMB_DEG
THR_BAND = assumptions.THR_BAND
W = assumptions.aircraft_weight      # Weight [N]
V = assumptions.velocity             # Design (max) speed [m/s]
D_MAX_M = assumptions.prop_dia       # Max prop diameter [m]
MACH_LIM = assumptions.M_tip_max     # Tip Mach limit
N_PROPS = assumptions.n_propellers   # Number of propellers
MOTOR_RPM_MAX = assumptions.rpm_max

# Propeller library
D_MIN_IN = assumptions.D_MIN_IN
RPM_TABLE_TOL = assumptions.RPM_TABLE_TOL
DATA_DIR = Path(__file__).resolve().parents[3] / "data" / "prop_Performance"

# Electrical system from assumptions
elec = ElectricalSystem(
    cells=assumptions.ESC_CELLS,
    cap_ah=assumptions.ESC_CAP_AH,
    c_rate=assumptions.ESC_C_RATE,
    v_cell=assumptions.ESC_V_CELL,
    r_pack=assumptions.ESC_R_PACK,
    esc_a=assumptions.ESC_A,
    esc_burst_a=assumptions.ESC_BURST_A,
    esc_burst_s=assumptions.ESC_BURST_S,
    esc_eff=assumptions.ESC_EFF,
    num_motors=assumptions.ESC_NUM_MOTORS,
    sag_factor=assumptions.ESC_SAG_FACTOR,
    kv_load_factor=assumptions.kv_load_factor,
)

# Derived totals
IN2M = 0.0254
D_MAX_IN = D_MAX_M / IN2M
T_CRUISE_TOTAL = W / LD
T_CLIMB_TOTAL = T_CRUISE_TOTAL + W * math.sin(math.radians(CLIMB_DEG))
T_STATIC_TOTAL = MIN_STATIC_TW * W

# Per-prop requirements
T_CRUISE = T_CRUISE_TOTAL / N_PROPS
T_CLIMB = T_CLIMB_TOTAL / N_PROPS
T_STATIC = T_STATIC_TOTAL / N_PROPS

# Motors from assumptions
MOTORS = assumptions.MOTORS

# Inputs for evaluation
INPUTS = {
    "V": V,
    "T_cruise": T_CRUISE,
    "T_climb": T_CLIMB,
    "T_static": T_STATIC,
    "MIN_STATIC_TW": MIN_STATIC_TW,
    "W": W,
    "n_props": N_PROPS,
    "MACH_LIM": MACH_LIM,
    "THR_BAND": THR_BAND,
    "motor_rpm_max": MOTOR_RPM_MAX,
}


def main() -> int:
    library = load_props(DATA_DIR, RPM_TABLE_TOL)
    library = [p for p in library if D_MIN_IN <= p.d_in <= D_MAX_IN + 1e-9]


    print(f"Prop library: {len(library)} PER3 files in {DATA_DIR}")
    print(f"V_max = {V:.1f} m/s ({V*3.6:.0f} km/h)")
    print(f"Per-prop thrust required: cruise {T_CRUISE:.3f}N | climb {T_CLIMB:.3f}N | hover {T_STATIC:.3f}N")
    print(f"ESC: {elec.esc_a:.0f}A cont / {elec.esc_burst_a:.0f}A burst ({elec.esc_burst_s}s)")
    print(f"Battery: {elec.cells}S {elec.cap_ah}Ah {elec.c_rate}C | V_sag={elec.vbat_sag:.1f}V")
    print(f"Limits: T/W>={MIN_STATIC_TW}, Mach<={MACH_LIM}, Throttle={THR_BAND[0]:.0%}-{THR_BAND[1]:.0%}, D={D_MIN_IN}-{D_MAX_IN:.1f}in\n")

    results = evaluate_all(library, MOTORS, INPUTS, elec)

    feasible = [r for r in results if r["feasible"]]
    failed = [r for r in results if not r["feasible"]]

    # Rank feasible: thrust (desc), power at cruise (asc), efficiency at V_max (desc)
    ranked = rank_results(results)

    # Report
    print("\n=== FULL-THROTTLE OVERCURRENT AT KNOWN CONDITIONS ===")
    print(f"ESC: {elec.esc_a:.0f}A cont / {elec.esc_burst_a:.0f}A burst ({elec.esc_burst_s}s)\n")
    for r in results:
        i_peak = max(r.get("i_ft", 0), r.get("i_static", 0))
        ok = i_peak <= elec.esc_a
        culprit = "static" if r.get("i_static", 0) > r.get("i_ft", 0) else "V_max"
        if ok:
            print(f"  {r['size']:8} {r['prop']:8} on {r['motor']:<10}: within {elec.esc_a:.0f}A (peak {i_peak:.1f}A)")
        else:
            print(f"  {r['size']:8} {r['prop']:8} on {r['motor']:<10}: EXCEEDS {elec.esc_a:.0f}A ({i_peak:.1f}A, {culprit})")

    # Summary
    print(f"\n=== SUMMARY: {len(feasible)} feasible / {len(results)} checked ===")
    for r in ranked:
        print(f"  PASS  {r['size']:<8} {r['prop']:<12} {r['motor']:<14} "
              f"P_cruise={r['p_batt_cruise']:.0f}W thr={r['throttle_cruise']:.0%}/{r['throttle_climb']:.0%}/{r['throttle_hover']:.0%}")
    for r in failed:
        reasons = "; ".join(f"{c[0]} ({c[1]})" for c in r["checks"] if not c[2])
        print(f"  FAIL  {r['size']:<8} {r['prop']:<12} {r['motor']:<14} {reasons}")

    if not feasible:
        print("\nNo feasible combos. Add more PER3 files or relax constraints.")
        return 1

    # Detailed results ranked by thrust, then power, then efficiency
    print("\n=== RANKED FEASIBLE (thrust, power, efficiency at V_max) ===")
    for rank, r in enumerate(ranked, 1):
        print(f"\n>>> #{rank} {r['size']} ({r['blades']}-blade, {r['prop']}) on {r['motor']}")
        print(f"  CRUISE:  {r['rpm_cruise']:.0f} RPM, J={r['J_cruise']:.2f}, J/pitch={r['j_over_pitch_cruise']:.2f}, {r['throttle_cruise']:.0%}, {r['i_cruise']:.1f}A, {r['p_batt_cruise']:.0f}W, eff={r['eff_cruise']:.0%}")
        print(f"  CLIMB:   {r['rpm_climb']:.0f} RPM, J={r['J_climb']:.2f}, {r['throttle_climb']:.0%}, {r['i_climb']:.1f}A, {r['p_batt_climb']:.0f}W, eff={r['eff_climb']:.0%}")
        print(f"  HOVER:   {r['rpm_hover']:.0f} RPM, {r['throttle_hover']:.0%}, {r['i_hover']:.1f}A, {r['p_batt_hover']:.0f}W, eff={r['eff_hover']:.0%}")
        print(f"  FULL V_max: {r['rpm_ft']:.0f} RPM, {r['t_ft']:.1f}N, {r['i_ft']:.1f}A, eff={r['eff_ft']:.2%}")
        print(f"  FULL STATIC: {r['rpm_static']:.0f} RPM, {r['t_static']:.1f}N, {r['i_static']:.1f}A, T/W={r['tw_static']:.2f}")
        print(f"  LIMITS: Mach={r['mach_tip']:.2f}, ESC margin={r['esc_margin']:.1f}A, Motor margin={r['motor_margin']:.1f}A")

    return 0


if __name__ == "__main__":
    sys.exit(main())