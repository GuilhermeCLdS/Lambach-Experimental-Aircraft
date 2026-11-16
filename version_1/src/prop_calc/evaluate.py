"""Evaluation: check prop+motor against requirements at known operating points.

Simple, reusable evaluation for prop + motor selection.
"""

from __future__ import annotations

import math

from electronics import ElectricalSystem
from physics import max_rpm_from_mach, thrust, tip_mach


def evaluate_prop(prop, motor, inputs: dict, elec: ElectricalSystem) -> dict:
    """Evaluate one prop+motor combination.

    Args:
        prop: Prop instance with Ct/Cp data
        motor: (name, kv_rpm_per_v, rm_ohm, i0_a, imax_a)
        inputs: Dict with V, T_cruise, T_climb, T_static_min, W, MACH_LIM, THR_BAND,
                MIN_STATIC_TW, motor_rpm_max, motor_imax
        elec: ElectricalSystem instance

    Returns dict with results and checks.
    """
    name, kv, rm, i0, imax = motor
    V = inputs["V"]
    T_CRUISE = inputs["T_cruise"]
    T_CLIMB = inputs["T_climb"]
    T_STATIC = inputs["T_static"]
    MACH_LIM = inputs["MACH_LIM"]
    THR_BAND = inputs["THR_BAND"]
    MIN_STATIC_TW = inputs["MIN_STATIC_TW"]
    W = inputs["W"]
    n_props = inputs.get("n_props", 4)
    MOTOR_RPM_MAX = inputs.get("motor_rpm_max", float("inf"))
    n_float = float("nan")
    checks = []

    # RPM bounds
    n_lo = prop.n_min()
    n_hi_V = min(max_rpm_from_mach(MACH_LIM, prop.D, V), prop.n_tab_max(), MOTOR_RPM_MAX / 60.0)
    n_hi_hover = min(prop.n_tab_max(), MOTOR_RPM_MAX / 60.0)

    # --- Full throttle at V_max and static ---
    # Uses nominal VBAT (worst case for current)
    n_ft, t_ft, i_ft = elec.full_throttle_solve(prop, kv, rm, i0, V, elec.vbat)
    n_static, t_static, i_static = elec.full_throttle_solve(prop, kv, rm, i0, 0.0, elec.vbat)

    # --- 1. CRUISE at V_max ---
    cruise = elec.rpm_for_thrust(
        prop, kv, rm, i0, V, T_CRUISE, n_lo, n_hi_V
    )
    if cruise is None:
        n_cruise = i_cruise = throttle_cruise = p_batt_cruise = v_batt_cruise = n_float
        checks.append(("Cruise thrust at V_max", "cannot achieve required thrust", False))
    else:
        n_cruise, i_cruise, throttle_cruise, p_batt_cruise = cruise
        v_batt_cruise = elec.vbat_sag - i_cruise * elec.r_pack
        checks.append(("Cruise thrust at V_max", f"{T_CRUISE:.2f} N at {n_cruise*60:.0f} RPM", True))
        checks.append(("Cruise throttle", f"{throttle_cruise:.0%} in [{THR_BAND[0]:.0%}-{THR_BAND[1]:.0%}]",
                       THR_BAND[0] <= throttle_cruise <= THR_BAND[1]))
        ok, msg = elec.check_current(i_cruise)
        checks.append(("Cruise current", msg, ok))

    # --- 2. CLIMB at V_max ---
    climb = elec.rpm_for_thrust(
        prop, kv, rm, i0, V, T_CLIMB, n_lo, n_hi_V
    )
    if climb is None:
        n_climb = i_climb = throttle_climb = p_batt_climb = v_batt_climb = n_float
        checks.append(("Climb thrust at V_max", "cannot achieve required thrust", False))
    else:
        n_climb, i_climb, throttle_climb, p_batt_climb = climb
        v_batt_climb = elec.vbat_sag - i_climb * elec.r_pack
        checks.append(("Climb thrust at V_max", f"{T_CLIMB:.2f} N at {n_climb*60:.0f} RPM", True))
        checks.append(("Climb throttle", f"{throttle_climb:.0%} in [{THR_BAND[0]:.0%}-{THR_BAND[1]:.0%}]",
                       THR_BAND[0] <= throttle_climb <= THR_BAND[1]))
        ok, msg = elec.check_current(i_climb)
        checks.append(("Climb current", msg, ok))

    # --- 3. HOVER (static) ---
    hover = elec.rpm_for_thrust(
        prop, kv, rm, i0, 0.0, T_STATIC, n_lo, n_hi_hover
    )
    if hover is None:
        n_hover = i_hover = throttle_hover = p_batt_hover = v_batt_hover = n_float
        t_hover = n_float
        checks.append(("Hover thrust", "cannot achieve required thrust", False))
    else:
        n_hover, i_hover, throttle_hover, p_batt_hover = hover
        v_batt_hover = elec.vbat_sag - i_hover * elec.r_pack
        t_hover = inputs["T_static"]
        t_hover_total = t_hover * n_props
        checks.append(("Hover thrust", f"{t_hover:.2f} N at {n_hover*60:.0f} RPM", True))
        ok, msg = elec.check_current(i_hover)
        checks.append(("Hover current", msg, ok))
        # T/W: total hover thrust / total weight >= MIN_STATIC_TW
        checks.append(("Hover T/W", f"{t_hover_total/inputs['W']:.2f} >= {inputs['MIN_STATIC_TW']}",
                       t_hover_total >= inputs['MIN_STATIC_TW'] * inputs['W']))

    # --- RPM inside data table ---
    rpm_need = max(n_static, n_ft) * 60.0
    rpm_limit = min(prop.rpm_max * prop.rpm_table_tol, MOTOR_RPM_MAX)
    checks.append(("RPM inside data table",
                   f"{rpm_need:.0f} vs {rpm_limit:.0f} RPM",
                   rpm_need <= rpm_limit))

    # Full-throttle current at known conditions (WARNING only)
    i_peak = max(i_ft, i_static)
    ok, msg = elec.check_current(i_peak)
    checks.append((f"Full-throttle peak current (V_max/static) [WARNING]", msg, True))

    ok, msg = elec.check_current(i_peak, burst=True)
    checks.append((f"Full-throttle peak current burst [WARNING]", msg, True))

    # Motor datasheet current
    checks.append(("Motor datasheet current",
                   f"{i_peak:.1f}A vs {imax:.1f}A",
                   i_peak <= imax))

    # Full-throttle climb check
    checks.append(("Full-throttle climb thrust",
                   f"{t_ft:.2f}N vs {T_CLIMB:.2f}N", t_ft >= T_CLIMB))

    # Full-throttle static T/W
    t_static_total = t_static * n_props
    checks.append(("Full-throttle static T/W",
                   f"{t_static_total/inputs['W']:.2f} vs {inputs['MIN_STATIC_TW']}",
                   t_static_total >= inputs['MIN_STATIC_TW'] * inputs['W']))

    # Tip Mach
    mach_tip = max(tip_mach(n_static, prop.D, 0.0), tip_mach(n_ft, prop.D, V))
    checks.append(("Tip Mach", f"{mach_tip:.2f} vs {MACH_LIM}", mach_tip <= MACH_LIM))

    # Efficiency at V_max (full throttle): thrust power / battery power
    # Efficiency = T_ft * V / P_batt_ft  where P_batt_ft = Vm * i_ft / esc_eff
    if i_ft > 0:
        kvr = kv * 2.0 * math.pi / 60.0
        vm_ft = i_ft * rm + 2.0 * math.pi * n_ft / kvr
        p_batt_ft = vm_ft * i_ft / elec.esc_eff
        eff_ft = t_ft * V / p_batt_ft if p_batt_ft > 0 else 0.0
    else:
        vm_ft = 0.0
        p_batt_ft = 0.0
        eff_ft = 0.0

    # Efficiency at cruise / climb / hover: thrust power / battery power
    # Efficiency = T * V / P_batt where P_batt = Vm * I / ESC_EFF
    eff_cruise = T_CRUISE * V / p_batt_cruise if p_batt_cruise > 0 else 0.0
    eff_climb = T_CLIMB * V / p_batt_climb if p_batt_climb > 0 else 0.0
    eff_hover = 0.0  # V = 0, so thrust power = 0 (use Figure of Merit instead)

    return {
        "prop": prop.label,
        "size": f"{prop.d_in:g}x{prop.p_in:g}",
        "blades": prop.blades,
        "motor": name,
        "checks": checks,
        "feasible": all(ok for _, _, ok in checks),
        "n_fail": sum(not ok for _, _, ok in checks),
        # Cruise
        "rpm_cruise": n_cruise * 60.0 if n_cruise == n_cruise else float("nan"),
        "J_cruise": V / (n_cruise * prop.D) if n_cruise == n_cruise else float("nan"),
        "j_over_pitch_cruise": (V / (n_cruise * prop.D)) / (prop.p_in / prop.d_in) if n_cruise == n_cruise else float("nan"),
        "throttle_cruise": throttle_cruise,
        "i_cruise": i_cruise,
        "p_batt_cruise": p_batt_cruise,
        "v_batt_cruise": elec.vbat_sag - i_cruise * elec.r_pack if i_cruise == i_cruise else float("nan"),
        # Climb
        "rpm_climb": n_climb * 60.0 if n_climb == n_climb else float("nan"),
        "J_climb": V / (n_climb * prop.D) if n_climb == n_climb else float("nan"),
        "j_over_pitch_climb": (V / (n_climb * prop.D)) / (prop.p_in / prop.d_in) if n_climb == n_climb else float("nan"),
        "throttle_climb": throttle_climb,
        "i_climb": i_climb,
        "p_batt_climb": p_batt_climb,
        "v_batt_climb": elec.vbat_sag - i_climb * elec.r_pack if i_climb == i_climb else float("nan"),
        # Hover
        "rpm_hover": n_hover * 60.0 if n_hover == n_hover else float("nan"),
        "throttle_hover": throttle_hover,
        "i_hover": i_hover,
        "p_batt_hover": p_batt_hover,
        "v_batt_hover": elec.vbat_sag - i_hover * elec.r_pack if i_hover == i_hover else float("nan"),
        "t_hover": t_hover,
        # Full throttle
        "rpm_ft": n_ft * 60.0, "t_ft": t_ft, "i_ft": i_ft,
        "vm_ft": vm_ft, "p_batt_ft": p_batt_ft, "eff_ft": eff_ft,
        "eff_cruise": eff_cruise, "eff_climb": eff_climb, "eff_hover": eff_hover,
        "rpm_static": n_static * 60.0, "t_static": t_static, "i_static": i_static,
        "tw_static": t_static * n_props / W,
        "esc_margin": elec.esc_a - i_peak,
        "motor_margin": imax - i_peak,
        "mach_tip": mach_tip,
    }


def evaluate_all(props, motors, inputs: dict, elec: ElectricalSystem):
    """Evaluate all prop x motor combinations."""
    results = []
    for prop in props:
        for motor in motors:
            try:
                result = evaluate_prop(prop, motor, inputs, elec)
            except (ValueError, RuntimeError) as e:
                result = {
                    "prop": prop.label, "size": f"{prop.d_in:g}x{prop.p_in:g}",
                    "blades": prop.blades, "motor": motor[0],
                    "checks": [("Solver", str(e), False)],
                    "feasible": False, "n_fail": 1,
                    "p_batt_cruise": float("nan"),
                }
            results.append(result)
    return results


def rank_results(results):
    """Rank feasible results by thrust at V_max, then power at cruise, then efficiency at V_max."""
    feasible = [r for r in results if r["feasible"]]
    return sorted(feasible, key=lambda r: (-r["t_ft"], r["p_batt_cruise"], -r["eff_ft"]))