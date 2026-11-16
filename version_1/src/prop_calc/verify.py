"""Quick verification: print thrust, J, and tip Mach for a known propeller."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from global_parameters.global_parameters import Assumptions

from data_loading import load_props
from physics import thrust, tip_mach

assumptions = Assumptions()

V = assumptions.velocity  # design (max) speed [m/s]
DATA_DIR = Path(__file__).resolve().parents[3] / "data" / "prop_Performance"
RPM_TABLE_TOL = 1.01
TARGET_LABEL = "5x5F"

# Operating points to check: (description, rpm, airspeed_mps)
POINTS = [
    ("Cruise", 26987, V),
    ("Full throttle @ V_max", 37141, V),
    ("Static", 37999, 0.0),
]


def main() -> int:
    library = load_props(DATA_DIR, RPM_TABLE_TOL)
    try:
        prop = next(p for p in library if p.label == TARGET_LABEL)
    except StopIteration:
        print(f"Prop {TARGET_LABEL} not found in {DATA_DIR}")
        return 1

    print(f"Prop: {prop.label} ({prop.d_in}x{prop.p_in}, {prop.blades}-blade)")
    print(f"Diameter: {prop.D:.4f} m")
    print(f"RPM range: {prop.rpm_min:.0f} - {prop.rpm_max:.0f}")
    print(f"V_design: {V:.1f} m/s\n")

    for name, rpm, v_mps in POINTS:
        n_rps = rpm / 60.0
        J = v_mps / (n_rps * prop.D) if n_rps > 0 else 0.0
        t_n = thrust(prop, n_rps, v_mps)
        m_tip = tip_mach(n_rps, prop.D, v_mps)

        print(f"{name} ({rpm:.0f} RPM):")
        print(f"  J = {J:.3f}")
        print(f"  Thrust = {t_n:.3f} N")
        print(f"  Tip Mach = {m_tip:.3f}")
        print(f"  Positive thrust: {t_n > 0}\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
