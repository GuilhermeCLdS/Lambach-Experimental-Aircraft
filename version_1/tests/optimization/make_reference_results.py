"""Regenerate reference_results.json from the legacy optimizer.

Run from the repository root:
    python version_1/tests/optimization/make_reference_results.py

The JSON is the golden record of what the legacy (pre-refactor) code
produces. Only regenerate it on purpose, when a physics change is intended.
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from conftest import REFERENCE_CASES, REFERENCE_FILE, load_legacy_module, solve_quietly  # noqa: E402

DESIGN_KEYS = (
    "AOA", "fuse_len", "fuse_dia", "pylon_len", "pylon_chord", "pylon_inc",
    "canard_len", "canard_chord", "canard_inc", "l_c", "l_cg", "l_p",
)
OUTPUT_KEYS = (
    "total_drag", "drag_profile", "drag_induced", "lift_aero", "lift_prop",
    "lift_tot", "moment_aero", "moment_thrust", "moment_thrust_diff",
    "moment_thrust_incidence", "x_np", "CLa", "Cma", "thrust_along_velocity",
)
PROPULSION_KEYS = (
    "rpm_top", "rpm_bottom", "T_top", "T_bottom", "P_top", "P_bottom",
    "J_top", "J_bottom", "M_tip_top", "M_tip_bottom", "delta_T", "power_tot",
)


def solve_case(module, overrides):
    cfg = module.Config()
    for key, value in overrides.items():
        setattr(cfg, key, value)
    optimizer = module.AircraftDesignOptimizer(cfg)
    sol = solve_quietly(optimizer.opti)

    def value(x):
        return float(sol.value(x))

    return {
        "design": {k: value(getattr(optimizer, k)) for k in DESIGN_KEYS},
        "outputs": {k: value(getattr(optimizer, k)) for k in OUTPUT_KEYS},
        "propulsion": {k: value(getattr(optimizer.propulsion, k)) for k in PROPULSION_KEYS},
        "weight": float(cfg.aircraft_weight),
        "J_tab_max": float(optimizer.prop_db.J_tab_max),
    }


def main():
    module = load_legacy_module()
    results = {name: solve_case(module, overrides) for name, overrides in REFERENCE_CASES.items()}
    with open(REFERENCE_FILE, "w") as file:
        json.dump(results, file, indent=2)
    print(f"Wrote {REFERENCE_FILE}")


if __name__ == "__main__":
    main()
