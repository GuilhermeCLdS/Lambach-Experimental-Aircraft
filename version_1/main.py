"""Integrates the src modules into one run."""

from pathlib import Path

import matplotlib.pyplot as plt

from src.global_parameters.global_parameters import Assumptions, Parameters
from src.optimization.modules.aircraft_design_optimizer import AircraftDesignOptimizer
from src.optimization.modules.print_design_report import print_design_report
from src.optimization.modules.propeller_performance_database import PropellerPerformanceDatabase

FIGURES_DIR = Path(__file__).resolve().parent / "figures"


# def run_design_optimization(assumptions=None, show=True):
#     """Optimize the aircraft, print the report and save a three-view figure."""
#     assumptions = assumptions or Assumptions()

#     prop_db = PropellerPerformanceDatabase(
#         assumptions.perf_file,
#         Parameters.AIR_DENSITY_SEA_LEVEL,
#         assumptions.prop_dia,
#         rpm_lo=assumptions.rpm_min,
#     )
#     optimizer = AircraftDesignOptimizer(assumptions, prop_db)
#     optimizer.solve()
#     print_design_report(optimizer)

#     airplane = optimizer.build_solved_airplane()
#     airplane.draw_three_view(style="shaded", show=False)
#     FIGURES_DIR.mkdir(exist_ok=True)
#     plt.savefig(FIGURES_DIR / "optimized_aircraft_three_view.png", dpi=200, bbox_inches="tight")
#     if show:
#         plt.show()
#     return optimizer


def run_design_optimization(assumptions=None, show=True,
                             max_iter=10, tol_kg=5e-4, damping=0.7):
    """Optimize with mass feedback: solve, read MassProperties output, update
    assumptions.aircraft_mass, repeat until the fixed point is reached."""
    assumptions = assumptions or Assumptions()

    prop_db = PropellerPerformanceDatabase(
        assumptions.perf_file,
        Parameters.AIR_DENSITY_SEA_LEVEL,
        assumptions.prop_dia,
        rpm_lo=assumptions.rpm_min,
    )

    optimizer = None
    for k in range(max_iter):
        optimizer = AircraftDesignOptimizer(assumptions, prop_db)  # fresh Opti each pass
        optimizer.solve(verbose=False)

        m_new = optimizer.solved_mass_kg()
        delta = m_new - assumptions.aircraft_mass
        print(f"[mass iter {k:02d}]  "
              f"m_fixed={assumptions.aircraft_mass:.4f} kg   "
              f"m_model={m_new:.4f} kg   Δ={delta:+.2e} kg")

        if abs(delta) < tol_kg:
            print(f"Mass converged after {k + 1} iteration(s).")
            break

        assumptions.aircraft_mass = ((1.0 - damping) * assumptions.aircraft_mass
                                     + damping * m_new)
    else:
        print(f"WARNING: mass did not converge in {max_iter} iterations "
              f"(last |Δ| = {abs(delta):.2e} kg).")

    print_design_report(optimizer)
    airplane = optimizer.build_solved_airplane()
    airplane.draw_three_view(style="shaded", show=False)
    FIGURES_DIR.mkdir(exist_ok=True)
    plt.savefig(FIGURES_DIR / "optimized_aircraft_three_view.png",
                dpi=200, bbox_inches="tight")
    if show:
        plt.show()
    return optimizer


def main():
    run_design_optimization()


if __name__ == "__main__":
    main()
