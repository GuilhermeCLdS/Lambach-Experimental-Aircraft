"""Integrates the src modules into one run."""

from pathlib import Path

import matplotlib.pyplot as plt

from src.global_parameters.global_parameters import Assumptions, Parameters
from src.optimization.modules.aircraft_design_optimizer import AircraftDesignOptimizer
from src.optimization.modules.print_design_report import print_design_report
from src.optimization.modules.propeller_performance_database import PropellerPerformanceDatabase

FIGURES_DIR = Path(__file__).resolve().parent / "figures"


def run_design_optimization(assumptions=None, show=True):
    """Optimize the aircraft, print the report and save a three-view figure."""
    assumptions = assumptions or Assumptions()

    prop_db = PropellerPerformanceDatabase(
        assumptions.perf_file,
        Parameters.AIR_DENSITY_SEA_LEVEL,
        assumptions.prop_dia,
        rpm_lo=assumptions.rpm_min,
    )
    optimizer = AircraftDesignOptimizer(assumptions, prop_db)
    optimizer.solve()
    print_design_report(optimizer)

    airplane = optimizer.build_solved_airplane()
    airplane.draw_three_view(style="shaded", show=False)
    FIGURES_DIR.mkdir(exist_ok=True)
    plt.savefig(FIGURES_DIR / "optimized_aircraft_three_view.png", dpi=200, bbox_inches="tight")
    if show:
        plt.show()
    return optimizer


def main():
    run_design_optimization()


if __name__ == "__main__":
    main()
