"""Shared fixtures for the optimization tests.

The legacy single-file optimizer is loaded straight from its file path so the
same tests work while it lives in ``src/Dev/`` and after it moves to
``src/deprecated/``.
"""

import importlib.util
import json
import os
import sys
from pathlib import Path

import pytest

TESTS_DIR = Path(__file__).resolve().parent
VERSION_DIR = TESTS_DIR.parents[1]
SRC_DIR = VERSION_DIR / "src"
REFERENCE_FILE = TESTS_DIR / "reference_results.json"

LEGACY_CANDIDATES = (
    SRC_DIR / "deprecated" / "optimizer.py",
    SRC_DIR / "Dev" / "optimizer.py",
)

# Configurations covered by the golden reference file. Each entry maps a case
# name to Config attribute overrides.
REFERENCE_CASES = {
    "baseline": {},
    "no_canards": {"use_canards": False},
    "no_diff_thrust": {"use_diff_thrust": False},
    "all_free": {"fix_l_cg": False, "fix_pylon_len": False, "fix_fuse_dia": False},
    "climb_allowed": {"steady_level": False},
}

if str(VERSION_DIR) not in sys.path:
    sys.path.insert(0, str(VERSION_DIR))


def load_legacy_module():
    """Import the legacy optimizer.py from wherever it currently lives.

    Set LEGACY_OPTIMIZER=/path/to/optimizer.py to test another copy.
    """
    override = os.environ.get("LEGACY_OPTIMIZER")
    for path in ((Path(override),) if override else LEGACY_CANDIDATES):
        if path.exists():
            spec = importlib.util.spec_from_file_location("legacy_optimizer", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    raise FileNotFoundError("legacy optimizer.py not found in src/Dev or src/deprecated")


def solve_quietly(opti):
    """Solve an asb.Opti without IPOPT's console output."""
    return opti.solve(verbose=False)


@pytest.fixture(scope="session")
def legacy():
    return load_legacy_module()


@pytest.fixture(scope="session")
def reference():
    with open(REFERENCE_FILE) as file:
        return json.load(file)


@pytest.fixture(scope="session")
def legacy_baseline(legacy):
    """Solved legacy optimizer with the default configuration."""
    optimizer = legacy.AircraftDesignOptimizer(legacy.Config())
    optimizer.sol = solve_quietly(optimizer.opti)
    return optimizer


# ---------------------------------------------------------------------------
# Refactored code (src/optimization)
# ---------------------------------------------------------------------------

def solve_refactored_case(overrides=None, rho=None):
    """Build and solve the refactored optimizer with Assumptions overrides."""
    from src.global_parameters.global_parameters import Assumptions, Parameters
    from src.optimization.modules.aircraft_design_optimizer import AircraftDesignOptimizer
    from src.optimization.modules.propeller_performance_database import PropellerPerformanceDatabase

    assumptions = Assumptions()
    for key, value in (overrides or {}).items():
        setattr(assumptions, key, value)
    prop_db = PropellerPerformanceDatabase(
        assumptions.perf_file,
        Parameters.AIR_DENSITY_SEA_LEVEL if rho is None else rho,
        assumptions.prop_dia,
        rpm_lo=assumptions.rpm_min,
    )
    optimizer = AircraftDesignOptimizer(assumptions, prop_db)
    optimizer.solve(verbose=False)
    return optimizer


def refactored_results(optimizer):
    """Same structure as reference_results.json, from the refactored optimizer."""
    from make_reference_results import DESIGN_KEYS, OUTPUT_KEYS, PROPULSION_KEYS

    v = optimizer.value
    return {
        "design": {k: v(getattr(optimizer.design, k)) for k in DESIGN_KEYS},
        "outputs": {k: v(getattr(optimizer, k)) for k in OUTPUT_KEYS},
        "propulsion": {k: v(getattr(optimizer.propulsion, k)) for k in PROPULSION_KEYS},
        "weight": float(optimizer.assumptions.aircraft_weight),
        "J_tab_max": float(optimizer.prop_db.J_tab_max),
    }


@pytest.fixture(scope="session")
def refactored_baseline():
    return solve_refactored_case()
