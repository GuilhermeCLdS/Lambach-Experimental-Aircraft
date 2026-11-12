"""Old vs new: the refactored src/optimization must reproduce the legacy code.

1. With the legacy physical constants patched in (g = 9.81, AeroSandbox ISA
   rho and a), the refactor must match reference_results.json to solver
   precision: same problem, same answer.
2. With the new global Parameters (g = G0 = 9.80665, rho = 1.225), the only
   change is ~0.035 % in weight, so results must agree within a small tolerance.
3. Component-level checks: identical propeller tables, identical AeroBuildup
   results for the same numeric design.
"""

from types import SimpleNamespace

import aerosandbox as asb
import numpy as onp
import pytest

from conftest import REFERENCE_CASES, refactored_results, solve_refactored_case
from src.global_parameters.global_parameters import Assumptions, Parameters
from src.optimization.modules.aircraft_aerodynamics import AircraftAerodynamics
from src.optimization.modules.propeller_performance_database import PropellerPerformanceDatabase

LEGACY_ATMOSPHERE = asb.Atmosphere(altitude=0)
LEGACY_RHO = float(LEGACY_ATMOSPHERE.density())
LEGACY_A = float(LEGACY_ATMOSPHERE.speed_of_sound())

RTOL_SAME_CONSTANTS = 1e-5
# Weight changes by 0.035 %; every dimensional output moves by < 0.07 %.
# Near-zero angles/moments (|x| ~ 1e-3) need the absolute tolerance instead:
# their largest change is 1.5e-5 rad (0.0009 deg).
RTOL_NEW_CONSTANTS = 1e-3
ATOL_NEW_CONSTANTS = 5e-5


def _compare(got, expected, rtol, atol):
    mismatches = []
    for group in ("design", "outputs", "propulsion"):
        for key, ref in expected[group].items():
            if got[group][key] != pytest.approx(ref, rel=rtol, abs=atol):
                mismatches.append(f"{group}.{key}: new={got[group][key]!r} legacy={ref!r}")
    return mismatches


@pytest.fixture
def legacy_constants(monkeypatch):
    monkeypatch.setattr(Parameters, "G0", 9.81)
    monkeypatch.setattr(Parameters, "AIR_DENSITY_SEA_LEVEL", LEGACY_RHO)
    monkeypatch.setattr(Parameters, "speed_of_sound_sea_level", classmethod(lambda cls: LEGACY_A))


@pytest.mark.parametrize("case", list(REFERENCE_CASES))
def test_same_constants_reproduce_legacy(legacy_constants, reference, case):
    got = refactored_results(solve_refactored_case(REFERENCE_CASES[case], rho=LEGACY_RHO))
    assert got["weight"] == reference[case]["weight"]
    assert got["J_tab_max"] == reference[case]["J_tab_max"]
    assert _compare(got, reference[case], RTOL_SAME_CONSTANTS, 1e-9) == []


@pytest.mark.parametrize("case", list(REFERENCE_CASES))
def test_global_parameters_change_results_only_slightly(reference, case):
    got = refactored_results(solve_refactored_case(REFERENCE_CASES[case]))
    assert got["weight"] == pytest.approx(reference[case]["weight"], rel=4e-4)
    assert _compare(got, reference[case], RTOL_NEW_CONSTANTS, ATOL_NEW_CONSTANTS) == []


def test_propeller_tables_identical(legacy):
    cfg, asm = legacy.Config(), Assumptions()
    old = legacy.PropellerPerformanceDatabase(cfg.PERF_FILE, 1.225, cfg.prop_dia, rpm_lo=cfg.rpm_min)
    new = PropellerPerformanceDatabase(asm.perf_file, 1.225, asm.prop_dia, rpm_lo=asm.rpm_min)
    onp.testing.assert_array_equal(old.rpms, new.rpms)
    onp.testing.assert_array_equal(old.J_grid, new.J_grid)
    onp.testing.assert_array_equal(old.Ct_tab, new.Ct_tab)
    onp.testing.assert_array_equal(old.Cp_tab, new.Cp_tab)
    for rpm, v in ((20000, 30.0), (31000, 55.0), (38000, 55.0)):
        for a, b in zip(old.evaluate(rpm, v), new.evaluate(rpm, v)):
            assert float(onp.ravel(a)[0]) == float(onp.ravel(b)[0])


@pytest.mark.parametrize("use_canards", [True, False])
def test_aerobuildup_identical(legacy, use_canards):
    values = dict(
        fuse_len=0.30, fuse_dia=0.08, pylon_len=0.065, pylon_chord=0.075, pylon_inc=0.02,
        canard_len=0.10, canard_chord=0.06, canard_inc=-0.01, l_p=0.20, l_c=0.05, l_cg=0.08,
        AOA=onp.radians(3.0),
    )
    cfg = legacy.Config()
    cfg.use_canards = use_canards
    _, old = legacy.AircraftAerodynamics(SimpleNamespace(cfg=cfg, **values)).analyze(cfg.velocity)
    _, new = AircraftAerodynamics(SimpleNamespace(**values), use_canards).analyze(cfg.velocity)
    for key in ("L", "D", "D_profile", "D_induced", "m_b", "CLa", "Cma", "x_np"):
        assert float(onp.ravel(new[key])[0]) == float(onp.ravel(old[key])[0]), key
