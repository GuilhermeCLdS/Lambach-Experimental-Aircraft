"""Characterisation tests for the legacy single-file optimizer.

These pin down how the original (bug-fixed) optimizer.py behaves, so the
refactored modules can be checked against exactly the same expectations.
"""

import re
from types import SimpleNamespace

import aerosandbox as asb
import numpy as onp
import pytest

from conftest import REFERENCE_CASES, solve_quietly

RTOL_GOLDEN = 1e-5


# ---------------------------------------------------------------------------
# Module-level helpers
# ---------------------------------------------------------------------------

def test_radius_from_ellipse_centre_ends_and_outside(legacy):
    a, b = 0.2, 0.04
    assert float(legacy.radius_from_ellipse(0.0, a, b)) == pytest.approx(b)
    assert float(legacy.radius_from_ellipse(a, a, b)) == pytest.approx(0.0)
    assert float(legacy.radius_from_ellipse(-a, a, b)) == pytest.approx(0.0)
    assert float(legacy.radius_from_ellipse(1.5 * a, a, b)) == 0.0
    x = 0.12
    assert float(legacy.radius_from_ellipse(x, a, b)) == pytest.approx(b * onp.sqrt(1 - (x / a) ** 2))


def test_polar_to_cartesian(legacy):
    y, z = legacy.polar_to_cartesian(2.0, onp.pi / 4)
    assert float(y) == pytest.approx(onp.sqrt(2))
    assert float(z) == pytest.approx(onp.sqrt(2))
    y, z = legacy.polar_to_cartesian(1.0, -onp.pi / 2)
    assert float(y) == pytest.approx(0.0, abs=1e-12)
    assert float(z) == pytest.approx(-1.0)


# ---------------------------------------------------------------------------
# Propeller database
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def prop_db(legacy):
    cfg = legacy.Config()
    return legacy.PropellerPerformanceDatabase(cfg.PERF_FILE, 1.225, cfg.prop_dia, rpm_lo=cfg.rpm_min)


def _raw_block(path, rpm):
    rows, current = [], None
    for line in open(path, errors="ignore"):
        match = re.search(r"PROP RPM =\s*([\d.]+)", line)
        if match:
            current = float(match.group(1))
            continue
        parts = line.split()
        if current == rpm and len(parts) == 15:
            try:
                rows.append([float(v) for v in parts])
            except ValueError:
                pass
    return onp.asarray(rows)


def test_prop_db_rpm_range_and_shapes(prop_db):
    assert prop_db.rpms[0] == 15000
    assert prop_db.rpms[-1] == 40000
    assert onp.all(onp.diff(prop_db.rpms) > 0)
    assert prop_db.Ct_tab.shape == (len(prop_db.rpms), 89)
    assert prop_db.Cp_tab.shape == prop_db.Ct_tab.shape
    assert prop_db.J_grid[0] == 0.0
    assert prop_db.J_grid[-1] == pytest.approx(prop_db.J_tab_max)


def test_prop_db_table_reaches_zero_thrust(prop_db):
    """The extended table must cover J up to (almost) zero thrust for every RPM.

    Before the fix the shortest block capped J at 0.86, leaving a thrust floor.
    """
    assert prop_db.J_tab_max > 0.875
    assert onp.all(prop_db.Ct_tab[:, -1] < 0.005)
    assert onp.all(prop_db.Ct_tab[:, -1] > -0.002)
    assert onp.all(prop_db.Cp_tab > 0)


def test_prop_db_matches_raw_file_at_data_points(legacy, prop_db):
    cfg = legacy.Config()
    raw = _raw_block(cfg.PERF_FILE, 30000.0)
    j_raw, ct_raw, cp_raw = raw[:, 1], raw[:, 3], raw[:, 4]
    ct_tab = onp.interp(j_raw, prop_db.J_grid, prop_db.Ct_tab[prop_db.rpms == 30000][0])
    cp_tab = onp.interp(j_raw, prop_db.J_grid, prop_db.Cp_tab[prop_db.rpms == 30000][0])
    inside = j_raw <= prop_db.J_tab_max
    onp.testing.assert_allclose(ct_tab[inside], ct_raw[inside], atol=2e-3)
    onp.testing.assert_allclose(cp_tab[inside], cp_raw[inside], atol=2e-3)


def test_extend_to_zero_thrust(legacy):
    # Synthetic block: columns [V, J, Pe, Ct, Cp, ...]; Ct = 0.1 - 0.1 J
    j = onp.array([0.0, 0.2, 0.4, 0.6])
    block = onp.zeros((4, 15))
    block[:, 1], block[:, 3], block[:, 4] = j, 0.1 - 0.1 * j, 0.05 - 0.02 * j
    j_ext, ct_ext, cp_ext = legacy.PropellerPerformanceDatabase._extend_to_zero_thrust(block)
    assert j_ext[-1] == pytest.approx(1.0)
    assert ct_ext[-1] == 0.0
    assert cp_ext[-1] == pytest.approx(0.03)

    # Blocks that already reach zero thrust are left unchanged.
    block[:, 3] = 0.06 - 0.1 * j  # last point exactly at Ct = 0
    j_ext, ct_ext, _ = legacy.PropellerPerformanceDatabase._extend_to_zero_thrust(block)
    assert len(j_ext) == 4
    block[:, 3] = 0.05 - 0.1 * j  # last point already negative
    j_ext, ct_ext, _ = legacy.PropellerPerformanceDatabase._extend_to_zero_thrust(block)
    assert len(j_ext) == 4


def test_prop_db_evaluate_dimensional_relations(prop_db):
    rpm, v = 32000.0, 50.0
    thrust, power, torque, j = (float(x) for x in prop_db.evaluate(rpm, v))
    n, d = rpm / 60, prop_db.prop_dia
    assert j == pytest.approx(v / (n * d))
    assert torque == pytest.approx(power / (2 * onp.pi * n))
    ct = thrust / (1.225 * n ** 2 * d ** 4)
    cp = power / (1.225 * n ** 3 * d ** 5)
    assert 0 < ct < 0.2
    assert 0 < cp < 0.2


def test_prop_db_rejects_file_without_enough_blocks(legacy, tmp_path):
    bad = tmp_path / "bad.txt"
    bad.write_text("PROP RPM = 20000\n" + " ".join(["1.0"] * 15) + "\n")
    with pytest.raises(ValueError):
        legacy.PropellerPerformanceDatabase(bad, 1.225, 0.127, rpm_lo=15000)


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def _numeric_design(legacy, use_canards=True):
    cfg = legacy.Config()
    cfg.use_canards = use_canards
    return SimpleNamespace(
        cfg=cfg, fuse_len=0.30, fuse_dia=0.08, pylon_len=0.065, pylon_chord=0.075,
        pylon_inc=0.0, canard_len=0.10, canard_chord=0.06, canard_inc=0.0,
        l_p=0.20, l_c=0.05, l_cg=0.08, AOA=onp.radians(3.0),
    )


@pytest.mark.parametrize("use_canards, n_wings", [(True, 3), (False, 2)])
def test_airplane_components(legacy, use_canards, n_wings):
    design = _numeric_design(legacy, use_canards)
    airplane = legacy.AircraftAerodynamics(design).build_airplane()
    assert len(airplane.wings) == n_wings
    assert len(airplane.fuselages) == 1
    assert airplane.xyz_ref[0] == pytest.approx(design.l_cg - design.fuse_len / 2)


def test_fuselage_is_ellipsoid(legacy):
    design = _numeric_design(legacy)
    fuselage = legacy.AircraftAerodynamics(design).build_fuselage()
    xs = onp.array([x.xyz_c[0] for x in fuselage.xsecs])
    rs = onp.array([float(x.width) / 2 for x in fuselage.xsecs])
    assert len(xs) == 31
    assert xs[0] == pytest.approx(-0.15)
    assert xs[-1] == pytest.approx(0.15)
    assert rs[0] == pytest.approx(0.0, abs=1e-12)
    assert rs[-1] == pytest.approx(0.0, abs=1e-12)
    assert rs.max() == pytest.approx(0.04)
    onp.testing.assert_allclose(rs, 0.04 * onp.sqrt(onp.clip(1 - (xs / 0.15) ** 2, 0, None)), atol=1e-12)


def test_pylon_arms_form_an_x(legacy):
    design = _numeric_design(legacy)
    aero = legacy.AircraftAerodynamics(design)
    upper, lower = aero.build_wing_surfaces()[:2]
    for wing, sign in ((upper, 1), (lower, -1)):
        root = onp.asarray(wing.xsecs[0].xyz_le, dtype=float)
        tip = onp.asarray(wing.xsecs[1].xyz_le, dtype=float)
        assert wing.symmetric
        assert root[1] == pytest.approx(sign * root[2])  # 45 degrees
        span = onp.hypot(tip[1] - root[1], tip[2] - root[2])
        assert span == pytest.approx(design.pylon_len)
        # Root lies on the fuselage surface at the pylon leading edge.
        x_le = -design.fuse_len / 2 + design.l_p - 0.25 * design.pylon_chord
        r_expected = float(legacy.radius_from_ellipse(x_le, design.fuse_len / 2, design.fuse_dia / 2))
        assert onp.hypot(root[1], root[2]) == pytest.approx(r_expected)
        assert root[0] == pytest.approx(x_le)


def test_aerobuildup_on_numeric_design_is_physical(legacy):
    design = _numeric_design(legacy)
    _, aero = legacy.AircraftAerodynamics(design).analyze(legacy.Config().velocity)
    assert _f(aero["L"]) > 0
    assert _f(aero["D"]) > 0
    assert _f(aero["D"]) == pytest.approx(_f(aero["D_profile"]) + _f(aero["D_induced"]), rel=1e-6)
    assert _f(aero["CLa"]) > 0


# ---------------------------------------------------------------------------
# Solved baseline: physics invariants
# ---------------------------------------------------------------------------

def _f(x):
    """Scalar float from a float, 0-d or 1-element array."""
    return float(onp.asarray(x, dtype=float).reshape(-1)[0])


def _v(opt, x):
    return float(opt.sol.value(x))


def test_baseline_force_and_moment_balance(legacy_baseline):
    opt, cfg = legacy_baseline, legacy_baseline.cfg
    assert _v(opt, opt.lift_tot) == pytest.approx(cfg.aircraft_weight, rel=1e-6)
    assert _v(opt, opt.thrust_along_velocity) == pytest.approx(_v(opt, opt.total_drag), rel=1e-6)
    assert abs(_v(opt, opt.moment_about_cg)) < 1e-8


def test_baseline_constraints_hold(legacy_baseline):
    opt, cfg, prop = legacy_baseline, legacy_baseline.cfg, legacy_baseline.propulsion
    v = lambda x: _v(opt, x)  # noqa: E731
    tol = 1e-7
    assert v(opt.x_np) - v(opt.l_cg) >= cfg.static_margin_frac * v(opt.fuse_len) - tol
    assert v(opt.l_cg) == pytest.approx(v(opt.l_p) - cfg.cg_offset)
    assert v(opt.pylon_len) == pytest.approx(opt.forced_pylon_len)
    for j in (prop.J_top, prop.J_bottom):
        assert 0 <= v(j) <= opt.prop_db.J_tab_max + tol
    for p in (prop.P_top, prop.P_bottom):
        assert v(p) <= cfg.P_motor_max + tol
    for m in (prop.M_tip_top, prop.M_tip_bottom):
        assert v(m) <= cfg.M_tip_max + tol
    assert abs(v(prop.delta_T)) <= cfg.max_diff_frac * v(prop.thrust_tot) + tol
    assert abs(v(opt.alpha_p)) <= onp.radians(cfg.alpha_stall_deg) + tol
    a = v(opt.fuse_len) / 2
    dia_at_station = v(opt.fuse_dia) * onp.sqrt(1 - ((opt.l_min_dia - a) / a) ** 2)
    assert dia_at_station >= opt.fuse_dia_min - tol


def test_baseline_thrust_moment_by_hand(legacy_baseline):
    """r x F for the four thrust vectors, recomputed independently."""
    opt, prop = legacy_baseline, legacy_baseline.propulsion
    v = lambda x: _v(opt, x)  # noqa: E731
    inc, z = v(opt.pylon_inc), v(opt.prop_z_arm)
    dx = v(opt.l_p) - v(opt.l_cg)
    m = 0.0
    for z_i, t_i in ((z, 2 * v(prop.T_top)), (-z, 2 * v(prop.T_bottom))):
        f_x, f_z = -t_i * onp.cos(inc), t_i * onp.sin(inc)  # geometry axes
        m += z_i * f_x - dx * f_z  # M_y = z F_x - x F_z
    assert v(opt.moment_thrust) == pytest.approx(m, rel=1e-9)


def test_baseline_differential_thrust_trims_nose_down_aero_moment(legacy_baseline):
    opt = legacy_baseline
    assert _v(opt, opt.moment_aero) < 0
    assert _v(opt, opt.propulsion.delta_T) < 0  # lower pair pushes harder -> nose up


def test_build_solved_airplane(legacy, legacy_baseline):
    airplane = legacy_baseline.build_solved_airplane()
    assert isinstance(airplane, asb.Airplane)
    assert len(airplane.wings) == 3
    unsolved = legacy.AircraftDesignOptimizer(legacy.Config())
    with pytest.raises(RuntimeError):
        unsolved.build_solved_airplane()


# ---------------------------------------------------------------------------
# Golden regression against reference_results.json
# ---------------------------------------------------------------------------

def solve_legacy_case(legacy, overrides):
    cfg = legacy.Config()
    for key, value in overrides.items():
        setattr(cfg, key, value)
    optimizer = legacy.AircraftDesignOptimizer(cfg)
    optimizer.sol = solve_quietly(optimizer.opti)
    return optimizer


@pytest.mark.parametrize("case", list(REFERENCE_CASES))
def test_legacy_matches_reference(legacy, reference, case):
    opt = solve_legacy_case(legacy, REFERENCE_CASES[case])
    expected = reference[case]
    for group, owner in (("design", opt), ("outputs", opt), ("propulsion", opt.propulsion)):
        for key, value in expected[group].items():
            got = _v(opt, getattr(owner, key))
            assert got == pytest.approx(value, rel=RTOL_GOLDEN, abs=1e-9), f"{case}.{group}.{key}"
