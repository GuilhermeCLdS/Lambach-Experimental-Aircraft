"""Tests for src/optimization/modules and the global Assumptions."""

from types import SimpleNamespace

import aerosandbox as asb
import numpy as onp
import pytest

from src.global_parameters.global_parameters import Assumptions, Parameters
from src.optimization.helpers.radius_from_ellipse import radius_from_ellipse
from src.optimization.modules.aircraft_aerodynamics import AircraftAerodynamics
from src.optimization.modules.aircraft_design_optimizer import AircraftDesignOptimizer
from src.optimization.modules.propeller_performance_database import PropellerPerformanceDatabase
from src.optimization.modules.propulsion_system import PropulsionSystem


def _f(x):
    return float(onp.asarray(x, dtype=float).reshape(-1)[0])


# ---------------- global parameters ----------------

def test_assumptions_derived_values():
    asm = Assumptions()
    assert asm.aircraft_weight == pytest.approx(0.85 * Parameters.G0)
    assert asm.prop_radius == pytest.approx(0.0635)
    assert asm.perf_file.exists()
    assert Parameters.speed_of_sound_sea_level() == pytest.approx(340.29, abs=0.01)


# ---------------- propeller database ----------------

@pytest.fixture(scope="module")
def prop_db():
    asm = Assumptions()
    return PropellerPerformanceDatabase(asm.perf_file, 1.225, asm.prop_dia, rpm_lo=asm.rpm_min)


def test_prop_db_table(prop_db):
    assert (prop_db.rpms[0], prop_db.rpms[-1]) == (15000, 40000)
    assert prop_db.Ct_tab.shape == prop_db.Cp_tab.shape == (26, 89)
    assert prop_db.J_tab_max > 0.875
    assert onp.all(onp.abs(prop_db.Ct_tab[:, -1]) < 0.005)
    assert onp.all(prop_db.Cp_tab > 0)


def test_prop_db_evaluate(prop_db):
    rpm, v = 32000.0, 50.0
    thrust, power, torque, j = (_f(x) for x in prop_db.evaluate(rpm, v))
    n, d = rpm / 60, prop_db.prop_dia
    assert j == pytest.approx(v / (n * d))
    assert torque == pytest.approx(power / (2 * onp.pi * n))
    assert thrust > 0 and power > 0


def test_prop_db_rejects_bad_file(tmp_path):
    bad = tmp_path / "bad.txt"
    bad.write_text("PROP RPM = 20000\n" + " ".join(["1.0"] * 15) + "\n")
    with pytest.raises(ValueError):
        PropellerPerformanceDatabase(bad, 1.225, 0.127, rpm_lo=15000)


# ---------------- propulsion system ----------------

@pytest.mark.parametrize("diff_thrust", [True, False])
def test_propulsion_system(prop_db, diff_thrust):
    asm = Assumptions()
    asm.use_diff_thrust = diff_thrust
    opti = asb.Opti()
    prop = PropulsionSystem(opti, asm, 340.0, asm.velocity, 0.05, -0.01, prop_db)
    assert (prop.rpm_bottom is prop.rpm_top) == (not diff_thrust)
    x0 = opti.initial()
    v = lambda e: float(opti.value(e, x0))  # noqa: E731
    assert v(prop.thrust_tot) == pytest.approx(2 * v(prop.T_top) + 2 * v(prop.T_bottom))
    assert v(prop.V_ax) == pytest.approx(asm.velocity * onp.cos(0.04))
    omega = v(prop.rpm_top) / 60 * 2 * onp.pi
    assert v(prop.M_tip_top) == pytest.approx(onp.hypot(omega * asm.prop_radius, v(prop.V_ax)) / 340.0)


def test_propulsion_rejects_disjoint_rpm_range(prop_db):
    asm = Assumptions()
    asm.rpm_min, asm.rpm_max = 50000, 60000
    with pytest.raises(ValueError):
        PropulsionSystem(asb.Opti(), asm, 340.0, asm.velocity, 0.0, 0.0, prop_db)


# ---------------- geometry ----------------

def _numeric_design():
    return SimpleNamespace(
        fuse_len=0.30, fuse_dia=0.08, pylon_len=0.065, pylon_chord=0.075, pylon_inc=0.0,
        canard_len=0.10, canard_chord=0.06, canard_inc=0.0, l_p=0.20, l_c=0.05, l_cg=0.08,
        AOA=onp.radians(3.0),
    )


@pytest.mark.parametrize("use_canards, n_wings", [(True, 3), (False, 2)])
def test_airplane_components(use_canards, n_wings):
    design = _numeric_design()
    airplane = AircraftAerodynamics(design, use_canards).build_airplane()
    assert len(airplane.wings) == n_wings
    assert airplane.xyz_ref[0] == pytest.approx(design.l_cg - design.fuse_len / 2)


def test_fuselage_is_ellipsoid():
    fuselage = AircraftAerodynamics(_numeric_design(), True).build_fuselage()
    xs = onp.array([x.xyz_c[0] for x in fuselage.xsecs])
    rs = onp.array([float(x.width) / 2 for x in fuselage.xsecs])
    assert len(xs) == 31 and xs[0] == pytest.approx(-0.15) and xs[-1] == pytest.approx(0.15)
    onp.testing.assert_allclose(rs, 0.04 * onp.sqrt(onp.clip(1 - (xs / 0.15) ** 2, 0, None)), atol=1e-12)


def test_pylon_arms_form_an_x():
    design = _numeric_design()
    upper, lower = AircraftAerodynamics(design, True).build_wing_surfaces()[:2]
    x_le = -design.fuse_len / 2 + design.l_p - 0.25 * design.pylon_chord
    r_root = float(radius_from_ellipse(x_le, design.fuse_len / 2, design.fuse_dia / 2))
    for wing, sign in ((upper, 1), (lower, -1)):
        root = onp.asarray(wing.xsecs[0].xyz_le, dtype=float)
        tip = onp.asarray(wing.xsecs[1].xyz_le, dtype=float)
        assert root[1] == pytest.approx(sign * root[2])
        assert onp.hypot(*(tip - root)[1:]) == pytest.approx(design.pylon_len)
        assert onp.hypot(root[1], root[2]) == pytest.approx(r_root)
        assert root[0] == pytest.approx(x_le)


def test_aerobuildup_on_numeric_design():
    _, aero = AircraftAerodynamics(_numeric_design(), True).analyze(Assumptions().velocity)
    assert _f(aero["L"]) > 0 and _f(aero["D"]) > 0 and _f(aero["CLa"]) > 0
    assert _f(aero["D"]) == pytest.approx(_f(aero["D_profile"]) + _f(aero["D_induced"]), rel=1e-6)


# ---------------- solved optimizer: physics invariants ----------------

def test_force_and_moment_balance(refactored_baseline):
    opt = refactored_baseline
    assert opt.value(opt.lift_tot) == pytest.approx(opt.assumptions.aircraft_weight, rel=1e-6)
    assert opt.value(opt.thrust_along_velocity) == pytest.approx(opt.value(opt.total_drag), rel=1e-6)
    assert abs(opt.value(opt.moment_about_cg)) < 1e-8


def test_constraints_hold(refactored_baseline):
    opt, asm = refactored_baseline, refactored_baseline.assumptions
    d, prop, v, tol = opt.design, opt.propulsion, opt.value, 1e-7
    assert v(opt.x_np) - v(d.l_cg) >= asm.static_margin_frac * v(d.fuse_len) - tol
    assert v(d.l_cg) == pytest.approx(v(d.l_p) - asm.cg_offset)
    assert v(d.pylon_len) == pytest.approx(asm.forced_pylon_len)
    assert v(d.l_t) >= asm.prop_dia + asm.prop_spacing_gap - tol
    for j in (prop.J_top, prop.J_bottom):
        assert 0 <= v(j) <= opt.prop_db.J_tab_max + tol
    for p, m in ((prop.P_top, prop.M_tip_top), (prop.P_bottom, prop.M_tip_bottom)):
        assert v(p) <= asm.P_motor_max + tol and v(m) <= asm.M_tip_max + tol
    assert abs(v(prop.delta_T)) <= asm.max_diff_frac * v(prop.thrust_tot) + tol
    a = v(d.fuse_len) / 2
    assert v(d.fuse_dia) * onp.sqrt(1 - ((asm.l_min_dia - a) / a) ** 2) >= asm.fuse_dia_min - tol


def test_solved_airplane(refactored_baseline):
    airplane = refactored_baseline.build_solved_airplane()
    assert isinstance(airplane, asb.Airplane) and len(airplane.wings) == 3
    asm = Assumptions()
    unsolved = AircraftDesignOptimizer(asm, PropellerPerformanceDatabase(asm.perf_file, 1.225, asm.prop_dia, asm.rpm_min))
    with pytest.raises(RuntimeError):
        unsolved.build_solved_airplane()


def test_print_design_report(refactored_baseline, capsys):
    from src.optimization.modules.print_design_report import print_design_report
    print_design_report(refactored_baseline)
    out = capsys.readouterr().out
    assert "Total aerodynamic drag:  1.308 N" in out
    assert "Canard length" in out
