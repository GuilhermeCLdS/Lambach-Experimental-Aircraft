"""Unit tests for src/optimization/helpers."""

import aerosandbox as asb
import numpy as onp
import pytest

from src.optimization.helpers.extend_to_zero_thrust import extend_to_zero_thrust
from src.optimization.helpers.polar_to_cartesian import polar_to_cartesian
from src.optimization.helpers.radius_from_ellipse import radius_from_ellipse
from src.optimization.helpers.read_apc_performance_blocks import read_apc_performance_blocks
from src.optimization.helpers.thrust_force_components import thrust_force_components
from src.optimization.helpers.thrust_pitching_moment import thrust_pitching_moment
from src.global_parameters.global_parameters import Assumptions


def test_radius_from_ellipse():
    a, b = 0.2, 0.04
    assert float(radius_from_ellipse(0.0, a, b)) == pytest.approx(b)
    assert float(radius_from_ellipse(a, a, b)) == pytest.approx(0.0)
    assert float(radius_from_ellipse(-a, a, b)) == pytest.approx(0.0)
    assert float(radius_from_ellipse(1.5 * a, a, b)) == 0.0
    assert float(radius_from_ellipse(0.12, a, b)) == pytest.approx(b * onp.sqrt(1 - 0.6 ** 2))


def test_polar_to_cartesian():
    y, z = polar_to_cartesian(2.0, onp.pi / 4)
    assert (float(y), float(z)) == pytest.approx((onp.sqrt(2), onp.sqrt(2)))
    y, z = polar_to_cartesian(1.0, -onp.pi / 2)
    assert (float(y), float(z)) == pytest.approx((0.0, -1.0), abs=1e-12)


def test_read_apc_performance_blocks():
    blocks = read_apc_performance_blocks(Assumptions().perf_file)
    assert sorted(blocks)[0] == 1000 and sorted(blocks)[-1] == 40000
    assert len(blocks) == 40
    assert all(rows.shape[1] == 15 for rows in blocks.values())
    first = blocks[1000.0][0]
    assert first[1] == 0.0 and first[3] == pytest.approx(0.2149) and first[4] == pytest.approx(0.1602)


def test_read_apc_performance_blocks_skips_non_numeric_rows(tmp_path):
    f = tmp_path / "p.txt"
    f.write_text("PROP RPM = 5000\n" + " ".join(["x"] * 15) + "\n" + " ".join(["2"] * 15) + "\n")
    blocks = read_apc_performance_blocks(f)
    assert list(blocks) == [5000.0]
    assert blocks[5000.0].shape == (1, 15)


def _block(ct_of_j):
    j = onp.array([0.0, 0.2, 0.4, 0.6])
    block = onp.zeros((4, 15))
    block[:, 1], block[:, 3], block[:, 4] = j, ct_of_j(j), 0.05 - 0.02 * j
    return block


def test_extend_to_zero_thrust_appends_zero_point():
    j, ct, cp = extend_to_zero_thrust(_block(lambda j: 0.1 - 0.1 * j))
    assert j[-1] == pytest.approx(1.0) and ct[-1] == 0.0 and cp[-1] == pytest.approx(0.03)


@pytest.mark.parametrize("ct_of_j", [lambda j: 0.06 - 0.1 * j, lambda j: 0.05 - 0.1 * j, lambda j: 0.01 + 0.1 * j])
def test_extend_to_zero_thrust_leaves_block_unchanged(ct_of_j):
    j, _, _ = extend_to_zero_thrust(_block(ct_of_j))
    assert len(j) == 4


@pytest.mark.parametrize("alpha_deg", [0.0, 4.0, 10.0])
@pytest.mark.parametrize("incidence_deg", [-5.0, 0.0, 3.0])
def test_thrust_force_components(alpha_deg, incidence_deg):
    op = asb.OperatingPoint(velocity=50, alpha=alpha_deg)
    lift, along = thrust_force_components(2.0, onp.radians(incidence_deg), op)
    axis = onp.radians(alpha_deg + incidence_deg)  # thrust axis to freestream
    assert float(along) == pytest.approx(2.0 * onp.cos(axis))
    assert float(lift) == pytest.approx(2.0 * onp.sin(axis))


def test_thrust_pitching_moment_matches_r_cross_f():
    op = asb.OperatingPoint(velocity=50, alpha=3.0)
    t_top, t_bot, inc, z, dx = 0.4, 0.9, onp.radians(2.0), 0.07, 0.15
    total, diff, incid = thrust_pitching_moment(
        delta_thrust=2 * (t_top - t_bot), thrust_total=2 * (t_top + t_bot),
        incidence=inc, z_prop=z, dx_prop_from_cg=dx, op_point=op,
    )
    expected = 0.0
    for z_i, t_i in ((z, 2 * t_top), (-z, 2 * t_bot)):
        expected += z_i * (-t_i * onp.cos(inc)) - dx * (t_i * onp.sin(inc))
    assert float(total) == pytest.approx(expected)
    assert float(diff) + float(incid) == pytest.approx(expected)
    assert float(total) > 0  # lower pair pushing harder -> nose up
