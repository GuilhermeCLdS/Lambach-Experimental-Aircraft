import aerosandbox.numpy as np
import numpy as onp

from src.optimization.helpers.extend_to_zero_thrust import extend_to_zero_thrust
from src.optimization.helpers.read_apc_performance_blocks import read_apc_performance_blocks

N_J_POINTS = 89


class PropellerPerformanceDatabase:
    """Ct(RPM, J) / Cp(RPM, J) tables read from an APC PER3 file.

    Every RPM block at or above ``rpm_lo`` is extended to zero thrust and
    resampled onto one shared J grid, so the tables can be interpolated with a
    smooth (differentiable) B-spline inside the optimizer.
    """

    def __init__(self, path, rho, prop_dia, rpm_lo=10000):
        self.rho = rho
        self.prop_dia = prop_dia
        self.rpms, self.J_grid, self.Ct_tab, self.Cp_tab = self._build_tables(path, rpm_lo)
        self.J_tab_max = float(self.J_grid[-1])

    @staticmethod
    def _build_tables(path, rpm_lo):
        blocks = read_apc_performance_blocks(path)
        valid_blocks = {
            rpm: rows for rpm, rows in blocks.items()
            if rpm >= rpm_lo and len(rows) >= 2
        }

        rpms = onp.asarray(sorted(valid_blocks), dtype=float)
        if len(rpms) < 2:
            raise ValueError(
                f"The propeller file '{path}' must contain at least two valid RPM blocks "
                f"at or above {rpm_lo} RPM."
            )

        extended = {rpm: extend_to_zero_thrust(valid_blocks[rpm]) for rpm in rpms}

        j_max = min(j[-1] for j, _, _ in extended.values())
        j_max = onp.floor(j_max * 1000) / 1000
        if j_max <= 0:
            raise ValueError(f"No positive advance-ratio range found in '{path}'.")

        j_grid = onp.linspace(0.0, j_max, N_J_POINTS)
        ct_table = onp.asarray([onp.interp(j_grid, extended[rpm][0], extended[rpm][1]) for rpm in rpms])
        cp_table = onp.asarray([onp.interp(j_grid, extended[rpm][0], extended[rpm][2]) for rpm in rpms])
        return rpms, j_grid, ct_table, cp_table

    def evaluate(self, rpm, axial_velocity):
        """Return thrust [N], shaft power [W], torque [Nm] and advance ratio J."""
        revolutions_per_second = rpm / 60
        advance_ratio = axial_velocity / (revolutions_per_second * self.prop_dia)
        query = np.array([rpm, advance_ratio])

        ct = np.interpn(
            (self.rpms, self.J_grid), self.Ct_tab, query,
            method="bspline", bounds_error=False, fill_value=None,
        )
        cp = np.interpn(
            (self.rpms, self.J_grid), self.Cp_tab, query,
            method="bspline", bounds_error=False, fill_value=None,
        )

        thrust = ct * self.rho * revolutions_per_second ** 2 * self.prop_dia ** 4
        power = cp * self.rho * revolutions_per_second ** 3 * self.prop_dia ** 5
        torque = power / (2 * np.pi * revolutions_per_second)
        return thrust, power, torque, advance_ratio
