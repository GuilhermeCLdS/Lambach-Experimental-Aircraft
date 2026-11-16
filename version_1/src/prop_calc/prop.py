"""Propeller model backed by APC PER3 performance data.

Each Prop instance stores tabulated thrust/power coefficients (Ct, Cp) versus
advance ratio J at multiple RPM sections. Coefficients are interpolated in J
within each RPM section, then linearly between adjacent RPM sections.
"""

from __future__ import annotations

import numpy as np

IN2M = 0.0254  # inches to meters


class Prop:
    """A single propeller with APC PER3 performance tables.

    Attributes:
        label: Propeller name from the PER3 file header (e.g. "5x5F").
        d_in: Diameter in inches.
        p_in: Pitch in inches.
        blades: Blade count.
        D: Diameter in meters.
        rpms: Sorted array of RPM values available in the data table.
        rpm_min: Minimum tabulated RPM.
        rpm_max: Maximum tabulated RPM.
        rpm_table_tol: Multiplier allowing evaluation slightly above rpm_max.
    """

    def __init__(
        self,
        label: str,
        d_in: float,
        p_in: float,
        blades: int,
        sections: dict[int, dict[str, np.ndarray]],
        rpm_table_tol: float = 1.01,
    ) -> None:
        self.label = label
        self.d_in = d_in
        self.p_in = p_in
        self.blades = blades
        self.D = d_in * IN2M  # meters

        # Store RPM sections in ascending order for interpolation.
        self.rpms = np.array(sorted(sections))
        self._sections = [sections[rpm] for rpm in self.rpms]

        self.rpm_min = float(self.rpms[0])
        self.rpm_max = float(self.rpms[-1])
        self.rpm_table_tol = rpm_table_tol

    def _coefficients(self, J: float, n_rps: float) -> tuple[float, float]:
        """Return (Ct, Cp) interpolated at advance ratio J and rotational speed n_rps [rev/s].

        - RPM is clamped to the tabulated range for coefficient lookup.
        - J interpolation: Ct returns 0 beyond the table's J range (tables end near zero thrust).
          Cp holds its last tabulated value beyond the range.
        """
        rpm = float(np.clip(n_rps * 60.0, self.rpm_min, self.rpm_max))

        # Find bracketing RPM sections.
        idx = int(np.searchsorted(self.rpms, rpm))
        if idx == 0:
            idx = 1
        lo = self._sections[idx - 1]
        hi = self._sections[idx]

        # Linear interpolation weight between the two RPM sections.
        rpm_lo = self.rpms[idx - 1]
        rpm_hi = self.rpms[idx]
        w = (rpm - rpm_lo) / (rpm_hi - rpm_lo) if rpm_hi != rpm_lo else 0.0

        ct = (1.0 - w) * np.interp(J, lo["J"], lo["Ct"], right=0.0) + w * np.interp(
            J, hi["J"], hi["Ct"], right=0.0
        )
        cp = (1.0 - w) * np.interp(J, lo["J"], lo["Cp"]) + w * np.interp(
            J, hi["J"], hi["Cp"]
        )
        return max(float(ct), 0.0), max(float(cp), 0.0)

    def ct(self, J: float, n_rps: float) -> float:
        """Thrust coefficient Ct at advance ratio J and speed n_rps [rev/s]."""
        return self._coefficients(J, n_rps)[0]

    def cp(self, J: float, n_rps: float) -> float:
        """Power coefficient Cp at advance ratio J and speed n_rps [rev/s]."""
        return self._coefficients(J, n_rps)[1]

    def n_tab_max(self) -> float:
        """Maximum allowed rotational speed [rev/s], including tolerance."""
        return self.rpm_max / 60.0 * self.rpm_table_tol

    def n_min(self) -> float:
        """Minimum tabulated rotational speed [rev/s]."""
        return self.rpm_min / 60.0
