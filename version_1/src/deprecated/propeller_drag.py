"""Blade-element strip integration for a single propeller."""
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import make_interp_spline

INCH = 0.0254


@dataclass
class PropellerResult:
    D: float             # total drag force on one propeller [N]
    D_axial: float       # axial component  [N]
    Q: float             # torque [Nm]
    alphas: np.ndarray   # per-station AoA [deg]


class XFoilPolar:
    """Parse an XFOIL polar file and expose CD(alpha) with stall extrapolation."""

    def __init__(self, filename: str, cd_max: float = 1.2, n_dense: int = 100):
        self.cd_max = cd_max
        self.alphas, self.cds = self._parse(filename)
        self._alpha_dense = np.linspace(self.alphas.min(), self.alphas.max(), n_dense)
        spline = make_interp_spline(self.alphas, self.cds, k=3)
        self._cd_dense = spline(self._alpha_dense)

    @staticmethod
    def _parse(filename: str) -> tuple[np.ndarray, np.ndarray]:
        alphas, cds = [], []
        with open(filename) as f:
            parsing = False
            for line in f:
                if "------" in line:
                    parsing = True
                    continue
                if not parsing:
                    continue
                parts = line.strip().split()
                if len(parts) >= 3:
                    try:
                        alphas.append(float(parts[0]))
                        cds.append(float(parts[2]))
                    except ValueError:
                        continue
        alphas = np.asarray(alphas)
        cds = np.asarray(cds)
        alphas, idx = np.unique(alphas, return_index=True)
        return alphas, cds[idx]

    def cd(self, alpha: float) -> float:
        if self.alphas[0] <= alpha <= self.alphas[-1]:
            return float(np.interp(alpha, self._alpha_dense, self._cd_dense))

        edge = self.alphas[0] if alpha < self.alphas[0] else self.alphas[-1]
        cd_edge = float(np.interp(edge, self._alpha_dense, self._cd_dense))
        s2_e = np.sin(np.radians(edge)) ** 2
        s2 = np.sin(np.radians(alpha)) ** 2
        return cd_edge + (self.cd_max - cd_edge) * (s2 - s2_e) / (1 - s2_e)


class BladeGeometry:
    """Read the 14-column blade file and derive spanwise quantities (SI)."""

    def __init__(self, path: str):
        rows = []
        with open(path) as f:
            for line in f:
                p = line.split()
                if len(p) == 14:
                    try:
                        rows.append([float(x) for x in p])
                    except ValueError:
                        continue
        g = np.asarray(rows)

        self.station = g[:, 0]              # inches
        self.chord   = g[:, 1]              # inches
        self.twist   = g[:, 8]              # deg
        self.r       = self.station * INCH  # m
        self.c       = self.chord   * INCH  # m

        self.edges = np.concatenate(
            ([self.r[0]], 0.5 * (self.r[1:] + self.r[:-1]), [self.r[-1]])
        )
        self.dr   = np.diff(self.edges)
        self.span = self.r[-1]


class AirfoilSection:
    """Blend CD between two airfoils across a transition region."""

    def __init__(self, inner: XFoilPolar, outer: XFoilPolar,
                 transition_start: float, transition_end: float):
        self.inner, self.outer = inner, outer
        self.t0, self.t1 = transition_start, transition_end

    def cd(self, alpha: float, x: float) -> float:
        if x <= self.t0:
            return self.inner.cd(alpha)
        if x >= self.t1:
            return self.outer.cd(alpha)
        f = (x - self.t0) / (self.t1 - self.t0)
        return (1 - f) * self.inner.cd(alpha) + f * self.outer.cd(alpha)


class PropellerAnalysis:
    """Single-propeller blade-element integration."""

    def __init__(self, geometry: BladeGeometry, airfoil: AirfoilSection,
                 *, velocity: float, rpm: float, rho: float, mu: float,
                 n_blades: int, aoa_deg: float = 0.0):
        self.geom = geometry
        self.airfoil = airfoil
        self.velocity = velocity
        self.omega = rpm / 60.0 * 2.0 * np.pi
        self.rho = rho
        self.mu = mu
        self.n_blades = n_blades
        self.aoa = np.radians(aoa_deg)

    def run(self) -> PropellerResult:
        g = self.geom
        D = D_ax = Q = 0.0
        alphas = np.empty(len(g.r))

        for i in range(len(g.r)):
            v_rel = np.sqrt(self.velocity ** 2 + (self.omega * g.r[i]) ** 2)
            s = g.c[i] * g.dr[i]

            rel_alpha = np.arctan2(self.omega * g.r[i], self.velocity)
            phi = np.pi / 2.0 - rel_alpha
            alpha = np.degrees(g.twist[i] * np.pi / 180.0 + self.aoa - phi)
            alphas[i] = alpha

            cd = self.airfoil.cd(alpha, g.r[i] / g.span)

            dD = 0.5 * self.rho * v_rel ** 2 * s * cd
            D    += dD
            D_ax += dD * np.cos(rel_alpha)
            Q    += dD * g.r[i] * np.sin(rel_alpha)

        return PropellerResult(D=D, D_axial=D_ax, Q=Q, alphas=alphas)