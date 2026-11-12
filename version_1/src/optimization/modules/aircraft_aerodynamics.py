import aerosandbox as asb
import aerosandbox.numpy as np
import numpy as onp

from src.optimization.helpers.polar_to_cartesian import polar_to_cartesian
from src.optimization.helpers.radius_from_ellipse import radius_from_ellipse

N_FUSE_SECTIONS = 31
AIRFOIL = "naca0012"


class AircraftAerodynamics:
    """Build the aircraft geometry and analyse it with one AeroBuildup run.

    ``design`` is any object with the design attributes (fuse_len, fuse_dia,
    pylon_*, canard_*, l_p, l_c, l_cg, AOA): either the symbolic
    DesignVariables during optimization or plain numbers afterwards.

    Geometry x = 0 is at the fuselage centre with +x aft; the design stations
    are measured from the nose, hence the ``- fuse_len / 2`` shifts.
    """

    def __init__(self, design, use_canards):
        self.design = design
        self.use_canards = use_canards
        self.a = design.fuse_len / 2
        self.b = design.fuse_dia / 2
        self.canard_le = design.l_c - 0.25 * design.canard_chord
        self.pylon_le = design.l_p - 0.25 * design.pylon_chord

    @staticmethod
    def aero_surface(name, root_chord, tip_chord, twist_deg, y_root, z_root, y_tip, z_tip):
        """One half-surface; AeroSandbox mirrors it across the XZ plane."""
        return asb.Wing(
            name=name,
            symmetric=True,
            xsecs=[
                asb.WingXSec(xyz_le=[0, y_root, z_root], chord=root_chord,
                             twist=twist_deg, airfoil=asb.Airfoil(AIRFOIL)),
                asb.WingXSec(xyz_le=[0, y_tip, z_tip], chord=tip_chord,
                             twist=twist_deg, airfoil=asb.Airfoil(AIRFOIL)),
            ],
        )

    def build_wing_surfaces(self):
        d = self.design
        wings = []

        # Pylon arms form an X in the y-z plane; each half-wing is mirrored
        # across y = 0, giving four arms.
        pylon_x = -self.a + self.pylon_le
        pylon_root_radius = radius_from_ellipse(pylon_x, self.a, self.b)
        pylon_tip_radius = pylon_root_radius + d.pylon_len
        for name, angle in (("pylons_upper", np.pi / 4), ("pylons_lower", -np.pi / 4)):
            y_root, z_root = polar_to_cartesian(pylon_root_radius, angle)
            y_tip, z_tip = polar_to_cartesian(pylon_tip_radius, angle)
            wing = self.aero_surface(
                name=name, root_chord=d.pylon_chord, tip_chord=d.pylon_chord,
                twist_deg=np.degrees(d.pylon_inc),
                y_root=y_root, z_root=z_root, y_tip=y_tip, z_tip=z_tip,
            )
            wings.append(wing.translate([pylon_x, 0, 0]))

        # One half-canard, mirrored into left and right canards.
        if self.use_canards:
            canard_x = -self.a + self.canard_le
            canard_root_radius = radius_from_ellipse(canard_x, self.a, self.b)
            canard = self.aero_surface(
                name="canards", root_chord=d.canard_chord, tip_chord=d.canard_chord,
                twist_deg=np.degrees(d.canard_inc),
                y_root=canard_root_radius, z_root=0.0,
                y_tip=canard_root_radius + d.canard_len, z_tip=0.0,
            )
            wings.append(canard.translate([canard_x, 0, 0]))

        return wings

    def build_fuselage(self):
        # With x = -a cos(beta) the ellipse radius is exactly b sin(beta); this
        # avoids a symbolic sqrt(0) (infinite derivative) at the nose and tail.
        beta = onp.linspace(0, onp.pi, N_FUSE_SECTIONS)
        return asb.Fuselage(
            name="fuselage",
            xsecs=[
                asb.FuselageXSec(xyz_c=[-self.a * onp.cos(b), 0, 0], radius=self.b * onp.sin(b))
                for b in beta
            ],
        )

    @property
    def l_cg_geometry(self):
        return self.design.l_cg - self.a

    def build_airplane(self):
        return asb.Airplane(
            name="Quadcopter",
            xyz_ref=[self.l_cg_geometry, 0, 0],
            wings=self.build_wing_surfaces(),
            fuselages=[self.build_fuselage()],
        )

    def analyze(self, velocity):
        """Run AeroBuildup with the alpha stability derivative.

        Returns (op_point, results); results carries L, D, D_profile,
        D_induced, m_b, CLa, Cma and x_np (geometry axes).
        """
        op_point = asb.OperatingPoint(velocity=velocity, alpha=np.degrees(self.design.AOA))
        results = asb.AeroBuildup(
            airplane=self.build_airplane(), op_point=op_point,
        ).run_with_stability_derivatives(alpha=True, beta=False, p=False, q=False, r=False)
        return op_point, results
