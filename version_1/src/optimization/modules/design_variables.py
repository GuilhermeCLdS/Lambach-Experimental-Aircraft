import aerosandbox.numpy as np

from src.optimization.helpers.radius_from_ellipse import radius_from_ellipse


class DesignVariables:
    """Geometry and attitude optimization variables plus derived geometry.

    All longitudinal stations (l_c, l_cg, l_p) are measured from the nose;
    l_c and l_p are quarter-chord (aerodynamic-centre) stations.
    """

    def __init__(self, opti, assumptions):
        self.use_canards = assumptions.use_canards

        # Attitude
        self.AOA = opti.variable(init_guess=4 * np.pi / 180, lower_bound=0.0, upper_bound=12 * np.pi / 180)

        # Fuselage
        self.fuse_len = opti.variable(init_guess=0.23625, lower_bound=0.20, upper_bound=0.60)
        self.fuse_dia = opti.variable(init_guess=0.073747, lower_bound=0.05, upper_bound=0.15)

        # Pylons
        self.pylon_len = opti.variable(init_guess=0.08, lower_bound=0.05, upper_bound=0.20)
        self.pylon_chord = opti.variable(init_guess=0.075, lower_bound=0.015, upper_bound=0.150)
        self.pylon_thick = self.pylon_chord * assumptions.tc_pylon

        # Canards (variables only exist when canards are enabled)
        if assumptions.use_canards:
            self.canard_len = opti.variable(init_guess=0.06, lower_bound=0.03, upper_bound=0.30)
            self.canard_chord = opti.variable(init_guess=0.06, lower_bound=0.06, upper_bound=0.20)
            self.canard_inc = opti.variable(
                init_guess=-0.034907, lower_bound=-6 * np.pi / 180, upper_bound=10 * np.pi / 180,
            )
            self.canard_thick = self.canard_chord * assumptions.tc_canard
            self.l_c = opti.variable(init_guess=0.03, lower_bound=0.03, upper_bound=0.50)
        else:
            self.canard_len = 0.0
            self.canard_chord = 0.0
            self.canard_inc = 0.0
            self.canard_thick = 0.0
            self.l_c = 0.0

        # Incidence
        self.pylon_inc = opti.variable(
            init_guess=-0.007302, lower_bound=-10 * np.pi / 180, upper_bound=10 * np.pi / 180,
        )

        # Longitudinal stations
        self.l_cg = opti.variable(init_guess=0.05, lower_bound=0.05, upper_bound=0.50)
        self.l_p = opti.variable(init_guess=0.15, lower_bound=0.08, upper_bound=0.60)
        self.forced_l_cg = self.l_p - assumptions.cg_offset

        # Propeller height above the axis and top-to-bottom separation. The
        # pylon root sits on the elliptical fuselage at the pylon leading edge.
        self.pylon_le = self.l_p - 0.25 * self.pylon_chord
        x_pylon_le = -self.fuse_len / 2 + self.pylon_le
        r_fuse_at_pylon = radius_from_ellipse(x_pylon_le, self.fuse_len / 2, self.fuse_dia / 2)
        self.prop_z_arm = (r_fuse_at_pylon + self.pylon_len) * np.sin(np.pi / 4)
        self.l_t = 2 * self.prop_z_arm
