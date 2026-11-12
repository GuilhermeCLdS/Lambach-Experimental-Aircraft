import aerosandbox.numpy as np
import numpy as onp

# Warm-start RPMs from a previous converged design.
RPM_TOP_GUESS = 29994.211989
RPM_BOTTOM_GUESS = 30286.681339


class PropulsionSystem:
    """RPM variables and thrust / power / torque for the four propellers.

    The two upper propellers share ``rpm_top`` and the two lower ones share
    ``rpm_bottom`` (equal to ``rpm_top`` when differential thrust is off).
    """

    def __init__(self, opti, assumptions, a_sound, velocity, AOA, pylon_inc, prop_db):
        self.prop_axis_angle_to_freestream = AOA + pylon_inc
        self.V_ax = velocity * np.cos(self.prop_axis_angle_to_freestream)

        # Never let the optimizer extrapolate beyond the RPM range in the data file.
        rpm_lower = max(float(assumptions.rpm_min), float(prop_db.rpms[0]))
        rpm_upper = min(float(assumptions.rpm_max), float(prop_db.rpms[-1]))
        if rpm_upper <= rpm_lower:
            raise ValueError("The configured RPM bounds do not overlap the propeller performance database.")

        self.rpm_top = opti.variable(
            init_guess=float(onp.clip(RPM_TOP_GUESS, rpm_lower, rpm_upper)),
            lower_bound=rpm_lower, upper_bound=rpm_upper,
        )
        if assumptions.use_diff_thrust:
            self.rpm_bottom = opti.variable(
                init_guess=float(onp.clip(RPM_BOTTOM_GUESS, rpm_lower, rpm_upper)),
                lower_bound=rpm_lower, upper_bound=rpm_upper,
            )
        else:
            self.rpm_bottom = self.rpm_top

        self.T_top, self.P_top, self.Q_top, self.J_top = prop_db.evaluate(self.rpm_top, self.V_ax)
        self.T_bottom, self.P_bottom, self.Q_bottom, self.J_bottom = prop_db.evaluate(self.rpm_bottom, self.V_ax)

        # Two propellers per pair.
        self.thrust_top_pair = 2 * self.T_top
        self.thrust_bottom_pair = 2 * self.T_bottom
        self.thrust_tot = self.thrust_top_pair + self.thrust_bottom_pair
        self.delta_T = self.thrust_top_pair - self.thrust_bottom_pair
        self.power_tot = 2 * self.P_top + 2 * self.P_bottom

        # Resultant tip speed includes rotational and axial components.
        self.M_tip_top = self._tip_mach(self.rpm_top, assumptions.prop_radius, a_sound)
        self.M_tip_bottom = self._tip_mach(self.rpm_bottom, assumptions.prop_radius, a_sound)

    def _tip_mach(self, rpm, radius, a_sound):
        omega = rpm / 60 * 2 * np.pi
        return np.sqrt((omega * radius) ** 2 + self.V_ax ** 2) / a_sound
