import re

import aerosandbox as asb
import aerosandbox.numpy as np
import numpy as onp
from types import SimpleNamespace
from pathlib import Path

# =============================================================================
# Module-level helpers
# =============================================================================

def radius_from_ellipse(x, a, b):
    """Elliptical fuselage radius at longitudinal coordinate x."""
    return b * np.sqrt(np.maximum(1 - (x / a) ** 2, 0.0))


def polar_to_cartesian(r, theta):
    """Convert a radius and angle in the y-z plane to y and z coordinates."""
    return r * np.cos(theta), r * np.sin(theta)


# =============================================================================
# 1) Configuration
# =============================================================================

class Config:
    """User-facing switches, flight condition, and file paths."""

    def __init__(self):
        # --- optimization switches ---
        self.fix_l_cg = True
        self.fix_pylon_len = True
        self.fix_fuse_dia = True

        self.use_canards = True
        self.use_diff_thrust = True
        self.max_diff_frac = 0.5

        # --- flight condition ---
        self.velocity = 200 / 3.6              # [m/s] 200 km/h
        self.aircraft_weight = 0.85 * 9.81     # [N] 0.85 kg aircraft
        self.static_margin_frac = 0.02         # (x_np - x_cg) / fuse_len
        self.steady_level = True

        # --- propeller / powertrain ---
        self.PERF_FILE = Path(__file__).resolve().parent / "PER3_5x37E-3_performance_file.txt"
        self.prop_dia = 5 * 0.0254             # [m]
        self.rpm_min = 15000
        self.rpm_max = 44000
        self.P_motor_max = 1000                # [W] per motor
        self.M_tip_max = 0.80

        # --- geometry / flight limits ---
        self.cg_offset = 0.15                  # [m] l_cg = l_p - cg_offset
        self.alpha_stall_deg = 10.0
        self.prop_axis_max_deg = 12.0

        # --- propeller arrangement ---
        # Two propellers are on the upper pair of arms and two on the lower pair.
        # Each pair is represented by one RPM variable and one per-propeller map.
        self.n_propellers = 4


# =============================================================================
# 2) Propeller performance database
# =============================================================================

class PropellerPerformanceDatabase:
    """Ct(J, RPM) / Cp(J, RPM) tables read from an APC PER3 file."""

    def __init__(self, path, rho, prop_dia, rpm_lo=10000):
        self.rho = rho
        self.prop_dia = prop_dia
        self.rpms, self.J_grid, self.Ct_tab, self.Cp_tab = self._load(path, rpm_lo)
        self.J_tab_max = float(self.J_grid[-1])

    def _load(self, path, rpm_lo):
        blocks = {}
        current_rpm = None

        with open(path, "r", errors="ignore") as file:
            for line in file:
                match = re.search(r"PROP RPM =\s*([\d.]+)", line)
                if match:
                    current_rpm = float(match.group(1))
                    blocks[current_rpm] = []
                    continue

                parts = line.split()
                if current_rpm is not None and len(parts) == 15:
                    try:
                        blocks[current_rpm].append([float(value) for value in parts])
                    except ValueError:
                        continue

        valid_blocks = {
            rpm: onp.asarray(rows, dtype=float)
            for rpm, rows in blocks.items()
            if rpm >= rpm_lo and len(rows) >= 2
        }

        rpms = onp.asarray(sorted(valid_blocks), dtype=float)
        if len(rpms) < 2:
            raise ValueError(
                f"The propeller file '{path}' must contain at least two valid RPM blocks "
                f"at or above {rpm_lo} RPM."
            )

        j_max = min(valid_blocks[rpm][-1, 1] for rpm in rpms)
        j_max = onp.floor(j_max * 100) / 100
        if j_max <= 0:
            raise ValueError(f"No positive advance-ratio range found in '{path}'.")

        j_grid = onp.linspace(0.0, j_max, 89)
        ct_table = onp.asarray([
            onp.interp(j_grid, valid_blocks[rpm][:, 1], valid_blocks[rpm][:, 3])
            for rpm in rpms
        ])
        cp_table = onp.asarray([
            onp.interp(j_grid, valid_blocks[rpm][:, 1], valid_blocks[rpm][:, 4])
            for rpm in rpms
        ])

        return rpms, j_grid, ct_table, cp_table

    def evaluate(self, rpm, axial_velocity):
        """Return thrust [N], shaft power [W], torque [Nm], and advance ratio."""
        revolutions_per_second = rpm / 60
        advance_ratio = axial_velocity / (revolutions_per_second * self.prop_dia)
        query = np.array([rpm, advance_ratio])

        ct = np.interpn(
            (self.rpms, self.J_grid), self.Ct_tab, query,
            method="bspline", bounds_error=False, fill_value=None
        )
        cp = np.interpn(
            (self.rpms, self.J_grid), self.Cp_tab, query,
            method="bspline", bounds_error=False, fill_value=None
        )

        thrust = ct * self.rho * revolutions_per_second ** 2 * self.prop_dia ** 4
        power = cp * self.rho * revolutions_per_second ** 3 * self.prop_dia ** 5
        torque = power / (2 * np.pi * revolutions_per_second)

        return thrust, power, torque, advance_ratio


# =============================================================================
# 3) Propulsion system
# =============================================================================

class PropulsionSystem:
    """RPM variables and thrust / power / torque for the four propellers."""

    def __init__(self, opti, cfg, rho, a_sound, velocity,
                 AOA, pylon_inc, perf_db):
        self.prop_axis_angle_to_freestream = AOA + pylon_inc
        self.V_ax = velocity * np.cos(self.prop_axis_angle_to_freestream)

        # Do not let the optimizer extrapolate beyond the RPM range in the data file.
        rpm_lower = max(float(cfg.rpm_min), float(perf_db.rpms[0]))
        rpm_upper = min(float(cfg.rpm_max), float(perf_db.rpms[-1]))
        if rpm_upper <= rpm_lower:
            raise ValueError(
                "The configured RPM bounds do not overlap the propeller performance database."
            )

        rpm_top_guess = float(onp.clip(29994.211989, rpm_lower, rpm_upper))
        rpm_bottom_guess = float(onp.clip(30286.681339, rpm_lower, rpm_upper))

        self.rpm_top = opti.variable(
            init_guess=rpm_top_guess,
            lower_bound=rpm_lower,
            upper_bound=rpm_upper
        )

        if cfg.use_diff_thrust:
            self.rpm_bottom = opti.variable(
                init_guess=rpm_bottom_guess,
                lower_bound=rpm_lower,
                upper_bound=rpm_upper
            )
        else:
            self.rpm_bottom = self.rpm_top

        (self.T_top, self.P_top, self.Q_top,
         self.J_top) = perf_db.evaluate(self.rpm_top, self.V_ax)
        (self.T_bottom, self.P_bottom, self.Q_bottom,
         self.J_bottom) = perf_db.evaluate(self.rpm_bottom, self.V_ax)

        # Two propellers per pair.
        self.thrust_top_pair = 2 * self.T_top
        self.thrust_bottom_pair = 2 * self.T_bottom
        self.thrust_tot = self.thrust_top_pair + self.thrust_bottom_pair
        self.delta_T = self.thrust_top_pair - self.thrust_bottom_pair
        self.power_tot = 2 * self.P_top + 2 * self.P_bottom

        # Resultant tip speed includes rotational and axial components.
        omega_top = self.rpm_top / 60 * 2 * np.pi
        omega_bottom = self.rpm_bottom / 60 * 2 * np.pi
        prop_radius = cfg.prop_dia / 2

        self.M_tip_top = np.sqrt(
            (omega_top * prop_radius) ** 2 + self.V_ax ** 2
        ) / a_sound
        self.M_tip_bottom = np.sqrt(
            (omega_bottom * prop_radius) ** 2 + self.V_ax ** 2
        ) / a_sound


# =============================================================================
# 4) Aircraft geometry and AeroBuildup analysis
# =============================================================================

class AircraftAerodynamics:
    """Build the complete aircraft and analyse it with one AeroBuildup model."""

    def __init__(self, design):
        self.design = design
        self.l_p = design.l_p
        self.l_c = design.l_c
        self.canard_chord = design.canard_chord
        self.canard_len = design.canard_len
        self.pylon_chord = design.pylon_chord
        self.pylon_len = design.pylon_len
        self.fuse_len = design.fuse_len
        self.fuse_dia = design.fuse_dia
        self.pylon_incidence = design.pylon_inc
        self.canard_incidence = design.canard_inc
        self.n_fuse_sections = 31

        self.a = self.fuse_len / 2
        self.b = self.fuse_dia / 2

        self.canard_le = self.l_c - 0.25 * self.canard_chord
        self.pylon_le = self.l_p - 0.25 * self.pylon_chord

    def aero_surface(self, name, root_chord, tip_chord, twist_deg,
                     y_root, z_root, y_tip, z_tip):
        """Create one half-surface; AeroSandbox mirrors it across the XZ plane."""
        return asb.Wing(
            name=name,
            symmetric=True,
            xsecs=[
                asb.WingXSec(
                    xyz_le=[0, y_root, z_root],
                    chord=root_chord,
                    twist=twist_deg,
                    airfoil=asb.Airfoil("naca0012"),
                ),
                asb.WingXSec(
                    xyz_le=[0, y_tip, z_tip],
                    chord=tip_chord,
                    twist=twist_deg,
                    airfoil=asb.Airfoil("naca0012"),
                ),
            ],
        )

    def build_wing_surfaces(self):
        wings = []

        # The pylon arms form an X in the y-z plane. Each listed half-wing is
        # mirrored across y=0, giving four arms in total.
        pylon_x = -self.fuse_len / 2 + self.pylon_le
        pylon_root_radius = radius_from_ellipse(pylon_x, self.a, self.b)
        pylon_tip_radius = pylon_root_radius + self.pylon_len
        pylon_twist_deg = np.degrees(self.pylon_incidence)

        for name, angle in (("pylons_upper", np.pi / 4),
                            ("pylons_lower", -np.pi / 4)):
            y_root, z_root = polar_to_cartesian(pylon_root_radius, angle)
            y_tip, z_tip = polar_to_cartesian(pylon_tip_radius, angle)

            wing = self.aero_surface(
                name=name,
                root_chord=self.pylon_chord,
                tip_chord=self.pylon_chord,
                twist_deg=pylon_twist_deg,
                y_root=y_root,
                z_root=z_root,
                y_tip=y_tip,
                z_tip=z_tip,
            )
            wings.append(wing.translate([pylon_x, 0, 0]))

        # One half-canard is mirrored to create the left and right canards.
        if self.design.cfg.use_canards:
            canard_x = -self.fuse_len / 2 + self.canard_le
            canard_root_radius = radius_from_ellipse(canard_x, self.a, self.b)
            canard_tip_radius = canard_root_radius + self.canard_len

            canard = self.aero_surface(
                name="canards",
                root_chord=self.canard_chord,
                tip_chord=self.canard_chord,
                twist_deg=np.degrees(self.canard_incidence),
                y_root=canard_root_radius,
                z_root=0.0,
                y_tip=canard_tip_radius,
                z_tip=0.0,
            )
            wings.append(canard.translate([canard_x, 0, 0]))

        return wings

    def build_fuselage(self):
        beta = onp.linspace(0, onp.pi, self.n_fuse_sections)
        x_sections = []

        for beta_i in beta:
            x = -(self.fuse_len / 2) * np.cos(beta_i)
            radius = (self.fuse_dia / 2) * np.sqrt(
                np.maximum(1 - (x / (self.fuse_len / 2)) ** 2, 0.0)
            )
            x_sections.append(
                asb.FuselageXSec(
                    xyz_c=[x, 0, 0],
                    radius=radius,
                )
            )

        return asb.Fuselage(name="fuselage", xsecs=x_sections)

    def build_airplane(self):
        # Geometry x=0 is at the fuselage centre; l_cg is measured from the nose.
        xyz_ref_cg = [self.l_cg_geometry, 0, 0]
        return asb.Airplane(
            name="Quadcopter",
            xyz_ref=xyz_ref_cg,
            wings=self.build_wing_surfaces(),
            fuselages=[self.build_fuselage()],
        )

    @property
    def l_cg_geometry(self):
        return self.design.l_cg - self.design.fuse_len / 2

    def analyze(self, velocity):
        airplane = self.build_airplane()
        op_point = asb.OperatingPoint(
            velocity=velocity,
            alpha=np.degrees(self.design.AOA),
        )

        # One full-aircraft analysis: forces, moments, drag breakdown, and
        # longitudinal stability derivatives all come from this result.
        results = asb.AeroBuildup(
            airplane=airplane,
            op_point=op_point,
        ).run_with_stability_derivatives(
            alpha=True,
            beta=False,
            p=False,
            q=False,
            r=False,
        )

        return op_point, results


# =============================================================================
# 5) Main optimizer
# =============================================================================

class AircraftDesignOptimizer:
    """Sets up and solves the aircraft design optimization problem."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.opti = asb.Opti()

        self._setup_atmosphere()
        self._create_geometry_variables()
        self._load_propeller_database()

        self.propulsion = PropulsionSystem(
            opti=self.opti,
            cfg=self.cfg,
            rho=self.rho,
            a_sound=self.a_sound,
            velocity=self.cfg.velocity,
            AOA=self.AOA,
            pylon_inc=self.pylon_inc,
            perf_db=self.prop_db,
        )

        self._compute_aerodynamics_and_equilibrium()
        self._apply_constraints()

    # ---------------- setup ----------------
    def _setup_atmosphere(self):
        atmosphere = asb.Atmosphere(altitude=0)
        self.rho = atmosphere.density()
        self.a_sound = atmosphere.speed_of_sound()

    def _create_geometry_variables(self):
        opti, cfg = self.opti, self.cfg

        # Attitude
        self.AOA = opti.variable(
            init_guess=4 * np.pi / 180,
            lower_bound=0.0,
            upper_bound=12 * np.pi / 180,
        )

        # Fuselage
        self.fuse_len = opti.variable(
            init_guess=0.23625, lower_bound=0.20, upper_bound=0.60
        )
        self.fuse_dia = opti.variable(
            init_guess=0.073747, lower_bound=0.05, upper_bound=0.15
        )
        self.fuse_dia_min = 0.07
        self.l_min_dia = 0.10

        # Pylons
        self.pylon_len = opti.variable(
            init_guess=0.08, lower_bound=0.05, upper_bound=0.20
        )
        self.pylon_chord = opti.variable(
            init_guess=0.075, lower_bound=0.015, upper_bound=0.150
        )
        self.tc_pylon = 0.16
        self.pylon_thick = self.pylon_chord * self.tc_pylon
        self.prop_clearance = 0.0635 + 0.01
        self.forced_pylon_len = 0.065

        # Canard variables only exist when canards are enabled.
        if cfg.use_canards:
            self.canard_len = opti.variable(
                init_guess=0.06, lower_bound=0.03, upper_bound=0.30
            )
            self.canard_chord = opti.variable(
                init_guess=0.06, lower_bound=0.06, upper_bound=0.20
            )
            self.canard_inc = opti.variable(
                init_guess=-0.034907,
                lower_bound=-6 * np.pi / 180,
                upper_bound=10 * np.pi / 180,
            )
            self.canard_thick = self.canard_chord * 0.12
            self.l_c = opti.variable(
                init_guess=0.03, lower_bound=0.03, upper_bound=0.50
            )
        else:
            self.canard_len = 0.0
            self.canard_chord = 0.0
            self.canard_inc = 0.0
            self.canard_thick = 0.0
            self.l_c = 0.0

        # Incidence
        self.pylon_inc = opti.variable(
            init_guess=-0.007302,
            lower_bound=-10 * np.pi / 180,
            upper_bound=10 * np.pi / 180,
        )

        # Longitudinal stations; all are measured from the nose.
        self.l_cg = opti.variable(
            init_guess=0.05, lower_bound=0.05, upper_bound=0.50
        )
        self.l_p = opti.variable(
            init_guess=0.15, lower_bound=0.08, upper_bound=0.60
        )
        self.forced_l_cg = self.l_p - cfg.cg_offset

        # Pylon half-span in z and top-to-bottom propeller separation.
        # The pylon root is on the elliptical fuselage at the pylon leading edge.
        self.pylon_le = self.l_p - 0.25 * self.pylon_chord
        x_pylon_le = -self.fuse_len / 2 + self.pylon_le
        r_fuse_at_pylon = radius_from_ellipse(
            x_pylon_le, self.fuse_len / 2, self.fuse_dia / 2
        )
        self.prop_z_arm = (r_fuse_at_pylon + self.pylon_len) * np.sin(np.pi / 4)
        self.l_t = 2 * self.prop_z_arm

    def _load_propeller_database(self):
        cfg = self.cfg
        self.prop_db = PropellerPerformanceDatabase(
            cfg.PERF_FILE,
            self.rho,
            cfg.prop_dia,
            rpm_lo=cfg.rpm_min,
        )

    # ---------------- aerodynamics and equilibrium ----------------
    def _compute_aerodynamics_and_equilibrium(self):
        estimator = AircraftAerodynamics(self)
        self.op_point, self.aero = estimator.analyze(self.cfg.velocity)

        # AeroBuildup dimensional forces and pitching moment.
        self.lift_aero = self.aero["L"]
        self.total_drag = self.aero["D"]
        self.drag_profile = self.aero["D_profile"]
        self.drag_induced = self.aero["D_induced"]
        self.moment_aero = self.aero["m_b"]
        self.CLa = self.aero["CLa"]
        self.Cma = self.aero["Cma"]

        # x_np is in AeroSandbox geometry coordinates (x=0 at fuselage centre).
        # Convert it to the nose-based coordinate used by l_cg and l_p.
        self.x_np = self.aero["x_np"] + self.fuse_len / 2

        # Local geometric incidence checks. AeroBuildup itself supplies the forces.
        self.alpha_p = self.AOA + self.pylon_inc
        self.alpha_c = self.AOA + self.canard_inc if self.cfg.use_canards else 0.0

        self._compute_propeller_force_components()
        self._compute_propeller_moment()

        self.lift_tot = self.lift_aero + self.lift_prop
        self.thrust_along_velocity = self.thrust_along_freestream
        self.moment_about_cg = self.moment_aero + self.moment_thrust

    def _compute_propeller_force_components(self):
        """Resolve total thrust into wind-axis lift and freestream components."""
        incidence = self.pylon_inc

        # Geometry axes: +x points aft and +z points up. Therefore forward
        # thrust is -x; positive incidence gives an upward +z component.
        thrust_x_geometry = -self.propulsion.thrust_tot * np.cos(incidence)
        thrust_z_geometry = self.propulsion.thrust_tot * np.sin(incidence)

        thrust_x_wind, _, thrust_z_wind = self.op_point.convert_axes(
            thrust_x_geometry,
            0.0,
            thrust_z_geometry,
            from_axes="geometry",
            to_axes="wind",
        )

        # AeroBuildup defines L = -F_w[z]. In wind axes, positive x is
        # along the freestream, so this is the propulsive contribution.
        self.lift_prop = -thrust_z_wind
        self.thrust_along_freestream = thrust_x_wind

    def _compute_propeller_moment(self):
        """Pitching moment of all four thrust forces about the aircraft CG.

        The two upper propellers are at +z_prop and each produce T_top. The two
        lower propellers are at -z_prop and each produce T_bottom. All four
        thrust vectors are parallel and have the same pylon incidence.
        The resulting y-axis moment is calculated from r x T.
        """
        incidence = self.pylon_inc
        dx_prop_from_cg = self.l_p - self.l_cg
        z_prop = self.prop_z_arm

        # Pair-differential term from the vertical separation of the two pairs.
        self.moment_thrust_diff = (
            -z_prop * self.propulsion.delta_T * np.cos(incidence)
        )

        # Additional pitch moment from the vertical thrust component acting
        # at the longitudinal distance between the propellers and CG.
        self.moment_thrust_incidence = (
            -dx_prop_from_cg
            * self.propulsion.thrust_tot
            * np.sin(incidence)
        )

        moment_y_geometry = (
            self.moment_thrust_diff + self.moment_thrust_incidence
        )

        # Convert the geometry-axis moment into body axes so it can be added
        # directly to AeroBuildup's m_b (which is nose-up positive).
        _, self.moment_thrust, _ = self.op_point.convert_axes(
            0.0,
            moment_y_geometry,
            0.0,
            from_axes="geometry",
            to_axes="body",
        )

    # ---------------- constraints ----------------
    def _apply_constraints(self):
        opti, cfg, prop = self.opti, self.cfg, self.propulsion

        # Minimum fuselage diameter at the specified nose-based station.
        if cfg.fix_fuse_dia:
            opti.subject_to(
                self.fuse_dia >= self.fuse_dia_min / np.sqrt(
                    1 - ((self.l_min_dia - self.fuse_len / 2)
                         / (self.fuse_len / 2)) ** 2
                )
            )
        else:
            opti.subject_to(self.fuse_dia >= self.fuse_dia_min)

        # Pylon geometry and propeller clearance.
        if cfg.fix_pylon_len:
            opti.subject_to(self.pylon_len == self.forced_pylon_len)
        else:
            opti.subject_to(self.pylon_len >= self.prop_clearance)

        opti.subject_to(self.l_t >= 2 * 0.0635 + 0.01)
        opti.subject_to(self.pylon_thick >= 0.012)

        # Steady-level vertical force balance includes aerodynamic and propeller lift.
        if cfg.steady_level:
            opti.subject_to(self.lift_tot == cfg.aircraft_weight)
        else:
            opti.subject_to(self.lift_tot >= cfg.aircraft_weight)

        # Pitch trim: aerodynamic moment from AeroBuildup plus thrust-force moment.
        opti.subject_to(self.moment_about_cg == 0)

        # CG and longitudinal static margin.
        if cfg.fix_l_cg:
            opti.subject_to(self.l_cg == self.forced_l_cg)
        else:
            opti.subject_to(self.l_cg <= self.x_np)

        opti.subject_to(
            self.x_np - self.l_cg >= cfg.static_margin_frac * self.fuse_len
        )

        # Longitudinal layout constraints.
        opti.subject_to(
            self.l_p + 0.75 * self.pylon_chord + 0.03 <= self.fuse_len
        )
        if cfg.use_canards:
            opti.subject_to(
                self.l_c + 0.75 * self.canard_chord
                <= self.l_p - 0.25 * self.pylon_chord
            )
            opti.subject_to(self.l_c + 0.02 <= self.l_cg)
            opti.subject_to(self.l_c - 0.25 * self.canard_chord >= 0)

        # Thrust balance along the freestream direction.
        if cfg.steady_level:
            opti.subject_to(self.thrust_along_velocity == self.total_drag)
        else:
            opti.subject_to(self.thrust_along_velocity >= self.total_drag)

        # Differential-thrust limit (only one pair of constraints is needed).
        opti.subject_to(prop.delta_T <= cfg.max_diff_frac * prop.thrust_tot)
        opti.subject_to(prop.delta_T >= -cfg.max_diff_frac * prop.thrust_tot)

        # Propeller axis angle relative to the freestream.
        prop_axis_angle = self.AOA + self.pylon_inc
        max_prop_axis = cfg.prop_axis_max_deg * np.pi / 180
        opti.subject_to(prop_axis_angle <= max_prop_axis)
        opti.subject_to(prop_axis_angle >= -max_prop_axis)

        # Local surface incidence / stall guards.
        max_stall = cfg.alpha_stall_deg * np.pi / 180
        opti.subject_to(self.alpha_p <= max_stall)
        opti.subject_to(self.alpha_p >= -max_stall)
        if cfg.use_canards:
            opti.subject_to(self.alpha_c <= max_stall)
            opti.subject_to(self.alpha_c >= -max_stall)

        # Propeller database, motor power, and tip-Mach limits.
        opti.subject_to(prop.J_top >= 0)
        opti.subject_to(prop.J_bottom >= 0)
        opti.subject_to(prop.J_top <= self.prop_db.J_tab_max)
        opti.subject_to(prop.J_bottom <= self.prop_db.J_tab_max)
        opti.subject_to(prop.P_top <= cfg.P_motor_max)
        opti.subject_to(prop.P_bottom <= cfg.P_motor_max)
        opti.subject_to(prop.M_tip_top <= cfg.M_tip_max)
        opti.subject_to(prop.M_tip_bottom <= cfg.M_tip_max)
        opti.subject_to(prop.T_top >= 0)
        opti.subject_to(prop.T_bottom >= 0)

        opti.minimize(self.total_drag)

    # ---------------- solve & report ----------------
    def solve(self):
        try:
            self.sol = self.opti.solve()
        except RuntimeError:
            print("\nSOLVER FAILED -> constraint violations at the last iterate:")
            self.opti.debug.show_infeasibilities(1e-4)
            print(
                "Check the propeller data-file RPM/J coverage, lift capability, "
                "CG/static-margin constraints, and pitch-trim sign conventions."
            )
            raise

        self.opti.debug.show_infeasibilities()
        return self.sol

    def build_solved_airplane(self):

        if not hasattr(self, "sol"):
            raise RuntimeError("Solve the optimization before drawing the aircraft.")

        def value(x):
            if isinstance(x, (int, float)):
                return float(x)
            return float(self.sol.value(x))

        # Create a numerical version of the optimized design
        solved_design = SimpleNamespace(
            cfg=self.cfg,

            fuse_len=value(self.fuse_len),
            fuse_dia=value(self.fuse_dia),

            pylon_len=value(self.pylon_len),
            pylon_chord=value(self.pylon_chord),
            pylon_inc=value(self.pylon_inc),

            canard_len=value(self.canard_len),
            canard_chord=value(self.canard_chord),
            canard_inc=value(self.canard_inc),

            l_p=value(self.l_p),
            l_c=value(self.l_c),
            l_cg=value(self.l_cg),
        )

    # Build the aircraft using optimized numerical values
        estimator = AircraftAerodynamics(solved_design)

        return estimator.build_airplane()


    def report(self):
        sol, cfg, prop = self.sol, self.cfg, self.propulsion
        value = sol.value

        print("--- AIRCRAFT AERODYNAMIC OPTIMIZATION ---")
        print(f"Canards: {'ON' if cfg.use_canards else 'OFF'} | "
              f"Differential thrust: {'ON' if cfg.use_diff_thrust else 'OFF'}")
        print(f"Total aerodynamic drag: {value(self.total_drag):.3f} N")
        print(f"  Profile drag:          {value(self.drag_profile):.3f} N")
        print(f"  Induced drag:          {value(self.drag_induced):.3f} N")
        print(f"Aerodynamic lift:        {value(self.lift_aero):.3f} N")
        print(f"Propeller lift:          {value(self.lift_prop):.3f} N")
        print(f"Total lift:              {value(self.lift_tot):.3f} N "
              f"(weight {cfg.aircraft_weight:.3f} N)")
        print(f"L/D:                     {value(self.lift_tot) / value(self.total_drag):.3f}")

        print(f"Fuselage length:         {value(self.fuse_len) * 1000:.1f} mm")
        print(f"Fuselage diameter:       {value(self.fuse_dia) * 1000:.1f} mm")
        if cfg.use_canards:
            print(f"Canard length:           {value(self.canard_len) * 1000:.1f} mm")
            print(f"Canard chord:            {value(self.canard_chord) * 1000:.1f} mm")
            print(f"Canard incidence:        {np.degrees(value(self.canard_inc)):.2f} deg")
            print(f"Canard AC (nose-based):  {value(self.l_c) * 1000:.1f} mm")

        print(f"Pylon length:            {value(self.pylon_len) * 1000:.1f} mm")
        print(f"Pylon chord:             {value(self.pylon_chord) * 1000:.1f} mm")
        print(f"Pylon thickness:         {value(self.pylon_thick) * 1000:.1f} mm")
        print(f"Pylon incidence:         {np.degrees(value(self.pylon_inc)):.2f} deg")
        print(f"Pylon AC (nose-based):   {value(self.l_p) * 1000:.1f} mm")
        print(f"CG (nose-based):         {value(self.l_cg) * 1000:.1f} mm")
        print(f"Neutral point:           {value(self.x_np) * 1000:.1f} mm")
        print(f"Static margin:           "
              f"{(value(self.x_np) - value(self.l_cg)) * 1000:.1f} mm")
        print(f"AOA:                     {np.degrees(value(self.AOA)):.2f} deg")
        print(f"AeroBuildup CLa:          {value(self.CLa):.4f} 1/rad")
        print(f"AeroBuildup Cma:          {value(self.Cma):.4f} 1/rad")

        print("--- PROPULSION ---")
        print(f"RPM upper / lower:       "
              f"{value(prop.rpm_top):.0f} / {value(prop.rpm_bottom):.0f}")
        print(f"Advance ratio J:         upper {value(prop.J_top):.3f} / "
              f"lower {value(prop.J_bottom):.3f} "
              f"(table limit {self.prop_db.J_tab_max:.3f})")
        print(f"Thrust per propeller:    upper {value(prop.T_top):.3f} N / "
              f"lower {value(prop.T_bottom):.3f} N")
        print(f"Total thrust:            {value(prop.thrust_tot):.3f} N")
        print(f"Differential thrust:     {value(prop.delta_T):.3f} N "
              f"(upper pair minus lower pair)")
        print(f"Thrust along freestream: {value(self.thrust_along_velocity):.3f} N")
        print(f"Propeller axis angle:    "
              f"{np.degrees(value(self.AOA + self.pylon_inc)):.2f} deg")
        print(f"Thrust pitch moment:     {value(self.moment_thrust):.5f} Nm")
        print(f"  Differential part:    {value(self.moment_thrust_diff):.5f} Nm")
        print(f"  Incidence part:        {value(self.moment_thrust_incidence):.5f} Nm")
        print(f"Aerodynamic pitch moment:{value(self.moment_aero):.5f} Nm")
        print(f"Total trim moment:       {value(self.moment_about_cg):.3e} Nm")
        print(f"Total shaft power:       {value(prop.power_tot):.1f} W")
        print(f"  Upper motor power:    {value(prop.P_top):.1f} W each")
        print(f"  Lower motor power:    {value(prop.P_bottom):.1f} W each")
        print(f"Shaft torque:            upper {value(prop.Q_top):.4f} / "
              f"lower {value(prop.Q_bottom):.4f} Nm per motor")
        print(f"Tip Mach:                upper {value(prop.M_tip_top):.3f} / "
              f"lower {value(prop.M_tip_bottom):.3f}")
        print(f"Propulsive efficiency*:  "
              f"{value(self.total_drag) * cfg.velocity / value(prop.power_tot):.3f}")
        print("*Based on D*V / total shaft power; does not include motor/ESC losses.")

        print("--- INCIDENCE CHECKS ---")
        print(f"Pylon local alpha:       {np.degrees(value(self.alpha_p)):.2f} deg")
        if cfg.use_canards:
            print(f"Canard local alpha:      {np.degrees(value(self.alpha_c)):.2f} deg")

        print(f"CG offset from pylon AC: "
              f"{(value(self.l_p) - value(self.l_cg)) * 1000:.1f} mm")
        print(f"CG location along body:  "
              f"{value(self.l_cg) / value(self.fuse_len) * 100:.1f} % of fuselage length")


# =============================================================================
# Entry point
# =============================================================================

def main():
    cfg = Config()
    optimizer = AircraftDesignOptimizer(cfg)

    optimizer.solve()
    optimizer.report()

    # Build and visualize the optimized aircraft
    airplane = optimizer.build_solved_airplane()

    airplane.draw_three_view(
        style="shaded"
    )


if __name__ == "__main__":
    main()
