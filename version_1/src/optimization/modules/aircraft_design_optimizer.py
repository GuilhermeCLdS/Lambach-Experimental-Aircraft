from types import SimpleNamespace

import aerosandbox as asb

from src.global_parameters.global_parameters import Parameters
from src.optimization.helpers.thrust_force_components import thrust_force_components
from src.optimization.helpers.thrust_pitching_moment import thrust_pitching_moment
from src.optimization.modules.aircraft_aerodynamics import AircraftAerodynamics
from src.optimization.modules.apply_design_constraints import apply_design_constraints
from src.optimization.modules.design_variables import DesignVariables
from src.optimization.modules.propulsion_system import PropulsionSystem

SOLVED_DESIGN_KEYS = (
    "AOA", "fuse_len", "fuse_dia", "pylon_len", "pylon_chord", "pylon_inc",
    "canard_len", "canard_chord", "canard_inc", "l_p", "l_c", "l_cg",
)


class AircraftDesignOptimizer:
    """Minimum-drag trimmed design at the cruise condition in ``assumptions``.

    The problem is set up on construction; call ``solve()`` to run IPOPT.
    """

    def __init__(self, assumptions, prop_db):
        self.assumptions = assumptions
        self.prop_db = prop_db
        self.opti = asb.Opti()

        self.rho = Parameters.AIR_DENSITY_SEA_LEVEL
        self.a_sound = Parameters.speed_of_sound_sea_level()

        self.design = DesignVariables(self.opti, assumptions)
        self.propulsion = PropulsionSystem(
            opti=self.opti,
            assumptions=assumptions,
            a_sound=self.a_sound,
            velocity=assumptions.velocity,
            AOA=self.design.AOA,
            pylon_inc=self.design.pylon_inc,
            prop_db=prop_db,
        )

        self._compute_aerodynamics_and_equilibrium()
        apply_design_constraints(self)
        self.opti.minimize(self.total_drag)

    def _compute_aerodynamics_and_equilibrium(self):
        d, prop = self.design, self.propulsion

        aero_model = AircraftAerodynamics(d, self.assumptions.use_canards)
        self.op_point, self.aero = aero_model.analyze(self.assumptions.velocity)

        self.lift_aero = self.aero["L"]
        self.total_drag = self.aero["D"]
        self.drag_profile = self.aero["D_profile"]
        self.drag_induced = self.aero["D_induced"]
        self.moment_aero = self.aero["m_b"]
        self.CLa = self.aero["CLa"]
        self.Cma = self.aero["Cma"]
        # AeroSandbox x_np is about the fuselage centre; convert to nose-based.
        self.x_np = self.aero["x_np"] + d.fuse_len / 2

        # Local geometric incidence (stall guards only; forces come from AeroBuildup).
        self.alpha_p = d.AOA + d.pylon_inc
        self.alpha_c = d.AOA + d.canard_inc if self.assumptions.use_canards else 0.0

        self.lift_prop, self.thrust_along_velocity = thrust_force_components(
            prop.thrust_tot, d.pylon_inc, self.op_point,
        )
        (self.moment_thrust,
         self.moment_thrust_diff,
         self.moment_thrust_incidence) = thrust_pitching_moment(
            delta_thrust=prop.delta_T,
            thrust_total=prop.thrust_tot,
            incidence=d.pylon_inc,
            z_prop=d.prop_z_arm,
            dx_prop_from_cg=d.l_p - d.l_cg,
            op_point=self.op_point,
        )

        self.lift_tot = self.lift_aero + self.lift_prop
        self.moment_about_cg = self.moment_aero + self.moment_thrust

    # ---------------- solve ----------------
    def solve(self, verbose=True):
        try:
            self.sol = self.opti.solve(verbose=verbose)
        except RuntimeError:
            print("\nSOLVER FAILED -> constraint violations at the last iterate:")
            self.opti.debug.show_infeasibilities(1e-4)
            print(
                "Check the propeller data-file RPM/J coverage, lift capability, "
                "CG/static-margin constraints, and pitch-trim sign conventions."
            )
            raise
        return self.sol

    def value(self, expression):
        """Numerical value of an expression at the solution."""
        if isinstance(expression, (int, float)):
            return float(expression)
        return float(self.sol.value(expression))

    def solved_design(self):
        """Plain-number copy of the design variables at the solution."""
        if not hasattr(self, "sol"):
            raise RuntimeError("Solve the optimization before reading the solved design.")
        return SimpleNamespace(**{k: self.value(getattr(self.design, k)) for k in SOLVED_DESIGN_KEYS})

    def build_solved_airplane(self):
        """asb.Airplane of the optimized design (e.g. for draw_three_view)."""
        return AircraftAerodynamics(self.solved_design(), self.assumptions.use_canards).build_airplane()
