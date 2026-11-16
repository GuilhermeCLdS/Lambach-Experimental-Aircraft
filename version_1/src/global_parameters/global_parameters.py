# Statically typed classes of parameters and assumptions

from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DATA_DIR = REPOSITORY_ROOT / "data" / "prop_Performance"


# For all known parameters
class Parameters():

    G0 = 9.80665                       # [N/kg]
    GAS_CONSTANT_AIR = 287.05          # [J/kg/K]
    GAMMA_AIR = 1.4
    PRESSURE_SEA_LEVEL = 101325        # [Pa]
    TEMPERATURE_SEA_LEVEL = 288.15     # [K]
    AIR_DENSITY_SEA_LEVEL = 1.225      # [kg/m^3]
    DYNAMIC_VISCOSITY_SEA_LEVEL = 1.789e-5   # [kg/m/s]

    @classmethod
    def speed_of_sound_sea_level(cls):
        """ISA sea-level speed of sound [m/s], a = sqrt(gamma R T)."""
        return np.sqrt(cls.GAMMA_AIR * cls.GAS_CONSTANT_AIR * cls.TEMPERATURE_SEA_LEVEL)


# For all Assumptions
class Assumptions():

    def __init__(self):

        # ------------------------------------------------- Prop / motor selection (editable)
        # Airframe / mission
        self.LD_propcalc = 2.5                    # [-] Lift-to-drag at cruise
        # TODO: ^ can't be defined in the future, used for required prop thrust, will come from optimizer
        self.MIN_STATIC_TW = 1.1         # [-] Static T/W per prop (>1.0 for VTOL hover)
        self.CLIMB_DEG = 10.0            # [deg] Climb angle for prop thrust
        self.THR_BAND = (0.40, 1.0)     # [-] Allowed cruise/throttle band

        # Propeller library filters
        self.D_MIN_IN = 3.0              # [in] Min prop diameter
        self.RPM_TABLE_TOL = 1.01        # [-] RPM tolerance factor beyond table max

        # Electrical system (GEPRC TAKER H60 BLS 60A 4in1)
        self.ESC_CELLS = 6               # [-] Battery cell count
        self.ESC_CAP_AH = 1.6            # [Ah] Battery capacity
        self.ESC_C_RATE = 160            # [C] Battery discharge rate
        self.ESC_V_CELL = 3.7            # [V] Loaded voltage per cell
        self.ESC_R_PACK = 0.025          # [ohm] Per-motor pack+wiring resistance
        self.ESC_A = 60.0                # [A] ESC continuous current limit per motor
        self.ESC_BURST_A = 65.0          # [A] ESC burst current limit
        self.ESC_BURST_S = 5.0           # [s] Burst duration
        self.ESC_EFF = 0.95              # [-] ESC efficiency
        self.ESC_NUM_MOTORS = 4          # [-] Number of motors
        self.ESC_SAG_FACTOR = 0.75       # [-] Battery sag factor (loaded voltage fraction)
        self.kv_load_factor = 0.8

        # Motor: (name, KV [rpm/V], Rm [ohm], I0 [A], Imax [A])
        self.MOTORS = [
            ("2207_1960kv", 1960, 0.06985, 0.70, 44.57),
        ]

        # ------------------------------------------------- Fixed aircraft parameters
        # Flight conditions
        self.velocity = 200 / 3.6              # [m/s] 200 km/h design (max) speed
        self.static_margin_frac = 0.02         # (x_np - x_cg) / fuse_len
        self.alpha_stall_deg = 10.0            # [deg] local surface incidence guard
        self.prop_axis_max_deg = 12.0          # [deg] prop axis to freestream

        # Structural properties
        self.tc_pylon = 0.16                   # pylon thickness-to-chord
        self.tc_canard = 0.12                  # canard thickness-to-chord
        self.pylon_thick_min = 0.012           # [m]

        # Engine parameters
        self.perf_file = DATA_DIR / "PER3_5x5F.dat"
        self.n_propellers = 4                  # [-] Number of propellers
        self.prop_dia = 5 * 0.0254             # [m] Max prop diameter
        self.rpm_min = 15000                   # [rpm] Min motor RPM TODO: change to fit prop or motor limit
        self.rpm_max = 44000                   # [rpm] Max motor RPM TODO: change to fit prop or motor limit
        # Max motor power per motor = sagged battery voltage * min(ESC continuous, battery C-rate per motor, motor datasheet Imax)
        i_lim_batt_per_motor = (self.ESC_C_RATE * self.ESC_CAP_AH) / self.ESC_NUM_MOTORS
        motor_imax = self.MOTORS[0][4] if self.MOTORS else float("inf")
        self.P_motor_max = self.ESC_CELLS * self.ESC_V_CELL * self.ESC_SAG_FACTOR * min(
            self.ESC_A, i_lim_batt_per_motor, motor_imax
        )                                       # [W] Max motor power per motor
        self.M_tip_max = 0.65                  # [-] Tip Mach limit
        self.max_diff_frac = 0.5               # [-] |delta_T| <= frac * total thrust
        self.prop_spacing_gap = 0.01           # [m] Gap between upper/lower discs

        # Mass properties
        self.aircraft_mass = 0.85              # [kg]
        self.cg_offset = 0.06                  # [m] l_cg = l_p - cg_offset

        # Fuselage dimensions
        self.fuse_dia_min = 0.07               # [m]
        self.l_min_dia = 0.10                  # [m] nose station of fuse_dia_min
        self.forced_pylon_len = 0.065          # [m]
        self.prop_clearance_gap = 0.01         # [m] prop tip to fuselage
        self.tail_margin = 0.03                # [m] pylon TE to fuselage tail
        self.canard_cg_gap = 0.02              # [m] canard AC ahead of CG TODO: should be removed, has already been in optimizer branch

        # ------------------------------------------------- Optimization switches (used by optimizer modules)
        self.use_canards = True                # [-] Enable canard surfaces
        self.use_diff_thrust = True            # [-] Enable differential thrust between top/bottom prop pairs
        self.steady_level = True               # [-] Equalities (True) or >= (False) for force balance
        self.fix_l_cg = True                   # [-] Fix CG location to l_p - cg_offset
        self.fix_pylon_len = True              # [-] Fix pylon length to forced_pylon_len
        self.fix_fuse_dia = True               # [-] Fix fuselage diameter to minimum

    @property
    def aircraft_weight(self):
        """[N] Weight from mass and standard gravity."""
        return self.aircraft_mass * Parameters.G0

    @property
    def prop_radius(self):
        return self.prop_dia / 2