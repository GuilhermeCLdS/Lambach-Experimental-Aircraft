# Statically typed classes of parameters and assumptions

from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DATA_DIR = REPOSITORY_ROOT / "data"


# For all known parameters
class Parameters():

    G0 = 9.80665 # [N/kg]
    GAS_CONSTANT_AIR = 287.05 # [J/kg/K]
    GAMMA_AIR = 1.4
    PRESSURE_SEA_LEVEL = 101325 # [Pa]
    TEMPERATURE_SEA_LEVEL = 288.15 # [K]
    AIR_DENSITY_SEA_LEVEL = 1.225 # [kg/m^3]
    DYNAMIC_VISCOSITY_SEA_LEVEL = 1.789e-5 # [kg/m/s]

    @classmethod
    def speed_of_sound_sea_level(cls):
        """ISA sea-level speed of sound [m/s], a = sqrt(gamma R T)."""
        return np.sqrt(cls.GAMMA_AIR * cls.GAS_CONSTANT_AIR * cls.TEMPERATURE_SEA_LEVEL)



# For all Assumptions
class Assumptions():

    def __init__(self):

        #Optimization switches
        self.fix_l_cg = True                   # l_cg = l_p - cg_offset
        self.fix_pylon_len = True              # pylon_len = forced_pylon_len
        self.fix_fuse_dia = True               # fuse_dia_min enforced at l_min_dia
        self.use_canards = True
        self.use_diff_thrust = True            # separate RPM for upper / lower pair
        self.steady_level = True               # equalities (True) or >= (False)

        #Flight conditions
        self.velocity = 200 / 3.6              # [m/s] 200 km/h
        self.static_margin_frac = 0.02         # (x_np - x_cg) / fuse_len
        self.alpha_stall_deg = 10.0            # [deg] local surface incidence guard
        self.prop_axis_max_deg = 12.0          # [deg] prop axis to freestream

        #Structural properties
        self.tc_pylon = 0.16                   # pylon thickness-to-chord
        self.tc_canard = 0.12                  # canard thickness-to-chord
        self.pylon_thick_min = 0.012           # [m]

        #Engine parameters
        self.perf_file = DATA_DIR / "PER3_5x37E-3_performance_file.txt"
        self.n_propellers = 4                  # two upper, two lower
        self.prop_dia = 5 * 0.0254             # [m]
        self.rpm_min = 15000
        self.rpm_max = 44000
        self.P_motor_max = 1000                # [W] per motor
        self.M_tip_max = 0.80
        self.max_diff_frac = 0.5               # |delta_T| <= frac * total thrust
        self.prop_spacing_gap = 0.01           # [m] gap between upper/lower discs

        #Mass properties
        self.aircraft_mass = 0.85              # [kg]
        self.cg_offset = 0.15                  # [m] l_cg = l_p - cg_offset

        #Fuselage dimensions
        self.fuse_dia_min = 0.07               # [m]
        self.l_min_dia = 0.10                  # [m] nose station of fuse_dia_min
        self.forced_pylon_len = 0.065          # [m]
        self.prop_clearance_gap = 0.01         # [m] prop tip to fuselage
        self.tail_margin = 0.03                # [m] pylon TE to fuselage tail
        self.canard_cg_gap = 0.02              # [m] canard AC ahead of CG

    @property
    def aircraft_weight(self):
        """[N] Weight from mass and standard gravity."""
        return self.aircraft_mass * Parameters.G0

    @property
    def prop_radius(self):
        return self.prop_dia / 2
