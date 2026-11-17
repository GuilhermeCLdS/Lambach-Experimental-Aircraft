import numpy as np

from src.global_parameters.global_parameters import Parameters as para, Assumptions as ass

#accurate for z _fws in [50,200]
#datum is pylons

ass = ass()


class MassProperties:
    def __init__(self, l_c, l_p):
        self.l_c = l_c
        self.l_p = l_p  # Assuming l_p is defined in DesignVariables
        self.z_fws = self.l_p - self.l_c
        self.z_fws_bat = 30
        self.pusher = ass.pusher
        self.rho_pla = para.rho_pla

    def calculate_mass_properties(self):
        self.components = {
        "battery": {"mass": 240, "z": self.z_fws + self.z_fws_bat + 35},
        "frame": {"mass": 100, "z": 0},
        "FS": {"mass": 23, "z": -13},
        "vtx": {"mass": 15.6, "z": self.z_fws + self.z_fws_bat + 85},
        "camera": {"mass": 16.4, "z": self.z_fws + self.z_fws_bat + 115},
        "gps": {"mass": 10, "z": 23},
        "receiver": {"mass": 7, "z": -25},
        "motor": {"mass": 34.5, "z": -50 if self.pusher else 25},
        "wiring":{"mass": 20, "z":0},
        "front_wing": {"mass": 80, "z": self.z_fws},
        "rear_wing": {"mass": 60, "z": 0},
        "fuselage": {"mass": (self.z_fws + self.z_fws_bat + 115 + 60 ) * np.pi * (35**2 - 32.75**2) * self.rho_pla,
            "z": (self.z_fws + self.z_fws_bat + 115) / 2,
        }  ,
        }

        masses = np.array([c["mass"] for c in self.components.values()])
        zs = np.array([c["z"] for c in self.components.values()])

        total_mass = masses.sum()  
        z_cg = np.sum(masses * zs) / total_mass          #from datum (pylon root)
        x_cg = self.l_p - z_cg                           #from nose (fuselage nose)
        return x_cg

    def calculate_total_mass(self):
        total_mass = sum(c["mass"] for c in self.components.values())
        return total_mass/1000  # Convert to kg