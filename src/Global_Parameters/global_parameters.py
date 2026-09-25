# Statically typed classes of parameters and assumptions

import numpy as np
import os
import sys

# For all known parameters
class Parameters():
    
    G0 = 9.80665 # [N/kg]
    GAS_CONSTANT_AIR = 287.05 # [J/kg/K]
    GAMMA_AIR = 1.4
    PRESSURE_SEA_LEVEL = 101325 # [Pa]
    TEMPERATURE_SEA_LEVEL = 288.15 # [K]
    AIR_DENSITY_SEA_LEVEL = 1.225 # [kg/m^3]
    DYNAMIC_VISCOSITY_SEA_LEVEL = 1.789e-5 # [kg/m/s]
 
 
 
# For all Assumptions    
class Assumptions():
    
    def __init__(self):
        
        #Flight conditions
            # X flight or t flight
        #Structural properties
        
        #Engine parameters
        
        #Mass properties
        
        #Fuselage dimensions