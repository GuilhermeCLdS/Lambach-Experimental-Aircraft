# Calculation of Longitudenal Static Stability for DART V1

import numpy as np



REL_TOL = 1e-4  
# dimensionless — fraction of the characteristic lift-slope/lift
# magnitude below which the denominator is treated as singular.
# Tune once real Roskam-derived magnitudes are available.



class LogitudinalStability:
    
    def __init__(self, assumptions: Assumptions, aero: Aerodynamics):
        self.assumptions = assumptions
        self.aero = aero
    
    # hidden method. All it does is take the actual values from the aero class
    def _terms(self, alpha: float):
        """
        Placeholder retrieval of the six quantities the formula needs.
        Replace each with the real call once the aero class's interface
        is settled — for now these stand in as constants so the rest of
        the class can be written/tested independently of that refactor.
        """
        l_s = 0.0          # self.aero.l_s(alpha) or similar
        l_p = 0.0          # self.aero.l_p(alpha) or similar
        l_total = 0.0
        L_s = 0.0          # self.aero.L_s(alpha)
        dLs_dalpha = 0.0   # self.aero.dLs_dalpha(alpha)
        L_p = 0.0          # self.aero.L_p(alpha)
        dLp_dalpha = 0.0   # self.aero.dLp_dalpha(alpha)
        dMacs_dalpha = 0.0 # self.aero.dMacs_dalpha(alpha)
        dMacp_dalpha = 0.0 # self.aero.dMacp_dalpha(alpha)
        return l_s, l_p, l_total, L_s, dLs_dalpha, L_p, dLp_dalpha, dMacs_dalpha, dMacp_dalpha
    

    def neutralPoint(self, alpha: float) -> float:
        
        l_s, l_p, l_total, L_s, dLs_dalpha, L_p, dLp_dalpha, dMacs_dalpha, dMacp_dalpha = self._terms(alpha)

        cos_a = np.cos(alpha)
        sin_a = np.sin(alpha)
        
        
        # See README for the derivation
        numerator = l_s * dLs_dalpha * cos_a - l_s * L_s * sin_a + l_p * dLp_dalpha * cos_a - l_p * L_p * sin_a - dMacs_dalpha - dMacp_dalpha
        
        denominator = dLs_dalpha * cos_a - L_s * sin_a + dLp_dalpha * cos_a - L_p * sin_a
        
        
        # Compares the order of the denominator with the order of 
        # the variables that makes it up to avoid dividing by near zero
        scale = max(abs(dLs_dalpha), abs(dLp_dalpha), abs(L_s), abs(L_p))
        if scale == 0.0 or abs(denominator) < REL_TOL * scale:
            return np.nan
        
        x_np = numerator / denominator
        
        # If x_np > total length of drone, then it physically makes no sense.
        # Returns np.nan
        if x_np > l_total:
            return np.nan 
    
        return x_np

    # Returns an array of x_np for given alpha range
    def range_np(self, alpha_array: np.ndarray) -> np.ndarray:
        
        return np.array([self.neutralPoint(a) for a in alpha_array])
    
    