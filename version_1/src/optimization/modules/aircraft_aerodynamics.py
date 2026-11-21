import aerosandbox as asb
import aerosandbox.numpy as np
import numpy as onp
from pathlib import Path

from src.optimization.helpers.polar_to_cartesian import polar_to_cartesian
from src.optimization.helpers.radius_from_ellipse import radius_from_ellipse
from src.deprecated.propeller_drag import (BladeGeometry, XFoilPolar, AirfoilSection, PropellerAnalysis)
from src.global_parameters.global_parameters import Parameters as para, Assumptions as ass
from src.optimization.modules.design_variables import DesignVariables

N_FUSE_SECTIONS = 31
AIRFOIL1 = "NACA2412"  # NACA 4412 is too thick for the pylon chord
AIRFOIL2 = "NACA0016"

NOSE_FRAC = 0.25
TAIL_FRAC = 0.30

N_MID = 6.0     # superellipse exponent at mid-fuselage (2 = ellipse, larger = boxier)
BLEND_POW = 2.0 # how quickly the boxiness ramps up (higher = stays elliptical longer)

#canard_taper_ratio = 0.75  # tip chord / root chord

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
        self.taper_ratio = 0.75
        self.a = design.fuse_len / 2
        self.b = design.fuse_dia / 2
        self.canard_le = design.l_c - 0.25 * design.canard_chord
        self.pylon_le = design.l_p - 0.25 * design.pylon_chord
        self.pylon_le_TE = design.l_p + 0.75 * design.pylon_chord  
        self.cyl_start = design.cyl_start
    @staticmethod
    def aero_surface(name, root_chord, tip_chord, twist_deg, y_root, z_root, y_tip, z_tip, AIRFOIL):
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
                y_root=y_root, z_root=z_root, y_tip=y_tip, z_tip=z_tip, AIRFOIL=AIRFOIL2
            )
            wings.append(wing.translate([pylon_x, 0, 0]))

        # One half-canard, mirrored into left and right canards.
        if self.use_canards:
            canard_x = -self.a + self.canard_le
            canard_root_radius = radius_from_ellipse(canard_x, self.a, self.b)
            canard = self.aero_surface(
                name="canards", root_chord=d.canard_chord, tip_chord=d.canard_chord * self.taper_ratio,
                twist_deg=np.degrees(d.canard_inc),
                y_root=canard_root_radius, z_root=0.0,
                y_tip=canard_root_radius + d.canard_len, z_tip=0.0, AIRFOIL=AIRFOIL1
            )
            wings.append(canard.translate([canard_x, 0, 0]))

        return wings

    def build_fuselage(self):

                beta1 = np.linspace(0, np.pi/2, N_FUSE_SECTIONS//2)
                beta2 = np.linspace(0, np.pi/2, N_FUSE_SECTIONS//2)
                n = 2
                rho = 0.3*self.b

                return asb.Fuselage(
                    name="fuselage",
                    #the droplet-shape part of the fuselage goes from the start to the start of the canards, this is because the larger the droplet-shaped part the more aerodynamic will be the fuselagea and therefore less drag
                    #the canards are therefore the constraint and impose the maximum length of the nose
                    xsecs = [
                        asb.FuselageXSec(xyz_c=[-self.a + self.cyl_start * (1 - np.cos(b)) - 0.03, 0, 0], radius=self.b * (1 - ((self.canard_le * (1 - np.cos(b)) - self.canard_le) / self.canard_le) ** 2) ** (1 / n)
                        )
                        for b in beta1
                    ] + [
                        asb.FuselageXSec(xyz_c=[-self.a + self.cyl_start, 0, 0], radius=self.b)
                    ] + [
                        asb.FuselageXSec(xyz_c=[-self.a + np.maximum(self.pylon_le_TE, self.design.l_p + 0.04), 0, 0], radius=self.b)
                    ] + [
                        asb.FuselageXSec(xyz_c=[-self.a + self.pylon_le_TE + ((2*self.a - self.pylon_le_TE) * np.sin(b)), 0, 0], radius=np.sqrt(np.maximum(
                        self.b**2 * (2*np.sin(b)**3 - 3*np.sin(b)**2 + 1)
                        - 2*rho*(2*self.a - self.pylon_le_TE) * (np.sin(b)**3 - np.sin(b)**2),
                        0.0)))
                        for b in beta2
                    ]
                )
            #Fractions of total fuselage length (2a) taken up by each cone


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

    # def compute_interference_drag(self, velocity):
    #         """Run VLM for only the canards, then only the pylons and then for the whole body
    #         and take the difference between the drag of both bodies and the summation of both 
    #         independently to know interference drag due to bodies
    #         """
    #         d = self.design
    #         pylon = []

    #         # Pylon arms form an X in the y-z plane; each half-wing is mirrored
    #         # across y = 0, giving four arms.
    #         pylon_x = -self.a + self.pylon_le
    #         pylon_root_radius = radius_from_ellipse(pylon_x, self.a, self.b)
    #         pylon_tip_radius = pylon_root_radius + d.pylon_len
    #         for name, angle in (("pylons_upper", np.pi / 4), ("pylons_lower", -np.pi / 4)):
    #             y_root, z_root = polar_to_cartesian(pylon_root_radius, angle)
    #             y_tip, z_tip = polar_to_cartesian(pylon_tip_radius, angle)
    #             wing = self.aero_surface(
    #                 name=name, root_chord=d.pylon_chord, tip_chord=d.pylon_chord,
    #                 twist_deg=np.degrees(d.pylon_inc),
    #                 y_root=y_root, z_root=z_root, y_tip=y_tip, z_tip=z_tip, AIRFOIL=AIRFOIL2
    #             )
    #             pylon.append(wing.translate([pylon_x, 0, 0]))

    #         canard = []
    #         canard_x = -self.a + self.canard_le
    #         canard_root_radius = radius_from_ellipse(canard_x, self.a, self.b)
    #         canard = self.aero_surface(
    #             name="canards", root_chord=d.canard_chord, tip_chord=d.canard_chord * self.taper_ratio,
    #             twist_deg=np.degrees(d.canard_inc),
    #             y_root=canard_root_radius, z_root=0.0,
    #             y_tip=canard_root_radius + d.canard_len, z_tip=0.0, AIRFOIL=AIRFOIL1
    #         )
    #         canard.append(canard.translate([canard_x, 0, 0]))
            
    #         op_point = asb.OperatingPoint(velocity=velocity, alpha=np.degrees(self.design.AOA))
    #         analysis_canard = asb.VortexLatticeMethod(
    #             airplane=canard,
    #             op_point=op_point,
    #         )
    #         aero_canard = analysis_canard.run()

    #         op_point = asb.OperatingPoint(velocity=velocity, alpha=np.degrees(self.design.AOA))
    #         analysis_pylon = asb.VortexLatticeMethod(
    #             airplane=pylon,
    #             op_point=op_point,
    #         )
    #         aero_pylon = analysis_pylon.run()
    #         wings = []
    #         wings.append(canard)
    #         wings.append(pylon)
    #         op_point = asb.OperatingPoint(velocity=velocity, alpha=np.degrees(self.design.AOA))
    #         analysis_wing = asb.VortexLatticeMethod(
    #             airplane=wing,
    #             op_point=op_point,
    #         )
    #         aero_wing = analysis_wing.run()
    #         return (aero_wing['D'] - aero_canard['D'] - aero_pylon['D'])

    def compute_induced_drag_wings(self, op_point):
          #Induced drag wing part
        analysis = asb.VortexLatticeMethod(
            airplane=self.build_airplane(),
            op_point=op_point,
        )
        aero_data = analysis.run()
        D_i_wings = aero_data['D']
        return D_i_wings
    def compute_viscous_drag_wings(self, rho, visc, velocity):
        #Viscous wing drag part with Neuralfoil
        Re_canard = rho * velocity * self.design.canard_chord/visc
        Re_pylon = rho * velocity * self.design.pylon_chord/visc
        M = velocity/np.sqrt(1.4*287*288.15)
        #Airfoil1 for canard airfoil2 for pylon
        Aero_canard = AIRFOIL1.get_aero_from_neuralfoil(np.degrees(self.design.AOA), Re_canard, M, n_crit=9.0, xtr_upper=1.0, xtr_lower=1.0, model_size='xlarge', control_surfaces=None, include_360_deg_effects=True)
        #Assume turbulent flow for pylons (xtr = 0), alpha is different since the pylons are at 45 degrees with respect to the x axis
        Aero_pylon =  AIRFOIL1.get_aero_from_neuralfoil(np.degrees(np.atan(np.sqrt(2)*(np.tan(self.design.AOA))/2)), Re_pylon, M, n_crit=9.0, xtr_upper=0.0, xtr_lower=0.0, model_size='xlarge', control_surfaces=None, include_360_deg_effects=True)
        
        D_v_canard = 0.5*rho*(velocity**2)*self.design.canard_len*self.design.canard_chord*Aero_canard['CD']
        #Since there are 2 pylonds the drag is multiplied by 2
        D_v_pylon = 2*0.5*rho*(velocity**2)*self.design.pylon_len*self.design.pylon_chord*Aero_pylon['CD']
        D_v = D_v_canard + D_v_pylon
        return D_v
    def compute_fuselage_drag(self, op_point):
        #The drag of the fuselage can be obtained from aerobuildup

        fuse_airplane = asb.Airplane(name="fuse only", fuselages=[self.build_fuselage()], xyz_ref=(self.l_cg_geometry, 0, 0))

        fuse_aero = asb.AeroBuildup(airplane=fuse_airplane, op_point=op_point).run() 
        #Still does not take into account IF
        D_fuse = fuse_aero['D']
    def prop_drag(self, rho, visc, velocity):
         REPO_ROOT = Path(__file__).resolve().parents[5]
         BLADE_PATH= REPO_ROOT / "docs" / '5x75E-PERF_geometry.txt'
         INNER_AIRFOIL_PATH = REPO_ROOT / "docs" / 'xf-naca4412-il-200000.txt'
         EXTERIOR_AIRFOIL_PATH = REPO_ROOT / "docs" / 'xf-e63-il-200000.txt'
         #prop_drag is computed with the deprecated code
         Blade = BladeGeometry(BLADE_PATH)
         Airfoil_blade = AirfoilSection(INNER_AIRFOIL_PATH, EXTERIOR_AIRFOIL_PATH,
                 0.2, 0.3)#transition points arbitrary
         analysis = PropellerAnalysis(Blade, Airfoil_blade,
                 velocity, 3000, rho, visc,
                 3, self.aoa_deg)#Check number of blades and rpm

         result = analysis.run()
         total_drag = result.D
         return total_drag

    def analyze_version2(self, rho, visc, velocity):
        """Run VLM for the wings, neuralfoil for the airfoils used and then integrate 
        over the wings to get an approximate of the viscous drag. 
        The fuselage is completely ignored by VLM, its drag and the interference have to 
        be computed apart
        """
        op_point = asb.OperatingPoint(velocity=velocity, alpha=np.degrees(self.design.AOA))
        D_i = self.compute_induced_drag_wings(op_point)
        D_v = self.compute_viscous_drag_wings(rho, visc, velocity)
        D_f = self.compute_fuselage_drag(op_point)
        D_p = 4*self.prop_drag(op_point, rho, visc, velocity)

        #Drag still does not take into account interference between fuselage, propellers and wings
        #but it does take into account pressure drag due to interaction between wings and pylons
        D = D_i + D_v + D_f + D_p
        
        return D
