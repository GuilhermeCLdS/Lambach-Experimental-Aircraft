import aerosandbox as asb
import aerosandbox.numpy as np
import time
import matplotlib.pyplot as plt
import plotly

#decide on proper format for quadcopter and component parameters
canards_on = True        
wing_ac = 0.25     
canard_chord = 0.03   
canard_le = 0.1   
canard_ac = canard_le + canard_chord * 0.25   
canard_incidence = 0

#pylon_incidence = 0#
#pylon_sweep = 0#
#pylon_chord = 0.035#
#pylon_length = 0.15#

#a = fuse_length/2
#b = fuse_radius


canard_length = 0.07



#canard later
class Quadcopter():

    global_parameters = {}

    def __init__(self, fuselage_type, pylon_type): # only start with fuselage and pylon
        self.fuselage_type = fuselage_type
        self.pylon_type = pylon_type

    def start_geometry(self, fuselage_parameters, pylon_parameters):
        self.fuselage_parameters = fuselage_parameters
        self.pylon_parameters = pylon_parameters
        self.wing_elements = []
        if self.fuselage_type.lower() == "ellipse":
            self.fuselage = self.make_fuselage_Ellipse(fuselage_parameters)
        else:
            pass
        if self.pylon_type.lower() == "symmetric":
            pylons = self.aero_surface_symmetric(None, pylon_parameters, ispylon = True)
            self.wing_elements.append(pylons[0])
            self.wing_elements.append(pylons[1])
        else:
            pass
            

    @staticmethod
    def make_fuselage_Ellipse(fuse_data):
        fuse_length = fuse_data["fuselage_length"]
        fuse_radius = fuse_data["fuselage_radius"]
        n_fuse_sections = fuse_data["n_fuselage_sections"]

        beta = np.linspace(0, np.pi, n_fuse_sections)
        xs = -(fuse_length / 2) * np.cos(beta)
        radii = fuse_radius * np.sqrt(np.maximum(1 - (xs / (fuse_length / 2)) ** 2, 0.0))
        return asb.Fuselage(
            name="Ellipse Fuselage",
            xsecs=[asb.FuselageXSec(xyz_c=[x, 0, 0], radius=r) for x, r in zip(xs, radii)],
        )

    def aero_surface_symmetric(self, name, surface_data, ispylon = False, y_root = 0, z_root = 0, y_tip = 0, z_tip=0):
        #surface_data needs span, root_chord, taper_ratio (root to tip), twist, airfoil
        span = surface_data["span"]
        root_chord = surface_data["root_chord"]
        twist = surface_data["twist"]
        airfoil = surface_data["airfoil"]
        tip_chord = root_chord * surface_data["taper_ratio"]
        a = self.fuselage_parameters["fuselage_length"]/2
        b = self.fuselage_parameters["fuselage_radius"]
        fuse_length = self.fuselage_parameters["fuselage_length"]
        wing_le = 0 # define
        pylons = []

        if ispylon:
            #pylon needs pylon_x_offset
            pylon_x_offset = surface_data["pylon_x_offset"]
            x_offset = fuse_length*pylon_x_offset
            root_top = self.polar_to_cartesian(self.radius_from_ellipse(-x_offset, a, b), np.pi/4)
            x_root_top,y_root_top, z_root_top = -x_offset, root_top[0], root_top[1]

            root_bottom = self.polar_to_cartesian(self.radius_from_ellipse(-x_offset, a, b), -np.pi/4)                                                                     
            x_root_bottom, y_root_bottom, z_root_bottom = -x_offset, root_bottom[0], root_bottom[1]

            tip_top = self.polar_to_cartesian(self.radius_from_ellipse(-x_offset, a, b)+span, np.pi/4)
            x_tip_top, y_tip_top, z_tip_top, = -x_offset, tip_top[0], tip_top[1]

            tip_bottom = self.polar_to_cartesian(self.radius_from_ellipse(-x_offset, a, b)+span, -np.pi/4)
            x_tip_bottom, y_tip_bottom, z_tip_bottom = -x_offset, tip_bottom[0], tip_bottom[1]
            return [asb.Wing(name="top pylons",symmetric=True,
            xsecs=[
                asb.WingXSec(
                    xyz_le=[x_root_top, y_root_top, z_root_top],
                    chord=root_chord,
                    twist = twist,
                    airfoil=asb.Airfoil(airfoil),
                ),
                asb.WingXSec(
                    xyz_le=[x_tip_top, y_tip_top, z_tip_top],
                    chord = tip_chord,
                    twist = twist,
                    airfoil=asb.Airfoil(airfoil),
                )
            ]), asb.Wing(name="bottom pylons", symmetric=True,
            xsecs=[
                asb.WingXSec(
                    xyz_le=[x_root_bottom, y_root_bottom, z_root_bottom],
                    chord=root_chord,
                    twist = twist,
                    airfoil=asb.Airfoil(airfoil),
                ),
                asb.WingXSec(
                    xyz_le=[x_tip_bottom, y_tip_bottom, z_tip_bottom],
                    chord = tip_chord,
                    twist = twist,
                    airfoil=asb.Airfoil(airfoil),
                )
            ])]
        else:
            return asb.Wing(
                name=name,
                symmetric=True,
                xsecs=[
                    asb.WingXSec(
                        xyz_le=[0, y_root, z_root],
                        chord=root_chord,
                        twist = twist,
                        airfoil=asb.Airfoil(airfoil),
                    ),
                    asb.WingXSec(
                        xyz_le=[0, y_tip, z_tip],
                        chord = tip_chord,
                        twist = twist,
                        airfoil=asb.Airfoil(airfoil),
                    )
                ]
            )
    def make_propeller(self, name, radius, position):
        pass

    def return_asb_airplane(self):
        return asb.Airplane(name = "Quadcopter", xyz_ref = [0,0,0], wings = self.wing_elements, fuselages=[self.fuselage], propulsors = [])

    @staticmethod
    def polar_to_cartesian(r, theta):
        y = r * np.cos(theta)
        z = r * np.sin(theta)
        return y, z

    @staticmethod
    def radius_from_ellipse(x, a, b):
        return b * np.sqrt(1 - (x / a) ** 2)


fuselage_data = {"fuselage_length":0.4, "fuselage_radius": 0.03, "n_fuselage_sections": 31}
pylon_data = {"airfoil": "naca0012","incidence": 0, "sweep":0, "root_chord":0.035, "span": 0.15, "taper_ratio": 0.75, "twist": 0, "pylon_x_offset": -0.5}

test = Quadcopter(fuselage_type="ellipse", pylon_type="symmetric")
test.start_geometry(fuselage_data, pylon_data)

asb_test = test.return_asb_airplane()
asb_test.draw(backend='plotly')
