import aerosandbox as asb
import aerosandbox.numpy as np
import time
import matplotlib.pyplot as plt
import plotly

#canard later
class Quadcopter():

    global_parameters = {}

    def __init__(self, fuselage_type, pylon_type): # only start with fuselage and pylon
        self.fuselage_type = fuselage_type
        self.pylon_type = pylon_type

    def start_geometry(self, fuselage_parameters, pylon_parameters, cannard_parameters = None):
        self.fuselage_parameters = fuselage_parameters
        self.pylon_parameters = pylon_parameters
        self.wing_elements = []
        if self.fuselage_type.lower() == "ellipse":
            self.fuselage = [self.make_fuselage_Ellipse(fuselage_parameters)]
        else:
            pass
        if self.pylon_type.lower() == "symmetric":
            self.pylon_count = pylon_parameters["pylon_count"]
            self.wing_elements = self.aero_surface(pylon_parameters)
        else:
            pass
        if cannard_parameters:
            cannards = self.aero_surface(cannard_parameters)
            if cannards:
                for cannard in cannards:
                    self.wing_elements.append(cannard)
            

    def __repr__(self):
        return f"Quadcopter with {self.fuselage_type} fuselage and {self.pylon_count} {self.pylon_type} pylons"

    def make_fuselage_Ellipse(self, fuse_data):
        fuse_length = fuse_data["fuselage_length"]
        fuse_radius = fuse_data["fuselage_radius"]
        n_fuse_sections = fuse_data["n_fuselage_sections"]

        beta = np.linspace(0, np.pi, n_fuse_sections)
        xs = -(fuse_length / 2) * np.cos(beta)
        radii = fuse_radius * np.sqrt(np.maximum(1 - (xs / (fuse_length / 2)) ** 2, 0.0))
        return asb.Fuselage(
            name="Ellipse Fuselage",
            #xsecs=[
            #asb.FuselageXSec(xyz_c=[0.0, 0, 0.0], radius=0.0),                        # nose tip
            #asb.FuselageXSec(xyz_c=[0.5, 0, 0.0], radius=0.30),
            #asb.FuselageXSec(xyz_c=[1.5, 0, 0.0], width=0.9, height=1.1, shape=4),  # boxier cabin
            #asb.FuselageXSec(xyz_c=[4.5, 0, 0.1], width=0.8, height=0.9, shape=4),
            #asb.FuselageXSec(xyz_c=[6.5, 0, 0.3], radius=0.10),                       # tail rises
            #asb.FuselageXSec(xyz_c=[7.0, 0, 0.3], radius=0.0),                        # tail tip
            #]
            xsecs=[asb.FuselageXSec(xyz_c=[x, 0, 0], radius=r) for x, r in zip(xs, radii)],
        )
    def make_fuselage_airfoil(self, fuse_data):
        tc = None #thickness to chord function
        cc = None #camber to chord
        boxiness = 0


    def aero_surface(self, surface_data):
        #surface_data needs span, root_chord, taper_ratio (root to tip), twist, airfoil
        span = surface_data["span"]
        sweep = surface_data["sweep"]
        root_chord = surface_data["root_chord"]
        twist = surface_data["twist"]
        airfoil = surface_data["airfoil"]
        tip_chord = root_chord * surface_data["taper_ratio"]
        a = self.fuselage_parameters["fuselage_length"]/2
        b = self.fuselage_parameters["fuselage_radius"]
        fuse_length = self.fuselage_parameters["fuselage_length"]

        pylon_x_offset = surface_data["x_offset"]
        x_offset = 0.5*fuse_length - ((fuse_length)*(1-pylon_x_offset))
        
        pylons = []

        if surface_data["surface_type"] == "pylon":
            pylon_count = surface_data['pylon_count']
            angle = np.pi/2
            
            for i in range(pylon_count):
                rad_offset = min([self.radius_from_ellipse(x_offset, a, b), self.radius_from_ellipse(x_offset - root_chord, a, b)])
                root = self.polar_to_cartesian(rad_offset, angle)
                tip = self.polar_to_cartesian(span + rad_offset, angle)
                xr,yr,zr = -x_offset, root[0], root[1]
                xt,yt,zt = -x_offset+(np.sin(sweep)*span), tip[0], tip[1]
                p = asb.Wing(name=f"pylon {i}",symmetric=False,
                xsecs=[
                asb.WingXSec(
                    xyz_le=[xr, yr,zr],
                    chord=root_chord,
                    twist = twist,
                    airfoil=asb.Airfoil(airfoil),
                ),
                asb.WingXSec(
                    xyz_le=[xt, yt, zt],
                    chord = tip_chord,
                    twist = twist,
                    airfoil=asb.Airfoil(airfoil),
                )
                ])
                pylons.append(p)
                angle += (2*np.pi)/pylon_count

            return pylons
        
        if surface_data["surface_type"] == "cannard":
            rad_offset = min([self.radius_from_ellipse(x_offset, a, b), self.radius_from_ellipse(x_offset - root_chord, a, b)])
            root = self.polar_to_cartesian(rad_offset, 0)
            tip = self.polar_to_cartesian(span + rad_offset, 0)
            xr,yr,zr = -x_offset, root[0], root[1]
            xt,yt,zt = -x_offset+(np.sin(sweep)*span), tip[0], tip[1]
            return [asb.Wing(
                name="cannards",
                symmetric=True,
                xsecs=[
                    asb.WingXSec(
                        xyz_le=[xr, yr, zr],
                        chord=root_chord,
                        twist = twist,
                        airfoil=asb.Airfoil(airfoil),
                    ),
                    asb.WingXSec(
                        xyz_le=[xt, yt, zt],
                        chord = tip_chord,
                        twist = twist,
                        airfoil=asb.Airfoil(airfoil),
                    )
                ]
            )]
        else:
            print(f"Aerodynamic body type '{surface_data["surface_type"]}' not valid")

    def make_propeller(self, name, radius, position):
        pass

    def return_asb_airplane(self):
        return asb.Airplane(name = "Quadcopter", xyz_ref = [0,0,0], wings = self.wing_elements, fuselages=self.fuselage, propulsors = [])

    @staticmethod
    def polar_to_cartesian(r, theta):
        y = r * np.cos(theta)
        z = r * np.sin(theta)
        return y, z

    @staticmethod
    def radius_from_ellipse(x, a, b):
        return b * np.sqrt(1 - (x / a) ** 2)

    


fuselage_data = {"fuselage_length":0.4, "fuselage_radius": 0.03, "n_fuselage_sections": 31}
pylon_data = {"surface_type": "pylon","airfoil": "naca0012","pylon_count": 4, "incidence": 0, "sweep":np.radians(25), "root_chord":0.035, "span": 0.15, "taper_ratio": 0.15, "twist": 0, "x_offset": 0.15}
canard_data = {"surface_type": "cannard", "airfoil": "naca0012", "incidence": 0, "sweep":np.radians(45), "root_chord":0.035, "span": 0.08, "taper_ratio": 0.15, "twist": 15, "x_offset": 0.75}

test = Quadcopter(f"ellipse", "symmetric")
test.start_geometry(fuselage_data, pylon_data, canard_data)

asb_test = test.return_asb_airplane()

print(test)
asb_test.draw(backend='plotly')
