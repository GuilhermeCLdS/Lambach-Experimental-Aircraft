import aerosandbox as asb
import aerosandbox.numpy as np
import time
import matplotlib.pyplot as plt
import plotly

canards_on = True        
wing_ac = 0.25     
canard_chord = 0.03   
canard_le = 0.1   
canard_ac = canard_le + canard_chord * 0.25   

pylon_incidence = 0
canard_incidence = 0
pylon_sweep = 0
pylon_chord = 0.035
pylon_length = 0.15
fuse_length = 0.4
fuse_radius = 0.03
a = fuse_length/2
b = fuse_radius
n_fuse_sections = 31
wing_le = wing_ac-pylon_chord*0.25
canard_length = 0.07


def polar_to_cartesian(r, theta):
    y = r * np.cos(theta)
    z = r * np.sin(theta)
    return y, z

def radius_from_ellipse(x, a, b):
    return b * np.sqrt(1 - (x / a) ** 2)

def aero_surface(name, span, root_chord, tip_chord, twist, airfoil="naca0012", y_root=0, z_root=0, y_tip=0, z_tip=0):
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
                chord=tip_chord,
                twist = twist,
                airfoil=asb.Airfoil(airfoil),
            )
        ]
    )

pylons_top = aero_surface(name="pylons_top", span=0.15, root_chord=pylon_chord, tip_chord=pylon_chord, twist=pylon_incidence, airfoil="naca0012", y_root = polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b), np.pi/4)[0], z_root = polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b), np.pi/4)[1], y_tip = polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b)+pylon_length, np.pi/4)[0], z_tip = polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b)+pylon_length, np.pi/4)[1])
pylons_top_translated = pylons_top.translate([-fuse_length/2+wing_le, 0, 0])
pylons_bottom = aero_surface(name="pylons_bottom", span=0.15, root_chord=pylon_chord, tip_chord=pylon_chord, twist=pylon_incidence, airfoil="naca0012", y_root = polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b), -np.pi/4)[0], z_root = polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b), -np.pi/4)[1], y_tip = polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b)+pylon_length, -np.pi/4)[0], z_tip = polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b)+pylon_length, -np.pi/4)[1])
pylons_bottom_translated = pylons_bottom.translate([-fuse_length/2+wing_le, 0, 0]) 
canards = aero_surface(name="canards", span=0.07, root_chord=canard_chord, tip_chord=canard_chord, twist=canard_incidence, airfoil="naca0012", y_root = polar_to_cartesian(radius_from_ellipse(-fuse_length/2+canard_le, a, b), 0)[0], z_root = polar_to_cartesian(radius_from_ellipse(-fuse_length/2+canard_le, a, b), 0)[1], y_tip = polar_to_cartesian(radius_from_ellipse(-fuse_length/2+canard_le, a, b)+canard_length, 0)[0], z_tip = polar_to_cartesian(radius_from_ellipse(-fuse_length/2+canard_le, a, b)+canard_length, 0)[1])
canards_translated = canards.translate([-fuse_length/2+canard_le, 0, 0]) 



def make_fuselage(name):
    beta = np.linspace(0, np.pi, n_fuse_sections)
    xs = -(fuse_length / 2) * np.cos(beta)
    radii = fuse_radius * np.sqrt(np.maximum(1 - (xs / (fuse_length / 2)) ** 2, 0.0))
    return asb.Fuselage(
        name=name,
        xsecs=[asb.FuselageXSec(xyz_c=[x, 0, 0], radius=r) for x, r in zip(xs, radii)],
    )

fuselage = make_fuselage(name="fuselage")

def make_propeller(name, radius, position):
    return asb.Propulsor(
        name=name,
        xyz_c=position,
        radius=radius
    )

propellers_top_left = make_propeller(name="propellers_top_left", radius=0.05, position=[-fuse_length/2+wing_le+pylon_chord, polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b)+pylon_length, np.pi/4)[0], polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b)+pylon_length, np.pi/4)[1]])
propellers_bottom_left = make_propeller(name="propellers_bottom_left", radius=0.05, position=[-fuse_length/2+wing_le+pylon_chord, polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b)+pylon_length, -np.pi/4)[0], -(polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b)+pylon_length, np.pi/4)[1])])
propellers_top_right = make_propeller(name="propellers_top_right", radius=0.05, position=[-fuse_length/2+wing_le+pylon_chord, -(polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b)+pylon_length, np.pi/4)[0]), polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b)+pylon_length, np.pi/4)[1]])
propellers_bottom_right = make_propeller(name="propellers_bottom_right", radius=0.05, position=[-fuse_length/2+wing_le+pylon_chord, -(polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b)+pylon_length, np.pi/4)[0]), -(polar_to_cartesian(radius_from_ellipse(-fuse_length/2+wing_le, a, b)+pylon_length, np.pi/4)[1])])

# Canards are only added to the airplane when canards_on is True
wing_list = [pylons_top_translated, pylons_bottom_translated]
if canards_on:
    wing_list.append(canards_translated)

airplane = asb.Airplane(name="Quadcopter", xyz_ref=[0, 0, 0], wings=wing_list, fuselages=[fuselage], propulsors = [propellers_top_left, propellers_bottom_left, propellers_top_right, propellers_bottom_right])

#axs = airplane.draw_three_view()

#airplane.draw(backend='plotly')

alpha = np.linspace(-15, 15, 300)

op_point = asb.OperatingPoint(
    velocity=300/3.6,  # m/s
    alpha=alpha,  # deg
)

aero = asb.AeroBuildup(
    airplane=airplane,
    op_point=op_point,
).run()


def scalar(x):
    return float(np.ravel(x)[0])


for key in ["L", "D", "CL", "CD", "Cm"]:
    print(f"{key:>2} = {scalar(aero[key]):8.4f}")

derivs = asb.AeroBuildup(
    airplane=airplane,
    op_point=asb.OperatingPoint(velocity=10, alpha=5),
).run_with_stability_derivatives()  # computes d/d{alpha, beta, p, q, r}

for key in ["CLa", "Cma", "Cmq", "Clb", "Clp", "Cnb", "Cnr", "x_np"]:
    print(f"{key:>4} = {scalar(derivs[key]):+8.4f}")

fig, ax = plt.subplots(4, 1, figsize=(7, 9.5), sharex=True)

ax[0].plot(alpha, aero["CL"])
ax[0].set_ylabel("$C_L$ [-]")

ax[1].plot(alpha, aero["CD"])
ax[1].set_ylabel("$C_D$ [-]")
ax[1].set_ylim(bottom=0)

ax[2].plot(alpha, aero["Cm"])
ax[2].axhline(0, linewidth=0.8, linestyle="--")
ax[2].set_ylabel("$C_m$ [-]")

ax[3].plot(alpha, aero["CL"] / aero["CD"])
ax[3].set_ylabel("$C_L/C_D$ [-]")
ax[3].set_xlabel(r"Angle of attack $\alpha$ [deg]")

plt.tight_layout()
plt.show()


vlm = asb.VortexLatticeMethod(
    airplane=airplane,
    op_point=asb.OperatingPoint(velocity=10, alpha=5),
)

vlm_aero = vlm.run()

for key in ["CL", "CD", "Cm"]:
    print(f"{key:>2} = {vlm_aero[key]:8.4f}")

vlm.draw(draw_streamlines=True, recalculate_streamlines=True)

#