import aerosandbox as asb
import aerosandbox.numpy as np
import numpy as onp                 # plain numpy, only used to draw the propeller disks
import plotly.graph_objects as go

# =====================================================================
# PARAMETERS  (one variable per row, lengths in metres)
# =====================================================================
# --- Optimizer
opti = asb.Opti()

# --- Flight Conditions
flight_speed = 55            # m/s
flight_altitude = 0          # m

# --- fuselage (ellipsoid)
fuselage_length = 0.2662     #set for now     
fuselage_radius = 0.0728 / 2 #set for now
n_fuselage_sections = 52     #usefull for simulation, not optimization related

# --- pylons
pylon_count = 4              #set for now
pylon_airfoil = "naca0016"   #set for now
#pylon_incidence = 0         #not used here
pylon_sweep = np.radians(0)  #set for now
pylon_root_chord = 0.075     #TBD
#pylon_span = 0.060          #incorrect limit, span not variable       # measured outwards from the fuselage surface
pylon_span = 0.11521         #LIMIT not variable
pylon_taper_ratio = 1        #set for now, tip chord / root chord
pylon_twist = 0              #set 
pylon_x_offset = opti.variable(init_guess=0.75, lower_bound= 0.5, upper_bound=((fuselage_length -pylon_root_chord)/fuselage_length))      #VARIABLE (init_guess must sit inside the bounds)
#pylon_x_offset = (180 -  pylon_x_ratio * 75) / (1000 * fuselage_length)   # leading edge, as a fraction of the fuselage length from the nose

# --- canard
canard_airfoil = "naca0012"  #set for now
#canard_incidence = 0        # not used by the geometry yet
canard_sweep = opti.variable(init_guess=0, lower_bound=0, upper_bound=45) # Variable
canard_root_chord = opti.variable(init_guess=0.0787, lower_bound=0.03, upper_bound=0.12)  #VARIABLE (placeholder bounds: set your own)
canard_span = opti.variable(init_guess=0.0749, lower_bound=0.02, upper_bound=0.12)        #VARIABLE, measured outwards from the fuselage surface (placeholder bounds: set your own)
canard_taper_ratio = opti.variable(init_guess=1, upper_bound=1, lower_bound=0.2)     #VARIABLE (lower bound must stay above 0: a zero tip chord gives NaNs in the aerodynamics)
canard_twist = 0             #set for now
canard_x_offset = opti.variable(init_guess=0.25, lower_bound = 0, upper_bound = 0.5)#((fuselage_length - canard_root_chord)/fuselage_length)) #VARIABLE
#canard_x_offset = (30 - 0.25 * 78.7) / (1000 * fuselage_length)

# --- propellers (one at the tip of every pylon)
propeller_diameter = 0.127          # m
propeller_location = "leading"      # "leading" = in front of the pylon tip, "trailing" = behind it
propeller_gap = 0.005               # m, clearance between the pylon tip and the propeller disk
propeller_thrust = 2.0              # N per propeller (placeholder: set this to your own value)
propeller_axis = [-1, 0, 0]         # direction the thrust acts in; -x is forward (the nose is at -x)
air_density = 1.225                 # kg/m^3, only used for the power estimate


# =====================================================================
# HELPERS
# =====================================================================

def polar_to_cartesian(r, theta):
    """Point at distance r from the fuselage axis, angle theta around it -> (y, z)."""
    return r * np.cos(theta), r * np.sin(theta)


def radius_from_ellipse(x, a, b):
    """Radius of the ellipsoid fuselage (semi-axes a along x, b across) at position x.
    Written with np.where / np.maximum instead of `if`, so x may be an optimization variable."""
    return np.where(
        np.abs(x) > a,
        0,
        b * np.sqrt(np.maximum(1 - (x / a) ** 2, 1e-9)),     # tiny floor: the slope of sqrt is infinite at 0
    )

# =====================================================================
# FUSELAGE
# =====================================================================

half_length = fuselage_length / 2

beta = np.linspace(0, np.pi, n_fuselage_sections)
fuselage_xs = -half_length * np.cos(beta)       # nose at -half_length, tail at +half_length
fuselage_radii = fuselage_radius * np.sqrt(np.maximum(1 - (fuselage_xs / half_length) ** 2, 0.0))

fuselage = asb.Fuselage(
    name="Ellipse Fuselage",
    xsecs=[asb.FuselageXSec(xyz_c=[x, 0, 0], radius=r) for x, r in zip(fuselage_xs, fuselage_radii)],
)

# =====================================================================
# PYLONS
# =====================================================================

pylon_tip_chord = pylon_root_chord * pylon_taper_ratio

# root leading edge sits at x = -pylon_x_from_centre (negative x is towards the nose)
pylon_x_from_centre = 0.5 * fuselage_length - fuselage_length * pylon_x_offset
pylon_root_x = -pylon_x_from_centre
pylon_tip_x = -pylon_x_from_centre + np.sin(pylon_sweep) * pylon_span

# the pylon root starts on the fuselage skin: take the smaller of the radii at the root's leading and trailing edge
# (np.softmin instead of min(): min() fails on optimization variables, and a hard minimum has a corner)
pylon_rad_offset = np.softmin(
    radius_from_ellipse(pylon_x_from_centre, half_length, fuselage_radius),
    radius_from_ellipse(pylon_x_from_centre - pylon_root_chord, half_length, fuselage_radius),
    softness=0.002,         # m; a smooth min, because a hard min() has a corner that makes the optimizer chatter
)
pylon_length_from_centerline = pylon_rad_offset + pylon_span

pylons = []
for i in range(pylon_count):
    angle = np.pi / pylon_count + i * 2 * np.pi / pylon_count     
    root_y, root_z = polar_to_cartesian(pylon_rad_offset, angle)
    #tip_y, tip_z = polar_to_cartesian(pylon_span + pylon_rad_offset, angle) #VARIABLE SPAN IMPLEMENTATION
    tip_y, tip_z = polar_to_cartesian(pylon_span, angle)

    pylons.append(asb.Wing(
        name=f"pylon {i}",
        symmetric=False,
        xsecs=[
            asb.WingXSec(
                xyz_le=[pylon_root_x, root_y, root_z],
                chord=pylon_root_chord,
                twist=pylon_twist,
                airfoil=asb.Airfoil(pylon_airfoil),
            ),
            asb.WingXSec(
                xyz_le=[pylon_tip_x, tip_y, tip_z],
                chord=pylon_tip_chord,
                twist=pylon_twist,
                airfoil=asb.Airfoil(pylon_airfoil),
            ),
        ],
    ))

# =====================================================================
# CANARD
# =====================================================================

canard_tip_chord = canard_root_chord * canard_taper_ratio

canard_x_from_centre = 0.5 * fuselage_length - fuselage_length * canard_x_offset
canard_root_x = -canard_x_from_centre
canard_tip_x = -canard_x_from_centre + np.sin(canard_sweep) * canard_span

canard_rad_offset = np.softmin(
    radius_from_ellipse(canard_x_from_centre, half_length, fuselage_radius),
    radius_from_ellipse(canard_x_from_centre - canard_root_chord, half_length, fuselage_radius),
    softness=0.002,
)
canard_root_y, canard_root_z = polar_to_cartesian(canard_rad_offset, 0)
canard_tip_y, canard_tip_z = polar_to_cartesian(canard_span + canard_rad_offset, 0)

canard = asb.Wing(
    name="canard",
    symmetric=True,                                      # mirrored to both sides
    xsecs=[
        asb.WingXSec(
            xyz_le=[canard_root_x, canard_root_y, canard_root_z],
            chord=canard_root_chord,
            twist=canard_twist,
            airfoil=asb.Airfoil(canard_airfoil),
        ),
        asb.WingXSec(
            xyz_le=[canard_tip_x, canard_tip_y, canard_tip_z],
            chord=canard_tip_chord,
            twist=canard_twist,
            airfoil=asb.Airfoil(canard_airfoil),
        ),
    ],
)

# =====================================================================
# PROPELLERS
# =====================================================================

# x position of every propeller disk (all pylon tips share the same x)
if propeller_location == "leading":
    propeller_x = pylon_tip_x - propeller_gap
elif propeller_location == "trailing":
    propeller_x = pylon_tip_x + pylon_tip_chord + propeller_gap      # (pylon twist is ignored)
else:
    raise ValueError("propeller_location must be 'leading' or 'trailing'")

propeller_axis = onp.array(propeller_axis, dtype=float) / onp.linalg.norm(propeller_axis)   # unit vector

propellers = []
propeller_thrust_vectors = []                      # AeroSandbox's Propulsor has no thrust field, so keep the thrust alongside
for i, pylon in enumerate(pylons):
    tip_y, tip_z = pylon.xsecs[-1].xyz_le[1], pylon.xsecs[-1].xyz_le[2]
    propellers.append(asb.Propulsor(
        name=f"propeller {i}",
        xyz_c=[propeller_x, tip_y, tip_z],
        xyz_normal=propeller_axis,
        radius=propeller_diameter / 2,
    ))
    propeller_thrust_vectors.append(propeller_thrust * propeller_axis)      # N, in the geometry axes

# --- numbers that follow from the thrust and the diameter
propeller_area = np.pi * (propeller_diameter / 2) ** 2
disk_loading = propeller_thrust / propeller_area                                      # N/m^2
ideal_hover_power = propeller_thrust ** 1.5 / (2 * air_density * propeller_area) ** 0.5   # W per propeller, actuator-disk theory (no losses)
total_thrust_vector = sum(propeller_thrust_vectors)

# =====================================================================
# AIRPLANE
# =====================================================================

airplane = asb.Airplane(
    name="Quadcopter",
    xyz_ref=[0, 0, 0],
    wings=pylons + [canard],
    fuselages=[fuselage],
    propulsors=propellers,
)

# =====================================================================
# OPTIMIZATION
# =====================================================================

alpha = opti.variable(init_guess=5, lower_bound=0, upper_bound=12)      # deg
v = 55

op_point = asb.OperatingPoint(
    velocity=v,
    alpha=alpha,
    beta=0,
    atmosphere=asb.Atmosphere(altitude=flight_altitude),
)

aero = asb.AeroBuildup(
    airplane=airplane,
    op_point=op_point,
    #xyz_ref=[-(fuselage_length/2), 0, 0], # CENTER OF ROTATION
    model_size="small"            # slower but more accurate airfoil model ("small" is the default)
).run()

lift_req = 10

opti.subject_to([
    aero["L"] == lift_req, 
    aero["Cm"] <= 0
])

#opti.maximize(op_point.velocity)

opti.minimize(aero["D"])             # minimize() takes one argument: the objective
#opti.minimize(alpha)
sol = opti.solve()

# plug the solution into the symbolic airplane to get one made of plain numbers
opt_airplane = sol(airplane)

# =====================================================================
# RESULTS + VISUALISATION
# =====================================================================

print(f"\nlift = {sol(aero['L']):.3f} N | drag = {sol(aero['D']):.3f} N | alpha = {sol(alpha):.2f} deg")
print(f"pylon_x_offset = {sol(pylon_x_offset):.4f} | pylon length from centerline = {sol(pylon_length_from_centerline):.4f} m")
print(f"canard: root chord = {sol(canard_root_chord):.4f} m | span = {sol(canard_span):.4f} m | "
      f"taper = {sol(canard_taper_ratio):.3f} | x_offset = {sol(canard_x_offset):.4f} | sweep = {sol(canard_sweep):.4f}")
print(f"{len(propellers)} propellers ({propeller_location} edge), diameter {propeller_diameter} m, {propeller_thrust} N each")
print(f"total thrust vector: {total_thrust_vector} N | disk loading {disk_loading:.1f} N/m^2 | ideal hover power {ideal_hover_power:.1f} W per propeller")
print(v)
# --- warn if the propellers hit the fuselage or each other (checked on the optimized numbers)
hub_distance = pylon_span                                         # hub distance from the fuselage axis (the tip is pylon_span from the axis)
hub_spacing = 2 * hub_distance * onp.sin(onp.pi / pylon_count)    # distance between neighbouring hubs
x_prop = float(sol(propeller_x))
if abs(x_prop) <= half_length and hub_distance - propeller_diameter / 2 < float(radius_from_ellipse(abs(x_prop), half_length, fuselage_radius)):
    print(f"Warning: a propeller of diameter {propeller_diameter} m reaches into the fuselage")
if pylon_count > 1 and propeller_diameter > hub_spacing:
    print(f"Warning: neighbouring propellers overlap (diameter {propeller_diameter} m > hub spacing {hub_spacing:.4f} m)")

fig = opt_airplane.draw(backend="plotly", show=False)

# AeroSandbox's plotly backend does not draw propulsors, so add the propeller disks here
disk_angles = onp.linspace(0, 2 * onp.pi, 64, endpoint=False)
helper = onp.array([0.0, 0.0, 1.0]) if abs(propeller_axis[2]) < 0.9 else onp.array([0.0, 1.0, 0.0])
disk_u = onp.cross(propeller_axis, helper)
disk_u /= onp.linalg.norm(disk_u)                          # two directions lying in the disk plane
disk_v = onp.cross(propeller_axis, disk_u)
for prop in opt_airplane.propulsors:
    hub = onp.array(prop.xyz_c, dtype=float)
    rim = hub + prop.radius * (onp.outer(onp.cos(disk_angles), disk_u) + onp.outer(onp.sin(disk_angles), disk_v))
    points = onp.vstack([hub, rim])                        # point 0 = hub, the rest = rim
    n = len(disk_angles)
    fig.add_trace(go.Mesh3d(
        x=points[:, 0], y=points[:, 1], z=points[:, 2],
        i=[0] * n, j=list(range(1, n + 1)), k=list(range(2, n + 1)) + [1],      # triangle fan
        color="crimson", opacity=0.35, name=f"{prop.name}: {propeller_thrust} N", showlegend=False,
    ))
    outline = onp.vstack([rim, rim[:1]])
    fig.add_trace(go.Scatter3d(
        x=outline[:, 0], y=outline[:, 1], z=outline[:, 2], mode="lines",
        line=dict(color="crimson", width=3), showlegend=False,
    ))

fig.show()