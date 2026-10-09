import aerosandbox as asb
import aerosandbox.numpy as np

#origin: where x=0 is the nose of the fuselage

opti = asb.Opti()

use_canards     = False
use_diff_thrust = True
max_diff_frac   = 0.5
canard_on = 1.0 if use_canards else 0.0

velocity = 300/3.6 #kmph -> m/s
aircraft_weight = 1.5*9.81 #N, assume a weight of 1.5 kg for the aircraft
AOA = 2*np.pi/180 #assume a 2 deg AOA for the fuselage, canard, and pylon
e_osw = 0.8 #oswald eff for low aspect ratio wings, assume 0.8 for the canard and pylon
static_margin_frac = 0.05       # [NEW] required (x_np - x_cg) as a fraction of fuselage length


def CL_alpha(AR):   # 2D to 3D lift curve slope conversion
    return 2*np.pi*AR / (2 + np.sqrt(4 + AR**2))


fuse_len = opti.variable(init_guess=0.27, lower_bound=0.20, upper_bound=0.40)
fuse_dia = opti.variable(init_guess=0.13, lower_bound=0.10, upper_bound=0.15)
fuse_dia_min = 0.10 #space for electronics and battery (depends on battery size, electronics, etc)

pylon_len = opti.variable(init_guess=0.15, lower_bound=0.10, upper_bound=0.20) #starting from the end of the fuselage, so the pylon length is the distance from the fuselage to the propeller
pylon_chord = opti.variable(init_guess=0.035, lower_bound=0.015, upper_bound=0.08)
pylon_thick = pylon_chord*0.12 #assume naca 0012 airfoil for the pylon, so thickness is 12% of chord. This is a constraint, not a variable.
prop_clearance = 0.0635 + 0.01  #5inch prop dia + 1cm clearance

canard_len = opti.variable(init_guess=0.10, lower_bound=0.07, upper_bound=0.12)
canard_chord = opti.variable(init_guess=0.03, lower_bound=0.02, upper_bound=0.05)
canard_thick = canard_chord*0.12 #assume naca 0012 airfoil for the canard, so thickness is 12% of chord. This is a constraint, not a variable.

#incidence angles of the canard and pylon, in radians. 
canard_inc = opti.variable(init_guess=3 * np.pi / 180, lower_bound=-2 * np.pi / 180, upper_bound=6 * np.pi / 180)
pylon_inc = opti.variable(init_guess=0.0, lower_bound=-3 * np.pi / 180, upper_bound=3 * np.pi / 180)

#locations:
l_s = fuse_len*0.25 #aero center of shell
l_cg = opti.variable(init_guess=0.16, lower_bound=0.10, upper_bound=0.25)
l_p = opti.variable(init_guess=0.20, lower_bound=0.15, upper_bound=0.30) # pylon aero center
l_c = opti.variable(init_guess=0.15, lower_bound=0.03, upper_bound=0.15) # canard aero center
l_t = pylon_len*np.sin(np.pi/4)*2+fuse_dia*np.sin(np.pi/4) # vert separation from top and bottom motors, assuming symmetric configuration and motors at the end of pylons

#geometry of fuselage:
a = fuse_len/2
b = fuse_dia/2
vol_fuse = (4/3)*np.pi*a*b**2
eccentricity = np.sqrt(1 - (b**2/a**2))
area_fuse_wetted = 2 * np.pi * (b ** 2) + 2 * np.pi * (a * b / eccentricity) * np.arcsin(eccentricity)

#geometry of pylon:
pylon_area = pylon_len*pylon_chord
pylon_vol = pylon_area*pylon_thick

#geometry of canard:
canard_area = canard_len*canard_chord
canard_vol = canard_area*canard_thick

#atmospheric properties:
atmosphere = asb.Atmosphere(altitude=0)
rho = atmosphere.density()
mu = atmosphere.dynamic_viscosity()
q = 0.5*rho*velocity**2 #dynamic pressure

Re_fuse = rho*velocity*fuse_len/mu
Re_pylon = rho*velocity*pylon_chord/mu
Re_canard = rho*velocity*canard_chord/mu

cf_fuse = 0.074/(Re_fuse**0.2)
cf_pylon = 0.074/(Re_pylon**0.2)
cf_canard = 0.074/(Re_canard**0.2)

fineness_ratio = fuse_len / fuse_dia
ff_ellipsoid = 1 + 1.5 * (1 / fineness_ratio) ** 1.5 + 7 * (1 / fineness_ratio) ** 3
ff_wing = 1 + 2 * 0.12 + 60 * 0.12 ** 4

area_pylon_wetted = 2*4*pylon_area  #2 sides of 4 pylons
area_canard_wetted = canard_on*2*canard_area #2 sides of 2 canards

AR_pylon = pylon_len/pylon_chord
AR_canard = canard_len/canard_chord

k_c = q * canard_area * CL_alpha(AR_canard)
k_p = q * pylon_area * CL_alpha(AR_pylon)

alpha_c = AOA + canard_inc
alpha_p = AOA*np.cos(np.pi/4) + pylon_inc

lift_pylon_single = k_p*alpha_p*np.cos(np.pi/4) #small angle approx, multiply by cos(pi/4) because the lift is at 45deg angle (X config)
lift_canard_single = canard_on * k_c*alpha_c #small angle approx

lift_tot = lift_pylon_single*4 + lift_canard_single*2

dLc_dalpha = canard_on*k_c #approximately #of the stabilizing surface (the canards)
dLp_dalpha = k_p*np.cos(np.pi/4)**2 #cos(phi) for local alpha, cos(phi) for vertical component

dMacc_dalpha = 2*(l_c-l_cg)*dLc_dalpha #of the stabilizing surface (the canards)
dMacp_dalpha = 4*(l_p-l_cg)*dLp_dalpha #of the propulsive surface (the pylons)

CL_c = CL_alpha(AR_canard) * alpha_c
CL_p = CL_alpha(AR_pylon) * alpha_p             # normal-force coefficient of the pylons

drag_induced = canard_on * q * canard_area * 2 * CL_c ** 2 / (np.pi * e_osw * AR_canard) + q * pylon_area * 4 * CL_p ** 2 / (np.pi * e_osw * AR_pylon)
drag_fuse = 0.5*rho*velocity**2*area_fuse_wetted*cf_fuse*ff_ellipsoid
drag_pylon = 0.5*rho*(velocity**2)*area_pylon_wetted*cf_pylon*ff_wing
drag_canard = 0.5*rho*(velocity**2)*area_canard_wetted*cf_canard*ff_wing

total_drag = drag_fuse + drag_pylon + drag_canard +drag_induced

if use_diff_thrust:
    delta_T = opti.variable(init_guess=0.0, lower_bound=-2.0, upper_bound=2.0)
else:
    delta_T = 0.0

moment_thrust = -delta_T * l_t / 2


e = eccentricity
lg = np.log((1 + e) / (1 - e))
alpha0 = 2 * (1 - e ** 2) / e ** 3 * (0.5 * lg - e)
beta0 = 1 / e ** 2 - (1 - e ** 2) / (2 * e ** 3) * lg
k1 = alpha0 / (2 - alpha0)
k2 = beta0 / (2 - beta0)
M_alpha_fuse = 2 * q * vol_fuse * (k2 - k1)     # N*m/rad, nose-up

#numerator = (l_s * dLs_dalpha * np.cos(AOA) - l_s * lift_canard_single*2 * np.sin(AOA) + lift_pylon_single*4 * dLp_dalpha * np.cos(AOA) - l_p * lift_pylon_single*4 * np.sin(AOA) - dMacs_dalpha - dMacp_dalpha)
#denominator = (dLs_dalpha * np.cos(AOA) - lift_canard_single*2 * np.sin(AOA) + dLp_dalpha * np.cos(AOA) - lift_pylon_single*4 * np.sin(AOA))
#x_np = numerator / denominator  #obtained from the derivation done by Guillerme

x_np = (2*dLc_dalpha * l_c + 4*dLp_dalpha * l_p - M_alpha_fuse) / (2*dLc_dalpha + 4*dLp_dalpha)

moment_about_cg = lift_canard_single * 2 * (l_cg - l_c) + M_alpha_fuse * AOA - lift_pylon_single * 4 * (l_p - l_cg) + moment_thrust

opti.subject_to(fuse_dia >= fuse_dia_min)
opti.subject_to(pylon_len >= prop_clearance)
opti.subject_to(l_t >= 2 * 0.0635 + 0.01) # adjacent props (dia 127 mm) do not touch
opti.subject_to(pylon_thick >= 0.003)   #min. 3 mm for structure/motor mount (assumption)
opti.subject_to(lift_tot >= aircraft_weight) #ensure lift is greater than weight
opti.subject_to(moment_about_cg == 0) #ensure nose-down moment about CG for stability
opti.subject_to(l_cg <= x_np) #ensure CG is in front of NP for stability
#opti.subject_to(l_cg >= 0.25*fuse_len) #just cuz
opti.subject_to(x_np - l_cg >= static_margin_frac * fuse_len)  # real static margin, was l_cg <= x_np
opti.subject_to(l_p + 0.75*pylon_chord + 0.03 <= fuse_len)   # pylon on the fuselage
opti.subject_to(l_c + 0.75*canard_chord <= l_p-0.25*pylon_chord)   # canard on the fuselage
opti.subject_to(alpha_c <= 8 * np.pi / 180)               # [NEW] stay below stall (rough)
opti.subject_to(alpha_p <= 8 * np.pi / 180)

if use_canards:                                           # canard constraints only when canards exist
    opti.subject_to(l_c + 0.02 <= l_cg)                       # canard ahead of CG
    opti.subject_to(l_c - 0.25*canard_chord >= 0)             # [FIX] canard leading edge not ahead of the nose
    opti.subject_to(l_c + 0.75*canard_chord <= l_p-0.25*pylon_chord)   # canard TE ahead of pylon LE
    opti.subject_to(alpha_c <= 8 * np.pi / 180)


if use_diff_thrust:                                       # each pair must stay >= 0 thrust, with headroom
    opti.subject_to(delta_T <= max_diff_frac * total_drag)
    opti.subject_to(delta_T >= -max_diff_frac * total_drag)


opti.minimize(total_drag)
sol = opti.solve()


print("--- ELLIPSOIDAL FUSELAGE OPTIMIZATION ---")
print(f"Canards: {'ON' if use_canards else 'OFF'} | Differential thrust: {'ON' if use_diff_thrust else 'OFF'}")
print(f"Total Drag Force:  {sol.value(total_drag):.2f} N")
print(f"  fuselage:        {sol.value(drag_fuse):.2f} N")
print(f"  pylons:          {sol.value(drag_pylon):.2f} N")
print(f"  canards:         {sol.value(drag_canard):.2f} N")
print(f"  induced:         {sol.value(drag_induced):.2f} N")
print(f"Fuselage Length:   {sol.value(fuse_len) * 1000:.1f} mm")
print(f"Fuselage Diameter: {sol.value(fuse_dia) * 1000:.1f} mm")
print(f"Fineness Ratio:    {sol.value(fineness_ratio):.2f}")
if use_canards:
    print(f"Canard Length:     {sol.value(canard_len) * 1000:.1f} mm")
    print(f"Canard Chord:      {sol.value(canard_chord) * 1000:.1f} mm")
    print(f"Canard Thickness:  {sol.value(canard_thick) * 1000:.1f} mm")
    print(f"Canard Incidence:  {np.degrees(sol.value(canard_inc)):.2f} deg")
    print(f"Canard AC (l_c):   {sol.value(l_c) * 1000:.1f} mm")
print(f"Pylon Length:      {sol.value(pylon_len) * 1000:.1f} mm")
print(f"Pylon Chord:       {sol.value(pylon_chord) * 1000:.1f} mm")
print(f"Pylon Thickness:   {sol.value(pylon_thick) * 1000:.1f} mm")
print(f"Pylon Incidence:   {np.degrees(sol.value(pylon_inc)):.2f} deg")
print(f"Lift:              {sol.value(lift_tot):.2f} N  (weight {aircraft_weight:.2f} N)")
print(f"L/D:               {sol.value(lift_tot) / sol.value(total_drag):.2f}")
print(f"Pylon AC (l_p):    {sol.value(l_p) * 1000:.1f} mm")
print(f"CG Location:       {sol.value(l_cg) * 1000:.1f} mm")
print(f"Neutral Point:     {sol.value(x_np) * 1000:.1f} mm")
print(f"Stability Margin:  {(sol.value(x_np) - sol.value(l_cg)) * 1000:.1f} mm")
if use_diff_thrust:
    D = sol.value(total_drag)
    dT = sol.value(delta_T)
    print(f"Delta T (top-bot): {dT:.3f} N  (top pair {(D + dT)/2:.2f} N, bottom pair {(D - dT)/2:.2f} N)")
    print(f"Thrust moment:     {sol.value(moment_thrust):.3e} N*m")
print(f"Trim moment:       {sol.value(moment_about_cg):.2e} N*m")