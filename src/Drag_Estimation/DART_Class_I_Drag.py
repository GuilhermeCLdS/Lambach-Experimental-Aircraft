# -*- coding: utf-8 -*-
"""
Created on Wed May 20 11:01:19 2026
@author: Daria
"""
import numpy as np
import math

#Geometrical parameters of the drone
motor_number = 4
l_main_body = 1 #main body of revolution length
x_main_body = np.arange(0, l_main_body, 0.001)          
l_motor_housing = 1 #motor housing body of revolution length
x_motor_housing = np.arange(0, l_motor_housing, 0.001)  

# Placeholder radius functions - replace with actual geometry.
d_max_main_body = 1  #max diameter of main body of revolution
r_main_body = (d_max_main_body / 2) * np.sqrt(
    np.clip(1 - ((x_main_body - l_main_body / 2) / (l_main_body / 2))**2, 0, None)
)

d_max_motor_housing = 1  #max diameter of motor housing body of revolution
r_motor_housing = (d_max_motor_housing / 2) * np.sqrt(
    np.clip(1 - ((x_motor_housing - l_motor_housing / 2) / (l_motor_housing / 2))**2, 0, None)
)

c_pylon = 1           #pylon (airfoil) chord
b_pylon = 1           #pylon span (length)
tc_max_pylon = 1      #maximum t/c ratio of pylon airfoil
max_tc_location_pylon = 1  #chord-wise location of max t/c on pylon airfoil
lambda_m = 1          #sweep angle at maximum t/c line

#Flow properties
M_infty = 0.57        #freestream Mach number
rho = 1.225           #air density
gamma = 1.4           #specific heat ratio of air
T = 288.15            #air temperature in Kelvin
R = 287               #gas constant (air)
v = M_infty * math.sqrt(gamma * R * T)  #freestream velocity
q = 1/2 * rho * v**2  #dynamic pressure
miu = 1.81e-5         #dynamic viscosity


def S_wet_pylon(c_pylon, b_pylon):
    k_pylon = 1.03  #assuming a thin airfoil, for a t/c larger than 15% we would have to use k = 1.06-1.09
    S_wet = 2 * c_pylon * b_pylon * k_pylon
    return S_wet


def S_wet_body_of_revolution(d_max, l):
    S_wet = math.pi * d_max * l * (1 - 2/3 * d_max / l)
    return S_wet


def pylon_drag(q, rho, v, miu, c_pylon, S_wet_pylon, tc_max_pylon, M_infty):
    """Estimates the (non-compressibility-corrected) drag of one pylon."""
    m = 6
    k_friction_2D = 0.44
    k_wave_drag = 1  #depends on fineness ratio of the body of revolution
    Rl = (rho * v * c_pylon) / miu  #Reynolds number experienced by the pylon

    #Friction drag:
    C_f0 = k_friction_2D / (Rl**(1/m))  #friction coefficient of 2D body

    #Form drag (Eq. 9):
    FF = (1 + 0.6 / max_tc_location_pylon * tc_max_pylon + 100 * tc_max_pylon**4) * \
         (1.34 * M_infty**0.18 * math.cos(lambda_m)**0.28)

    #Critical Mach number and wave drag
    K = 0.87  #for conventional (non-supercritical) airfoil
    C_L = 0   #we assume symmetrical airfoil so no lift
    M_dd = K - 1/10 * C_L - tc_max_pylon          #drag divergence Mach (Eq. 13)
    M_cr = M_dd - 0.00125**(1/3)                   #critical Mach (Eq. 14)

    if M_infty > M_dd:
        # FIX: missing * operator before (M_infty-M_dd)**2
        C_d_wave = 20 / tc_max_pylon**15 * (M_infty - M_dd)**2   #Eq. 19-20
    elif M_infty > M_cr:
        C_d_wave = k_wave_drag * (M_infty - M_cr)**(3/2)          #Eq. 21
    else:
        C_d_wave = 0

    C_d_pylon = FF * (C_f0 + C_d_wave)
    pylon_drag = S_wet_pylon(c_pylon, b_pylon) * q * C_d_pylon

    return pylon_drag


def body_of_revolution_drag(l, d_max, r_array, x_array, rho, q, v, miu, K1):
    """
    Estimates the (non-compressibility-corrected) drag of one body of revolution.

    Parameters
    ----------
    l       : float   – body length
    d_max   : float   – maximum diameter
    r_array : ndarray – radius r(x) sampled at the x stations in x_array
    x_array : ndarray – x stations along the body axis (length l)
    rho     : float   – air density
    q       : float   – dynamic pressure
    v       : float   – freestream velocity
    miu     : float   – dynamic viscosity
    K1      : float   – wave-drag pre-divergence factor (airfoil-dependent,
                        see McDevitt & Okuno 1973)

    Returns
    -------
    revolve_drag : float – drag force [N] (without compressibility correction)
    """
    m = 6
    k_friction_2D = 0.44
    k_friction_3D = 0.025
    f = l / d_max                            # fineness ratio
    Rl = (rho * v * l) / miu                 # Reynolds number based on body length

    # Friction drag (Eqs. 7-8)
    C_f0 = k_friction_2D / (Rl**(1/m))
    C_f  = C_f0 * (1 + k_friction_3D * l / (d_max * Rl**(1/5)))

    
    # Form drag (Eqs. 10-11)
    if f < 5:
        FF = 0.9 + 5 / f**1.5 + f / 400
    else:
        FF = 0.9 + 60 / f**3 + f / 400

    # Critical Mach number for a body of revolution (Eq. 23)

    # dr/dx and d2r/dx2 using numpy gradient (second-order central differences)
    dr_dx  = np.gradient(r_array, x_array)
    d2r_dx2 = np.gradient(dr_dx,  x_array)

    M_crit = 1.0 - 0.1 * np.max(np.abs(dr_dx))   # Eq. 23

#TODO: Also estimate Mdd and the drag above Mdd? (I don't think its really necessary)
    # Wave drag (Eq. 24)
    # use M_crit as the threshold consistent with Eq. 21/24.
    if M_infty > M_crit:
        # Integral of (d2r/dx2)^2 over the body length (trapezoidal rule)
        integrand = d2r_dx2**2
        integral  = np.trapz(integrand, x_array)
        C_d_wave  = K1 * (M_infty - M_crit)**(3/2) * integral  # Eq. 24
    else:
        C_d_wave = 0.0

    
    # Total drag coefficient (friction + form + wave), then drag force
    S_wet = S_wet_body_of_revolution(d_max, l)
    C_d_total = FF * (C_f + C_d_wave)
    revolve_drag = C_d_total * q * S_wet

    return revolve_drag


def interference_drag(tc_max_pylon, c_pylon):
    # Eq. 12: C_Dt = 17*(t/c)^2 - 0.05, referenced to q*t^2
    t = tc_max_pylon * c_pylon           # maximum thickness
    interference_drag = (17 * tc_max_pylon**2 - 0.05) * q * t**2
    return interference_drag

def estimate_total_drag(tc_max_pylon, c_pylon, b_pylon, l_main_body, r_main_body, x_main_body, l_motor_housing, r_motor_housing, x_motor_housing, d_max_main_body, d_max_motor_housing, M_infty, rho, q, v, miu, K1):
    main_body_drag = body_of_revolution_drag(l_main_body, d_max_main_body, r_main_body, x_main_body, rho, q, v, miu)
    motor_housing_drag = body_of_revolution_drag(l_motor_housing, d_max_motor_housing, r_motor_housing, x_motor_housing, rho, q, v, miu)
    S_wet_pylon = S_wet_pylon(c_pylon, b_pylon)
    one_pylon_drag = pylon_drag(q, rho, v, miu, c_pylon, S_wet_pylon, tc_max_pylon, M_infty)
    total_piecewise_drag = main_body_drag + (motor_housing_drag + pylon_drag)*motor_number
    total_drag = total_piecewise_drag + interference_drag(tc_max_pylon, c_pylon)