# -*- coding: utf-8 -*-
"""
Created on Sat May 23 12:08:06 2026

@author: Daria
"""
import numpy as np
import math

#---------------------------------------
#Geometrical parameters of the drone
#---------------------------------------

#Motor housings (bodies of revolution)
motor_number = 4
l_motor_housing = 1 #motor housing length
x_motor_housing = np.arange(0, l_motor_housing, 0.001)  #do not change
d_max_motor_housing = 1  #max diameter of motor housing body of revolution
r_motor_housing = (d_max_motor_housing / 2) * np.sqrt(
    np.clip(1 - ((x_motor_housing - l_motor_housing / 2) / (l_motor_housing / 2))**2, 0, None)
)#placeholder for radius of body of revolution as a function of distance downstream from the nose

#Main body (body of revolution)
l_main_body = 1 #main body of revolution length
x_main_body = np.arange(0, l_main_body, 0.001) #do not change
d_max_main_body = 1  #max diameter of main body of revolution
r_main_body = (d_max_main_body / 2) * np.sqrt(
    np.clip(1 - ((x_main_body - l_main_body / 2) / (l_main_body / 2))**2, 0, None)
) #placeholder for radius of body of revolution as a function of distance downstream from the nose



#Pylons (thin airfoils)
c_pylon = 1           #pylon (airfoil) chord
b_pylon = 1           #pylon span (length)
tc_max_pylon = 1      #maximum t/c ratio of pylon airfoil
max_tc_location_pylon = 1  #chord-wise location of max t/c on pylon airfoil
lambda_m = 15*math.pi/180        #sweep angle at maximum t/c line (in radians)
k_wave_drag = 0.87      #compiled by Mc-Devit and Okuno for a number of popular airfoils

#------------------------------------------
#Flow properties
#------------------------------------------
M_infty = 0.57 #freestream Mach number
#all other atmospheric properties will be assumed ISA sea level for now

#--------------------------------------------
#Wetted areas
#--------------------------------------------

#Eq. 3
def S_wet_pylon(c_pylon, b_pylon):
    k_pylon = 1.03  #assuming a thin airfoil, for a t/c larger than 15% we would have to use k = 1.06-1.09
    S_wet = 2 * c_pylon * b_pylon * k_pylon
    return S_wet

#Eq. 5
def S_wet_slender_body_of_revolution(d_max, l): #approximation valid for l/d_max > 4
    S_wet = math.pi * d_max * l * (1 - 2/3 * d_max / l) 
    return S_wet

#Eq. 4
def S_wet_body_of_revolution(r, x, d_max):  # exact formula for the area of a body of revolution
    drdx = np.gradient(r, x)
    integrand = r * np.sqrt(1 + drdx**2)
    return 2 * np.pi * np.trapezoid(integrand, x)

#--------------------------------------------------
#Friction, wave, and form drag per component
#--------------------------------------------------

#these are not compressibility-corrected yet !

def pylon_drag( c_pylon, tc_max_pylon, M_infty, k_wave_drag):
    rho = 1.225           #atmospheric density
    miu = 1.81e-5         #dynamic viscosity
    gamma = 1.4           #air specific heat ratio
    T = 288.15            #air temperature in Kelvin
    R = 287               #gas constant (air)
    v = M_infty * math.sqrt(gamma * R * T)  #freestream velocity
    q = 1/2 * rho * v**2  #dynamic pressure
    m = 6
    k_friction_2D = 0.44
    Rl = (rho * v * c_pylon) / miu  #Reynolds number experienced by the pylon

    #Friction drag (Eq. 7):
    C_f0 = k_friction_2D / (Rl**(1/m))  #friction coefficient of 2D body

    #Form factor (Eq. 12):
    # Fixed: Replaced the undeclared variable with the explicit numerical input value (0.30)
    FF = (1 + 0.6 / 0.30 * tc_max_pylon + 100 * tc_max_pylon**4) * \
         (1.34 * M_infty**0.18 * math.cos(lambda_m)**0.28)

    #Critical Mach number and wave drag: (chose the Korn Equation for now over Prandtl-Glauert)
    K = 0.87            #for conventional (non-supercritical) airfoil
    C_L = 0             #we assume symmetrical airfoil so no lift
    M_dd = K - 1/10 * C_L - tc_max_pylon          #drag divergence Mach (Eq. 16)
    M_cr = M_dd - 0.00125**(1/3)                  #critical Mach (Eq. 17)

    if M_infty > M_dd:
        C_d_wave = 20 / tc_max_pylon**15 * (M_infty - M_dd)**2   #Eq. 22-23
    elif M_infty > M_cr:
        # Added np.clip to prevent mathematical imaginary numbers from micro-negative floating point artifacts
        C_d_wave = k_wave_drag * np.clip(M_infty - M_cr, 0, None)**(3/2)          #Eq. 24
    else:
        C_d_wave = 0

    C_d_pylon = FF * C_f0 + C_d_wave #Non-compressibility-corrected drag coeffcicient of one pylon alone (no interference)
    pylon_drag = S_wet_pylon(c_pylon, b_pylon) * q * C_d_pylon
    return pylon_drag

def body_of_revolution_drag(l, d_max, r_array, x_array):

    rho = 1.225           #atmospheric density
    miu = 1.81e-5         #dynamic viscosity
    gamma = 1.4           #air specific heat ratio
    T = 288.15            #air temperature in Kelvin
    R = 287               #gas constant (air)
    v = M_infty * math.sqrt(gamma * R * T)  #freestream velocity
    q = 1/2 * rho * v**2  #dynamic pressure
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

    # Critical Mach number for a body of revolution
    dr_dx  = np.gradient(r_array, x_array)
    d2r_dx2 = np.gradient(dr_dx,  x_array)
    M_crit = 1.0 - 0.1 * np.max(np.abs(dr_dx))   # Eq. 26

    #TODO: Also estimate Mdd and the drag above Mdd? (I don't think its really necessary)

    if M_infty > M_crit:
        integrand = d2r_dx2**2
        integral  = np.trapezoid(integrand, x_array)
        # Added np.clip to prevent mathematical imaginary numbers from negative bases
        C_d_wave  = 4/(math.pi*f**2) * np.clip(M_infty - M_crit, 0, None)**(3/2) * integral  # Eq. 27-28
    else:
        C_d_wave = 0.0

    # Total drag coefficient (friction + form + wave), then drag force
    S_wet = S_wet_body_of_revolution(r_array, x_array, d_max)
    C_d_total = FF *C_f + C_d_wave #Non-compressibility-corrected drag coeffcicient of one body of revolution (no interference)
    revolve_drag = C_d_total * q * S_wet
    
    return revolve_drag
#--------------------------------------------------
#Interference drag
#--------------------------------------------------


def interference_drag(tc_max_pylon, c_pylon, b_pylon):
    rho = 1.225           #atmospheric density
    gamma = 1.4           #air specific heat ratio
    T = 288.15            #air temperature in Kelvin
    R = 287               #gas constant (air)
    v = M_infty * math.sqrt(gamma * R * T)  #freestream velocity
    q = 1/2 * rho * v**2  #dynamic pressure
    
    interference_drag = (17 * tc_max_pylon**2 - 0.05) * q * S_wet_pylon(c_pylon, b_pylon) #Eq. 15 modified to use the S_wet formula from Hoerner 

    return interference_drag


#--------------------------------------------------
#Total drag
#--------------------------------------------------

def estimate_total_drag(tc_max_pylon, c_pylon, b_pylon, l_main_body, r_main_body, x_main_body, l_motor_housing, r_motor_housing, x_motor_housing, d_max_main_body, d_max_motor_housing, M_infty, k_wave_drag):
    main_body_drag = body_of_revolution_drag(l_main_body, d_max_main_body , r_main_body , x_main_body)
    one_motor_housing_drag = body_of_revolution_drag(l_motor_housing, d_max_motor_housing, r_motor_housing, x_motor_housing)
    one_pylon_drag = pylon_drag(c_pylon, tc_max_pylon, M_infty, k_wave_drag)
    total_piecewise_drag = main_body_drag + (one_motor_housing_drag + one_pylon_drag)*motor_number #only the individual contributions, no compressibility or interactions
    total_drag = total_piecewise_drag + interference_drag(tc_max_pylon, c_pylon, b_pylon)
    compressibility_corrected_drag = total_drag/math.sqrt(1-M_infty**2)*(1 + M_infty**2/(1 + math.sqrt(1-M_infty**2))) #Eq. 28
    S_wet = S_wet_body_of_revolution(r_main_body , x_main_body, d_max_main_body) + motor_number*(S_wet_body_of_revolution(r_motor_housing , x_motor_housing , d_max_motor_housing ) + S_wet_pylon(c_pylon,b_pylon))
    
    return compressibility_corrected_drag, S_wet

