import numpy as np
import DART_ClassI_Drag_Functions
import math

# A smooth, 200-point symmetric distribution
x_main = np.linspace(0.001, 0.349, 200)
r_main = (0.075 / 2) * np.sqrt(np.clip(1 - ((x_main - 0.35 / 2) / (0.35 / 2))**2, 0, None))

x_motor = np.linspace(0.001, 0.099, 200)
r_motor = (0.045 / 2) * np.sqrt(np.clip(1 - ((x_motor - 0.10 / 2) / (0.10 / 2))**2, 0, None))

D_total, S_wet = DART_ClassI_Drag_Functions.estimate_total_drag(
    tc_max_pylon=0.09,              
    c_pylon=0.08,                   
    b_pylon=0.12,                   
    l_main_body=0.5,               
    r_main_body=r_main, 
    x_main_body=x_main, 
    l_motor_housing=0.10,           
    r_motor_housing=r_motor, 
    x_motor_housing=x_motor, 
    d_max_main_body=0.065,          
    d_max_motor_housing=0.045,      
    M_infty=0.57, 
    k_wave_drag = 0.87                  
)

print(f"Calculated Total Drag Force: {D_total:.2f} Newtons")

M_infty = 0.57
rho = 1.225           #atmospheric density
miu = 1.81e-5         #dynamic viscosity
gamma = 1.4           #air specific heat ratio
T = 288.15            #air temperature in Kelvin
R = 287               #gas constant (air)
v = M_infty * math.sqrt(gamma * R * T)  #freestream velocity
q = 1/2 * rho * v**2  #dynamic pressure
C_d = D_total/S_wet/q
print ("C_d", C_d)