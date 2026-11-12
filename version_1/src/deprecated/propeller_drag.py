import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

GEOM_FILE  = "5x37E-3-PERF_geometry.txt"


velocity = 300/3.6
rpm = 2000
rho = 1.225
mu = 1.81e-5
n_blades = 3
AOA = 2*np.pi/180
omega = rpm/60*2*np.pi
inch = 0.0254

def parse_xfoil_polar(filename):
    alphas = []
    cds = []
    
    with open(filename, 'r') as file:
        start_parsing = False
        for line in file:
            # Look for the header separator line to begin parsing data
            if '------' in line:
                start_parsing = True
                continue
            
            # Read numerical data rows after the header
            if start_parsing:
                parts = line.strip().split()
                if len(parts) >= 3:
                    try:
                        alpha = float(parts[0])
                        cd = float(parts[2])  # 3rd column is CD
                        alphas.append(alpha)
                        cds.append(cd)
                    except ValueError:
                        continue
                        
    return alphas, cds


cd_naca4412 = parse_xfoil_polar("xf-naca4412-il-200000.txt")[1]
alpha_of_naca4412 = np.array(parse_xfoil_polar("xf-naca4412-il-200000.txt")[0])
cd_e63 = parse_xfoil_polar("xf-e63-il-200000.txt")[1]
alpha_of_e63 = np.array(parse_xfoil_polar("xf-e63-il-200000.txt")[0])
airfoil1_end = 0.66/2.5
airfoil2_start = 2.4/2.5
cd_max = 1.2

alpha_of_naca4412_dense = np.linspace(alpha_of_naca4412.min(), alpha_of_naca4412.max(), 100)
spline_naca4412 = make_interp_spline(alpha_of_naca4412, cd_naca4412, k=3)
y_spline_naca4412 = spline_naca4412(alpha_of_naca4412_dense)

alpha_of_e63_dense = np.linspace(alpha_of_e63.min(), alpha_of_e63.max(), 100)
spline_e63 = make_interp_spline(alpha_of_e63, cd_e63, k=3)
y_spline_e63 = spline_e63(alpha_of_e63_dense)

def read_geometry(path):
    """Return station, chord, twist(deg), thickness ratio arrays (inches / deg)."""
    rows = []
    for line in open(path):
        p = line.split()
        if len(p) == 14:
            try:
                rows.append([float(x) for x in p])
            except ValueError:
                pass
    g = np.array(rows)
    return g[:, 0], g[:, 1], g[:, 8], g[:, 7]

st, chord, twist, tc = read_geometry(GEOM_FILE)
r = st*inch
c = chord*inch
edges = np.concatenate(([r[0]], 0.5 * (r[1:] + r[:-1]), [r[-1]]))
dr = np.diff(edges)

D = D_ax = Q = 0.0

for i in range(len(r)):
    v_rel = np.sqrt(velocity**2 + (omega*r[i])**2)
    s = c[i]*dr[i]

    rel_alpha = np.arctan2(omega*r[i], velocity)
    phi = np.pi/2 - rel_alpha

    alpha = np.degrees(twist[i]*np.pi/180 + AOA - phi)
    if r[i]/r[-1] < 0.264:
        cd = np.interp(alpha, alpha_of_e63_dense, y_spline_e63)
    if r[i]/r[-1] > 0.264 and r[i]/r[-1] < 0.96:
        f = np.clip((st[i] - airfoil1_end) / (airfoil2_start - airfoil1_end), 0, 1)
        cd = (1 - f) * np.interp(alpha, alpha_of_e63_dense, y_spline_e63) + f * np.interp(alpha, alpha_of_naca4412_dense, y_spline_naca4412)
    if r[i]/r[-1] > 0.96:
        cd = np.interp(alpha, alpha_of_naca4412_dense, y_spline_naca4412)

    dD = 0.5*rho*v_rel**2*s*cd
    D += dD
    D_ax += dD*np.cos(rel_alpha)
    Q += dD*r[i]*np.sin(rel_alpha)

D_ax_tot = D_ax*n_blades

print(D_ax_tot)