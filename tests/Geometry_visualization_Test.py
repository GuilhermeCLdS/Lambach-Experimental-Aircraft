import aerosandbox as asb
import numpy as np
import plotly

n_xsecs = 12
x_positions = np.linspace(0, 4, n_xsecs)  # 4 m long
radius_profile = 0.35 * np.sin(np.pi * (x_positions / 4) ** 0.7) ** 0.7

fuselage = asb.Fuselage(
    name="Demo Fuselage",
    xsecs=[
        asb.FuselageXSec(xyz_c=[x, 0, 0], radius=r, shape=5.0)
        for x, r in zip(x_positions, radius_profile)
    ],
)

airplane = asb.Airplane(
    name="My Airplane",
    wings=[],
    fuselages=[fuselage],
)

airplane.draw(backend='plotly')