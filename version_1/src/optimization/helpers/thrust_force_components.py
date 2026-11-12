import aerosandbox.numpy as np


def thrust_force_components(thrust_total, incidence, op_point):
    """Split total thrust into (lift, along-freestream) wind-axis components.

    Geometry axes: +x aft, +z up, so forward thrust is -x and positive
    incidence tilts it upward. AeroBuildup defines L = -F_w[z] and
    D = -F_w[x]: wind-axis +x points forward, so +x thrust opposes drag.
    """
    thrust_x_geometry = -thrust_total * np.cos(incidence)
    thrust_z_geometry = thrust_total * np.sin(incidence)

    thrust_x_wind, _, thrust_z_wind = op_point.convert_axes(
        thrust_x_geometry, 0.0, thrust_z_geometry,
        from_axes="geometry", to_axes="wind",
    )
    return -thrust_z_wind, thrust_x_wind
