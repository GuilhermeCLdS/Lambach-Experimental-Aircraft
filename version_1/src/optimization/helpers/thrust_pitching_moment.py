import aerosandbox.numpy as np


def thrust_pitching_moment(delta_thrust, thrust_total, incidence,
                           z_prop, dx_prop_from_cg, op_point):
    """Pitching moment of the four thrust vectors about the CG (nose-up +).

    The upper pair sits at +z_prop, the lower pair at -z_prop, all at
    dx_prop_from_cg aft of the CG with the same incidence. From M_y = z F_x - x F_z:
      differential part = -z_prop * delta_T * cos(i)
      incidence part    = -dx * T_total * sin(i)

    Returns (moment_body, differential_part, incidence_part); the total is
    converted to body axes so it adds directly to AeroBuildup's m_b.
    """
    moment_diff = -z_prop * delta_thrust * np.cos(incidence)
    moment_incidence = -dx_prop_from_cg * thrust_total * np.sin(incidence)

    _, moment_body, _ = op_point.convert_axes(
        0.0, moment_diff + moment_incidence, 0.0,
        from_axes="geometry", to_axes="body",
    )
    return moment_body, moment_diff, moment_incidence
