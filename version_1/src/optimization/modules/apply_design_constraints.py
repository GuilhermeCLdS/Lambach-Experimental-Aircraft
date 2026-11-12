import aerosandbox.numpy as np


def apply_design_constraints(optimizer):
    """Add every constraint of the design problem to ``optimizer.opti``.

    Groups: fuselage/pylon geometry, steady-level force balance, pitch trim,
    CG and static margin, longitudinal layout, differential-thrust limit,
    prop-axis and stall guards, and propeller table / power / tip-Mach limits.
    """
    opti, asm = optimizer.opti, optimizer.assumptions
    d, prop, prop_db = optimizer.design, optimizer.propulsion, optimizer.prop_db

    # Minimum fuselage diameter at the nose-based station l_min_dia.
    if asm.fix_fuse_dia:
        opti.subject_to(
            d.fuse_dia >= asm.fuse_dia_min / np.sqrt(
                1 - ((asm.l_min_dia - d.fuse_len / 2) / (d.fuse_len / 2)) ** 2
            )
        )
    else:
        opti.subject_to(d.fuse_dia >= asm.fuse_dia_min)

    # Pylon geometry and propeller clearance.
    if asm.fix_pylon_len:
        opti.subject_to(d.pylon_len == asm.forced_pylon_len)
    else:
        opti.subject_to(d.pylon_len >= asm.prop_radius + asm.prop_clearance_gap)

    opti.subject_to(d.l_t >= asm.prop_dia + asm.prop_spacing_gap)
    opti.subject_to(d.pylon_thick >= asm.pylon_thick_min)

    # Vertical force balance: aerodynamic plus propeller lift.
    if asm.steady_level:
        opti.subject_to(optimizer.lift_tot == asm.aircraft_weight)
    else:
        opti.subject_to(optimizer.lift_tot >= asm.aircraft_weight)

    # Pitch trim about the CG.
    opti.subject_to(optimizer.moment_about_cg == 0)

    # CG and longitudinal static margin.
    if asm.fix_l_cg:
        opti.subject_to(d.l_cg == d.forced_l_cg)
    else:
        opti.subject_to(d.l_cg <= optimizer.x_np)
    opti.subject_to(optimizer.x_np - d.l_cg >= asm.static_margin_frac * d.fuse_len)

    # Longitudinal layout.
    opti.subject_to(d.l_p + 0.75 * d.pylon_chord + asm.tail_margin <= d.fuse_len)
    if asm.use_canards:
        opti.subject_to(d.l_c + 0.75 * d.canard_chord <= d.l_p - 0.25 * d.pylon_chord)
        opti.subject_to(d.l_c + asm.canard_cg_gap <= d.l_cg)
        opti.subject_to(d.l_c - 0.25 * d.canard_chord >= 0)

    # Thrust balance along the freestream.
    if asm.steady_level:
        opti.subject_to(optimizer.thrust_along_velocity == optimizer.total_drag)
    else:
        opti.subject_to(optimizer.thrust_along_velocity >= optimizer.total_drag)

    # Differential-thrust limit.
    opti.subject_to(prop.delta_T <= asm.max_diff_frac * prop.thrust_tot)
    opti.subject_to(prop.delta_T >= -asm.max_diff_frac * prop.thrust_tot)

    # Propeller axis angle relative to the freestream.
    max_prop_axis = asm.prop_axis_max_deg * np.pi / 180
    opti.subject_to(prop.prop_axis_angle_to_freestream <= max_prop_axis)
    opti.subject_to(prop.prop_axis_angle_to_freestream >= -max_prop_axis)

    # Local surface incidence / stall guards.
    max_stall = asm.alpha_stall_deg * np.pi / 180
    opti.subject_to(optimizer.alpha_p <= max_stall)
    opti.subject_to(optimizer.alpha_p >= -max_stall)
    if asm.use_canards:
        opti.subject_to(optimizer.alpha_c <= max_stall)
        opti.subject_to(optimizer.alpha_c >= -max_stall)

    # Propeller table range, motor power and tip Mach.
    opti.subject_to(prop.J_top >= 0)
    opti.subject_to(prop.J_bottom >= 0)
    opti.subject_to(prop.J_top <= prop_db.J_tab_max)
    opti.subject_to(prop.J_bottom <= prop_db.J_tab_max)
    opti.subject_to(prop.P_top <= asm.P_motor_max)
    opti.subject_to(prop.P_bottom <= asm.P_motor_max)
    opti.subject_to(prop.M_tip_top <= asm.M_tip_max)
    opti.subject_to(prop.M_tip_bottom <= asm.M_tip_max)
    opti.subject_to(prop.T_top >= 0)
    opti.subject_to(prop.T_bottom >= 0)
