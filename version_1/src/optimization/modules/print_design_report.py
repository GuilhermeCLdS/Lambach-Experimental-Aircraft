import numpy as onp


def print_design_report(optimizer):
    """Print the solved design, aerodynamics and propulsion to the console."""
    asm, d, prop = optimizer.assumptions, optimizer.design, optimizer.propulsion
    v = optimizer.value
    deg = onp.degrees

    print("--- AIRCRAFT AERODYNAMIC OPTIMIZATION ---")
    print(f"Canards: {'ON' if asm.use_canards else 'OFF'} | "
          f"Differential thrust: {'ON' if asm.use_diff_thrust else 'OFF'}")
    print(f"Total aerodynamic drag:  {v(optimizer.total_drag):.3f} N")
    print(f"  Profile drag:          {v(optimizer.drag_profile):.3f} N")
    print(f"  Induced drag:          {v(optimizer.drag_induced):.3f} N")
    print(f"Aerodynamic lift:        {v(optimizer.lift_aero):.3f} N")
    print(f"Propeller lift:          {v(optimizer.lift_prop):.3f} N")
    print(f"Total lift:              {v(optimizer.lift_tot):.3f} N (weight {asm.aircraft_weight:.3f} N)")
    print(f"Aerodynamic L/D:         {v(optimizer.lift_aero) / v(optimizer.total_drag):.3f}")
    print(f"Total L/D (incl. props): {v(optimizer.lift_tot) / v(optimizer.total_drag):.3f}")

    print(f"Fuselage length:         {v(d.fuse_len) * 1000:.1f} mm")
    print(f"Fuselage diameter:       {v(d.fuse_dia) * 1000:.1f} mm")
    if asm.use_canards:
        print(f"Canard length:           {v(d.canard_len) * 1000:.1f} mm")
        print(f"Canard chord:            {v(d.canard_chord) * 1000:.1f} mm")
        print(f"Canard incidence:        {deg(v(d.canard_inc)):.2f} deg")
        print(f"Canard AC (nose-based):  {v(d.l_c) * 1000:.1f} mm")
    print(f"Pylon length:            {v(d.pylon_len) * 1000:.1f} mm")
    print(f"Pylon chord:             {v(d.pylon_chord) * 1000:.1f} mm")
    print(f"Pylon thickness:         {v(d.pylon_thick) * 1000:.1f} mm")
    print(f"Pylon incidence:         {deg(v(d.pylon_inc)):.2f} deg")
    print(f"Pylon AC (nose-based):   {v(d.l_p) * 1000:.1f} mm")
    print(f"CG (nose-based):         {v(d.l_cg) * 1000:.1f} mm")
    print(f"Neutral point:           {v(optimizer.x_np) * 1000:.1f} mm")
    print(f"Static margin:           {(v(optimizer.x_np) - v(d.l_cg)) * 1000:.1f} mm")
    print(f"AOA:                     {deg(v(d.AOA)):.2f} deg")
    print(f"AeroBuildup CLa:         {v(optimizer.CLa):.4f} 1/rad")
    print(f"AeroBuildup Cma:         {v(optimizer.Cma):.4f} 1/rad")

    print("--- PROPULSION ---")
    print(f"RPM upper / lower:       {v(prop.rpm_top):.0f} / {v(prop.rpm_bottom):.0f}")
    print(f"Advance ratio J:         upper {v(prop.J_top):.3f} / lower {v(prop.J_bottom):.3f} "
          f"(table limit {optimizer.prop_db.J_tab_max:.3f})")
    print(f"Thrust per propeller:    upper {v(prop.T_top):.3f} N / lower {v(prop.T_bottom):.3f} N")
    print(f"Total thrust:            {v(prop.thrust_tot):.3f} N")
    print(f"Differential thrust:     {v(prop.delta_T):.3f} N (upper pair minus lower pair)")
    print(f"Thrust along freestream: {v(optimizer.thrust_along_velocity):.3f} N")
    print(f"Propeller axis angle:    {deg(v(prop.prop_axis_angle_to_freestream)):.2f} deg")
    print(f"Thrust pitch moment:     {v(optimizer.moment_thrust):.5f} Nm")
    print(f"  Differential part:     {v(optimizer.moment_thrust_diff):.5f} Nm")
    print(f"  Incidence part:        {v(optimizer.moment_thrust_incidence):.5f} Nm")
    print(f"Aerodynamic pitch moment:{v(optimizer.moment_aero):.5f} Nm")
    print(f"Total trim moment:       {v(optimizer.moment_about_cg):.3e} Nm")
    print(f"Total shaft power:       {v(prop.power_tot):.1f} W")
    print(f"  Upper motor power:     {v(prop.P_top):.1f} W each")
    print(f"  Lower motor power:     {v(prop.P_bottom):.1f} W each")
    print(f"Shaft torque:            upper {v(prop.Q_top):.4f} / lower {v(prop.Q_bottom):.4f} Nm per motor")
    print(f"Tip Mach:                upper {v(prop.M_tip_top):.3f} / lower {v(prop.M_tip_bottom):.3f}")
    print(f"Propulsive efficiency*:  {v(optimizer.total_drag) * asm.velocity / v(prop.power_tot):.3f}")
    print("*Based on D*V / total shaft power; does not include motor/ESC losses.")

    print("--- INCIDENCE CHECKS ---")
    print(f"Pylon local alpha:       {deg(v(optimizer.alpha_p)):.2f} deg")
    if asm.use_canards:
        print(f"Canard local alpha:      {deg(v(optimizer.alpha_c)):.2f} deg")
    print(f"CG offset from pylon AC: {(v(d.l_p) - v(d.l_cg)) * 1000:.1f} mm")
    print(f"CG location along body:  {v(d.l_cg) / v(d.fuse_len) * 100:.1f} % of fuselage length")
