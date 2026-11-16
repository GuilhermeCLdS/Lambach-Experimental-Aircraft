"""Core aerodynamic and propulsion physics for propeller evaluation.

All forces/torques use standard propeller coefficient definitions:
    J  = V / (n * D)
    T  = Ct * rho * n^2 * D^4
    Q  = Cp * rho * n^2 * D^5 / (2*pi)

Units:
    n: rev/s
    V: m/s
    D: m
    rho: kg/m^3
"""

from __future__ import annotations

import math

from global_parameters.global_parameters import Parameters

_parameters = Parameters()
RHO = _parameters.AIR_DENSITY_SEA_LEVEL  # kg/m^3
A_SND = _parameters.speed_of_sound_sea_level()  # m/s


def thrust(prop, n_rps: float, v_mps: float) -> float:
    """Return propeller thrust [N] at rotational speed n_rps [rev/s] and airspeed v_mps [m/s]."""
    J = v_mps / (n_rps * prop.D)
    ct = prop.ct(J, n_rps)
    return ct * RHO * n_rps**2 * prop.D**4


def torque(prop, n_rps: float, v_mps: float) -> float:
    """Return propeller torque [Nm] at rotational speed n_rps [rev/s] and airspeed v_mps [m/s]."""
    J = v_mps / (n_rps * prop.D)
    cp = prop.cp(J, n_rps)
    return cp * RHO * n_rps**2 * prop.D**5 / (2.0 * math.pi)


def motor_current(prop, kv_rpm_per_v: float, i0_a: float, n_rps: float, v_mps: float) -> float:
    """Return motor current [A] for a given prop operating point.

    Model: I = Q / Kt + I0, where Kt = 1 / (KV_rad_s_per_V).
    """
    kv_rad_s_per_v = kv_rpm_per_v * 2.0 * math.pi / 60.0
    q_nm = torque(prop, n_rps, v_mps)
    return q_nm * kv_rad_s_per_v + i0_a


def tip_mach(n_rps: float, diameter_m: float, v_mps: float) -> float:
    """Return helical tip Mach number at speed v_mps [m/s]."""
    v_tip_tangential = math.pi * n_rps * diameter_m
    return math.hypot(v_tip_tangential, v_mps) / A_SND


def max_rpm_from_mach(mach_limit: float, diameter_m: float, v_mps: float) -> float:
    """Return maximum rotational speed [rev/s] allowed by tip Mach limit at airspeed v_mps."""
    # Need: sqrt((pi*n*D)^2 + V^2) <= mach_limit * a  =>  pi*n*D <= sqrt((mach_limit*a)^2 - V^2)
    max_tip_tangential = (mach_limit * A_SND) ** 2 - v_mps**2
    if max_tip_tangential <= 0:
        return 0.0
    return math.sqrt(max_tip_tangential) / (math.pi * diameter_m)
